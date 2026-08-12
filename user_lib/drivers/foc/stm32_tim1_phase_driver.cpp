#include "stm32_tim1_phase_driver.h"

#include <math.h>

/**
 * @brief 创建 TIM1 三相 PWM 驱动适配器
 *
 * @param config TIM1、使能引脚和安全许可配置
 */
stm32_tim1_phase_driver::stm32_tim1_phase_driver(
    const stm32_tim1_phase_driver_config &config)
    : config(config)
{
}

/**
 * @brief 初始化驱动并强制关闭功率输出
 *
 * @return 硬件配置有效时返回 OK
 */
foc_result stm32_tim1_phase_driver::init()
{
    initialized = false;
    output_enabled = false;

    if(!config.timer || config.timer->Instance != TIM1 ||
        !config.enable_port || config.enable_pin == 0U ||
        config.timer->Init.Period == 0U)
    {
        return foc_result::INVALID_CONFIG;
    }

    write_enable_pin(false);
    config.timer->Instance->BDTR &= ~TIM_BDTR_MOE;
    write_neutral_duty();
    initialized = true;
    return foc_result::OK;
}

/**
 * @brief 在明确许可时启动三相 PWM 并使能门极驱动
 *
 * @return 首版安全配置会固定返回 DISABLED
 */
foc_result stm32_tim1_phase_driver::enable_output_task()
{
    if(!initialized){return foc_result::NOT_INITIALIZED;}
    if(!config.allow_output){return foc_result::DISABLED;}
    if(fault_active_from_isr()){return foc_result::DRIVER_FAULT;}

    write_neutral_duty();
    if(HAL_TIM_PWM_Start(config.timer, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(config.timer, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(config.timer, TIM_CHANNEL_3) != HAL_OK)
    {
        disable_output();
        return foc_result::OUTPUT_FAULT;
    }

    config.timer->Instance->BDTR |= TIM_BDTR_MOE;
    write_enable_pin(true);
    output_enabled = true;
    return foc_result::OK;
}

/**
 * @brief 立即关闭门极使能和 TIM1 主输出
 */
void stm32_tim1_phase_driver::disable_output()
{
    write_enable_pin(false);
    if(config.timer && config.timer->Instance)
    {
        config.timer->Instance->BDTR &= ~TIM_BDTR_MOE;
        write_neutral_duty();
    }

    output_enabled = false;
}

/**
 * @brief 在控制 ISR 中写入三相比较值
 *
 * @param duty 三相归一化占空比
 *
 * @return 输出已使能且占空比有效时返回 OK
 */
foc_result stm32_tim1_phase_driver::write_duty_from_isr(
    const phase_duty &duty)
{
    if(!initialized){return foc_result::NOT_INITIALIZED;}
    if(!output_enabled){return foc_result::DISABLED;}
    if(!isfinite(duty.phase_a) || !isfinite(duty.phase_b) ||
        !isfinite(duty.phase_c) || duty.phase_a < 0.0f ||
        duty.phase_a > 1.0f || duty.phase_b < 0.0f ||
        duty.phase_b > 1.0f || duty.phase_c < 0.0f ||
        duty.phase_c > 1.0f)
    {
        return foc_result::INVALID_ARGUMENT;
    }

    uint32_t period = config.timer->Init.Period + 1U;
    config.timer->Instance->CCR1 = (uint32_t)(duty.phase_a *
        (float)period);
    config.timer->Instance->CCR2 = (uint32_t)(duty.phase_b *
        (float)period);
    config.timer->Instance->CCR3 = (uint32_t)(duty.phase_c *
        (float)period);
    return foc_result::OK;
}

/**
 * @brief 查询 TIM1 硬件 Break 标志
 *
 * @return 检测到 Break 标志时返回 true
 */
bool stm32_tim1_phase_driver::fault_active_from_isr() const
{
    if(!config.timer || !config.timer->Instance){return true;}
    return (config.timer->Instance->SR & TIM_SR_BIF) != 0U;
}

/**
 * @brief 直接写门极驱动使能 GPIO
 *
 * @param enabled 期望的逻辑使能状态
 */
void stm32_tim1_phase_driver::write_enable_pin(bool enabled)
{
    bool pin_high = enabled == config.enable_active_high;
    if(pin_high)
    {
        config.enable_port->BSRR = config.enable_pin;
    }
    else
    {
        config.enable_port->BSRR = (uint32_t)config.enable_pin << 16U;
    }
}

/**
 * @brief 把三相比较寄存器恢复为百分之五十中性占空比
 */
void stm32_tim1_phase_driver::write_neutral_duty()
{
    if(!config.timer || !config.timer->Instance){return;}

    uint32_t neutral_compare = (config.timer->Init.Period + 1U) / 2U;
    config.timer->Instance->CCR1 = neutral_compare;
    config.timer->Instance->CCR2 = neutral_compare;
    config.timer->Instance->CCR3 = neutral_compare;
}

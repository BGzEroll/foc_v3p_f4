#include "tim1_phase_driver.h"

#include <math.h>

/**
 * @brief 创建 TIM1 三相 PWM 驱动适配器
 *
 * @param config TIM1、使能引脚和安全许可配置
 */
tim1_phase_driver::tim1_phase_driver(
    const tim1_phase_driver_config &config)
    : config(config)
{
}

/**
 * @brief 初始化驱动并强制关闭功率输出
 *
 * @return 硬件配置有效时返回 OK
 */
foc_result tim1_phase_driver::init()
{
    initialized = false;
    pwm_started = false;
    output_enabled = false;

    if(!config.timer || config.timer->Instance != TIM1 ||
        !config.enable_port || config.enable_pin == 0U ||
        config.timer->Init.Period == 0U || !channels_valid())
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
 * @brief 在明确许可时启动或重新使能三相 PWM
 *
 * @return 输出启动结果
 */
foc_result tim1_phase_driver::enable_output_task()
{
    if(!initialized){return foc_result::NOT_INITIALIZED;}
    if(!config.allow_output){return foc_result::DISABLED;}
    if(fault_active_from_isr()){return foc_result::DRIVER_FAULT;}

    write_neutral_duty();
    if(!pwm_started)
    {
        if(HAL_TIM_PWM_Start(config.timer, TIM_CHANNEL_1) != HAL_OK)
        {
            disable_output();
            return foc_result::OUTPUT_FAULT;
        }
        if(HAL_TIM_PWM_Start(config.timer, TIM_CHANNEL_2) != HAL_OK)
        {
            HAL_TIM_PWM_Stop(config.timer, TIM_CHANNEL_1);
            disable_output();
            return foc_result::OUTPUT_FAULT;
        }
        if(HAL_TIM_PWM_Start(config.timer, TIM_CHANNEL_3) != HAL_OK)
        {
            HAL_TIM_PWM_Stop(config.timer, TIM_CHANNEL_2);
            HAL_TIM_PWM_Stop(config.timer, TIM_CHANNEL_1);
            disable_output();
            return foc_result::OUTPUT_FAULT;
        }

        pwm_started = true;
    }

    config.timer->Instance->BDTR |= TIM_BDTR_MOE;
    write_enable_pin(true);
    output_enabled = true;
    return foc_result::OK;
}

/**
 * @brief 立即关闭门极使能和 TIM1 主输出
 */
void tim1_phase_driver::disable_output()
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
foc_result tim1_phase_driver::write_duty_from_isr(
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
    write_compare(config.phase_a_channel,
        (uint32_t)(duty.phase_a * (float)period));
    write_compare(config.phase_b_channel,
        (uint32_t)(duty.phase_b * (float)period));
    write_compare(config.phase_c_channel,
        (uint32_t)(duty.phase_c * (float)period));
    return foc_result::OK;
}

/**
 * @brief 查询 TIM1 硬件 Break 标志
 *
 * @return 检测到 Break 标志时返回 true
 */
bool tim1_phase_driver::fault_active_from_isr() const
{
    if(!config.timer || !config.timer->Instance){return true;}
    return (config.timer->Instance->SR & TIM_SR_BIF) != 0U;
}

/**
 * @brief 校验三个逻辑相通道是否完整且互不重复
 *
 * @return 通道恰好覆盖一至三时返回 true
 */
bool tim1_phase_driver::channels_valid() const
{
    uint8_t channel_mask = 0U;
    const uint8_t channels[3] =
    {
        config.phase_a_channel,
        config.phase_b_channel,
        config.phase_c_channel
    };

    for(uint8_t channel : channels)
    {
        if(channel < 1U || channel > 3U)
        {
            return false;
        }

        uint8_t channel_bit = (uint8_t)(1U << (channel - 1U));
        if((channel_mask & channel_bit) != 0U)
        {
            return false;
        }

        channel_mask |= channel_bit;
    }

    return channel_mask == 0x07U;
}

/**
 * @brief 按配置的通道编号写入比较值
 *
 * @param channel 通道编号
 * @param compare 比较值
 */
void tim1_phase_driver::write_compare(uint8_t channel,
    uint32_t compare)
{
    switch(channel)
    {
        case 1U:
            config.timer->Instance->CCR1 = compare;
            break;
        case 2U:
            config.timer->Instance->CCR2 = compare;
            break;
        case 3U:
            config.timer->Instance->CCR3 = compare;
            break;
        default:
            break;
    }
}

/**
 * @brief 直接写门极驱动使能 GPIO
 *
 * @param enabled 期望的逻辑使能状态
 */
void tim1_phase_driver::write_enable_pin(bool enabled)
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
void tim1_phase_driver::write_neutral_duty()
{
    if(!config.timer || !config.timer->Instance){return;}

    uint32_t neutral_compare = (config.timer->Init.Period + 1U) / 2U;
    config.timer->Instance->CCR1 = neutral_compare;
    config.timer->Instance->CCR2 = neutral_compare;
    config.timer->Instance->CCR3 = neutral_compare;
}

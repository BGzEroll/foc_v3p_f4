#include "sys_time.h"

#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"

static volatile bool initialized = false;

/**
 * @brief 确保 CubeMX 已配置的微秒定时器开始自由运行计数
 *
 * @note 所有公共 API 都会先调用此函数，仅首次调用会启动定时器。
 */
static void init()
{
    if(initialized){return;}

    uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();

    if(!initialized)
    {
        if(htim5.Instance != TIM5 ||
            HAL_TIM_Base_Start(&htim5) != HAL_OK)
        {
            Error_Handler();
        }

        initialized = true;
    }

    if(interrupt_mask == 0U){__enable_irq();}
}

/**
 * @brief 延迟指定毫秒数
 *
 * @param duration_ms 延迟时长，单位毫秒
 *
 * @note 调度器运行后的任务上下文使用 vTaskDelay()；中断上下文使用
 *       微秒定时器忙等，不应在中断中请求较长延时。
 */
void sys_time::delay_ms(uint32_t duration_ms)
{
    init();
    if(duration_ms == 0U){return;}

    if(__get_IPSR() == 0U &&
        xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        TickType_t delay_ticks = pdMS_TO_TICKS(duration_ms);
        if(delay_ticks == 0U){delay_ticks = 1U;}
        vTaskDelay(delay_ticks);
        return;
    }

    if(__get_IPSR() == 0U)
    {
        HAL_Delay(duration_ms);
        return;
    }

    while(duration_ms > 0U)
    {
        delay_us(1000U);
        duration_ms--;
    }
}

/**
 * @brief 使用微秒定时器进行微秒级忙等待
 *
 * @param duration_us 延迟时长，单位微秒
 *
 * @note 此函数不会让出 CPU，适合短暂硬件时序，不应用于任务级长延时。
 */
void sys_time::delay_us(uint32_t duration_us)
{
    init();
    uint32_t start_us = get_us_tick();
    while(get_us_tick() - start_us < duration_us)
    {
    }
}

/**
 * @brief 获取系统毫秒时基计数
 *
 * @return 系统启动后的毫秒计数，约 49.7 天自然回卷
 */
uint32_t sys_time::get_ms_tick()
{
    init();
    return HAL_GetTick();
}

/**
 * @brief 获取定时器微秒时基计数
 *
 * @return 定时器启动后的微秒计数，约 71.6 分钟自然回卷
 */
uint32_t sys_time::get_us_tick()
{
    init();
    return __HAL_TIM_GET_COUNTER(&htim5);
}

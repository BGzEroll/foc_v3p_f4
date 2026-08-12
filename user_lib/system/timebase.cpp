#include "timebase.h"

#include "stm32f4xx_hal.h"

static constexpr uint32_t TIMEBASE_FREQUENCY_HZ = 1000000U;

static bool initialized = false;

/**
 * @brief 获取 APB1 定时器输入时钟频率
 *
 * @return APB1 定时器时钟频率，单位赫兹
 */
static uint32_t timer_clock_hz()
{
    uint32_t timer_clock = HAL_RCC_GetPCLK1Freq();
    if((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
    {
        timer_clock *= 2U;
    }

    return timer_clock;
}

/**
 * @brief 读取 TIM5 自由运行微秒计数
 *
 * @return 当前 32 位微秒时间戳，约 71.6 分钟自然回卷
 */
uint32_t timebase::now_us()
{
    if(!initialized){return 0U;}
    return TIM5->CNT;
}

/**
 * @brief 初始化 TIM5 为 1 MHz 32 位自由运行时基
 *
 * @return 时钟可以精确分频且初始化成功时返回 true
 */
bool timebase::init()
{
    if(initialized){return true;}

    uint32_t input_clock_hz = timer_clock_hz();
    if(input_clock_hz < TIMEBASE_FREQUENCY_HZ ||
        input_clock_hz % TIMEBASE_FREQUENCY_HZ != 0U)
    {
        return false;
    }

    __HAL_RCC_TIM5_CLK_ENABLE();
    __HAL_RCC_TIM5_FORCE_RESET();
    __HAL_RCC_TIM5_RELEASE_RESET();

    TIM5->PSC = input_clock_hz / TIMEBASE_FREQUENCY_HZ - 1U;
    TIM5->ARR = UINT32_MAX;
    TIM5->CNT = 0U;
    TIM5->EGR = TIM_EGR_UG;
    TIM5->SR = 0U;
    TIM5->CR1 = TIM_CR1_CEN;

    initialized = true;
    return true;
}

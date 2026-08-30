#include "foc_dev.h"

#include "drivers/foc/sensors/encoder/as5600_rotor_sensor.h"
#include "system/sys_time.h"
#include "adc.h"
#include "main.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"

extern "C"
{
#include "third-parts/SguanFOC/SguanFOC.h"
}

static constexpr uint8_t AS5600_I2C_BUS_ID = 0;
static constexpr uint8_t AS5600_I2C_ADDRESS = 0x36;
static constexpr uint16_t FOC_SENSOR_TASK_STACK_DEPTH = 512;
static constexpr uint16_t SGUAN_TASK_STACK_DEPTH = 768;
static constexpr UBaseType_t FOC_SENSOR_TASK_PRIORITY =
    tskIDLE_PRIORITY + 4;
static constexpr UBaseType_t SGUAN_TASK_PRIORITY = tskIDLE_PRIORITY + 2;
static constexpr uint32_t FOC_SENSOR_UPDATE_PERIOD_MS = 1;
static constexpr uint32_t SGUAN_UPDATE_PERIOD_MS = 1;
static constexpr float ADC_REFERENCE_VOLTAGE_V = 3.3f;
static constexpr float ADC_FULL_SCALE_COUNT = 4095.0f;
static constexpr float BUS_VOLTAGE_DIVIDER_RATIO = 11.0f;

static as5600_rotor_sensor rotor(AS5600_I2C_BUS_ID,
    AS5600_I2C_ADDRESS);
static volatile bool rotor_ready = false;

extern "C"
{
    volatile float sguan_encoder_angle_rad = 0.0f;
    volatile float sguan_encoder_velocity_rad_s = 0.0f;
    volatile uint32_t sguan_encoder_timestamp_us = 0;
}

/**
 * @brief 启动母线电压连续采样
 *
 * ADC3 的连续转换结果供 SguanFOC 的低频保护任务读取。
 *
 * @return ADC 启动成功时返回 true
 */
static bool start_bus_voltage_sampling()
{
    return HAL_ADC_Start(&hadc3) == HAL_OK;
}

/**
 * @brief 启动 TIM8 三相 PWM 和 ADC2 注入采样
 *
 * ADC2 的注入组由 TIM8_CC4 触发，转换完成中断驱动 SguanFOC 高速环。
 * 三个相 PWM 通道和 CH4 触发通道都需要启动。
 *
 * @return 外设全部启动成功时返回 true
 */
static bool start_current_sampling()
{
    MOTOR_EN_GPIO_Port->BSRR = (uint32_t)MOTOR_EN_Pin << 16;
    TIM8->BDTR &= ~TIM_BDTR_MOE;

    uint32_t neutral_compare = (htim8.Init.Period + 1) / 2;
    TIM8->CCR1 = neutral_compare;
    TIM8->CCR2 = neutral_compare;
    TIM8->CCR3 = neutral_compare;

    HAL_NVIC_SetPriority(ADC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);

    if(HAL_ADCEx_InjectedStart_IT(&hadc2) != HAL_OK)
    {
        HAL_NVIC_DisableIRQ(ADC_IRQn);
        return false;
    }

    if(HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4) != HAL_OK)
    {
        HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_4);
        HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
        HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
        HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
        HAL_ADCEx_InjectedStop_IT(&hadc2);
        HAL_NVIC_DisableIRQ(ADC_IRQn);
        return false;
    }

    TIM8->BDTR &= ~TIM_BDTR_MOE;
    return true;
}

/**
 * @brief 周期读取 AS5600，并把最新机械角缓存给高速环
 *
 * @param argument FreeRTOS 任务参数
 */
static void foc_sensor_task_entry(void *argument)
{
    (void)argument;
    TickType_t last_wake_time = xTaskGetTickCount();

    while(rotor.init() != foc_result::OK)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    rotor_sample initial_sample{};
    if(rotor.read_task(initial_sample) == foc_result::OK)
    {
        sguan_encoder_angle_rad = initial_sample.mechanical_angle_rad;
        sguan_encoder_velocity_rad_s =
            initial_sample.mechanical_velocity_rad_s;
        sguan_encoder_timestamp_us = initial_sample.timestamp_us;
        rotor_ready = true;
    }

    while(true)
    {
        if(rotor.update_task() == foc_result::OK)
        {
            rotor_sample sample{};
            if(rotor.read_task(sample) == foc_result::OK)
            {
                sguan_encoder_angle_rad = sample.mechanical_angle_rad;
                sguan_encoder_velocity_rad_s =
                    sample.mechanical_velocity_rad_s;
                sguan_encoder_timestamp_us = sample.timestamp_us;
            }
        }

        vTaskDelayUntil(&last_wake_time,
            pdMS_TO_TICKS(FOC_SENSOR_UPDATE_PERIOD_MS));
    }
}

/**
 * @brief 运行 SguanFOC 的主循环和低频状态机
 *
 * 高速电流环由 ADC2 注入转换中断驱动，本任务只负责初始化、保护和状态机。
 *
 * @param argument FreeRTOS 任务参数
 */
static void sguan_task_entry(void *argument)
{
    (void)argument;
    TickType_t last_wake_time = xTaskGetTickCount();

    while(!rotor_ready)
    {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    while(true)
    {
        SguanFOC_main_Loop();
        SguanFOC_Low_Loop();

        vTaskDelayUntil(&last_wake_time,
            pdMS_TO_TICKS(SGUAN_UPDATE_PERIOD_MS));
    }
}

/**
 * @brief 初始化 AS5600、母线采样、TIM8/ADC2 和 SguanFOC 调度任务
 */
void foc_dev::init()
{
    if(!start_bus_voltage_sampling() ||
        !start_current_sampling())
    {
        Error_Handler();
    }

    // SguanFOC 全局对象复位后处于 STANDBY，需要显式进入初始化状态。
    Sguan.status = MOTOR_STATUS_UNINITIALIZED;

    BaseType_t sensor_task_result = xTaskCreate(foc_sensor_task_entry,
        "foc_sensor",
        FOC_SENSOR_TASK_STACK_DEPTH,
        nullptr,
        FOC_SENSOR_TASK_PRIORITY,
        nullptr);
    BaseType_t sguan_task_result = xTaskCreate(sguan_task_entry,
        "sguan_foc",
        SGUAN_TASK_STACK_DEPTH,
        nullptr,
        SGUAN_TASK_PRIORITY,
        nullptr);

    if(sensor_task_result != pdPASS || sguan_task_result != pdPASS)
    {
        Error_Handler();
    }
}

/**
 * @brief 处理 ADC 全局中断并分派 ADC2 注入转换完成事件
 */
extern "C" void ADC_IRQHandler(void)
{
    HAL_ADC_IRQHandler(&hadc2);
}

/**
 * @brief 在 ADC2 注入转换完成时推进 SguanFOC 高速电流环
 *
 * @param adc ADC 外设句柄
 */
extern "C" void HAL_ADCEx_InjectedConvCpltCallback(
    ADC_HandleTypeDef *adc)
{
    if(adc && adc->Instance == ADC2)
    {
        SguanFOC_High_Loop();
    }
}

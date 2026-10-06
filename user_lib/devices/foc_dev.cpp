#include "foc_dev.h"

#include "drivers/foc/sensors/current_sense/stm32_two_shunt_current_sensor.h"
#include "drivers/foc/sensors/encoder/as5600_rotor_sensor.h"
#include "adc.h"
#include "main.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"
#include "drivers/foc/sensorless/pll_experiment.h"
#ifdef PLL_ENABLE_RTT
#include "debug/pll_rtt.h"
#endif

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
static constexpr float ADC_FULL_SCALE_COUNTS = 4096.0f;
static constexpr float CURRENT_AMPLIFIER_GAIN = 50.0f;
static constexpr float CURRENT_SHUNT_RESISTANCE_OHM = 0.01f;
static constexpr float CURRENT_AMPERE_PER_COUNT =
    ADC_REFERENCE_VOLTAGE_V /
    (ADC_FULL_SCALE_COUNTS * CURRENT_AMPLIFIER_GAIN *
        CURRENT_SHUNT_RESISTANCE_OHM);

static as5600_rotor_sensor rotor(AS5600_I2C_BUS_ID,
    AS5600_I2C_ADDRESS);
static stm32_two_shunt_current_config current_sense_config{
    &hadc2,
    CURRENT_AMPERE_PER_COUNT,
    CURRENT_AMPERE_PER_COUNT,
    1,
    -1,
    two_shunt_phase_mapping::AC
};
static stm32_two_shunt_current_sensor current_sense(current_sense_config);
static sguan_foc_wrapper motor_instance;
static volatile bool rotor_ready = false;

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

    // ADC2 TIM8_CC4 trigger follows the timer channel output and is gated by
    // MOE on STM32F407. Keep neutral PWM alive with MOTOR_EN low, otherwise
    // the zero-current calibration never receives a conversion.
    TIM8->BDTR |= TIM_BDTR_MOE;
    return true;
}

/**
 * @brief 在任务上下文初始化 AS5600 并持续更新其 Topic
 *
 * @param argument FreeRTOS 任务参数
 */
static void foc_sensor_task_entry(void *argument)
{
    while(rotor.init() != foc_result::OK)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    rotor_sample initial_sample{};
    while(rotor.read_task(initial_sample) != foc_result::OK)
    {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    rotor_ready = true;

    TickType_t last_wake_time = xTaskGetTickCount();
    while(true)
    {
        rotor.update_task();
        vTaskDelayUntil(&last_wake_time,
            pdMS_TO_TICKS(FOC_SENSOR_UPDATE_PERIOD_MS));
    }
}

/**
 * @brief 运行 SguanFOC 初始化、主循环和低频状态机
 *
 * 高速电流环由 ADC2 注入转换中断驱动，本任务负责依赖初始化、校准、
 * 第三方 FOC 初始化和低频状态机。
 *
 * @param argument FreeRTOS 任务参数
 */
static void sguan_task_entry(void *argument)
{
    while(!rotor_ready)
    {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    sguan_foc_config config = sguan_foc_wrapper::default_config();
    if(motor_instance.init(config) != foc_result::OK)
    {
        Error_Handler();
    }

    foc_result init_foc_result = motor_instance.init_foc();
    if(init_foc_result != foc_result::CALIBRATING &&
        init_foc_result != foc_result::OK)
    {
        Error_Handler();
    }

    bool default_command_sent = false;
    TickType_t last_wake_time = xTaskGetTickCount();
    while(true)
    {
        motor_instance.service();
        if(motor_instance.ready() && !default_command_sent)
        {
#ifdef PLL_AUTO_RUN_ENCODER
            motor_instance.set_controller(
                motion_control_type::TORQUE);
            motor_instance.move(0.10f);
            motor_instance.enable();
#else
            motor_instance.disable();
#endif
            default_command_sent = true;
        }
        if(pll_experiment.request == 3 && motor_instance.ready() && !pll_experiment.fault)
        {
            // This task owns start commands; the ISR owns trial/stop commands.
            pll_experiment.request = 0;
            motor_instance.set_controller(motion_control_type::TORQUE);
            motor_instance.move(0.10f);
            motor_instance.enable();
        }
#ifdef PLL_ENABLE_RTT
        pll_rtt_poll();
#endif

        vTaskDelayUntil(&last_wake_time,
            pdMS_TO_TICKS(SGUAN_UPDATE_PERIOD_MS));
    }
}

/**
 * @brief 获取板级使用的 SguanFOC wrapper
 *
 * @return 板级唯一的 SguanFOC wrapper 实例
 */
sguan_foc_wrapper &foc_dev::motor()
{
    return motor_instance;
}

/**
 * @brief 绑定传感器、启动 TIM8/ADC2 并创建 FOC 调度任务
 */
void foc_dev::init()
{
    if(motor_instance.link_sensor(rotor) != foc_result::OK ||
        motor_instance.link_current_sense(current_sense) !=
            foc_result::OK)
    {
        Error_Handler();
    }

    if(!start_bus_voltage_sampling() ||
        !start_current_sampling())
    {
        Error_Handler();
    }

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
        motor_instance.loop_foc();
    }
}

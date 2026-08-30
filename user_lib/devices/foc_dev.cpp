#include "foc_dev.h"

#include "drivers/foc/foc.h"
#include "drivers/foc/foc_commissioning.h"
#include "drivers/foc/foc_runtime.h"
#include "drivers/foc/phase_driver/tim1_phase_driver.h"
#include "drivers/foc/sensors/current_sense/stm32_two_shunt_current_sensor.h"
#include "drivers/foc/sensors/encoder/as5600_rotor_sensor.h"
#include "system/sys_time.h"
#include "adc.h"
#include "main.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"

static constexpr uint8_t AS5600_I2C_BUS_ID = 0;
static constexpr uint8_t AS5600_I2C_ADDRESS = 0x36;
static constexpr uint16_t FOC_SENSOR_TASK_STACK_DEPTH = 512;
static constexpr uint16_t FOC_SAFETY_TASK_STACK_DEPTH = 768;
static constexpr UBaseType_t FOC_SENSOR_TASK_PRIORITY =
    tskIDLE_PRIORITY + 4;
static constexpr UBaseType_t FOC_SAFETY_TASK_PRIORITY =
    tskIDLE_PRIORITY + 2;
static constexpr uint32_t FOC_SENSOR_UPDATE_PERIOD_MS = 1;
static constexpr uint32_t FOC_SAFETY_UPDATE_PERIOD_MS = 10;
static constexpr uint16_t FOC_CONTROL_ISR_DIVIDER = 2;
static constexpr float ADC_REFERENCE_VOLTAGE_V = 3.3f;
static constexpr float ADC_FULL_SCALE_COUNT = 4095.0f;
static constexpr float CURRENT_SHUNT_RESISTANCE_OHM = 0.01f;
static constexpr float CURRENT_AMPLIFIER_GAIN = 50.0f;
static constexpr float CURRENT_AMPERE_PER_COUNT =
    ADC_REFERENCE_VOLTAGE_V /
    (ADC_FULL_SCALE_COUNT * CURRENT_SHUNT_RESISTANCE_OHM *
        CURRENT_AMPLIFIER_GAIN);
static constexpr uint8_t MOTOR_POLE_PAIRS = 7;
static constexpr float MOTOR_BUS_VOLTAGE_V = 12.0f;
static constexpr float MOTOR_VOLTAGE_LIMIT_V = 3.0f;
static constexpr float MOTOR_PHASE_CURRENT_LIMIT_A = 1.8f;
static constexpr float CURRENT_PI_PROPORTIONAL_GAIN = 0.1251f;
static constexpr float CURRENT_PI_INTEGRAL_GAIN = 370.8825f;
static constexpr float CURRENT_PI_INTEGRAL_LIMIT_V = 1.0f;
static constexpr float BUS_VOLTAGE_DIVIDER_RATIO = 11.0f;

static as5600_rotor_sensor rotor(AS5600_I2C_BUS_ID,
    AS5600_I2C_ADDRESS);
static const stm32_two_shunt_current_config CURRENT_SENSOR_CONFIG =
{
    &hadc1,
    CURRENT_AMPERE_PER_COUNT,
    CURRENT_AMPERE_PER_COUNT,
    1,
    -1
};
static stm32_two_shunt_current_sensor phase_current(
    CURRENT_SENSOR_CONFIG);
static const tim1_phase_driver_config PHASE_DRIVER_CONFIG =
{
    &htim1,
    MOTOR_EN_GPIO_Port,
    MOTOR_EN_Pin,
    3,
    2,
    1,
    true,
    true
};
static tim1_phase_driver phase_output(PHASE_DRIVER_CONFIG);
static bool bus_voltage_sampling_started = false;
static foc::instance motor;
static foc::commissioner motorCommissioner;

/**
 * @brief 启动母线电压 ADC 连续采样
 *
 * @return ADC 成功启动时返回 true
 */
static bool start_bus_voltage_sampling()
{
    if(HAL_ADC_Start(&hadc3) != HAL_OK)
    {
        return false;
    }

    bus_voltage_sampling_started = true;
    return true;
}

/**
 * @brief 读取母线电压连续转换结果
 *
 * @return 母线电压，单位伏特
 */
static float read_bus_voltage_v()
{
    if(!bus_voltage_sampling_started)
    {
        return 0.0f;
    }

    uint32_t raw_count = hadc3.Instance->DR;
    return (float)raw_count * ADC_REFERENCE_VOLTAGE_V *
        BUS_VOLTAGE_DIVIDER_RATIO / ADC_FULL_SCALE_COUNT;
}

/**
 * @brief 启动功率禁用状态下的同步 ADC 电流采样
 *
 * @return 采样硬件成功启动时返回 true
 */
static bool start_current_sampling()
{
    sys_time::get_us_tick();

    HAL_NVIC_SetPriority(ADC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);

    if(HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK ||
        HAL_TIM_Base_Start(&htim1) != HAL_OK)
    {
        HAL_NVIC_DisableIRQ(ADC_IRQn);
        HAL_ADCEx_InjectedStop_IT(&hadc1);
        HAL_TIM_Base_Stop(&htim1);
        return false;
    }

    return true;
}

/**
 * @brief 创建创客工坊 2804 与驱动板的受限控制配置
 *
 * @return 电流环调试配置
 */
static foc_config make_control_config()
{
    foc_config config{};
    config.monitor_only = false;
    config.pole_pairs = MOTOR_POLE_PAIRS;
    config.rotor_direction = 1;
    config.electrical_zero_offset_rad = 0.0f;
    config.control_period_s = 0.0001f;
    config.bus_voltage_v = MOTOR_BUS_VOLTAGE_V;
    config.voltage_limit_v = MOTOR_VOLTAGE_LIMIT_V;
    config.max_phase_current_a = MOTOR_PHASE_CURRENT_LIMIT_A;
    config.rotor_extrapolation_limit_us = 2000;
    config.rotor_hard_timeout_us = 5000;
    config.rotor_slow_timeout_us = 50000;
    config.command_timeout_ms = 100;
    config.communication_error_limit = 10;
    config.telemetry_divider = 20;
    config.control_isr_divider = FOC_CONTROL_ISR_DIVIDER;
    config.d_axis_pi.proportional_gain =
        CURRENT_PI_PROPORTIONAL_GAIN;
    config.d_axis_pi.integral_gain = CURRENT_PI_INTEGRAL_GAIN;
    config.d_axis_pi.integral_limit = CURRENT_PI_INTEGRAL_LIMIT_V;
    config.q_axis_pi = config.d_axis_pi;
    return config;
}

/**
 * @brief 周期驱动 AS5600 并发布最新转子快照
 *
 * @param argument FreeRTOS 任务参数
 */
static void foc_sensor_task_entry(void *argument)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    while(true)
    {
        TickType_t update_start_tick = xTaskGetTickCount();
        foc_result result = foc::runtime::update_sensors(motor);
        TickType_t update_elapsed_ticks =
            xTaskGetTickCount() - update_start_tick;

        if(result != foc_result::OK ||
            update_elapsed_ticks >=
                pdMS_TO_TICKS(FOC_SENSOR_UPDATE_PERIOD_MS))
        {
            vTaskDelay(pdMS_TO_TICKS(FOC_SENSOR_UPDATE_PERIOD_MS));
            last_wake_time = xTaskGetTickCount();
        }
        else
        {
            vTaskDelayUntil(&last_wake_time,
                pdMS_TO_TICKS(FOC_SENSOR_UPDATE_PERIOD_MS));
        }
    }
}

/**
 * @brief 周期执行 FOC 投运状态机与安全检查
 *
 * @param argument FreeRTOS 任务参数
 */
static void foc_safety_task_entry(void *argument)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    while(true)
    {
        uint32_t timestampMs = sys_time::get_ms_tick();
        motorCommissioner.update(timestampMs,
            read_bus_voltage_v());
        foc::runtime::update_safety(motor, timestampMs);

        vTaskDelayUntil(&last_wake_time,
            pdMS_TO_TICKS(FOC_SAFETY_UPDATE_PERIOD_MS));
    }
}

/**
 * @brief 初始化板级 FOC 硬件、控制核心和调度任务
 */
void foc_dev::init()
{
    foc_hardware hardware{
        &rotor,
        &phase_current,
        &phase_output
    };
    if(motor.init(make_control_config(), hardware) != foc_result::OK)
    {
        Error_Handler();
    }

    foc_commissioning_config commissioningConfig{};
    if(motorCommissioner.init(motor,
        commissioningConfig,
        sys_time::get_ms_tick()) != foc_result::OK)
    {
        Error_Handler();
    }

    if(!start_bus_voltage_sampling() || !start_current_sampling())
    {
        Error_Handler();
    }

    BaseType_t sensor_task_result = xTaskCreate(foc_sensor_task_entry,
        "foc_sensor",
        FOC_SENSOR_TASK_STACK_DEPTH,
        nullptr,
        FOC_SENSOR_TASK_PRIORITY,
        nullptr);
    BaseType_t safety_task_result = xTaskCreate(foc_safety_task_entry,
        "foc_safety",
        FOC_SAFETY_TASK_STACK_DEPTH,
        nullptr,
        FOC_SAFETY_TASK_PRIORITY,
        nullptr);

    if(sensor_task_result != pdPASS || safety_task_result != pdPASS)
    {
        Error_Handler();
    }
}

/**
 * @brief 处理 ADC 全局中断并分派注入转换完成事件
 */
extern "C" void ADC_IRQHandler(void)
{
    HAL_ADC_IRQHandler(&hadc1);
}

/**
 * @brief 在同步 ADC 注入转换完成时推进电流采样与控制核心
 *
 * @param adc ADC 外设句柄
 */
extern "C" void HAL_ADCEx_InjectedConvCpltCallback(
    ADC_HandleTypeDef *adc)
{
    if(adc && adc->Instance == ADC1)
    {
        foc::runtime::run_control_from_isr(motor,
            sys_time::get_us_tick());
    }
}

foc::instance &foc_dev::get_motor()
{
    return motor;
}

foc::commissioner &foc_dev::get_motor_commissioner()
{
    return motorCommissioner;
}

#include "foc_dev.h"

#include "drivers/foc/foc_core.h"
#include "drivers/foc/phase_driver/tim1_phase_driver.h"
#include "drivers/foc/sensors/encoder/as5600_rotor_sensor.h"
#include "system/sys_time.h"
#include "main.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"

static constexpr uint8_t AS5600_I2C_BUS_ID = 0U;
static constexpr uint8_t AS5600_I2C_ADDRESS = 0x36U;
static constexpr uint16_t FOC_SENSOR_TASK_STACK_DEPTH = 512U;
static constexpr uint16_t FOC_SAFETY_TASK_STACK_DEPTH = 512U;
static constexpr UBaseType_t FOC_SENSOR_TASK_PRIORITY =
    tskIDLE_PRIORITY + 4U;
static constexpr UBaseType_t FOC_SAFETY_TASK_PRIORITY =
    tskIDLE_PRIORITY + 2U;
static constexpr uint32_t FOC_SENSOR_UPDATE_PERIOD_MS = 1U;
static constexpr uint32_t FOC_SAFETY_UPDATE_PERIOD_MS = 10U;

static as5600_rotor_sensor rotor(AS5600_I2C_BUS_ID,
    AS5600_I2C_ADDRESS);
static const tim1_phase_driver_config PHASE_DRIVER_CONFIG =
{
    &htim1,
    MOTOR_EN_GPIO_Port,
    MOTOR_EN_Pin,
    true,
    false
};
static tim1_phase_driver phase_output(PHASE_DRIVER_CONFIG);

/**
 * @brief 创建首版只监视、不允许功率输出的 FOC 配置
 *
 * @return 安全监视配置
 */
static foc_config make_monitor_config()
{
    foc_config config{};
    config.monitor_only = true;
    config.rotor_direction = 1;
    config.control_period_s = 0.00005f;
    config.rotor_extrapolation_limit_us = 2000U;
    config.rotor_hard_timeout_us = 5000U;
    config.rotor_slow_timeout_us = 50000U;
    config.command_timeout_ms = 100U;
    config.communication_error_limit = 10U;
    config.telemetry_divider = 20U;
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
        foc_result result = foc_core::update_bus_sensors();
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
 * @brief 周期执行 FOC 慢速安全检查并生成调试快照
 *
 * @param argument FreeRTOS 任务参数
 */
static void foc_safety_task_entry(void *argument)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    while(true)
    {
        foc_core::update_safety(sys_time::get_us_tick() / 1000U);
        vTaskDelayUntil(&last_wake_time,
            pdMS_TO_TICKS(FOC_SAFETY_UPDATE_PERIOD_MS));
    }
}

/**
 * @brief 初始化微秒时基、FOC 核心并创建三层框架的两个任务
 */
void foc_dev::init()
{
    if(foc_core::link_rotor_sensor(rotor) != foc_result::OK ||
        foc_core::link_phase_driver(phase_output) != foc_result::OK ||
        foc_core::init(make_monitor_config()) != foc_result::OK)
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

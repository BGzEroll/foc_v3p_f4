#include "as5600_dev.h"

#include "drivers/as5600.h"
#include "drivers/bus/uart_bus.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

static constexpr uint8_t AS5600_I2C_BUS_ID = 0U;
static constexpr uint8_t AS5600_I2C_ADDRESS = 0x36U;
static constexpr uint8_t TELEMETRY_UART_BUS_ID = 0U;

static constexpr uint16_t AS5600_TASK_STACK_DEPTH = 512U;
static constexpr UBaseType_t AS5600_TASK_PRIORITY = tskIDLE_PRIORITY + 3U;
static constexpr uint32_t AS5600_UPDATE_PERIOD_MS = 5U;
static constexpr uint32_t AS5600_OUTPUT_PERIOD_MS = 100U;
static constexpr uint32_t AS5600_RETRY_DELAY_MS = 1000U;
static constexpr uint16_t UART_MESSAGE_BUFFER_SIZE = 160U;
static constexpr float RADIAN_TO_CENTIDEGREE =
    18000.0f / 3.14159265358979323846f;
static constexpr float RADIAN_PER_SECOND_TO_MILLIRADIAN_PER_SECOND = 1000.0f;

static as5600 encoder(AS5600_I2C_BUS_ID, AS5600_I2C_ADDRESS);
static uart_bus telemetry_uart(TELEMETRY_UART_BUS_ID);

/**
 * @brief 通过 UART 发送一个以空字符结尾的文本
 *
 * @param text 待发送文本
 *
 * @return UART 发送结果
 */
static uart_result send_uart_text(const char *text)
{
    size_t text_length = strlen(text);
    if(text_length == 0U || text_length > UINT16_MAX)
    {
        return uart_result::INVALID_ARGUMENT;
    }

    return telemetry_uart.write_bytes(
        reinterpret_cast<const uint8_t *>(text),
        (uint16_t)text_length,
        uart_bus::DEFAULT_LOCK_TIMEOUT_MS,
        100U);
}

/**
 * @brief 将浮点值四舍五入为带符号整数
 *
 * @param value 待转换的浮点值
 *
 * @return 转换后的整数
 */
static int32_t round_to_int32(float value)
{
    return (int32_t)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

/**
 * @brief 获取带符号整数的绝对值
 *
 * @param value 待处理的带符号整数
 *
 * @return 整数绝对值
 */
static uint32_t magnitude(int32_t value)
{
    return value < 0 ? (uint32_t)(-value) : (uint32_t)value;
}

/**
 * @brief 格式化并发送 AS5600 角度和角速度
 *
 * @return UART 发送结果
 */
static uart_result send_as5600_telemetry()
{
    const as5600_sample &sample = encoder.sample();
    int32_t angle = round_to_int32(
        sample.angle_rad * RADIAN_TO_CENTIDEGREE);
    int32_t full_angle = round_to_int32(
        sample.full_angle_rad * RADIAN_TO_CENTIDEGREE);
    int32_t velocity = round_to_int32(sample.angular_velocity_rad_s *
        RADIAN_PER_SECOND_TO_MILLIRADIAN_PER_SECOND);
    uint32_t angle_magnitude = magnitude(angle);
    uint32_t full_angle_magnitude = magnitude(full_angle);
    uint32_t velocity_magnitude = magnitude(velocity);

    char message[UART_MESSAGE_BUFFER_SIZE]{};
    int message_length = snprintf(message,
        sizeof(message),
        "AS5600 angle=%c%lu.%02lu full=%c%lu.%02lu deg "
        "velocity=%c%lu.%03lu rad/s raw=%u sequence=%lu "
        "timestamp_ms=%lu\r\n",
        angle < 0 ? '-' : '+',
        (unsigned long)(angle_magnitude / 100U),
        (unsigned long)(angle_magnitude % 100U),
        full_angle < 0 ? '-' : '+',
        (unsigned long)(full_angle_magnitude / 100U),
        (unsigned long)(full_angle_magnitude % 100U),
        velocity < 0 ? '-' : '+',
        (unsigned long)(velocity_magnitude / 1000U),
        (unsigned long)(velocity_magnitude % 1000U),
        (unsigned int)sample.raw_count,
        (unsigned long)sample.sequence,
        (unsigned long)sample.timestamp_ms);

    if(message_length <= 0 ||
        (uint32_t)message_length >= sizeof(message))
    {
        return uart_result::INVALID_ARGUMENT;
    }

    return telemetry_uart.write_bytes(
        reinterpret_cast<const uint8_t *>(message),
        (uint16_t)message_length,
        uart_bus::DEFAULT_LOCK_TIMEOUT_MS,
        100U);
}

/**
 * @brief 发送 AS5600 的 I2C 错误状态
 *
 * @param prefix 状态文本前缀
 * @param result I2C 操作结果
 */
static void send_i2c_error(const char *prefix, i2c_result result)
{
    char message[UART_MESSAGE_BUFFER_SIZE]{};
    int message_length = snprintf(message,
        sizeof(message),
        "%s error=%u\r\n",
        prefix,
        (unsigned int)result);

    if(message_length > 0 &&
        (uint32_t)message_length < sizeof(message))
    {
        telemetry_uart.write_bytes(
            reinterpret_cast<const uint8_t *>(message),
            (uint16_t)message_length,
            uart_bus::DEFAULT_LOCK_TIMEOUT_MS,
            100U);
    }
}

/**
 * @brief 初始化 AS5600，周期采样并通过 UART 输出测量值
 *
 * @param argument FreeRTOS 任务参数
 */
static void as5600_task_entry(void *argument)
{
    if(telemetry_uart.init() != uart_result::OK)
    {
        Error_Handler();
    }

    i2c_result init_result;
    do
    {
        send_uart_text("AS5600 initializing...\r\n");
        init_result = encoder.init();

        if(init_result != i2c_result::OK)
        {
            send_i2c_error("AS5600 init", init_result);
            vTaskDelay(pdMS_TO_TICKS(AS5600_RETRY_DELAY_MS));
        }
    }
    while(init_result != i2c_result::OK);

    send_uart_text("AS5600 ready\r\n");

    TickType_t last_wake_time = xTaskGetTickCount();
    TickType_t last_output_time = last_wake_time;
    TickType_t last_error_time = last_wake_time;

    while(true)
    {
        i2c_result update_result = encoder.update();
        TickType_t current_time = xTaskGetTickCount();

        if(update_result == i2c_result::OK)
        {
            if((current_time - last_output_time) >=
                pdMS_TO_TICKS(AS5600_OUTPUT_PERIOD_MS))
            {
                send_as5600_telemetry();
                last_output_time = current_time;
            }
        }
        else if((current_time - last_error_time) >=
            pdMS_TO_TICKS(AS5600_RETRY_DELAY_MS))
        {
            send_i2c_error("AS5600 update", update_result);
            last_error_time = current_time;
        }

        vTaskDelayUntil(&last_wake_time,
            pdMS_TO_TICKS(AS5600_UPDATE_PERIOD_MS));
    }
}

/**
 * @brief 创建 AS5600 采样与 UART 输出任务
 */
void as5600_dev::init()
{
    BaseType_t result = xTaskCreate(as5600_task_entry,
        "as5600",
        AS5600_TASK_STACK_DEPTH,
        nullptr,
        AS5600_TASK_PRIORITY,
        nullptr);

    if(result != pdPASS)
    {
        Error_Handler();
    }
}

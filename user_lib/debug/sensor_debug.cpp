#include "sensor_debug.h"

#include "drivers/bus/uart_bus.h"
#include "devices/as5600_dev.h"
#include "devices/mpu6050_dev.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

static constexpr uint8_t DEBUG_UART_BUS_ID = 0U;
static constexpr uint16_t SENSOR_DEBUG_TASK_STACK_DEPTH = 512U;
static constexpr UBaseType_t SENSOR_DEBUG_TASK_PRIORITY = tskIDLE_PRIORITY + 2U;
static constexpr uint32_t SENSOR_DEBUG_OUTPUT_PERIOD_MS = 100U;
static constexpr uint16_t UART_MESSAGE_BUFFER_SIZE = 192U;
static constexpr float RADIAN_TO_CENTIDEGREE =
    18000.0f / 3.14159265358979323846f;
static constexpr float RADIAN_PER_SECOND_TO_MILLIRADIAN_PER_SECOND = 1000.0f;

static uart_bus debug_uart(DEBUG_UART_BUS_ID);

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
 * @brief 发送已经格式化的 UART 调试文本
 *
 * @param message 调试文本缓冲区
 * @param message_length 调试文本长度
 *
 * @return UART 发送结果
 */
static uart_result send_debug_message(const char *message,
    int message_length)
{
    if(!message || message_length <= 0 ||
        message_length > UINT16_MAX)
    {
        return uart_result::INVALID_ARGUMENT;
    }

    return debug_uart.write_bytes(
        reinterpret_cast<const uint8_t *>(message),
        (uint16_t)message_length,
        uart_bus::DEFAULT_LOCK_TIMEOUT_MS,
        100U);
}

/**
 * @brief 通过 UART 输出 MPU6050 最新样本
 *
 * @param sample MPU6050 样本快照
 *
 * @return UART 发送结果
 */
static uart_result send_mpu6050_sample(const mpu6050_sample &sample)
{
    int32_t roll = round_to_int32(
        sample.angle_rad[0] * RADIAN_TO_CENTIDEGREE);
    int32_t pitch = round_to_int32(
        sample.angle_rad[1] * RADIAN_TO_CENTIDEGREE);
    int32_t yaw = round_to_int32(
        sample.angle_rad[2] * RADIAN_TO_CENTIDEGREE);
    uint32_t roll_magnitude = magnitude(roll);
    uint32_t pitch_magnitude = magnitude(pitch);
    uint32_t yaw_magnitude = magnitude(yaw);

    char message[UART_MESSAGE_BUFFER_SIZE]{};
    int message_length = snprintf(message,
        sizeof(message),
        "MPU6050 roll=%c%lu.%02lu pitch=%c%lu.%02lu "
        "yaw=%c%lu.%02lu deg sequence=%lu timestamp_ms=%lu\r\n",
        roll < 0 ? '-' : '+',
        (unsigned long)(roll_magnitude / 100U),
        (unsigned long)(roll_magnitude % 100U),
        pitch < 0 ? '-' : '+',
        (unsigned long)(pitch_magnitude / 100U),
        (unsigned long)(pitch_magnitude % 100U),
        yaw < 0 ? '-' : '+',
        (unsigned long)(yaw_magnitude / 100U),
        (unsigned long)(yaw_magnitude % 100U),
        (unsigned long)sample.sequence,
        (unsigned long)sample.timestamp_ms);

    if((uint32_t)message_length >= sizeof(message))
    {
        return uart_result::INVALID_ARGUMENT;
    }

    return send_debug_message(message, message_length);
}

/**
 * @brief 通过 UART 输出 AS5600 最新样本
 *
 * @param sample AS5600 样本快照
 *
 * @return UART 发送结果
 */
static uart_result send_as5600_sample(const as5600_sample &sample)
{
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

    if((uint32_t)message_length >= sizeof(message))
    {
        return uart_result::INVALID_ARGUMENT;
    }

    return send_debug_message(message, message_length);
}

/**
 * @brief 周期读取传感器话题并通过 UART 输出新样本
 *
 * @param argument FreeRTOS 任务参数
 */
static void sensor_debug_task_entry(void *argument)
{
    if(debug_uart.init() != uart_result::OK)
    {
        Error_Handler();
    }

    uint32_t previous_mpu6050_sequence = 0U;
    uint32_t previous_as5600_sequence = 0U;
    TickType_t last_wake_time = xTaskGetTickCount();

    while(true)
    {
        mpu6050_sample mpu6050_data{};
        if(mpu6050_dev::peek_latest(mpu6050_data) &&
            mpu6050_data.sequence != previous_mpu6050_sequence)
        {
            send_mpu6050_sample(mpu6050_data);
            previous_mpu6050_sequence = mpu6050_data.sequence;
        }

        as5600_sample as5600_data{};
        if(as5600_dev::peek_latest(as5600_data) &&
            as5600_data.sequence != previous_as5600_sequence)
        {
            send_as5600_sample(as5600_data);
            previous_as5600_sequence = as5600_data.sequence;
        }

        vTaskDelayUntil(&last_wake_time,
            pdMS_TO_TICKS(SENSOR_DEBUG_OUTPUT_PERIOD_MS));
    }
}

/**
 * @brief 创建传感器话题 UART 调试消费者任务
 */
void sensor_debug::init()
{
    BaseType_t result = xTaskCreate(sensor_debug_task_entry,
        "sensor_debug",
        SENSOR_DEBUG_TASK_STACK_DEPTH,
        nullptr,
        SENSOR_DEBUG_TASK_PRIORITY,
        nullptr);

    if(result != pdPASS)
    {
        Error_Handler();
    }
}

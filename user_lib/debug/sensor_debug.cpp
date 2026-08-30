#include "sensor_debug.h"

#include "drivers/bus/uart_bus.h"
#include "devices/mpu6050_dev.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

extern "C"
{
#include "third-parts/SguanFOC/SguanFOC.h"
}

static constexpr uint8_t DEBUG_UART_BUS_ID = 0;
static constexpr uint16_t SENSOR_DEBUG_TASK_STACK_DEPTH = 512;
static constexpr UBaseType_t SENSOR_DEBUG_TASK_PRIORITY = tskIDLE_PRIORITY + 2;
static constexpr uint32_t SENSOR_DEBUG_OUTPUT_PERIOD_MS = 100;
static constexpr uint16_t UART_MESSAGE_BUFFER_SIZE = 640;
static constexpr float RADIAN_TO_CENTIDEGREE =
    18000.0f / 3.14159265358979323846f;

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
        100);
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
        "yaw=%c%lu.%02lu deg sequence=%lu timestamp_us=%lu\r\n",
        roll < 0 ? '-' : '+',
        (unsigned long)(roll_magnitude / 100),
        (unsigned long)(roll_magnitude % 100),
        pitch < 0 ? '-' : '+',
        (unsigned long)(pitch_magnitude / 100),
        (unsigned long)(pitch_magnitude % 100),
        yaw < 0 ? '-' : '+',
        (unsigned long)(yaw_magnitude / 100),
        (unsigned long)(yaw_magnitude % 100),
        (unsigned long)sample.sequence,
        (unsigned long)sample.timestamp_us);

    if(message_length < 0 || (uint32_t)message_length >= sizeof(message))
    {
        return uart_result::INVALID_ARGUMENT;
    }

    return send_debug_message(message, message_length);
}

/**
 * @brief 通过 UART 输出 SguanFOC 的电流环运行数据
 *
 * @return UART 发送结果
 */
static uart_result send_sguan_status()
{
    int32_t angle = round_to_int32(
        Sguan.encoder.Real_Rad * RADIAN_TO_CENTIDEGREE);
    int32_t speed = round_to_int32(Sguan.encoder.Real_Speed * 1000.0f);
    int32_t current_d = round_to_int32(Sguan.current.Real_Id * 1000.0f);
    int32_t current_q = round_to_int32(Sguan.current.Real_Iq * 1000.0f);
    int32_t target_q = round_to_int32(Sguan.foc.Target_Iq * 1000.0f);
    int32_t voltage_q = round_to_int32(Sguan.foc.Uq_in * 1000.0f);

    char message[UART_MESSAGE_BUFFER_SIZE]{};
    int message_length = snprintf(message,
        sizeof(message),
        "SGUAN status=%u mode=%u angle_mdeg=%ld speed_mrads=%ld "
        "id_ma=%ld iq_ma=%ld target_iq_ma=%ld uq_mv=%ld "
        "duty=%u,%u,%u\r\n",
        (unsigned int)Sguan.status,
        (unsigned int)Sguan.mode,
        (long)angle,
        (long)speed,
        (long)current_d,
        (long)current_q,
        (long)target_q,
        (long)voltage_q,
        (unsigned int)Sguan.foc.Duty_u,
        (unsigned int)Sguan.foc.Duty_v,
        (unsigned int)Sguan.foc.Duty_w);

    if(message_length < 0 || (uint32_t)message_length >= sizeof(message))
    {
        return uart_result::INVALID_ARGUMENT;
    }

    return send_debug_message(message, message_length);
}

/**
 * @brief 周期读取传感器和 FOC 话题并通过 UART 输出新样本
 *
 * @param argument FreeRTOS 任务参数
 */
static void sensor_debug_task_entry(void *argument)
{
    if(debug_uart.init() != uart_result::OK)
    {
        Error_Handler();
    }

    uint32_t previous_mpu6050_sequence = 0;
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

        send_sguan_status();

        vTaskDelayUntil(&last_wake_time,
            pdMS_TO_TICKS(SENSOR_DEBUG_OUTPUT_PERIOD_MS));
    }
}

/**
 * @brief 创建传感器和 FOC 话题 UART 调试消费者任务
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

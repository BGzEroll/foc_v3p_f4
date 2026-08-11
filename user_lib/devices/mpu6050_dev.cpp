#include "mpu6050_dev.h"

#include "drivers/bus/uart_bus.h"
#include "drivers/mpu6050.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static constexpr uint8_t MPU6050_I2C_BUS_ID = 1U;
static constexpr uint8_t MPU6050_I2C_ADDRESS = 0x68U;
static constexpr float MPU6050_ACCELEROMETER_WEIGHT = 0.02f;
static constexpr uint8_t TELEMETRY_UART_BUS_ID = 0U;

static constexpr uint16_t MPU6050_TASK_STACK_DEPTH = 512U;
static constexpr UBaseType_t MPU6050_TASK_PRIORITY = tskIDLE_PRIORITY + 3U;
static constexpr uint32_t MPU6050_UPDATE_PERIOD_MS = 5U;
static constexpr uint32_t MPU6050_OUTPUT_PERIOD_MS = 100U;
static constexpr uint32_t MPU6050_RETRY_DELAY_MS = 1000U;
static constexpr uint16_t UART_MESSAGE_BUFFER_SIZE = 128U;
static constexpr float RADIAN_TO_CENTIDEGREE =
    18000.0f / 3.14159265358979323846f;

static mpu6050 imu(MPU6050_I2C_BUS_ID,
    MPU6050_I2C_ADDRESS,
    MPU6050_ACCELEROMETER_WEIGHT);
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
 * @brief 把弧度转换为带符号的百分之一度整数
 *
 * @param angle_rad 角度，单位弧度
 *
 * @return 角度，单位百分之一度
 */
static int32_t radian_to_centidegree(float angle_rad)
{
    float scaled_angle = angle_rad * RADIAN_TO_CENTIDEGREE;
    return (int32_t)(scaled_angle >= 0.0f ?
        scaled_angle + 0.5f : scaled_angle - 0.5f);
}

/**
 * @brief 获取带符号角度的绝对值
 *
 * @param angle_centidegree 角度，单位百分之一度
 *
 * @return 角度绝对值，单位百分之一度
 */
static uint32_t angle_magnitude(int32_t angle_centidegree)
{
    return angle_centidegree < 0 ?
        (uint32_t)(-angle_centidegree) :
        (uint32_t)angle_centidegree;
}

/**
 * @brief 格式化并发送三轴角度
 *
 * @return UART 发送结果
 */
static uart_result send_angle_telemetry()
{
    const mpu6050_sample &sample = imu.sample();
    int32_t roll = radian_to_centidegree(sample.angle_rad[0]);
    int32_t pitch = radian_to_centidegree(sample.angle_rad[1]);
    int32_t yaw = radian_to_centidegree(sample.angle_rad[2]);
    uint32_t roll_magnitude = angle_magnitude(roll);
    uint32_t pitch_magnitude = angle_magnitude(pitch);
    uint32_t yaw_magnitude = angle_magnitude(yaw);

    char message[UART_MESSAGE_BUFFER_SIZE]{};
    int message_length = snprintf(message,
        sizeof(message),
        "ANGLE roll=%c%lu.%02lu pitch=%c%lu.%02lu yaw=%c%lu.%02lu deg\r\n",
        roll < 0 ? '-' : '+',
        (unsigned long)(roll_magnitude / 100U),
        (unsigned long)(roll_magnitude % 100U),
        pitch < 0 ? '-' : '+',
        (unsigned long)(pitch_magnitude / 100U),
        (unsigned long)(pitch_magnitude % 100U),
        yaw < 0 ? '-' : '+',
        (unsigned long)(yaw_magnitude / 100U),
        (unsigned long)(yaw_magnitude % 100U));

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
 * @brief 发送 MPU6050 的 I2C 错误状态
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
 * @brief 初始化 MPU6050，采样并通过 UART 周期输出三轴角度
 *
 * @param argument FreeRTOS 任务参数
 */
static void mpu6050_task_entry(void *argument)
{
    if(telemetry_uart.init() != uart_result::OK)
    {
        Error_Handler();
    }

    i2c_result init_result;
    do
    {
        send_uart_text("MPU6050 initializing, keep sensor still...\r\n");
        init_result = imu.init(true);

        if(init_result != i2c_result::OK)
        {
            send_i2c_error("MPU6050 init", init_result);
            vTaskDelay(pdMS_TO_TICKS(MPU6050_RETRY_DELAY_MS));
        }
    }
    while(init_result != i2c_result::OK);

    send_uart_text("MPU6050 ready\r\n");

    TickType_t last_wake_time = xTaskGetTickCount();
    TickType_t last_output_time = last_wake_time;
    TickType_t last_error_time = last_wake_time;

    while(true)
    {
        i2c_result update_result = imu.update();
        TickType_t current_time = xTaskGetTickCount();

        if(update_result == i2c_result::OK)
        {
            if((current_time - last_output_time) >=
                pdMS_TO_TICKS(MPU6050_OUTPUT_PERIOD_MS))
            {
                send_angle_telemetry();
                last_output_time = current_time;
            }
        }
        else if((current_time - last_error_time) >=
            pdMS_TO_TICKS(MPU6050_RETRY_DELAY_MS))
        {
            send_i2c_error("MPU6050 update", update_result);
            last_error_time = current_time;
        }

        vTaskDelayUntil(&last_wake_time,
            pdMS_TO_TICKS(MPU6050_UPDATE_PERIOD_MS));
    }
}

/**
 * @brief 创建 MPU6050 采样与 UART 输出任务
 */
void mpu6050_dev::init()
{
    BaseType_t result = xTaskCreate(mpu6050_task_entry,
        "mpu6050",
        MPU6050_TASK_STACK_DEPTH,
        nullptr,
        MPU6050_TASK_PRIORITY,
        nullptr);

    if(result != pdPASS)
    {
        Error_Handler();
    }
}

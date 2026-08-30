#include "sensor_debug.h"

#include "drivers/bus/uart_bus.h"
#include "drivers/foc/foc.h"
#include "devices/foc_dev.h"
#include "devices/mpu6050_dev.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

static constexpr uint8_t DEBUG_UART_BUS_ID = 0;
static constexpr uint16_t SENSOR_DEBUG_TASK_STACK_DEPTH = 512;
static constexpr UBaseType_t SENSOR_DEBUG_TASK_PRIORITY = tskIDLE_PRIORITY + 2;
static constexpr uint32_t SENSOR_DEBUG_OUTPUT_PERIOD_MS = 100;
static constexpr uint16_t UART_MESSAGE_BUFFER_SIZE = 640;
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
 * @brief 把 FOC 状态转换为稳定的调试文本
 *
 * @param state FOC 状态
 *
 * @return 状态文本
 */
static const char *foc_state_text(foc_state state)
{
    switch(state)
    {
        case foc_state::UNINITIALIZED:
            return "UNINITIALIZED";
        case foc_state::MONITORING:
            return "MONITORING";
        case foc_state::READY:
            return "READY";
        case foc_state::RUNNING:
            return "RUNNING";
        case foc_state::FAULT:
            return "FAULT";
        default:
            return "UNKNOWN";
    }
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
 * @brief 通过 UART 输出 FOC 状态和转子最新快照
 *
 * @param snapshot FOC 调试快照
 *
 * @return UART 发送结果
 */
static uart_result send_foc_snapshot(const foc_snapshot &snapshot)
{
    int32_t angle = round_to_int32(
        snapshot.rotor.mechanical_angle_rad * RADIAN_TO_CENTIDEGREE);
    int32_t velocity = round_to_int32(
        snapshot.rotor.mechanical_velocity_rad_s *
        RADIAN_PER_SECOND_TO_MILLIRADIAN_PER_SECOND);
    int32_t age = round_to_int32(snapshot.rotor_sample_age_ms * 1000.0f);
    uint32_t angle_magnitude = magnitude(angle);
    uint32_t velocity_magnitude = magnitude(velocity);
    uint32_t age_magnitude = magnitude(age);
    int32_t current_a = round_to_int32(snapshot.current.current_a * 1000.0f);
    int32_t current_b = round_to_int32(snapshot.current.current_b * 1000.0f);
    int32_t current_c = round_to_int32(snapshot.current.current_c * 1000.0f);
    int32_t offset_a = round_to_int32(snapshot.current.offset_count_a * 100.0f);
    int32_t offset_b = round_to_int32(snapshot.current.offset_count_b * 100.0f);
    uint32_t current_a_magnitude = magnitude(current_a);
    uint32_t current_b_magnitude = magnitude(current_b);
    uint32_t current_c_magnitude = magnitude(current_c);
    uint32_t offset_a_magnitude = magnitude(offset_a);
    uint32_t offset_b_magnitude = magnitude(offset_b);

    char message[UART_MESSAGE_BUFFER_SIZE]{};
    int message_length = snprintf(message,
        sizeof(message),
        "FOC state=%s monitor=%u output=%u fault=0x%08lx "
        "angle=%c%lu.%02lu deg velocity=%c%lu.%03lu rad/s "
        "raw=%u rotor_seq=%lu timestamp_us=%lu age=%c%lu.%03lu ms "
        "current_valid=%u ia=%c%lu.%03lu ib=%c%lu.%03lu "
        "ic=%c%lu.%03lu A adc_a=%u adc_b=%u "
        "offset_a=%c%lu.%02lu offset_b=%c%lu.%02lu "
        "control_seq=%lu bus_error=%lu consecutive=%lu\r\n",
        foc_state_text(snapshot.state),
        snapshot.monitor_only ? 1 : 0,
        snapshot.output_active ? 1 : 0,
        (unsigned long)snapshot.fault_flags,
        angle < 0 ? '-' : '+',
        (unsigned long)(angle_magnitude / 100),
        (unsigned long)(angle_magnitude % 100),
        velocity < 0 ? '-' : '+',
        (unsigned long)(velocity_magnitude / 1000),
        (unsigned long)(velocity_magnitude % 1000),
        (unsigned int)snapshot.rotor.raw_count,
        (unsigned long)snapshot.rotor.sequence,
        (unsigned long)snapshot.rotor.timestamp_us,
        age < 0 ? '-' : '+',
        (unsigned long)(age_magnitude / 1000),
        (unsigned long)(age_magnitude % 1000),
        snapshot.current.valid ? 1 : 0,
        current_a < 0 ? '-' : '+',
        (unsigned long)(current_a_magnitude / 1000),
        (unsigned long)(current_a_magnitude % 1000),
        current_b < 0 ? '-' : '+',
        (unsigned long)(current_b_magnitude / 1000),
        (unsigned long)(current_b_magnitude % 1000),
        current_c < 0 ? '-' : '+',
        (unsigned long)(current_c_magnitude / 1000),
        (unsigned long)(current_c_magnitude % 1000),
        (unsigned int)snapshot.current.raw_count_a,
        (unsigned int)snapshot.current.raw_count_b,
        offset_a < 0 ? '-' : '+',
        (unsigned long)(offset_a_magnitude / 100),
        (unsigned long)(offset_a_magnitude % 100),
        offset_b < 0 ? '-' : '+',
        (unsigned long)(offset_b_magnitude / 100),
        (unsigned long)(offset_b_magnitude % 100),
        (unsigned long)snapshot.control_sequence,
        (unsigned long)snapshot.bus_update_error_count,
        (unsigned long)snapshot.consecutive_bus_error_count);

    if(message_length < 0 || (uint32_t)message_length >= sizeof(message))
    {
        return uart_result::INVALID_ARGUMENT;
    }

    return send_debug_message(message, message_length);
}

/**
 * @brief 通过 UART 输出 FOC 硬件投运进度
 *
 * @param status 最新投运状态
 *
 * @return UART 发送结果
 */
static uart_result send_foc_commissioning_status(
    const foc_commissioning_status &status)
{
    int32_t zero_offset = round_to_int32(
        status.electrical_zero_offset_rad * 1000.0f);
    int32_t first_angle = round_to_int32(
        status.first_mechanical_angle_rad * 1000.0f);
    int32_t second_angle = round_to_int32(
        status.second_mechanical_angle_rad * 1000.0f);
    int32_t d_axis_current = round_to_int32(
        status.measured_d_axis_current_a * 1000.0f);
    int32_t q_axis_current = round_to_int32(
        status.measured_q_axis_current_a * 1000.0f);
    int32_t bus_voltage = round_to_int32(
        status.bus_voltage_v * 1000.0f);
    int32_t minimum_bus_voltage = round_to_int32(
        status.minimum_bus_voltage_v * 1000.0f);
    int32_t open_loop_voltage = round_to_int32(
        status.open_loop_voltage_v * 1000.0f);
    int32_t open_loop_mechanical_velocity = round_to_int32(
        status.open_loop_mechanical_velocity_rad_s * 1000.0f);
    int32_t open_loop_electrical_velocity = round_to_int32(
        status.open_loop_electrical_velocity_rad_s * 1000.0f);
    int32_t open_loop_forward_delta = round_to_int32(
        status.open_loop_forward_delta_rad * 1000.0f);
    int32_t open_loop_reverse_delta = round_to_int32(
        status.open_loop_reverse_delta_rad * 1000.0f);
    int32_t vector_current_a[3]{};
    int32_t vector_current_b[3]{};
    int32_t vector_current_c[3]{};
    int32_t vector_angle[3]{};
    for(uint8_t index = 0; index < 3; index++)
    {
        vector_current_a[index] = round_to_int32(
            status.phase_vector_current_a[index] * 1000.0f);
        vector_current_b[index] = round_to_int32(
            status.phase_vector_current_b[index] * 1000.0f);
        vector_current_c[index] = round_to_int32(
            status.phase_vector_current_c[index] * 1000.0f);
        vector_angle[index] = round_to_int32(
            status.phase_vector_mechanical_angle_rad[index] * 1000.0f);
    }

    char message[UART_MESSAGE_BUFFER_SIZE]{};
    int message_length = snprintf(message,
        sizeof(message),
        "FOC commissioning stage=%u result=%u vectors_ok=%u "
        "vbus_mv=%ld min_vbus_mv=%ld current_dir=%d,%d "
        "rotor_dir=%d zero_offset_mrad=%ld first_mrad=%ld "
        "second_mrad=%ld vec0_ma=%ld,%ld,%ld "
        "vec1_ma=%ld,%ld,%ld vec2_ma=%ld,%ld,%ld "
        "vec_angle_mrad=%ld,%ld,%ld measured_id_ma=%ld "
        "measured_iq_ma=%ld open_loop_mv=%ld "
        "open_loop_mrads_milli=%ld open_loop_erads_milli=%ld "
        "forward_mrad=%ld "
        "reverse_mrad=%ld motion=%u\r\n",
        (unsigned int)status.stage,
        (unsigned int)status.result,
        status.phase_vector_check_passed ? 1 : 0,
        (long)bus_voltage,
        (long)minimum_bus_voltage,
        (int)status.current_direction_a,
        (int)status.current_direction_b,
        (int)status.rotor_direction,
        (long)zero_offset,
        (long)first_angle,
        (long)second_angle,
        (long)vector_current_a[0],
        (long)vector_current_b[0],
        (long)vector_current_c[0],
        (long)vector_current_a[1],
        (long)vector_current_b[1],
        (long)vector_current_c[1],
        (long)vector_current_a[2],
        (long)vector_current_b[2],
        (long)vector_current_c[2],
        (long)vector_angle[0],
        (long)vector_angle[1],
        (long)vector_angle[2],
        (long)d_axis_current,
        (long)q_axis_current,
        (long)open_loop_voltage,
        (long)open_loop_mechanical_velocity,
        (long)open_loop_electrical_velocity,
        (long)open_loop_forward_delta,
        (long)open_loop_reverse_delta,
        status.open_loop_motion_detected ? 1 : 0);

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
    uint32_t previous_foc_sequence = 0;
    uint32_t previous_commissioning_sequence = 0;
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

        foc_snapshot foc_data{};
        if(foc::peek_snapshot(foc_data) &&
            foc_data.sequence != previous_foc_sequence)
        {
            send_foc_snapshot(foc_data);
            previous_foc_sequence = foc_data.sequence;
        }

        foc_commissioning_status commissioning{};
        if(foc_dev::peek_commissioning_status(commissioning) &&
            commissioning.sequence != previous_commissioning_sequence)
        {
            send_foc_commissioning_status(commissioning);
            previous_commissioning_sequence = commissioning.sequence;
        }

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

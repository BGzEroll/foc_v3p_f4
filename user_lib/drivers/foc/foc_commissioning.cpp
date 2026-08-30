#include "foc_commissioning.h"

#include "foc.h"
#include "foc_math.h"
#include "internal/foc_internal_runtime.h"
#include "system/sys_time.h"
#include "system/topic.h"
#include <math.h>

static constexpr float ALIGNMENT_VOLTAGE_V = 3.0f;
static constexpr float FIRST_ALIGNMENT_ANGLE_RAD = 0.0f;
static constexpr float SECOND_PHASE_VECTOR_ANGLE_RAD =
    2.09439510239319549231f;
static constexpr float THIRD_PHASE_VECTOR_ANGLE_RAD =
    4.18879020478639098462f;
static constexpr float ALIGNMENT_SWEEP_ANGLE_RAD =
    6.28318530717958647692f;
static constexpr uint32_t PHASE_VECTOR_SETTLE_TIME_MS = 1000;
static constexpr uint32_t ALIGNMENT_SWEEP_TIME_MS = 3000;
static constexpr uint32_t CURRENT_POLARITY_VERIFY_TIME_MS = 500;
static constexpr uint32_t CURRENT_LOOP_VERIFY_TIME_MS = 500;
static constexpr uint32_t Q_AXIS_VERIFY_TIME_MS = 300;
static constexpr float CURRENT_POLARITY_MINIMUM_A = 0.005f;
static constexpr float PHASE_VECTOR_MINIMUM_CURRENT_A = 0.05f;
static constexpr float PHASE_VECTOR_RATIO_MINIMUM = 0.25f;
static constexpr float PHASE_VECTOR_RATIO_MAXIMUM = 0.75f;
static constexpr float PHASE_VECTOR_EQUAL_RATIO_MINIMUM = 0.50f;
static constexpr float PHASE_VECTOR_EQUAL_RATIO_MAXIMUM = 2.00f;
static constexpr float ROTOR_ALIGNMENT_MINIMUM_MOVE_RAD = 0.03f;
static constexpr float D_AXIS_VERIFY_TARGET_A = 0.05f;
static constexpr float Q_AXIS_VERIFY_TARGET_A = 0.03f;
static constexpr float OPEN_LOOP_TEST_VOLTAGE_V = 3.0f;
static constexpr float OPEN_LOOP_MECHANICAL_VELOCITY_RAD_S = 10.0f;
static constexpr float OPEN_LOOP_MINIMUM_BUS_VOLTAGE_V = 9.0f;
static constexpr float OPEN_LOOP_MAXIMUM_BUS_VOLTAGE_V = 15.0f;
static constexpr float OPEN_LOOP_MINIMUM_MECHANICAL_MOVE_RAD = 0.25f;
static constexpr uint32_t OPEN_LOOP_HOLD_TIME_MS = 500;
static constexpr uint32_t OPEN_LOOP_ACCELERATION_TIME_MS = 3000;
static constexpr uint32_t OPEN_LOOP_CONSTANT_SPEED_TIME_MS = 2000;
static constexpr uint32_t OPEN_LOOP_RUN_TIME_MS =
    OPEN_LOOP_ACCELERATION_TIME_MS * 2 +
    OPEN_LOOP_CONSTANT_SPEED_TIME_MS;
static constexpr uint32_t OPEN_LOOP_PAUSE_TIME_MS = 500;

static foc_commissioning_config commissioning_config{};
static bool commissioning_initialized = false;
static uint32_t commissioning_start_ms = 0;

static topic::latest_topic<foc_commissioning_status>
    commissioning_topic;
static foc_commissioning_status commissioning_status;
static uint32_t commissioning_stage_start_ms = 0;
static float stage_current_sum_a = 0.0f;
static float stage_current_sum_b = 0.0f;
static float stage_d_axis_current_sum = 0.0f;
static float stage_q_axis_current_sum = 0.0f;
static uint32_t stage_sample_count = 0;
static bool current_calibration_finished = false;
static bool calibration_task_started = false;
static float open_loop_stage_start_mechanical_angle_rad = 0.0f;
static float open_loop_stage_start_electrical_angle_rad = 0.0f;

/**
 * @brief 发布调试用的硬件投运状态
 */
static void publish_commissioning_status()
{
    commissioning_status.sequence++;
    commissioning_topic.publish(commissioning_status);
}

/**
 * @brief 清空当前阶段的电流统计值
 */
static void reset_stage_samples()
{
    stage_current_sum_a = 0.0f;
    stage_current_sum_b = 0.0f;
    stage_d_axis_current_sum = 0.0f;
    stage_q_axis_current_sum = 0.0f;
    stage_sample_count = 0;
}

/**
 * @brief 保存一个静态电压矢量的平均相电流与机械角
 *
 * @param vector_index 静态矢量编号
 * @param snapshot 最新 FOC 快照
 *
 * @return 存在有效阶段样本时返回 true
 */
static bool save_phase_vector_result(uint8_t vector_index,
    const foc_snapshot &snapshot)
{
    if(vector_index >= 3 || stage_sample_count == 0)
    {
        return false;
    }

    commissioning_status.phase_vector_current_a[vector_index] =
        stage_current_sum_a / (float)stage_sample_count;
    commissioning_status.phase_vector_current_b[vector_index] =
        stage_current_sum_b / (float)stage_sample_count;
    commissioning_status.phase_vector_current_c[vector_index] =
        -commissioning_status.phase_vector_current_a[vector_index] -
        commissioning_status.phase_vector_current_b[vector_index];
    commissioning_status.phase_vector_mechanical_angle_rad[vector_index] =
        snapshot.rotor.full_angle_rad;
    return true;
}

/**
 * @brief 判断两个数值的绝对值比例是否处于给定范围
 *
 * @param numerator 比例分子
 * @param denominator 比例分母
 * @param minimum 最小允许比例
 * @param maximum 最大允许比例
 *
 * @return 比例有效且位于允许范围时返回 true
 */
static bool magnitude_ratio_in_range(float numerator,
    float denominator,
    float minimum,
    float maximum)
{
    float denominator_magnitude = fabsf(denominator);
    if(denominator_magnitude < PHASE_VECTOR_MINIMUM_CURRENT_A)
    {
        return false;
    }

    float ratio = fabsf(numerator) / denominator_magnitude;
    return isfinite(ratio) && ratio >= minimum && ratio <= maximum;
}

/**
 * @brief 联合检查三个静态矢量下的两路相电流关系
 *
 * @return 电流幅值、比例和相对方向均符合三相关系时返回 true
 */
static bool phase_vector_results_valid()
{
    float first_a = commissioning_status.phase_vector_current_a[0];
    float first_b = commissioning_status.phase_vector_current_b[0];
    float second_a = commissioning_status.phase_vector_current_a[1];
    float second_b = commissioning_status.phase_vector_current_b[1];
    float third_a = commissioning_status.phase_vector_current_a[2];
    float third_b = commissioning_status.phase_vector_current_b[2];

    bool first_valid = fabsf(first_a) >= PHASE_VECTOR_MINIMUM_CURRENT_A &&
        first_a * first_b < 0.0f &&
        magnitude_ratio_in_range(first_b,
            first_a,
            PHASE_VECTOR_RATIO_MINIMUM,
            PHASE_VECTOR_RATIO_MAXIMUM);
    bool second_valid = fabsf(second_b) >=
        PHASE_VECTOR_MINIMUM_CURRENT_A && second_a * second_b < 0.0f &&
        magnitude_ratio_in_range(second_a,
            second_b,
            PHASE_VECTOR_RATIO_MINIMUM,
            PHASE_VECTOR_RATIO_MAXIMUM);
    bool third_valid = third_a * third_b > 0.0f &&
        magnitude_ratio_in_range(third_a,
            third_b,
            PHASE_VECTOR_EQUAL_RATIO_MINIMUM,
            PHASE_VECTOR_EQUAL_RATIO_MAXIMUM);
    return first_valid && second_valid && third_valid;
}

/**
 * @brief 更新投运期间的母线电压观测值
 *
 * @param bus_voltage_v 当前母线电压，单位伏特
 */
static void update_bus_voltage_status(float bus_voltage_v)
{
    if(!isfinite(bus_voltage_v) || bus_voltage_v <= 0.0f)
    {
        return;
    }

    commissioning_status.bus_voltage_v = bus_voltage_v;
    if(commissioning_status.minimum_bus_voltage_v <= 0.0f ||
        bus_voltage_v < commissioning_status.minimum_bus_voltage_v)
    {
        commissioning_status.minimum_bus_voltage_v = bus_voltage_v;
    }
}

/**
 * @brief 进入新的投运阶段并重置阶段统计
 *
 * @param stage 新阶段
 * @param timestamp_ms 阶段起始时间戳
 */
static void enter_commissioning_stage(foc_commissioning_stage stage,
    uint32_t timestamp_ms)
{
    commissioning_status.stage = stage;
    commissioning_stage_start_ms = timestamp_ms;
    reset_stage_samples();
    publish_commissioning_status();
}

/**
 * @brief 锁定投运失败状态并关闭功率输出
 *
 * @param result 失败原因
 */
static void fail_commissioning(foc_result result)
{
    foc::disable();
    commissioning_status.stage = foc_commissioning_stage::FAILED;
    commissioning_status.result = result;
    publish_commissioning_status();
}

/**
 * @brief 发布固定电角度的受限开环电压目标
 *
 * @param timestamp_ms 目标时间戳
 * @param electrical_angle_rad 电角度
 *
 * @return 目标发布结果
 */
static foc_result set_alignment_target(uint32_t timestamp_ms,
    float electrical_angle_rad)
{
    foc_command command{};
    command.timestamp_ms = timestamp_ms;
    command.target.mode = foc_control_mode::VOLTAGE;
    command.runtime_mode = foc_runtime_mode::OPEN_LOOP_VOLTAGE;
    command.target.d_axis_voltage_v = ALIGNMENT_VOLTAGE_V;
    command.electrical_angle_rad = electrical_angle_rad;
    command.electrical_angle_timestamp_us = sys_time::get_us_tick();
    return foc::internal::set_command(command);
}

/**
 * @brief 发布限压开环旋转磁场目标
 *
 * @param timestamp_ms 目标时间戳
 * @param electrical_angle_rad 电角度
 * @param q_axis_voltage_v Q 轴电压
 * @param electrical_velocity_rad_s 电角速度
 *
 * @return 目标发布结果
 */
static foc_result set_open_loop_target(uint32_t timestamp_ms,
    float electrical_angle_rad,
    float q_axis_voltage_v,
    float electrical_velocity_rad_s)
{
    foc_command command{};
    command.timestamp_ms = timestamp_ms;
    command.electrical_angle_timestamp_us = sys_time::get_us_tick();
    command.target.mode = foc_control_mode::VOLTAGE;
    command.runtime_mode = foc_runtime_mode::OPEN_LOOP_VOLTAGE;
    command.target.q_axis_voltage_v = q_axis_voltage_v;
    command.electrical_angle_rad = electrical_angle_rad;
    command.electrical_velocity_rad_s = electrical_velocity_rad_s;
    return foc::internal::set_command(command);
}

/**
 * @brief 判断母线电压是否处于开环验证允许范围
 *
 * @return 母线电压有效且处于允许范围时返回 true
 */
static bool open_loop_bus_voltage_valid()
{
    return commissioning_status.bus_voltage_v >=
        OPEN_LOOP_MINIMUM_BUS_VOLTAGE_V &&
        commissioning_status.bus_voltage_v <=
        OPEN_LOOP_MAXIMUM_BUS_VOLTAGE_V;
}

/**
 * @brief 启动一个限压开环速度验证阶段
 *
 * @param stage 正转或反转阶段
 * @param snapshot 最新 FOC 快照
 * @param timestamp_ms 阶段起始时间戳
 * @param electrical_angle_rad 起始电角度
 *
 * @return 输出启动结果
 */
static foc_result start_open_loop_stage(foc_commissioning_stage stage,
    const foc_snapshot &snapshot,
    uint32_t timestamp_ms,
    float electrical_angle_rad)
{
    foc_result result = set_open_loop_target(timestamp_ms,
        electrical_angle_rad,
        0.0f,
        0.0f);
    if(result != foc_result::OK){return result;}

    result = foc::enable();
    if(result != foc_result::OK){return result;}

    open_loop_stage_start_mechanical_angle_rad =
        snapshot.rotor.full_angle_rad;
    open_loop_stage_start_electrical_angle_rad =
        electrical_angle_rad;
    enter_commissioning_stage(stage, timestamp_ms);
    return foc_result::OK;
}

/**
 * @brief 计算梯形速度曲线累计转过的电角度
 *
 * @param rotation_elapsed_ms 旋转阶段经过时间
 *
 * @return 从本阶段起点累计的电角度
 */
static float calculate_open_loop_rotation_rad(
    uint32_t rotation_elapsed_ms)
{
    float acceleration_time_s =
        (float)OPEN_LOOP_ACCELERATION_TIME_MS * 0.001f;
    float constant_speed_time_s =
        (float)OPEN_LOOP_CONSTANT_SPEED_TIME_MS * 0.001f;
    float elapsed_s = (float)rotation_elapsed_ms * 0.001f;
    float electrical_velocity_rad_s =
        OPEN_LOOP_MECHANICAL_VELOCITY_RAD_S *
        (float)commissioning_config.motor_pole_pairs;
    float acceleration_rad_s2 =
        electrical_velocity_rad_s / acceleration_time_s;
    float acceleration_angle_rad = 0.5f * acceleration_rad_s2 *
        acceleration_time_s * acceleration_time_s;

    if(elapsed_s <= acceleration_time_s)
    {
        return 0.5f * acceleration_rad_s2 * elapsed_s * elapsed_s;
    }

    float constant_speed_end_s = acceleration_time_s +
        constant_speed_time_s;
    if(elapsed_s <= constant_speed_end_s)
    {
        return acceleration_angle_rad +
            electrical_velocity_rad_s *
            (elapsed_s - acceleration_time_s);
    }

    float deceleration_elapsed_s = elapsed_s - constant_speed_end_s;
    if(deceleration_elapsed_s > acceleration_time_s)
    {
        deceleration_elapsed_s = acceleration_time_s;
    }

    return acceleration_angle_rad +
        electrical_velocity_rad_s * constant_speed_time_s +
        electrical_velocity_rad_s * deceleration_elapsed_s -
        0.5f * acceleration_rad_s2 * deceleration_elapsed_s *
        deceleration_elapsed_s;
}

/**
 * @brief 计算梯形速度曲线当前电角速度
 *
 * @param rotation_elapsed_ms 旋转阶段经过时间
 *
 * @return 当前电角速度
 */
static float calculate_open_loop_velocity_rad_s(
    uint32_t rotation_elapsed_ms)
{
    float acceleration_time_s =
        (float)OPEN_LOOP_ACCELERATION_TIME_MS * 0.001f;
    float constant_speed_time_s =
        (float)OPEN_LOOP_CONSTANT_SPEED_TIME_MS * 0.001f;
    float elapsed_s = (float)rotation_elapsed_ms * 0.001f;
    float electrical_velocity_rad_s =
        OPEN_LOOP_MECHANICAL_VELOCITY_RAD_S *
        (float)commissioning_config.motor_pole_pairs;
    float acceleration_rad_s2 =
        electrical_velocity_rad_s / acceleration_time_s;

    if(elapsed_s <= acceleration_time_s)
    {
        return acceleration_rad_s2 * elapsed_s;
    }

    float constant_speed_end_s = acceleration_time_s +
        constant_speed_time_s;
    if(elapsed_s <= constant_speed_end_s)
    {
        return electrical_velocity_rad_s;
    }

    float deceleration_elapsed_s = elapsed_s - constant_speed_end_s;
    if(deceleration_elapsed_s >= acceleration_time_s)
    {
        return 0.0f;
    }

    return electrical_velocity_rad_s -
        acceleration_rad_s2 * deceleration_elapsed_s;
}

/**
 * @brief 更新限压开环速度验证目标
 *
 * @param timestamp_ms 当前时间戳
 * @param stage_elapsed_ms 当前阶段经过时间
 * @param direction 旋转方向
 *
 * @return 目标发布结果
 */
static foc_result update_open_loop_target(uint32_t timestamp_ms,
    uint32_t stage_elapsed_ms,
    int8_t direction)
{
    float voltage_progress = (float)stage_elapsed_ms /
        (float)OPEN_LOOP_HOLD_TIME_MS;
    if(voltage_progress > 1.0f){voltage_progress = 1.0f;}

    uint32_t rotation_elapsed_ms = stage_elapsed_ms >
        OPEN_LOOP_HOLD_TIME_MS ?
        stage_elapsed_ms - OPEN_LOOP_HOLD_TIME_MS : 0;
    float electrical_angle_rad =
        open_loop_stage_start_electrical_angle_rad +
        (float)direction *
        calculate_open_loop_rotation_rad(rotation_elapsed_ms);
    float q_axis_voltage_v = OPEN_LOOP_TEST_VOLTAGE_V *
        voltage_progress;
    float electrical_velocity_rad_s = (float)direction *
        calculate_open_loop_velocity_rad_s(rotation_elapsed_ms);
    return set_open_loop_target(timestamp_ms,
        electrical_angle_rad,
        q_axis_voltage_v,
        electrical_velocity_rad_s);
}

/**
 * @brief 完成正向开环验证并进入停机间隔
 *
 * @param snapshot 最新 FOC 快照
 * @param timestamp_ms 当前时间戳
 */
static void finish_open_loop_forward(const foc_snapshot &snapshot,
    uint32_t timestamp_ms)
{
    commissioning_status.open_loop_forward_delta_rad =
        snapshot.rotor.full_angle_rad -
        open_loop_stage_start_mechanical_angle_rad;
    open_loop_stage_start_electrical_angle_rad =
        foc_math::normalize_angle(
            open_loop_stage_start_electrical_angle_rad +
            calculate_open_loop_rotation_rad(OPEN_LOOP_RUN_TIME_MS));
    foc::disable();
    enter_commissioning_stage(
        foc_commissioning_stage::OPEN_LOOP_PAUSE,
        timestamp_ms);
}

/**
 * @brief 完成反向开环验证并关闭功率输出
 *
 * @param snapshot 最新 FOC 快照
 */
static void finish_open_loop_reverse(const foc_snapshot &snapshot)
{
    commissioning_status.open_loop_reverse_delta_rad =
        snapshot.rotor.full_angle_rad -
        open_loop_stage_start_mechanical_angle_rad;
    float forward_delta =
        commissioning_status.open_loop_forward_delta_rad;
    float reverse_delta =
        commissioning_status.open_loop_reverse_delta_rad;
    commissioning_status.open_loop_motion_detected =
        fabsf(forward_delta) >= OPEN_LOOP_MINIMUM_MECHANICAL_MOVE_RAD &&
        fabsf(reverse_delta) >= OPEN_LOOP_MINIMUM_MECHANICAL_MOVE_RAD &&
        forward_delta * reverse_delta < 0.0f;

    foc::disable();
    commissioning_status.stage = commissioning_status.open_loop_motion_detected ?
        foc_commissioning_stage::COMPLETE :
        foc_commissioning_stage::FAILED;
    commissioning_status.result = commissioning_status.open_loop_motion_detected ?
        foc_result::OK : foc_result::ROTOR_ALIGNMENT_FAILED;
    publish_commissioning_status();
}

/**
 * @brief 发布闭环 D-Q 电流目标
 *
 * @param timestamp_ms 目标时间戳
 * @param d_axis_current_a D 轴目标电流
 * @param q_axis_current_a Q 轴目标电流
 *
 * @return 目标发布结果
 */
static foc_result set_current_target(uint32_t timestamp_ms,
    float d_axis_current_a,
    float q_axis_current_a)
{
    foc_command command{};
    command.timestamp_ms = timestamp_ms;
    command.electrical_angle_timestamp_us = sys_time::get_us_tick();
    command.target.mode = foc_control_mode::CURRENT;
    command.runtime_mode = foc_runtime_mode::CURRENT;
    command.target.d_axis_current_a = d_axis_current_a;
    command.target.q_axis_current_a = q_axis_current_a;
    return foc::internal::set_command(command);
}

/**
 * @brief 启动固定电角度的受限对齐阶段
 *
 * @param stage 新阶段
 * @param timestamp_ms 阶段起始时间戳
 * @param electrical_angle_rad 固定电角度
 *
 * @return 输出启动结果
 */
static foc_result start_alignment_stage(foc_commissioning_stage stage,
    uint32_t timestamp_ms,
    float electrical_angle_rad)
{
    foc_result result = set_alignment_target(timestamp_ms,
        electrical_angle_rad);
    if(result != foc_result::OK){return result;}

    result = foc::enable();
    if(result != foc_result::OK){return result;}

    enter_commissioning_stage(stage, timestamp_ms);
    return foc_result::OK;
}

/**
 * @brief 累积当前阶段可用的电流快照
 *
 * @param snapshot 最新 FOC 快照
 */
static void accumulate_stage_sample(const foc_snapshot &snapshot)
{
    if(!snapshot.current.valid){return;}

    stage_current_sum_a += snapshot.current.current_a;
    stage_current_sum_b += snapshot.current.current_b;
    stage_d_axis_current_sum += snapshot.d_axis_current_a;
    stage_q_axis_current_sum += snapshot.q_axis_current_a;
    stage_sample_count++;
}

/**
 * @brief 保存第一个静态矢量并进入第二个矢量诊断
 *
 * @param snapshot 最新 FOC 快照
 * @param timestamp_ms 当前时间戳
 */
static void finish_first_alignment(const foc_snapshot &snapshot,
    uint32_t timestamp_ms)
{
    if(!save_phase_vector_result(0, snapshot))
    {
        fail_commissioning(foc_result::SAMPLE_NOT_READY);
        return;
    }

    foc_result start_result = start_alignment_stage(
        foc_commissioning_stage::DIAGNOSE_PHASE_B,
        timestamp_ms,
        SECOND_PHASE_VECTOR_ANGLE_RAD);
    if(start_result != foc_result::OK)
    {
        fail_commissioning(start_result);
    }
}

/**
 * @brief 保存第二个静态矢量并进入第三个矢量诊断
 *
 * @param snapshot 最新 FOC 快照
 * @param timestamp_ms 当前时间戳
 */
static void finish_second_phase_vector(const foc_snapshot &snapshot,
    uint32_t timestamp_ms)
{
    if(!save_phase_vector_result(1, snapshot))
    {
        fail_commissioning(foc_result::SAMPLE_NOT_READY);
        return;
    }

    foc_result start_result = start_alignment_stage(
        foc_commissioning_stage::DIAGNOSE_PHASE_C,
        timestamp_ms,
        THIRD_PHASE_VECTOR_ANGLE_RAD);
    if(start_result != foc_result::OK)
    {
        fail_commissioning(start_result);
    }
}

/**
 * @brief 保存第三个静态矢量并联合验证相电流关系
 *
 * @param snapshot 最新 FOC 快照
 * @param timestamp_ms 当前时间戳
 */
static void finish_third_phase_vector(const foc_snapshot &snapshot,
    uint32_t timestamp_ms)
{
    if(!save_phase_vector_result(2, snapshot))
    {
        fail_commissioning(foc_result::SAMPLE_NOT_READY);
        return;
    }

    foc::disable();
    commissioning_status.phase_vector_check_passed =
        phase_vector_results_valid();
    if(!commissioning_status.phase_vector_check_passed)
    {
        fail_commissioning(foc_result::SENSOR_ERROR);
        return;
    }

    commissioning_status.first_mechanical_angle_rad =
        commissioning_status.phase_vector_mechanical_angle_rad[0];
    commissioning_status.current_direction_a = 1;
    commissioning_status.current_direction_b = -1;

    foc_result direction_result =
        foc::internal::set_current_directions_task(
            commissioning_status.current_direction_a,
            commissioning_status.current_direction_b);
    if(direction_result != foc_result::OK)
    {
        fail_commissioning(direction_result);
        return;
    }

    foc_result start_result = start_alignment_stage(
        foc_commissioning_stage::VERIFY_CURRENT_POLARITY,
        timestamp_ms,
        FIRST_ALIGNMENT_ANGLE_RAD);
    if(start_result != foc_result::OK)
    {
        fail_commissioning(start_result);
    }
}

/**
 * @brief 验证电流极性后进入第二个电角度对齐点
 *
 * @param timestamp_ms 当前时间戳
 */
static void finish_current_polarity_verification(uint32_t timestamp_ms)
{
    if(stage_sample_count == 0)
    {
        fail_commissioning(foc_result::SAMPLE_NOT_READY);
        return;
    }

    float average_a = stage_current_sum_a / (float)stage_sample_count;
    float average_b = stage_current_sum_b / (float)stage_sample_count;
    if(average_a < CURRENT_POLARITY_MINIMUM_A ||
        average_b > -CURRENT_POLARITY_MINIMUM_A)
    {
        fail_commissioning(foc_result::SENSOR_ERROR);
        return;
    }

    foc::disable();
    foc_result start_result = start_alignment_stage(
        foc_commissioning_stage::ALIGN_SECOND,
        timestamp_ms,
        FIRST_ALIGNMENT_ANGLE_RAD);
    if(start_result != foc_result::OK)
    {
        fail_commissioning(start_result);
    }
}

/**
 * @brief 根据第二个对齐点确定转子方向与电角度零偏
 *
 * @param snapshot 最新 FOC 快照
 * @param timestamp_ms 当前时间戳
 */
static void finish_second_alignment(const foc_snapshot &snapshot,
    uint32_t timestamp_ms)
{
    foc::disable();
    commissioning_status.second_mechanical_angle_rad =
        snapshot.rotor.full_angle_rad;
    float movement = commissioning_status.second_mechanical_angle_rad -
        commissioning_status.first_mechanical_angle_rad;
    if(!isfinite(movement) ||
        fabsf(movement) < ROTOR_ALIGNMENT_MINIMUM_MOVE_RAD)
    {
        fail_commissioning(foc_result::ROTOR_ALIGNMENT_FAILED);
        return;
    }

    commissioning_status.rotor_direction = movement > 0.0f ? 1 : -1;
    commissioning_status.electrical_zero_offset_rad =
        foc_math::normalize_angle(
            ALIGNMENT_SWEEP_ANGLE_RAD -
            commissioning_status.second_mechanical_angle_rad *
            (float)commissioning_status.rotor_direction *
            (float)commissioning_config.motor_pole_pairs);

    foc_result alignment_result = foc::internal::set_rotor_alignment(
        commissioning_status.rotor_direction,
        commissioning_status.electrical_zero_offset_rad);
    if(alignment_result != foc_result::OK)
    {
        fail_commissioning(alignment_result);
        return;
    }

    foc_result target_result = set_current_target(timestamp_ms,
        D_AXIS_VERIFY_TARGET_A,
        0.0f);
    if(target_result != foc_result::OK)
    {
        fail_commissioning(target_result);
        return;
    }

    foc_result enable_result = foc::enable();
    if(enable_result != foc_result::OK)
    {
        fail_commissioning(enable_result);
        return;
    }

    enter_commissioning_stage(
        foc_commissioning_stage::VERIFY_D_AXIS_CURRENT,
        timestamp_ms);
}

/**
 * @brief 验证 D 轴电流环并进入受限 Q 轴测试
 *
 * @param timestamp_ms 当前时间戳
 */
static void finish_d_axis_verification(uint32_t timestamp_ms)
{
    if(stage_sample_count == 0)
    {
        fail_commissioning(foc_result::SAMPLE_NOT_READY);
        return;
    }

    commissioning_status.measured_d_axis_current_a =
        stage_d_axis_current_sum / (float)stage_sample_count;
    commissioning_status.measured_q_axis_current_a =
        stage_q_axis_current_sum / (float)stage_sample_count;
    if(!isfinite(commissioning_status.measured_d_axis_current_a) ||
        !isfinite(commissioning_status.measured_q_axis_current_a) ||
        commissioning_status.measured_d_axis_current_a < 0.005f ||
        commissioning_status.measured_d_axis_current_a > 0.20f ||
        fabsf(commissioning_status.measured_q_axis_current_a) > 0.15f)
    {
        fail_commissioning(foc_result::SENSOR_ERROR);
        return;
    }

    foc_result target_result = set_current_target(timestamp_ms,
        0.0f,
        Q_AXIS_VERIFY_TARGET_A);
    if(target_result != foc_result::OK)
    {
        fail_commissioning(target_result);
        return;
    }

    enter_commissioning_stage(
        foc_commissioning_stage::VERIFY_Q_AXIS_CURRENT,
        timestamp_ms);
}

/**
 * @brief 结束受限 Q 轴测试并让功率级保持关闭
 */
static void finish_q_axis_verification()
{
    foc::disable();
    commissioning_status.stage = foc_commissioning_stage::COMPLETE;
    commissioning_status.result = foc_result::OK;
    publish_commissioning_status();
}

/**
 * @brief 推进一次上电投运状态机
 *
 * @param snapshot 上一安全周期生成的 FOC 快照
 * @param snapshot_available 是否存在有效快照
 * @param timestamp_ms 当前时间戳
 */
static void update_commissioning(const foc_snapshot &snapshot,
    bool snapshot_available,
    uint32_t timestamp_ms)
{
    if(commissioning_status.stage == foc_commissioning_stage::COMPLETE ||
        commissioning_status.stage == foc_commissioning_stage::FAILED)
    {
        return;
    }

    if(snapshot_available && snapshot.state == foc_state::FAULT)
    {
        fail_commissioning(foc_result::OUTPUT_FAULT);
        return;
    }

    uint32_t stage_elapsed_ms = timestamp_ms -
        commissioning_stage_start_ms;

    switch(commissioning_status.stage)
    {
        case foc_commissioning_stage::WAIT_CALIBRATION:
            if(current_calibration_finished)
            {
                enter_commissioning_stage(
                    foc_commissioning_stage::WAIT_ROTOR,
                    timestamp_ms);
            }
            break;

        case foc_commissioning_stage::WAIT_ROTOR:
            if(snapshot_available && snapshot.rotor.valid &&
                snapshot.current.valid && open_loop_bus_voltage_valid())
            {
                commissioning_status.open_loop_voltage_v =
                    OPEN_LOOP_TEST_VOLTAGE_V;
                commissioning_status.open_loop_mechanical_velocity_rad_s =
                    OPEN_LOOP_MECHANICAL_VELOCITY_RAD_S;
                commissioning_status.open_loop_electrical_velocity_rad_s =
                    OPEN_LOOP_MECHANICAL_VELOCITY_RAD_S *
                    (float)commissioning_config.motor_pole_pairs;
                foc_result result = start_open_loop_stage(
                    foc_commissioning_stage::OPEN_LOOP_FORWARD,
                    snapshot,
                    timestamp_ms,
                    FIRST_ALIGNMENT_ANGLE_RAD);
                if(result != foc_result::OK)
                {
                    fail_commissioning(result);
                }
            }
            break;

        case foc_commissioning_stage::OPEN_LOOP_FORWARD:
            if(!open_loop_bus_voltage_valid())
            {
                fail_commissioning(foc_result::OUTPUT_FAULT);
                break;
            }
            if(update_open_loop_target(timestamp_ms,
                stage_elapsed_ms,
                1) != foc_result::OK)
            {
                fail_commissioning(foc_result::TOPIC_ERROR);
                break;
            }
            if(stage_elapsed_ms >=
                OPEN_LOOP_HOLD_TIME_MS + OPEN_LOOP_RUN_TIME_MS &&
                snapshot_available && snapshot.rotor.valid)
            {
                finish_open_loop_forward(snapshot, timestamp_ms);
            }
            break;

        case foc_commissioning_stage::OPEN_LOOP_PAUSE:
            if(stage_elapsed_ms >= OPEN_LOOP_PAUSE_TIME_MS &&
                snapshot_available && snapshot.rotor.valid &&
                open_loop_bus_voltage_valid())
            {
                foc_result result = start_open_loop_stage(
                    foc_commissioning_stage::OPEN_LOOP_REVERSE,
                    snapshot,
                    timestamp_ms,
                    open_loop_stage_start_electrical_angle_rad);
                if(result != foc_result::OK)
                {
                    fail_commissioning(result);
                }
            }
            break;

        case foc_commissioning_stage::OPEN_LOOP_REVERSE:
            if(!open_loop_bus_voltage_valid())
            {
                fail_commissioning(foc_result::OUTPUT_FAULT);
                break;
            }
            if(update_open_loop_target(timestamp_ms,
                stage_elapsed_ms,
                -1) != foc_result::OK)
            {
                fail_commissioning(foc_result::TOPIC_ERROR);
                break;
            }
            if(stage_elapsed_ms >=
                OPEN_LOOP_HOLD_TIME_MS + OPEN_LOOP_RUN_TIME_MS &&
                snapshot_available && snapshot.rotor.valid)
            {
                finish_open_loop_reverse(snapshot);
            }
            break;

        case foc_commissioning_stage::ALIGN_FIRST:
            if(set_alignment_target(timestamp_ms,
                FIRST_ALIGNMENT_ANGLE_RAD) != foc_result::OK)
            {
                fail_commissioning(foc_result::TOPIC_ERROR);
                break;
            }
            if(stage_elapsed_ms >= PHASE_VECTOR_SETTLE_TIME_MS / 2)
            {
                accumulate_stage_sample(snapshot);
            }
            if(stage_elapsed_ms >= PHASE_VECTOR_SETTLE_TIME_MS)
            {
                finish_first_alignment(snapshot, timestamp_ms);
            }
            break;

        case foc_commissioning_stage::DIAGNOSE_PHASE_B:
            if(set_alignment_target(timestamp_ms,
                SECOND_PHASE_VECTOR_ANGLE_RAD) != foc_result::OK)
            {
                fail_commissioning(foc_result::TOPIC_ERROR);
                break;
            }
            if(stage_elapsed_ms >= PHASE_VECTOR_SETTLE_TIME_MS / 2)
            {
                accumulate_stage_sample(snapshot);
            }
            if(stage_elapsed_ms >= PHASE_VECTOR_SETTLE_TIME_MS)
            {
                finish_second_phase_vector(snapshot, timestamp_ms);
            }
            break;

        case foc_commissioning_stage::DIAGNOSE_PHASE_C:
            if(set_alignment_target(timestamp_ms,
                THIRD_PHASE_VECTOR_ANGLE_RAD) != foc_result::OK)
            {
                fail_commissioning(foc_result::TOPIC_ERROR);
                break;
            }
            if(stage_elapsed_ms >= PHASE_VECTOR_SETTLE_TIME_MS / 2)
            {
                accumulate_stage_sample(snapshot);
            }
            if(stage_elapsed_ms >= PHASE_VECTOR_SETTLE_TIME_MS)
            {
                finish_third_phase_vector(snapshot, timestamp_ms);
            }
            break;

        case foc_commissioning_stage::VERIFY_CURRENT_POLARITY:
            if(set_alignment_target(timestamp_ms,
                FIRST_ALIGNMENT_ANGLE_RAD) != foc_result::OK)
            {
                fail_commissioning(foc_result::TOPIC_ERROR);
                break;
            }
            accumulate_stage_sample(snapshot);
            if(stage_elapsed_ms >= CURRENT_POLARITY_VERIFY_TIME_MS)
            {
                finish_current_polarity_verification(timestamp_ms);
            }
            break;

        case foc_commissioning_stage::ALIGN_SECOND:
        {
            float sweep_progress = (float)stage_elapsed_ms /
                (float)ALIGNMENT_SWEEP_TIME_MS;
            if(sweep_progress > 1.0f)
            {
                sweep_progress = 1.0f;
            }

            float electrical_angle_rad = ALIGNMENT_SWEEP_ANGLE_RAD *
                sweep_progress;
            if(set_alignment_target(timestamp_ms,
                electrical_angle_rad) != foc_result::OK)
            {
                fail_commissioning(foc_result::TOPIC_ERROR);
                break;
            }
            if(stage_elapsed_ms >= ALIGNMENT_SWEEP_TIME_MS)
            {
                finish_second_alignment(snapshot, timestamp_ms);
            }
            break;
        }

        case foc_commissioning_stage::VERIFY_D_AXIS_CURRENT:
            if(set_current_target(timestamp_ms,
                D_AXIS_VERIFY_TARGET_A,
                0.0f) != foc_result::OK)
            {
                fail_commissioning(foc_result::TOPIC_ERROR);
                break;
            }
            accumulate_stage_sample(snapshot);
            if(stage_elapsed_ms >= CURRENT_LOOP_VERIFY_TIME_MS)
            {
                finish_d_axis_verification(timestamp_ms);
            }
            break;

        case foc_commissioning_stage::VERIFY_Q_AXIS_CURRENT:
            if(set_current_target(timestamp_ms,
                0.0f,
                Q_AXIS_VERIFY_TARGET_A) != foc_result::OK)
            {
                fail_commissioning(foc_result::TOPIC_ERROR);
                break;
            }
            if(stage_elapsed_ms >= Q_AXIS_VERIFY_TIME_MS)
            {
                finish_q_axis_verification();
            }
            break;

        default:
            fail_commissioning(foc_result::INVALID_STATE);
            break;
    }
}


/**
 * @brief 初始化 FOC 投运状态机并发布初始状态
 *
 * @param config 投运配置
 * @param timestamp_ms 投运开始时间戳
 *
 * @return 初始化结果
 */
foc_result foc::commissioning::init(
    const foc_commissioning_config &config,
    uint32_t timestamp_ms)
{
    if(commissioning_initialized)
    {
        return foc_result::INVALID_STATE;
    }
    if(config.motor_pole_pairs == 0 ||
        config.current_calibration_sample_count == 0)
    {
        return foc_result::INVALID_ARGUMENT;
    }
    if(!commissioning_topic.init())
    {
        return foc_result::TOPIC_ERROR;
    }

    commissioning_config = config;
    commissioning_start_ms = timestamp_ms;
    commissioning_stage_start_ms = timestamp_ms;
    current_calibration_finished = false;
    calibration_task_started = false;
    commissioning_status = {};
    commissioning_status.result = foc_result::NOT_READY;
    commissioning_initialized = true;
    publish_commissioning_status();
    return foc_result::OK;
}

/**
 * @brief 周期推进 current calibration 与 commissioning 状态机
 *
 * @param timestamp_ms 当前毫秒时间戳
 * @param bus_voltage_v 当前母线电压，单位伏特
 */
void foc::commissioning::update(uint32_t timestamp_ms,
    float bus_voltage_v)
{
    if(!commissioning_initialized)
    {
        return;
    }

    update_bus_voltage_status(bus_voltage_v);

    if(!current_calibration_finished &&
        commissioning_status.stage != foc_commissioning_stage::FAILED)
    {
        if(timestamp_ms - commissioning_start_ms <
            commissioning_config.current_sensor_settle_time_ms)
        {
            return;
        }

        bool first_calibration_attempt = !calibration_task_started;
        foc_result calibration_result =
            foc::internal::calibrate_current_task(
                commissioning_config.current_calibration_sample_count);
        calibration_task_started = true;
        if(calibration_result == foc_result::OK)
        {
            current_calibration_finished = true;
        }
        else if(calibration_result != foc_result::CALIBRATING)
        {
            fail_commissioning(first_calibration_attempt ?
                foc_result::SENSOR_ERROR : calibration_result);
            return;
        }
    }

    if(!current_calibration_finished)
    {
        return;
    }

    foc_snapshot snapshot{};
    bool snapshot_available = foc::peek_snapshot(snapshot);
    update_commissioning(snapshot,
        snapshot_available,
        timestamp_ms);
}

/**
 * @brief 读取最新 FOC 投运状态
 *
 * @param status 用于接收投运状态的对象
 *
 * @return 已发布状态时返回 true
 */
bool foc::commissioning::peek_status(foc_commissioning_status &status)
{
    return commissioning_topic.peek(status);
}

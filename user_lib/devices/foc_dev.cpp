#include "foc_dev.h"

#include "drivers/foc/foc_core.h"
#include "drivers/foc/foc_math.h"
#include "drivers/foc/phase_driver/tim1_phase_driver.h"
#include "drivers/foc/sensors/current_sense/stm32_two_shunt_current_sensor.h"
#include "drivers/foc/sensors/encoder/as5600_rotor_sensor.h"
#include "system/sys_time.h"
#include "system/topic.h"
#include "adc.h"
#include "main.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

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
static constexpr uint32_t CURRENT_SENSOR_SETTLE_TIME_MS = 500;
static constexpr uint32_t CURRENT_CALIBRATION_SAMPLE_COUNT = 4096;
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
static constexpr float BUS_VOLTAGE_DIVIDER_RATIO = 11.0f;
static constexpr float OPEN_LOOP_TEST_VOLTAGE_V = 3.0f;
static constexpr float OPEN_LOOP_MECHANICAL_VELOCITY_RAD_S = 10.0f;
static constexpr float OPEN_LOOP_ELECTRICAL_VELOCITY_RAD_S =
    OPEN_LOOP_MECHANICAL_VELOCITY_RAD_S * (float)MOTOR_POLE_PAIRS;
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
static bool bus_voltage_sampling_started = false;
static float open_loop_stage_start_mechanical_angle_rad = 0.0f;
static float open_loop_stage_start_electrical_angle_rad = 0.0f;

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
 */
static void update_bus_voltage_status()
{
    float bus_voltage_v = read_bus_voltage_v();
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
    foc_core::disable();
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
    foc_target target{};
    target.timestamp_ms = timestamp_ms;
    target.mode = foc_control_mode::OPEN_LOOP_VOLTAGE;
    target.d_axis_voltage_v = ALIGNMENT_VOLTAGE_V;
    target.electrical_angle_rad = electrical_angle_rad;
    target.electrical_angle_timestamp_us = sys_time::get_us_tick();
    return foc_core::set_target(target);
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
    foc_target target{};
    target.timestamp_ms = timestamp_ms;
    target.electrical_angle_timestamp_us = sys_time::get_us_tick();
    target.mode = foc_control_mode::OPEN_LOOP_VOLTAGE;
    target.q_axis_voltage_v = q_axis_voltage_v;
    target.electrical_angle_rad = electrical_angle_rad;
    target.electrical_velocity_rad_s = electrical_velocity_rad_s;
    return foc_core::set_target(target);
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

    result = foc_core::enable();
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
    float acceleration_rad_s2 =
        OPEN_LOOP_ELECTRICAL_VELOCITY_RAD_S / acceleration_time_s;
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
            OPEN_LOOP_ELECTRICAL_VELOCITY_RAD_S *
            (elapsed_s - acceleration_time_s);
    }

    float deceleration_elapsed_s = elapsed_s - constant_speed_end_s;
    if(deceleration_elapsed_s > acceleration_time_s)
    {
        deceleration_elapsed_s = acceleration_time_s;
    }

    return acceleration_angle_rad +
        OPEN_LOOP_ELECTRICAL_VELOCITY_RAD_S * constant_speed_time_s +
        OPEN_LOOP_ELECTRICAL_VELOCITY_RAD_S * deceleration_elapsed_s -
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
    float acceleration_rad_s2 =
        OPEN_LOOP_ELECTRICAL_VELOCITY_RAD_S / acceleration_time_s;

    if(elapsed_s <= acceleration_time_s)
    {
        return acceleration_rad_s2 * elapsed_s;
    }

    float constant_speed_end_s = acceleration_time_s +
        constant_speed_time_s;
    if(elapsed_s <= constant_speed_end_s)
    {
        return OPEN_LOOP_ELECTRICAL_VELOCITY_RAD_S;
    }

    float deceleration_elapsed_s = elapsed_s - constant_speed_end_s;
    if(deceleration_elapsed_s >= acceleration_time_s)
    {
        return 0.0f;
    }

    return OPEN_LOOP_ELECTRICAL_VELOCITY_RAD_S -
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
    foc_core::disable();
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

    foc_core::disable();
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
    foc_target target{};
    target.timestamp_ms = timestamp_ms;
    target.mode = foc_control_mode::CURRENT;
    target.d_axis_current_a = d_axis_current_a;
    target.q_axis_current_a = q_axis_current_a;
    return foc_core::set_target(target);
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

    result = foc_core::enable();
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

    foc_core::disable();
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

    foc_result direction_result = phase_current.set_directions_task(
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

    foc_core::disable();
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
    foc_core::disable();
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
            (float)MOTOR_POLE_PAIRS);

    foc_result alignment_result = foc_core::set_rotor_alignment(
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

    foc_result enable_result = foc_core::enable();
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
    foc_core::disable();
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
                    OPEN_LOOP_ELECTRICAL_VELOCITY_RAD_S;
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
        foc_result result = foc_core::update_sensors();
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
 * @brief 周期执行 FOC 安全检查与受限硬件投运
 *
 * @param argument FreeRTOS 任务参数
 */
static void foc_safety_task_entry(void *argument)
{
    vTaskDelay(pdMS_TO_TICKS(CURRENT_SENSOR_SETTLE_TIME_MS));
    foc_result calibration_result =
        foc_core::calibrate_current_task(
            CURRENT_CALIBRATION_SAMPLE_COUNT);
    if(calibration_result != foc_result::OK &&
        calibration_result != foc_result::CALIBRATING)
    {
        fail_commissioning(foc_result::SENSOR_ERROR);
    }
    else if(calibration_result == foc_result::OK)
    {
        current_calibration_finished = true;
    }

    TickType_t last_wake_time = xTaskGetTickCount();

    while(true)
    {
        if(!current_calibration_finished &&
            commissioning_status.stage !=
                foc_commissioning_stage::FAILED)
        {
            foc_result calibration_result =
                foc_core::calibrate_current_task(
                    CURRENT_CALIBRATION_SAMPLE_COUNT);
            if(calibration_result == foc_result::OK)
            {
                current_calibration_finished = true;
            }
            else if(calibration_result != foc_result::CALIBRATING)
            {
                fail_commissioning(calibration_result);
            }
        }

        uint32_t timestamp_ms = sys_time::get_ms_tick();
        update_bus_voltage_status();
        foc_snapshot snapshot{};
        bool snapshot_available = foc_core::peek_snapshot(snapshot);
        update_commissioning(snapshot,
            snapshot_available,
            timestamp_ms);
        foc_core::update_safety(timestamp_ms);

        vTaskDelayUntil(&last_wake_time,
            pdMS_TO_TICKS(FOC_SAFETY_UPDATE_PERIOD_MS));
    }
}

/**
 * @brief 初始化受限 FOC 硬件、控制核心和三层调度任务
 */
void foc_dev::init()
{
    if(!commissioning_topic.init())
    {
        Error_Handler();
    }

    commissioning_status = {};
    commissioning_status.result = foc_result::NOT_READY;
    publish_commissioning_status();

    if(foc_core::link_rotor_sensor(rotor) != foc_result::OK ||
        foc_core::link_current_sensor(phase_current) != foc_result::OK ||
        foc_core::link_phase_driver(phase_output) != foc_result::OK ||
        foc_core::init(make_control_config()) != foc_result::OK)
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
 * @brief 读取最新硬件投运状态
 *
 * @param status 用于接收状态的对象
 *
 * @return 已发布状态时返回 true
 */
bool foc_dev::peek_commissioning_status(
    foc_commissioning_status &status)
{
    return commissioning_topic.peek(status);
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
        foc_core::run_control_from_isr(sys_time::get_us_tick());
    }
}

#include "foc_core.h"

#include "foc_math.h"
#include "system/sys_time.h"
#include "system/topic.h"
#include "FreeRTOS.h"
#include <math.h>

struct control_telemetry
{
    uint32_t sequence = 0;
    uint32_t timestamp_us = 0;
    rotor_sample rotor{};
    phase_current_sample current{};
    float rotor_sample_age_ms = 0.0f;
    float d_axis_current_a = 0.0f;
    float q_axis_current_a = 0.0f;
    float d_axis_voltage_v = 0.0f;
    float q_axis_voltage_v = 0.0f;
    phase_duty duty{};
};

struct fault_request
{
    uint32_t sequence = 0;
    uint32_t fault_flags = 0;
};

struct foc_context
{
    foc_config config{};
    rotor_sensor *rotor = nullptr;
    current_sensor *current = nullptr;
    phase_driver *driver = nullptr;
    pi_state d_axis_pi{};
    pi_state q_axis_pi{};
    phase_current_sample fault_current{};
    volatile foc_state state = foc_state::UNINITIALIZED;
    volatile uint32_t fault_flags = 0;
    uint32_t command_sequence = 0;
    uint32_t control_isr_prescaler = 0;
    uint32_t control_sequence = 0;
    uint32_t snapshot_sequence = 0;
    uint32_t fault_request_sequence = 0;
    volatile uint32_t bus_update_error_count = 0;
    volatile uint32_t consecutive_bus_error_count = 0;
    bool rotor_initialized = false;
    bool current_initialized = false;
    bool driver_initialized = false;
    bool initialized = false;
    bool fault_current_valid = false;
    bool current_calibration_started = false;
    bool current_calibration_done = false;
    volatile bool calibration_output_active = false;
};

static foc_context core_context;
static topic::latest_topic<foc_command> command_topic;
static topic::latest_topic<control_telemetry> telemetry_topic;
static topic::latest_topic<fault_request> fault_request_topic;
static topic::latest_topic<phase_current_sample> current_sample_topic;
static topic::latest_topic<foc_snapshot> snapshot_topic;

/**
 * @brief 判断浮点值是否有限且非负
 *
 * @param value 待检查的数值
 *
 * @return 有限且非负时返回 true
 */
static bool finite_non_negative(float value)
{
    return isfinite(value) && value >= 0.0f;
}

/**
 * @brief 锁存故障并立即关闭硬件输出
 *
 * @param fault_flags 本次新增故障位
 */
static void latch_fault(uint32_t fault_flags)
{
    core_context.fault_flags |= fault_flags;
    core_context.state = foc_state::FAULT;
    foc_math::reset_pi(core_context.d_axis_pi);
    foc_math::reset_pi(core_context.q_axis_pi);

    if(core_context.driver)
    {
        core_context.driver->disable_output();
    }
    core_context.calibration_output_active = false;
}

/**
 * @brief 计算允许样本在任务抢占下略晚于观察时刻的无符号年龄
 *
 * @param timestamp_us 当前观察时间戳
 * @param sample_timestamp_us 样本时间戳
 *
 * @return 样本年龄；样本略处于未来时返回零
 */
static uint32_t sample_age_us(uint32_t timestamp_us,
    uint32_t sample_timestamp_us)
{
    uint32_t elapsed = timestamp_us - sample_timestamp_us;
    return elapsed <= INT32_MAX ? elapsed : 0;
}

/**
 * @brief 按调试分频发布最新相电流样本
 *
 * @param sample 本周期同步相电流样本
 */
static void publish_current_sample_from_isr(
    const phase_current_sample &sample,
    bool force_publish = false)
{
    if(!force_publish &&
        sample.sequence % core_context.config.telemetry_divider != 0)
    {
        return;
    }

    BaseType_t higher_priority_task_woken = pdFALSE;
    if(!current_sample_topic.publish_from_isr(sample,
        higher_priority_task_woken))
    {
        latch_fault(foc_fault_mask(foc_fault::INTERNAL));
    }
}

/**
 * @brief 发布普通业务目标并由核心生成命令元数据
 *
 * @param target D-Q 控制目标
 *
 * @return 目标发布结果
 */
foc_result foc::set_target(const foc_target &target)
{
    // 开环角度和速度属于 commissioning 的内部命令，普通目标不携带这些字段。
    if(target.mode == foc_control_mode::OPEN_LOOP_VOLTAGE)
    {
        return foc_result::INVALID_ARGUMENT;
    }

    foc_command command{};
    command.target = target;
    command.timestamp_ms = sys_time::get_ms_tick();
    command.electrical_angle_timestamp_us = sys_time::get_us_tick();
    return foc::internal::set_command(command);
}

/**
 * @brief 一次性绑定 FOC 所需硬件抽象
 *
 * @param hardware 转子、电流和功率驱动接口
 *
 * @return 绑定结果
 */
foc_result foc::internal::bind_hardware(const foc_hardware &hardware)
{
    if(core_context.initialized || core_context.rotor ||
        core_context.current || core_context.driver)
    {
        return foc_result::INVALID_STATE;
    }

    if(!hardware.rotor || !hardware.driver)
    {
        return foc_result::NOT_LINKED;
    }

    core_context.rotor = hardware.rotor;
    core_context.current = hardware.current;
    core_context.driver = hardware.driver;
    return foc_result::OK;
}

/**
 * @brief 发布核心内部控制命令
 *
 * @param command 新控制命令
 *
 * @return 命令校验和发布结果
 */
foc_result foc::internal::set_command(const foc_command &command)
{
    if(!core_context.initialized){return foc_result::NOT_INITIALIZED;}

    // 命令会被控制 ISR 直接读取，统一拦截 NaN/Inf，避免无效值进入控制链。
    if(!isfinite(command.target.d_axis_current_a) ||
        !isfinite(command.target.q_axis_current_a) ||
        !isfinite(command.target.d_axis_voltage_v) ||
        !isfinite(command.target.q_axis_voltage_v) ||
        !isfinite(command.electrical_angle_rad) ||
        !isfinite(command.electrical_velocity_rad_s))
    {
        return foc_result::INVALID_ARGUMENT;
    }

    foc_command pending_command = command;
    pending_command.sequence = ++core_context.command_sequence;
    if(!command_topic.publish(pending_command))
    {
        return foc_result::TOPIC_ERROR;
    }

    return foc_result::OK;
}

/**
 * @brief 在功率输出关闭时更新转子方向与电角度零偏
 *
 * @param rotor_direction 转子传感器方向
 * @param electrical_zero_offset_rad 电角度零偏
 *
 * @return 参数和状态均有效时返回 OK
 */
foc_result foc::internal::set_rotor_alignment(int8_t rotor_direction,
    float electrical_zero_offset_rad)
{
    if(!core_context.initialized){return foc_result::NOT_INITIALIZED;}
    if(core_context.state == foc_state::RUNNING)
    {
        return foc_result::INVALID_STATE;
    }
    if((rotor_direction != 1 && rotor_direction != -1) ||
        !isfinite(electrical_zero_offset_rad))
    {
        return foc_result::INVALID_ARGUMENT;
    }

    core_context.config.rotor_direction = rotor_direction;
    core_context.config.electrical_zero_offset_rad =
        foc_math::normalize_angle(electrical_zero_offset_rad);
    foc_math::reset_pi(core_context.d_axis_pi);
    foc_math::reset_pi(core_context.q_axis_pi);
    return foc_result::OK;
}

/**
 * @brief 在三相中性 PWM 条件下推进一次相电流零偏校准
 *
 * @param sample_count 需要累计的同步 ADC 样本数量
 *
 * @return 校准未完成时返回 CALIBRATING，完成时返回 OK
 */
foc_result foc::internal::calibrate_current_task(uint32_t sample_count)
{
    if(!core_context.initialized){return foc_result::NOT_INITIALIZED;}
    if(!core_context.current || !core_context.current_initialized)
    {
        return foc_result::NOT_LINKED;
    }
    if(core_context.state == foc_state::RUNNING)
    {
        return foc_result::INVALID_STATE;
    }

    if(!core_context.current_calibration_started)
    {
        core_context.driver->disable_output();
        core_context.calibration_output_active = false;
        foc_result calibration_result =
            core_context.current->calibrate_task(sample_count);
        if(calibration_result != foc_result::CALIBRATING &&
            calibration_result != foc_result::OK)
        {
            return calibration_result;
        }

        if(!core_context.config.monitor_only)
        {
            foc_result output_result =
                core_context.driver->enable_output_task();
            if(output_result != foc_result::OK)
            {
                core_context.driver->disable_output();
                core_context.calibration_output_active = false;
                return output_result;
            }

            core_context.calibration_output_active = true;
        }

        core_context.current_calibration_started = true;
        if(calibration_result == foc_result::OK)
        {
            core_context.current_calibration_done = true;
        }

        return calibration_result;
    }

    foc_result calibration_result =
        core_context.current->calibrate_task(sample_count);
    if(calibration_result == foc_result::OK)
    {
        core_context.current_calibration_done = true;
    }
    return calibration_result;
}

/**
 * @brief 尝试进入真实 FOC 运行状态并开启功率输出
 *
 * @return 首版监视配置固定拒绝使能
 */
foc_result foc::enable()
{
    if(!core_context.initialized){return foc_result::NOT_INITIALIZED;}
    if(core_context.config.monitor_only){return foc_result::DISABLED;}
    if(core_context.fault_flags != 0){return foc_result::INVALID_STATE;}
    if(!core_context.rotor_initialized ||
        !core_context.current_initialized ||
        !core_context.driver_initialized ||
        !core_context.current_calibration_done)
    {
        return foc_result::NOT_READY;
    }

    foc_command command{};
    if(!command_topic.peek(command) ||
        command.target.mode == foc_control_mode::DISABLED)
    {
        return foc_result::NOT_READY;
    }

    foc_result result = core_context.driver->enable_output_task();
    if(result != foc_result::OK){return result;}

    foc_math::reset_pi(core_context.d_axis_pi);
    foc_math::reset_pi(core_context.q_axis_pi);
    core_context.calibration_output_active = false;
    core_context.state = foc_state::RUNNING;
    return foc_result::OK;
}

/**
 * @brief 关闭硬件输出并退出运行状态
 */
void foc::disable()
{
    if(core_context.driver)
    {
        core_context.driver->disable_output();
    }
    core_context.calibration_output_active = false;

    foc_math::reset_pi(core_context.d_axis_pi);
    foc_math::reset_pi(core_context.q_axis_pi);
    if(core_context.initialized && core_context.fault_flags == 0)
    {
        core_context.state = core_context.config.monitor_only ?
            foc_state::MONITORING : foc_state::READY;
    }
}

/**
 * @brief 在输出关闭且硬件无 Fault 时清除锁存的软件故障
 *
 * @return 故障清除结果
 */
foc_result foc::clear_fault()
{
    if(!core_context.initialized){return foc_result::NOT_INITIALIZED;}
    core_context.driver->disable_output();
    core_context.calibration_output_active = false;
    if(core_context.driver->fault_active_from_isr())
    {
        return foc_result::DRIVER_FAULT;
    }

    core_context.fault_flags = 0;
    core_context.fault_current_valid = false;
    core_context.consecutive_bus_error_count = 0;
    fault_request empty_request{};
    empty_request.sequence = ++core_context.fault_request_sequence;
    if(!fault_request_topic.publish(empty_request))
    {
        return foc_result::TOPIC_ERROR;
    }

    core_context.state = core_context.config.monitor_only ?
        foc_state::MONITORING : foc_state::READY;
    return foc_result::OK;
}

/**
 * @brief 执行一次 ADC 同步的完整 FOC 控制周期
 *
 * @param timestamp_us 本控制周期的微秒时间戳
 *
 * @return 本周期执行结果
 */
foc_result foc::internal::run_control_from_isr(uint32_t timestamp_us)
{
    if(!core_context.initialized){return foc_result::NOT_INITIALIZED;}

    phase_current_sample current{};
    foc_result current_result = foc_result::NOT_LINKED;
    if(core_context.current && core_context.current_initialized)
    {
        current_result =
            core_context.current->read_conversion_from_isr(timestamp_us,
                current);
        if(current_result == foc_result::OK && current.valid &&
            (core_context.state == foc_state::RUNNING ||
                core_context.calibration_output_active))
        {
            publish_current_sample_from_isr(current);
        }
    }

    if(core_context.state != foc_state::RUNNING)
    {
        if(current_result == foc_result::CALIBRATING)
        {
            return foc_result::CALIBRATING;
        }
        if(current_result != foc_result::OK)
        {
            return current_result;
        }
        return foc_result::DISABLED;
    }

    fault_request requested_fault{};
    if(fault_request_topic.peek_from_isr(requested_fault) &&
        requested_fault.fault_flags != 0)
    {
        latch_fault(requested_fault.fault_flags);
        return foc_result::OUTPUT_FAULT;
    }

    if(core_context.driver->fault_active_from_isr())
    {
        latch_fault(foc_fault_mask(foc_fault::DRIVER));
        return foc_result::DRIVER_FAULT;
    }

    if(current_result != foc_result::OK || !current.valid)
    {
        latch_fault(foc_fault_mask(foc_fault::CURRENT_NOT_READY));
        return current_result;
    }

    if(!isfinite(current.current_a) || !isfinite(current.current_b) ||
        !isfinite(current.current_c))
    {
        latch_fault(foc_fault_mask(foc_fault::INVALID_NUMBER));
        return foc_result::SENSOR_ERROR;
    }

    // 过流判定放在坐标变换前，并保留故障瞬间的原始样本用于安全快照。
    float current_limit = core_context.config.max_phase_current_a;
    if(fabsf(current.current_a) > current_limit ||
        fabsf(current.current_b) > current_limit ||
        fabsf(current.current_c) > current_limit)
    {
        core_context.fault_current = current;
        core_context.fault_current_valid = true;
        publish_current_sample_from_isr(current, true);
        latch_fault(foc_fault_mask(foc_fault::OVER_CURRENT));
        return foc_result::SENSOR_ERROR;
    }

    core_context.control_isr_prescaler++;
    if(core_context.control_isr_prescaler %
        core_context.config.control_isr_divider != 0)
    {
        return foc_result::OK;
    }

    rotor_sample rotor{};
    foc_result rotor_result = core_context.rotor->read_from_isr(rotor);
    if(rotor_result != foc_result::OK || !rotor.valid)
    {
        latch_fault(foc_fault_mask(foc_fault::ROTOR_NOT_READY));
        return rotor_result;
    }

    uint32_t rotor_age_us = sample_age_us(timestamp_us,
        rotor.timestamp_us);
    if(rotor_age_us > core_context.config.rotor_hard_timeout_us)
    {
        latch_fault(foc_fault_mask(foc_fault::ROTOR_STALE));
        return foc_result::SAMPLE_STALE;
    }

    foc_command command{};
    if(!command_topic.peek_from_isr(command) ||
        command.target.mode == foc_control_mode::DISABLED)
    {
        latch_fault(foc_fault_mask(foc_fault::COMMAND_TIMEOUT));
        return foc_result::NOT_READY;
    }

    const foc_target &target = command.target;

    float mechanical_angle = rotor.mechanical_angle_rad;
    if(rotor_age_us <= core_context.config.rotor_extrapolation_limit_us)
    {
        mechanical_angle += rotor.mechanical_velocity_rad_s *
            (float)rotor_age_us * 1.0e-6f;
    }

    float electrical_angle = 0.0f;
    if(target.mode == foc_control_mode::OPEN_LOOP_VOLTAGE)
    {
        uint32_t target_age_us = timestamp_us -
            command.electrical_angle_timestamp_us;
        electrical_angle = foc_math::normalize_angle(
            command.electrical_angle_rad +
            command.electrical_velocity_rad_s *
            (float)target_age_us * 1.0e-6f);
    }
    else
    {
        electrical_angle = foc_math::normalize_angle(
            mechanical_angle *
            (float)core_context.config.rotor_direction *
            (float)core_context.config.pole_pairs +
            core_context.config.electrical_zero_offset_rad);
    }
    alpha_beta_value stationary_current =
        foc_math::clarke_transform(current);
    d_q_value rotating_current = foc_math::park_transform(
        stationary_current,
        electrical_angle);
    d_q_value rotating_voltage{};

    if(target.mode == foc_control_mode::OPEN_LOOP_VOLTAGE ||
        target.mode == foc_control_mode::VOLTAGE)
    {
        rotating_voltage.d = target.d_axis_voltage_v;
        rotating_voltage.q = target.q_axis_voltage_v;
    }
    else if(target.mode == foc_control_mode::CURRENT)
    {
        rotating_voltage.d = foc_math::run_pi(
            target.d_axis_current_a - rotating_current.d,
            core_context.config.control_period_s,
            core_context.config.d_axis_pi,
            core_context.config.voltage_limit_v,
            core_context.d_axis_pi);
        rotating_voltage.q = foc_math::run_pi(
            target.q_axis_current_a - rotating_current.q,
            core_context.config.control_period_s,
            core_context.config.q_axis_pi,
            core_context.config.voltage_limit_v,
            core_context.q_axis_pi);
    }
    else
    {
        latch_fault(foc_fault_mask(foc_fault::COMMAND_TIMEOUT));
        return foc_result::INVALID_STATE;
    }

    foc_math::limit_vector(rotating_voltage,
        core_context.config.voltage_limit_v);
    alpha_beta_value stationary_voltage =
        foc_math::inverse_park_transform(rotating_voltage,
            electrical_angle);
    phase_duty duty{};
    if(!foc_math::calculate_svpwm(stationary_voltage,
        core_context.config.bus_voltage_v,
        duty))
    {
        latch_fault(foc_fault_mask(foc_fault::OUTPUT_RANGE));
        return foc_result::OUTPUT_FAULT;
    }

    foc_result output_result =
        core_context.driver->write_duty_from_isr(duty);
    if(output_result != foc_result::OK)
    {
        latch_fault(foc_fault_mask(foc_fault::OUTPUT_RANGE));
        return output_result;
    }

    core_context.control_sequence++;
    control_telemetry telemetry{};
    telemetry.sequence = core_context.control_sequence;
    telemetry.timestamp_us = timestamp_us;
    telemetry.rotor = rotor;
    telemetry.current = current;
    telemetry.rotor_sample_age_ms = (float)rotor_age_us * 0.001f;
    telemetry.d_axis_current_a = rotating_current.d;
    telemetry.q_axis_current_a = rotating_current.q;
    telemetry.d_axis_voltage_v = rotating_voltage.d;
    telemetry.q_axis_voltage_v = rotating_voltage.q;
    telemetry.duty = duty;

    // 遥测只按配置分频发布，避免每个控制周期都向 Topic 发送调试数据。
    if(core_context.control_sequence %
        core_context.config.telemetry_divider == 0)
    {
        BaseType_t higher_priority_task_woken = pdFALSE;
        if(!telemetry_topic.publish_from_isr(telemetry,
            higher_priority_task_woken))
        {
            latch_fault(foc_fault_mask(foc_fault::INTERNAL));
        }
    }
    return foc_result::OK;
}

/**
 * @brief 在高频 FreeRTOS 任务中推进一次传感器采集
 *
 * @return 本次初始化或采样结果
 */
foc_result foc::internal::update_sensors()
{
    if(!core_context.initialized){return foc_result::NOT_INITIALIZED;}

    foc_result result;
    if(!core_context.rotor_initialized)
    {
        result = core_context.rotor->init();
        core_context.rotor_initialized = result == foc_result::OK;
    }
    else
    {
        result = core_context.rotor->update_task();
    }

    if(result == foc_result::OK)
    {
        core_context.consecutive_bus_error_count = 0;
    }
    else
    {
        core_context.bus_update_error_count++;
        core_context.consecutive_bus_error_count++;
    }

    return result;
}

/**
 * @brief 在低频 FreeRTOS 任务中检查慢速故障并发布调试快照
 *
 * @param timestamp_ms 当前毫秒时间戳
 *
 * @return 本次安全检查结果
 */
foc_result foc::internal::update_safety(uint32_t timestamp_ms)
{
    if(!core_context.initialized){return foc_result::NOT_INITIALIZED;}

    rotor_sample rotor{};
    foc_result rotor_result = core_context.rotor_initialized ?
        core_context.rotor->read_task(rotor) :
        foc_result::SAMPLE_NOT_READY;
    uint32_t now_us = timestamp_ms * 1000;
    uint32_t rotor_age_us = rotor_result == foc_result::OK ?
        sample_age_us(now_us, rotor.timestamp_us) : now_us;
    uint32_t new_fault_flags = 0;
    foc_command command{};
    command_topic.peek(command);
    const foc_target &target = command.target;

    if(core_context.consecutive_bus_error_count >=
        core_context.config.communication_error_limit)
    {
        new_fault_flags |= foc_fault_mask(foc_fault::ROTOR_COMMUNICATION);
    }

    if(rotor_result != foc_result::OK &&
        now_us > core_context.config.rotor_slow_timeout_us)
    {
        new_fault_flags |= foc_fault_mask(foc_fault::ROTOR_NOT_READY);
    }
    else if(rotor_result == foc_result::OK &&
        rotor_age_us > core_context.config.rotor_slow_timeout_us)
    {
        new_fault_flags |= foc_fault_mask(foc_fault::ROTOR_STALE);
    }

    if(!core_context.config.monitor_only &&
        core_context.state == foc_state::RUNNING &&
        target.mode != foc_control_mode::DISABLED &&
        timestamp_ms - command.timestamp_ms >
            core_context.config.command_timeout_ms)
    {
        new_fault_flags |= foc_fault_mask(foc_fault::COMMAND_TIMEOUT);
    }

    if(new_fault_flags != 0)
    {
        fault_request request{};
        request.sequence = ++core_context.fault_request_sequence;
        request.fault_flags = new_fault_flags;
        if(!fault_request_topic.publish(request))
        {
            latch_fault(foc_fault_mask(foc_fault::INTERNAL));
            return foc_result::TOPIC_ERROR;
        }

        latch_fault(new_fault_flags);
    }

    control_telemetry telemetry{};
    telemetry_topic.peek(telemetry);

    foc_snapshot snapshot{};
    snapshot.sequence = ++core_context.snapshot_sequence;
    snapshot.timestamp_us = now_us;
    snapshot.state = core_context.state;
    snapshot.fault_flags = core_context.fault_flags;
    snapshot.monitor_only = core_context.config.monitor_only;
    snapshot.output_active = core_context.state == foc_state::RUNNING ||
        core_context.calibration_output_active;
    snapshot.rotor = rotor;
    if(core_context.fault_current_valid)
    {
        snapshot.current = core_context.fault_current;
    }
    else if(!current_sample_topic.peek(snapshot.current))
    {
        snapshot.current = telemetry.current;
    }
    if(!snapshot.output_active && !core_context.fault_current_valid)
    {
        snapshot.current.valid = false;
    }
    snapshot.target = target;
    snapshot.rotor_sample_age_ms = rotor_result == foc_result::OK ?
        (float)rotor_age_us * 0.001f : -1.0f;
    snapshot.d_axis_current_a = telemetry.d_axis_current_a;
    snapshot.q_axis_current_a = telemetry.q_axis_current_a;
    snapshot.d_axis_voltage_v = telemetry.d_axis_voltage_v;
    snapshot.q_axis_voltage_v = telemetry.q_axis_voltage_v;
    snapshot.duty = telemetry.duty;
    snapshot.control_sequence = core_context.control_sequence;
    snapshot.bus_update_error_count =
        core_context.bus_update_error_count;
    snapshot.consecutive_bus_error_count =
        core_context.consecutive_bus_error_count;

    if(!snapshot_topic.publish(snapshot))
    {
        latch_fault(foc_fault_mask(foc_fault::INTERNAL));
        return foc_result::TOPIC_ERROR;
    }

    return new_fault_flags == 0 ? foc_result::OK :
        foc_result::SENSOR_ERROR;
}

/**
 * @brief 在任务上下文读取最新 FOC 调试快照
 *
 * @param snapshot 用于接收快照的对象
 *
 * @return 已存在快照时返回 true
 */
bool foc::peek_snapshot(foc_snapshot &snapshot)
{
    return snapshot_topic.peek(snapshot);
}

/**
 * @brief 初始化单实例 FOC 核心、Topic 和安全硬件状态
 *
 * @param config FOC 配置
 *
 * @return 初始化结果
 */
foc_result foc::init(const foc_config &config)
{
    if(core_context.initialized){return foc_result::INVALID_STATE;}
    if(!core_context.rotor || !core_context.driver)
    {
        return foc_result::NOT_LINKED;
    }
    if(!config.monitor_only && !core_context.current)
    {
        return foc_result::NOT_LINKED;
    }
    
    // 先校验所有模式共用的时序、方向和超时约束。
    if((config.rotor_direction != 1 && config.rotor_direction != -1) ||
        !isfinite(config.electrical_zero_offset_rad) ||
        !isfinite(config.control_period_s) ||
        config.control_period_s <= 0.0f ||
        config.rotor_extrapolation_limit_us >
            config.rotor_hard_timeout_us ||
        config.rotor_hard_timeout_us == 0 ||
        config.rotor_slow_timeout_us < config.rotor_hard_timeout_us ||
        config.communication_error_limit == 0 ||
        config.telemetry_divider == 0 ||
        config.control_isr_divider == 0)
    {
        return foc_result::INVALID_CONFIG;
    }

    // 监视模式不驱动功率级，因此只在功率输出模式校验功率和 PI 参数。
    if(!config.monitor_only &&
        (config.pole_pairs == 0 ||
        !isfinite(config.bus_voltage_v) || config.bus_voltage_v <= 0.0f ||
        !isfinite(config.voltage_limit_v) ||
            config.voltage_limit_v <= 0.0f ||
        config.voltage_limit_v > config.bus_voltage_v ||
        !isfinite(config.max_phase_current_a) ||
            config.max_phase_current_a <= 0.0f ||
        !finite_non_negative(config.d_axis_pi.proportional_gain) ||
        !finite_non_negative(config.d_axis_pi.integral_gain) ||
        !finite_non_negative(config.d_axis_pi.integral_limit) ||
        !finite_non_negative(config.q_axis_pi.proportional_gain) ||
        !finite_non_negative(config.q_axis_pi.integral_gain) ||
        !finite_non_negative(config.q_axis_pi.integral_limit)))
    {
        return foc_result::INVALID_CONFIG;
    }

    if(!command_topic.init() || !telemetry_topic.init() ||
        !fault_request_topic.init() || !current_sample_topic.init() ||
        !snapshot_topic.init())
    {
        return foc_result::TOPIC_ERROR;
    }

    foc_result driver_result = core_context.driver->init();
    if(driver_result != foc_result::OK){return driver_result;}
    core_context.driver_initialized = true;

    if(core_context.current)
    {
        foc_result current_result = core_context.current->init();
        if(current_result != foc_result::OK){return current_result;}
        core_context.current_initialized = true;
    }

    core_context.config = config;
    core_context.fault_flags = 0;
    core_context.fault_current_valid = false;
    core_context.current_calibration_started = false;
    core_context.current_calibration_done = false;
    core_context.calibration_output_active = false;
    core_context.state = config.monitor_only ? foc_state::MONITORING :
        foc_state::READY;
    core_context.initialized = true;

    foc_command initial_command{};
    initial_command.sequence = ++core_context.command_sequence;
    if(!command_topic.publish(initial_command))
    {
        latch_fault(foc_fault_mask(foc_fault::INTERNAL));
        return foc_result::TOPIC_ERROR;
    }

    fault_request initial_request{};
    initial_request.sequence = ++core_context.fault_request_sequence;
    if(!fault_request_topic.publish(initial_request))
    {
        latch_fault(foc_fault_mask(foc_fault::INTERNAL));
        return foc_result::TOPIC_ERROR;
    }

    foc_snapshot initial_snapshot{};
    initial_snapshot.sequence = ++core_context.snapshot_sequence;
    initial_snapshot.state = core_context.state;
    initial_snapshot.monitor_only = config.monitor_only;
    if(!snapshot_topic.publish(initial_snapshot))
    {
        latch_fault(foc_fault_mask(foc_fault::INTERNAL));
        return foc_result::TOPIC_ERROR;
    }

    return foc_result::OK;
}

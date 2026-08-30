#include "foc.h"
#include "foc_runtime.h"
#include "foc_math.h"
#include "internal/foc_internal_runtime.h"
#include "system/sys_time.h"
#include "system/topic.h"
#include "FreeRTOS.h"
#include <math.h>

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
 * @param faultFlags 本次新增故障位
 */
void foc::instance::latchFault(uint32_t faultFlags)
{
    coreContext.faultFlags |= faultFlags;
    coreContext.commandValid = false;
    coreContext.state = foc_state::FAULT;
    foc_math::reset_pi(coreContext.dAxisPi);
    foc_math::reset_pi(coreContext.qAxisPi);

    if(coreContext.driver)
    {
        coreContext.driver->disable_output();
    }
    coreContext.calibrationOutputActive = false;
}

/**
 * @brief 计算允许样本在任务抢占下略晚于观察时刻的无符号年龄
 *
 * @param timestampUs 当前观察时间戳
 * @param sample_timestamp_us 样本时间戳
 *
 * @return 样本年龄；样本略处于未来时返回零
 */
static uint32_t sample_age_us(uint32_t timestampUs,
    uint32_t sample_timestamp_us)
{
    uint32_t elapsed = timestampUs - sample_timestamp_us;
    return elapsed <= INT32_MAX ? elapsed : 0;
}

/**
 * @brief 按调试分频发布最新相电流样本
 *
 * @param sample 本周期同步相电流样本
 */
void foc::instance::publishCurrentSampleFromIsr(
    const phase_current_sample &sample,
    bool forcePublish)
{
    if(!forcePublish &&
        sample.sequence % coreContext.config.telemetry_divider != 0)
    {
        return;
    }

    BaseType_t higher_priority_task_woken = pdFALSE;
    if(!coreContext.currentSampleTopic.publish_from_isr(sample,
        higher_priority_task_woken))
    {
        latchFault(foc_fault_mask(foc_fault::INTERNAL));
    }
}

/**
 * @brief 发布普通业务目标并由核心生成命令元数据
 *
 * @param target D-Q 控制目标
 *
 * @return 目标发布结果
 */
foc_result foc::instance::set_target(const foc_target &target)
{
    foc_command command{};
    command.target = target;
    command.timestamp_ms = sys_time::get_ms_tick();
    switch(target.mode)
    {
        case foc_control_mode::VOLTAGE:
            command.runtime_mode = foc_runtime_mode::VOLTAGE;
            break;

        case foc_control_mode::CURRENT:
            command.runtime_mode = foc_runtime_mode::CURRENT;
            break;

        default:
            return foc_result::INVALID_ARGUMENT;
    }

    return foc::internal::set_command(*this, command);
}

/**
 * @brief 发布核心内部控制命令
 *
 * @param command 新控制命令
 *
 * @return 命令校验和发布结果
 */
foc_result foc::internal::set_command(foc::instance &motor,
    const foc_command &command)
{
    auto &coreContext = motor.coreContext;
    if(!coreContext.initialized){return foc_result::NOT_INITIALIZED;}

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
    pending_command.sequence = ++coreContext.commandSequence;
    if(!coreContext.commandTopic.publish(pending_command))
    {
        return foc_result::TOPIC_ERROR;
    }

    coreContext.commandValid =
        pending_command.runtime_mode != foc_runtime_mode::DISABLED;

    return foc_result::OK;
}

/**
 * @brief 读取 FOC 配置中的电机极对数
 *
 * @param polePairs 用于接收极对数的对象
 *
 * @return 查询结果
 */
foc_result foc::internal::get_pole_pairs(
    const foc::instance &motor,
    uint8_t &polePairs)
{
    const auto &coreContext = motor.coreContext;
    if(!coreContext.initialized){return foc_result::NOT_INITIALIZED;}
    if(coreContext.config.pole_pairs == 0)
    {
        return foc_result::INVALID_CONFIG;
    }

    polePairs = coreContext.config.pole_pairs;
    return foc_result::OK;
}

/**
 * @brief 在功率输出关闭时更新转子方向与电角度零偏
 *
 * @param rotorDirection 转子传感器方向
 * @param electricalZeroOffsetRad 电角度零偏
 *
 * @return 参数和状态均有效时返回 OK
 */
foc_result foc::internal::set_rotor_alignment(
    foc::instance &motor,
    int8_t rotorDirection,
    float electricalZeroOffsetRad)
{
    auto &coreContext = motor.coreContext;
    if(!coreContext.initialized){return foc_result::NOT_INITIALIZED;}
    if(coreContext.state == foc_state::RUNNING)
    {
        return foc_result::INVALID_STATE;
    }
    if((rotorDirection != 1 && rotorDirection != -1) ||
        !isfinite(electricalZeroOffsetRad))
    {
        return foc_result::INVALID_ARGUMENT;
    }

    coreContext.config.rotor_direction = rotorDirection;
    coreContext.config.electrical_zero_offset_rad =
        foc_math::normalize_angle(electricalZeroOffsetRad);
    foc_math::reset_pi(coreContext.dAxisPi);
    foc_math::reset_pi(coreContext.qAxisPi);
    return foc_result::OK;
}

/**
 * @brief 在三相中性 PWM 条件下推进一次相电流零偏校准
 *
 * @param sampleCount 需要累计的同步 ADC 样本数量
 *
 * @return 校准未完成时返回 CALIBRATING，完成时返回 OK
 */
foc_result foc::internal::calibrate_current_task(
    foc::instance &motor,
    uint32_t sampleCount)
{
    auto &coreContext = motor.coreContext;
    if(!coreContext.initialized){return foc_result::NOT_INITIALIZED;}
    if(!coreContext.current || !coreContext.currentInitialized)
    {
        return foc_result::NOT_LINKED;
    }
    if(coreContext.state == foc_state::RUNNING)
    {
        return foc_result::INVALID_STATE;
    }

    if(!coreContext.currentCalibrationStarted)
    {
        coreContext.driver->disable_output();
        coreContext.calibrationOutputActive = false;
        foc_result calibration_result =
            coreContext.current->calibrate_task(sampleCount);
        if(calibration_result != foc_result::CALIBRATING &&
            calibration_result != foc_result::OK)
        {
            return calibration_result;
        }

        if(!coreContext.config.monitor_only)
        {
            foc_result output_result =
                coreContext.driver->enable_output_task();
            if(output_result != foc_result::OK)
            {
                coreContext.driver->disable_output();
                coreContext.calibrationOutputActive = false;
                return output_result;
            }

            coreContext.calibrationOutputActive = true;
        }

        coreContext.currentCalibrationStarted = true;
        if(calibration_result == foc_result::OK)
        {
            coreContext.currentCalibrationDone = true;
        }

        return calibration_result;
    }

    foc_result calibration_result =
        coreContext.current->calibrate_task(sampleCount);
    if(calibration_result == foc_result::OK)
    {
        coreContext.currentCalibrationDone = true;
    }
    return calibration_result;
}

/**
 * @brief 在任务上下文更新电流传感器的测量方向
 *
 * @param directionA A 相测量方向
 * @param directionB B 相测量方向
 *
 * @return 方向更新结果
 */
foc_result foc::internal::set_current_directions_task(
    foc::instance &motor,
    int8_t directionA,
    int8_t directionB)
{
    auto &coreContext = motor.coreContext;
    if(!coreContext.initialized){return foc_result::NOT_INITIALIZED;}
    if(!coreContext.current || !coreContext.currentInitialized)
    {
        return foc_result::NOT_LINKED;
    }

    return coreContext.current->set_directions_task(
        directionA,
        directionB);
}

/**
 * @brief 尝试进入真实 FOC 运行状态并开启功率输出
 *
 * @return 当前目标新鲜且硬件就绪时返回 OK，否则返回相应错误
 */
foc_result foc::instance::enable()
{
    if(!coreContext.initialized){return foc_result::NOT_INITIALIZED;}
    if(coreContext.config.monitor_only){return foc_result::DISABLED;}
    if(coreContext.faultFlags != 0){return foc_result::INVALID_STATE;}
    if(!coreContext.rotorInitialized ||
        !coreContext.currentInitialized ||
        !coreContext.driverInitialized ||
        !coreContext.currentCalibrationDone)
    {
        return foc_result::NOT_READY;
    }

    foc_command command{};
    if(!coreContext.commandValid ||
        !coreContext.commandTopic.peek(command) ||
        command.runtime_mode == foc_runtime_mode::DISABLED)
    {
        return foc_result::NOT_READY;
    }

    uint32_t now_ms = sys_time::get_ms_tick();
    if(now_ms - command.timestamp_ms >
        coreContext.config.command_timeout_ms)
    {
        return foc_result::NOT_READY;
    }

    foc_result result = coreContext.driver->enable_output_task();
    if(result != foc_result::OK){return result;}

    foc_math::reset_pi(coreContext.dAxisPi);
    foc_math::reset_pi(coreContext.qAxisPi);
    coreContext.calibrationOutputActive = false;
    coreContext.state = foc_state::RUNNING;
    return foc_result::OK;
}

/**
 * @brief 关闭硬件输出、退出运行状态并使当前命令失效
 */
void foc::instance::disable()
{
    if(coreContext.initialized && coreContext.faultFlags == 0)
    {
        coreContext.state = coreContext.config.monitor_only ?
            foc_state::MONITORING : foc_state::READY;
    }
    coreContext.commandValid = false;

    if(coreContext.driver)
    {
        coreContext.driver->disable_output();
    }
    coreContext.calibrationOutputActive = false;

    foc_math::reset_pi(coreContext.dAxisPi);
    foc_math::reset_pi(coreContext.qAxisPi);
}

/**
 * @brief 在输出关闭且硬件无 Fault 时清除锁存的软件故障
 *
 * @return 故障清除结果
 */
foc_result foc::instance::clear_fault()
{
    if(!coreContext.initialized){return foc_result::NOT_INITIALIZED;}
    coreContext.commandValid = false;
    coreContext.driver->disable_output();
    coreContext.calibrationOutputActive = false;
    if(coreContext.driver->fault_active_from_isr())
    {
        return foc_result::DRIVER_FAULT;
    }

    coreContext.faultFlags = 0;
    coreContext.faultCurrentValid = false;
    coreContext.consecutiveBusErrorCount = 0;
    foc_fault_request empty_request{};
    empty_request.sequence = ++coreContext.faultRequestSequence;
    if(!coreContext.faultRequestTopic.publish(empty_request))
    {
        return foc_result::TOPIC_ERROR;
    }

    coreContext.state = coreContext.config.monitor_only ?
        foc_state::MONITORING : foc_state::READY;
    return foc_result::OK;
}

/**
 * @brief 执行一次 ADC 同步的完整 FOC 控制周期
 *
 * @param timestampUs 本控制周期的微秒时间戳
 *
 * @return 本周期执行结果
 */
foc_result foc::runtime::run_control_from_isr(
    foc::instance &motor,
    uint32_t timestampUs)
{
    auto &coreContext = motor.coreContext;
    if(!coreContext.initialized){return foc_result::NOT_INITIALIZED;}

    phase_current_sample current{};
    foc_result current_result = foc_result::NOT_LINKED;
    if(coreContext.current && coreContext.currentInitialized)
    {
        current_result =
            coreContext.current->read_conversion_from_isr(timestampUs,
                current);
        if(current_result == foc_result::OK && current.valid &&
            (coreContext.state == foc_state::RUNNING ||
                coreContext.calibrationOutputActive))
        {
            motor.publishCurrentSampleFromIsr(current);
        }
    }

    if(coreContext.state != foc_state::RUNNING)
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

    foc_fault_request requested_fault{};
    if(coreContext.faultRequestTopic.peek_from_isr(requested_fault) &&
        requested_fault.fault_flags != 0)
    {
        motor.latchFault(requested_fault.fault_flags);
        return foc_result::OUTPUT_FAULT;
    }

    if(coreContext.driver->fault_active_from_isr())
    {
        motor.latchFault(foc_fault_mask(foc_fault::DRIVER));
        return foc_result::DRIVER_FAULT;
    }

    if(current_result != foc_result::OK || !current.valid)
    {
        motor.latchFault(foc_fault_mask(foc_fault::CURRENT_NOT_READY));
        return current_result;
    }

    if(!isfinite(current.current_a) || !isfinite(current.current_b) ||
        !isfinite(current.current_c))
    {
        motor.latchFault(foc_fault_mask(foc_fault::INVALID_NUMBER));
        return foc_result::SENSOR_ERROR;
    }

    // 过流判定放在坐标变换前，并保留故障瞬间的原始样本用于安全快照。
    float current_limit = coreContext.config.max_phase_current_a;
    if(fabsf(current.current_a) > current_limit ||
        fabsf(current.current_b) > current_limit ||
        fabsf(current.current_c) > current_limit)
    {
        coreContext.faultCurrent = current;
        coreContext.faultCurrentValid = true;
        motor.publishCurrentSampleFromIsr(current, true);
        motor.latchFault(foc_fault_mask(foc_fault::OVER_CURRENT));
        return foc_result::SENSOR_ERROR;
    }

    coreContext.controlIsrPrescaler++;
    if(coreContext.controlIsrPrescaler %
        coreContext.config.control_isr_divider != 0)
    {
        return foc_result::OK;
    }

    rotor_sample rotor{};
    foc_result rotor_result = coreContext.rotor->read_from_isr(rotor);
    if(rotor_result != foc_result::OK || !rotor.valid)
    {
        motor.latchFault(foc_fault_mask(foc_fault::ROTOR_NOT_READY));
        return rotor_result;
    }

    uint32_t rotor_age_us = sample_age_us(timestampUs,
        rotor.timestamp_us);
    if(rotor_age_us > coreContext.config.rotor_hard_timeout_us)
    {
        motor.latchFault(foc_fault_mask(foc_fault::ROTOR_STALE));
        return foc_result::SAMPLE_STALE;
    }

    foc_command command{};
    if(!coreContext.commandValid ||
        !coreContext.commandTopic.peek_from_isr(command) ||
        command.runtime_mode == foc_runtime_mode::DISABLED)
    {
        motor.latchFault(foc_fault_mask(foc_fault::COMMAND_TIMEOUT));
        return foc_result::NOT_READY;
    }

    const foc_target &target = command.target;

    float mechanical_angle = rotor.mechanical_angle_rad;
    if(rotor_age_us <= coreContext.config.rotor_extrapolation_limit_us)
    {
        mechanical_angle += rotor.mechanical_velocity_rad_s *
            (float)rotor_age_us * 1.0e-6f;
    }

    float electrical_angle = 0.0f;
    if(command.runtime_mode == foc_runtime_mode::OPEN_LOOP_VOLTAGE)
    {
        uint32_t target_age_us = timestampUs -
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
            (float)coreContext.config.rotor_direction *
            (float)coreContext.config.pole_pairs +
            coreContext.config.electrical_zero_offset_rad);
    }
    alpha_beta_value stationary_current =
        foc_math::clarke_transform(current);
    d_q_value rotating_current = foc_math::park_transform(
        stationary_current,
        electrical_angle);
    d_q_value rotating_voltage{};

    if(command.runtime_mode == foc_runtime_mode::OPEN_LOOP_VOLTAGE ||
        command.runtime_mode == foc_runtime_mode::VOLTAGE)
    {
        rotating_voltage.d = target.d_axis_voltage_v;
        rotating_voltage.q = target.q_axis_voltage_v;
    }
    else if(command.runtime_mode == foc_runtime_mode::CURRENT)
    {
        rotating_voltage.d = foc_math::run_pi(
            target.d_axis_current_a - rotating_current.d,
            coreContext.config.control_period_s,
            coreContext.config.d_axis_pi,
            coreContext.config.voltage_limit_v,
            coreContext.dAxisPi);
        rotating_voltage.q = foc_math::run_pi(
            target.q_axis_current_a - rotating_current.q,
            coreContext.config.control_period_s,
            coreContext.config.q_axis_pi,
            coreContext.config.voltage_limit_v,
            coreContext.qAxisPi);
    }
    else
    {
        motor.latchFault(foc_fault_mask(foc_fault::COMMAND_TIMEOUT));
        return foc_result::INVALID_STATE;
    }

    foc_math::limit_vector(rotating_voltage,
        coreContext.config.voltage_limit_v);
    alpha_beta_value stationary_voltage =
        foc_math::inverse_park_transform(rotating_voltage,
            electrical_angle);
    phase_duty duty{};
    if(!foc_math::calculate_svpwm(stationary_voltage,
        coreContext.config.bus_voltage_v,
        duty))
    {
        motor.latchFault(foc_fault_mask(foc_fault::OUTPUT_RANGE));
        return foc_result::OUTPUT_FAULT;
    }

    foc_result output_result =
        coreContext.driver->write_duty_from_isr(duty);
    if(output_result != foc_result::OK)
    {
        motor.latchFault(foc_fault_mask(foc_fault::OUTPUT_RANGE));
        return output_result;
    }

    coreContext.controlSequence++;
    foc_control_telemetry telemetry{};
    telemetry.sequence = coreContext.controlSequence;
    telemetry.timestamp_us = timestampUs;
    telemetry.rotor = rotor;
    telemetry.current = current;
    telemetry.rotor_sample_age_ms = (float)rotor_age_us * 0.001f;
    telemetry.d_axis_current_a = rotating_current.d;
    telemetry.q_axis_current_a = rotating_current.q;
    telemetry.d_axis_voltage_v = rotating_voltage.d;
    telemetry.q_axis_voltage_v = rotating_voltage.q;
    telemetry.duty = duty;

    // 遥测只按配置分频发布，避免每个控制周期都向 Topic 发送调试数据。
    if(coreContext.controlSequence %
        coreContext.config.telemetry_divider == 0)
    {
        BaseType_t higher_priority_task_woken = pdFALSE;
        if(!coreContext.telemetryTopic.publish_from_isr(telemetry,
            higher_priority_task_woken))
        {
            motor.latchFault(foc_fault_mask(foc_fault::INTERNAL));
        }
    }
    return foc_result::OK;
}

/**
 * @brief 在高频 FreeRTOS 任务中推进一次传感器采集
 *
 * @return 本次初始化或采样结果
 */
foc_result foc::runtime::update_sensors(foc::instance &motor)
{
    auto &coreContext = motor.coreContext;
    if(!coreContext.initialized){return foc_result::NOT_INITIALIZED;}

    foc_result result;
    if(!coreContext.rotorInitialized)
    {
        result = coreContext.rotor->init();
        coreContext.rotorInitialized = result == foc_result::OK;
    }
    else
    {
        result = coreContext.rotor->update_task();
    }

    if(result == foc_result::OK)
    {
        coreContext.consecutiveBusErrorCount = 0;
    }
    else
    {
        coreContext.busUpdateErrorCount++;
        coreContext.consecutiveBusErrorCount++;
    }

    return result;
}

/**
 * @brief 在低频 FreeRTOS 任务中检查慢速故障并发布调试快照
 *
 * @param timestampMs 当前毫秒时间戳
 *
 * @return 本次安全检查结果
 */
foc_result foc::runtime::update_safety(
    foc::instance &motor,
    uint32_t timestampMs)
{
    auto &coreContext = motor.coreContext;
    if(!coreContext.initialized){return foc_result::NOT_INITIALIZED;}

    rotor_sample rotor{};
    foc_result rotor_result = coreContext.rotorInitialized ?
        coreContext.rotor->read_task(rotor) :
        foc_result::SAMPLE_NOT_READY;
    uint32_t now_us = timestampMs * 1000;
    uint32_t rotor_age_us = rotor_result == foc_result::OK ?
        sample_age_us(now_us, rotor.timestamp_us) : now_us;
    uint32_t new_fault_flags = 0;
    foc_command command{};
    bool command_available = coreContext.commandValid &&
        coreContext.commandTopic.peek(command) &&
        command.runtime_mode != foc_runtime_mode::DISABLED;
    if(!command_available)
    {
        command = {};
    }
    const foc_target &target = command.target;

    if(coreContext.consecutiveBusErrorCount >=
        coreContext.config.communication_error_limit)
    {
        new_fault_flags |= foc_fault_mask(foc_fault::ROTOR_COMMUNICATION);
    }

    if(rotor_result != foc_result::OK &&
        now_us > coreContext.config.rotor_slow_timeout_us)
    {
        new_fault_flags |= foc_fault_mask(foc_fault::ROTOR_NOT_READY);
    }
    else if(rotor_result == foc_result::OK &&
        rotor_age_us > coreContext.config.rotor_slow_timeout_us)
    {
        new_fault_flags |= foc_fault_mask(foc_fault::ROTOR_STALE);
    }

    if(!coreContext.config.monitor_only &&
        coreContext.state == foc_state::RUNNING &&
        (!command_available ||
            timestampMs - command.timestamp_ms >
                coreContext.config.command_timeout_ms))
    {
        new_fault_flags |= foc_fault_mask(foc_fault::COMMAND_TIMEOUT);
    }

    if(new_fault_flags != 0)
    {
        foc_fault_request request{};
        request.sequence = ++coreContext.faultRequestSequence;
        request.fault_flags = new_fault_flags;
        if(!coreContext.faultRequestTopic.publish(request))
        {
            motor.latchFault(foc_fault_mask(foc_fault::INTERNAL));
            return foc_result::TOPIC_ERROR;
        }

        motor.latchFault(new_fault_flags);
    }

    foc_control_telemetry telemetry{};
    coreContext.telemetryTopic.peek(telemetry);

    foc_snapshot snapshot{};
    snapshot.sequence = ++coreContext.snapshotSequence;
    snapshot.timestamp_us = now_us;
    snapshot.state = coreContext.state;
    snapshot.fault_flags = coreContext.faultFlags;
    snapshot.monitor_only = coreContext.config.monitor_only;
    snapshot.output_active = coreContext.state == foc_state::RUNNING ||
        coreContext.calibrationOutputActive;
    snapshot.rotor = rotor;
    if(coreContext.faultCurrentValid)
    {
        snapshot.current = coreContext.faultCurrent;
    }
    else if(!coreContext.currentSampleTopic.peek(snapshot.current))
    {
        snapshot.current = telemetry.current;
    }
    if(!snapshot.output_active && !coreContext.faultCurrentValid)
    {
        snapshot.current.valid = false;
    }
    snapshot.target = target;
    snapshot.runtime_mode = command.runtime_mode;
    snapshot.command_electrical_angle_rad =
        command.electrical_angle_rad;
    snapshot.command_electrical_velocity_rad_s =
        command.electrical_velocity_rad_s;
    snapshot.rotor_sample_age_ms = rotor_result == foc_result::OK ?
        (float)rotor_age_us * 0.001f : -1.0f;
    snapshot.d_axis_current_a = telemetry.d_axis_current_a;
    snapshot.q_axis_current_a = telemetry.q_axis_current_a;
    snapshot.d_axis_voltage_v = telemetry.d_axis_voltage_v;
    snapshot.q_axis_voltage_v = telemetry.q_axis_voltage_v;
    snapshot.duty = telemetry.duty;
    snapshot.control_sequence = coreContext.controlSequence;
    snapshot.bus_update_error_count =
        coreContext.busUpdateErrorCount;
    snapshot.consecutive_bus_error_count =
        coreContext.consecutiveBusErrorCount;

    if(!coreContext.snapshotTopic.publish(snapshot))
    {
        motor.latchFault(foc_fault_mask(foc_fault::INTERNAL));
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
bool foc::instance::peek_snapshot(foc_snapshot &snapshot)
{
    return coreContext.snapshotTopic.peek(snapshot);
}

/**
 * @brief 初始化单实例 FOC 核心、Topic 和安全硬件状态
 *
 * @param config FOC 配置
 * @param hardware FOC 所需硬件抽象
 *
 * @return 初始化结果
 */
foc_result foc::instance::init(const foc_config &config,
    const foc_hardware &hardware)
{
    if(coreContext.initialized){return foc_result::INVALID_STATE;}
    rotor_sensor *rotor = hardware.rotor;
    current_sensor *current = hardware.current;
    phase_driver *driver = hardware.driver;
    if(!rotor || !driver)
    {
        return foc_result::NOT_LINKED;
    }
    if(!config.monitor_only && !current)
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

    if(!coreContext.commandTopic.init() || !coreContext.telemetryTopic.init() ||
        !coreContext.faultRequestTopic.init() || !coreContext.currentSampleTopic.init() ||
        !coreContext.snapshotTopic.init())
    {
        return foc_result::TOPIC_ERROR;
    }

    foc_result driver_result = driver->init();
    if(driver_result != foc_result::OK){return driver_result;}

    if(current)
    {
        foc_result current_result = current->init();
        if(current_result != foc_result::OK){return current_result;}
    }

    // 所有硬件初始化成功后再提交指针和状态，避免留下部分绑定的 context。
    coreContext.rotor = rotor;
    coreContext.current = current;
    coreContext.driver = driver;
    coreContext.config = config;
    coreContext.rotorInitialized = false;
    coreContext.currentInitialized = current != nullptr;
    coreContext.driverInitialized = true;
    coreContext.faultFlags = 0;
    coreContext.faultCurrentValid = false;
    coreContext.currentCalibrationStarted = false;
    coreContext.currentCalibrationDone = false;
    coreContext.commandValid = false;
    coreContext.calibrationOutputActive = false;
    coreContext.state = config.monitor_only ? foc_state::MONITORING :
        foc_state::READY;
    coreContext.initialized = true;

    foc_command initial_command{};
    initial_command.sequence = ++coreContext.commandSequence;
    if(!coreContext.commandTopic.publish(initial_command))
    {
        latchFault(foc_fault_mask(foc_fault::INTERNAL));
        return foc_result::TOPIC_ERROR;
    }

    foc_fault_request initial_request{};
    initial_request.sequence = ++coreContext.faultRequestSequence;
    if(!coreContext.faultRequestTopic.publish(initial_request))
    {
        latchFault(foc_fault_mask(foc_fault::INTERNAL));
        return foc_result::TOPIC_ERROR;
    }

    foc_snapshot initial_snapshot{};
    initial_snapshot.sequence = ++coreContext.snapshotSequence;
    initial_snapshot.state = coreContext.state;
    initial_snapshot.monitor_only = config.monitor_only;
    if(!coreContext.snapshotTopic.publish(initial_snapshot))
    {
        latchFault(foc_fault_mask(foc_fault::INTERNAL));
        return foc_result::TOPIC_ERROR;
    }

    return foc_result::OK;
}

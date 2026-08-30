#include "sguan_foc_wrapper.h"

#include "main.h"
#include "tim.h"
#include "system/sys_time.h"
#include <math.h>
extern "C"
{
#include "../SguanFOC.h"
}

static constexpr uint32_t CURRENT_CALIBRATION_SAMPLE_COUNT = 1000;
static constexpr uint32_t SNAPSHOT_DIVIDER = 20;
static constexpr uint32_t ROTOR_EXTRAPOLATION_LIMIT_US = 1000;
static constexpr uint32_t ROTOR_HARD_TIMEOUT_US = 5000;
static constexpr float CONTROL_PERIOD_TOLERANCE_S = 0.0000005f;

sguan_foc_wrapper *sguan_foc_wrapper::active_instance_ = nullptr;

/**
 * @brief 在初始化前绑定非拥有式转子传感器
 *
 * @param sensor 由板级代码长期持有的转子传感器
 *
 * @return 绑定成功时返回 OK
 */
foc_result sguan_foc_wrapper::link_sensor(rotor_sensor &sensor)
{
    if(initialized_)
    {
        return foc_result::INVALID_STATE;
    }

    sensor_ = &sensor;
    return foc_result::OK;
}

/**
 * @brief 在初始化前绑定非拥有式相电流传感器
 *
 * @param current_sense 由板级代码长期持有的相电流传感器
 *
 * @return 绑定成功时返回 OK
 */
foc_result sguan_foc_wrapper::link_current_sense(
    current_sensor &current_sense)
{
    if(initialized_)
    {
        return foc_result::INVALID_STATE;
    }

    current_sense_ = &current_sense;
    return foc_result::OK;
}

/**
 * @brief 校验配置、依赖和 TIM8 周期后占用 SguanFOC backend
 *
 * @param config 电机、PI、保护和实时周期配置
 *
 * @return 初始化结果
 */
foc_result sguan_foc_wrapper::init(const sguan_foc_config &config)
{
    if(!isfinite(config.motor.resistance_ohm) ||
        !isfinite(config.motor.ld_h) ||
        !isfinite(config.motor.lq_h) ||
        !isfinite(config.motor.ls_h) ||
        !isfinite(config.motor.flux_wb) ||
        !isfinite(config.current_pi.kp) ||
        !isfinite(config.current_pi.ki) ||
        !isfinite(config.current_pi.output_limit_v) ||
        !isfinite(config.current_pi.integral_limit_v) ||
        !isfinite(config.limits.max_id_a) ||
        !isfinite(config.limits.max_iq_a) ||
        !isfinite(config.limits.min_bus_voltage_v) ||
        !isfinite(config.limits.max_bus_voltage_v) ||
        !isfinite(config.nominal_bus_voltage_v) ||
        !isfinite(config.control_period_s) ||
        config.motor.pole_pairs == 0 ||
        config.motor.resistance_ohm <= 0.0f ||
        config.motor.ld_h <= 0.0f ||
        config.motor.lq_h <= 0.0f ||
        config.motor.ls_h <= 0.0f ||
        config.motor.flux_wb < 0.0f ||
        (config.motor.motor_direction != 1 &&
            config.motor.motor_direction != -1) ||
        (config.motor.encoder_direction != 1 &&
            config.motor.encoder_direction != -1) ||
        (config.motor.pwm_direction != 1 &&
            config.motor.pwm_direction != -1) ||
        config.current_pi.kp < 0.0f ||
        config.current_pi.ki < 0.0f ||
        config.current_pi.output_limit_v <= 0.0f ||
        config.current_pi.integral_limit_v <= 0.0f ||
        config.limits.max_id_a <= 0.0f ||
        config.limits.max_iq_a <= 0.0f ||
        config.limits.min_bus_voltage_v >=
            config.limits.max_bus_voltage_v ||
        config.nominal_bus_voltage_v <= 0.0f ||
        config.nominal_bus_voltage_v < config.limits.min_bus_voltage_v ||
        config.nominal_bus_voltage_v > config.limits.max_bus_voltage_v ||
        config.pwm_period == 0 ||
        config.control_period_s <= 0.0f)
    {
        last_result_ = foc_result::INVALID_CONFIG;
        return last_result_;
    }

    if(sensor_ == nullptr || current_sense_ == nullptr)
    {
        last_result_ = foc_result::NOT_LINKED;
        return last_result_;
    }

    if(active_instance_ != nullptr && active_instance_ != this)
    {
        last_result_ = foc_result::INVALID_STATE;
        return last_result_;
    }

    if(initialized_)
    {
        last_result_ = foc_result::OK;
        return last_result_;
    }

    RCC_ClkInitTypeDef clock_config{};
    uint32_t flash_latency = 0;
    HAL_RCC_GetClockConfig(&clock_config, &flash_latency);
    uint32_t timer_clock_hz = HAL_RCC_GetPCLK2Freq();
    if(clock_config.APB2CLKDivider != RCC_HCLK_DIV1)
    {
        timer_clock_hz *= 2;
    }

    float timer_control_period_s = 0.0f;
    if(timer_clock_hz != 0)
    {
        timer_control_period_s =
            2.0f * (float)(htim8.Init.Period + 1) *
            (float)(htim8.Init.Prescaler + 1) /
            (float)timer_clock_hz;
    }

    // TIM8 CH4 的每个中心对齐周期触发一次 ADC2 注入采样，当前应为 20 kHz。
    if(htim8.Instance != TIM8 ||
        htim8.Init.CounterMode != TIM_COUNTERMODE_CENTERALIGNED1 ||
        (uint32_t)(htim8.Init.Period + 1) != config.pwm_period ||
        timer_clock_hz == 0 ||
        fabsf(timer_control_period_s - config.control_period_s) >
            CONTROL_PERIOD_TOLERANCE_S)
    {
        last_result_ = foc_result::INVALID_CONFIG;
        return last_result_;
    }

    foc_result result = sensor_->init();
    if(result != foc_result::OK)
    {
        last_result_ = result;
        return result;
    }

    result = current_sense_->init();
    if(result != foc_result::OK)
    {
        last_result_ = result;
        return result;
    }

    config_ = config;
    active_instance_ = this;
    initialized_ = true;
    output_enabled_ = false;
    state_ = sguan_foc_state::UNINITIALIZED;
    current_calibration_done_ = false;
    sensor_fault_ = false;
    current_fault_ = false;
    last_rotor_sample_valid_ = false;
    snapshot_guard_sequence_ = 0;
    snapshot_publish_sequence_ = 0;
    snapshot_divider_ = 0;
    last_result_ = foc_result::OK;

    apply_config_to_backend();
    Sguan.status = MOTOR_STATUS_UNINITIALIZED;
    return foc_result::OK;
}

/**
 * @brief 在任务上下文推进 CurrentSense 校准和 SguanFOC 初始化
 *
 * @return 当前初始化阶段结果
 */
foc_result sguan_foc_wrapper::init_foc()
{
    if(!initialized_)
    {
        return foc_result::NOT_INITIALIZED;
    }

    if(sensor_ == nullptr || current_sense_ == nullptr)
    {
        last_result_ = foc_result::NOT_LINKED;
        state_ = sguan_foc_state::FAULT;
        return last_result_;
    }

    if(state_ == sguan_foc_state::READY)
    {
        return foc_result::OK;
    }

    if(state_ == sguan_foc_state::FAULT)
    {
        return last_result_;
    }

    if(state_ == sguan_foc_state::UNINITIALIZED)
    {
        foc_result result = current_sense_->calibrate_task(
            CURRENT_CALIBRATION_SAMPLE_COUNT);
        if(result == foc_result::CALIBRATING)
        {
            // 先让 CurrentSense 进入 calibrating，再开放 ISR 校准分支，
            // 避免 ADC 中断在两次状态写入之间看到 NOT_READY。
            state_ = sguan_foc_state::CALIBRATING;
            last_result_ = result;
            return result;
        }

        if(result != foc_result::OK)
        {
            current_fault_ = true;
            output_enabled_ = false;
            state_ = sguan_foc_state::FAULT;
            last_result_ = result;
            return result;
        }

        current_calibration_done_ = true;
        state_ = sguan_foc_state::INITIALIZING;
        Sguan.status = MOTOR_STATUS_UNINITIALIZED;
    }

    if(state_ == sguan_foc_state::CALIBRATING)
    {
        foc_result result = current_sense_->calibrate_task(
            CURRENT_CALIBRATION_SAMPLE_COUNT);
        if(result == foc_result::CALIBRATING)
        {
            last_result_ = result;
            return result;
        }

        if(result != foc_result::OK)
        {
            current_fault_ = true;
            output_enabled_ = false;
            state_ = sguan_foc_state::FAULT;
            last_result_ = result;
            return result;
        }

        current_calibration_done_ = true;
        state_ = sguan_foc_state::INITIALIZING;
        Sguan.status = MOTOR_STATUS_UNINITIALIZED;
    }

    if(state_ == sguan_foc_state::INITIALIZING)
    {
        SguanFOC_main_Loop();
        if(Sguan.status >= MOTOR_STATUS_IDLE &&
            Sguan.status < MOTOR_STATUS_OVERVOLTAGE)
        {
            state_ = sguan_foc_state::READY;
            last_result_ = foc_result::OK;
            return foc_result::OK;
        }

        if(backend_faulted())
        {
            output_enabled_ = false;
            state_ = sguan_foc_state::FAULT;
            last_result_ = foc_result::DRIVER_FAULT;
            return last_result_;
        }

        last_result_ = foc_result::NOT_READY;
        return last_result_;
    }

    return last_result_;
}

/**
 * @brief 原子地切换上层运动控制模式并清零全部模式目标
 *
 * @param controller 目标运动控制模式
 */
void sguan_foc_wrapper::set_controller(
    motion_control_type controller)
{
    if(!initialized_)
    {
        return;
    }

    switch(controller)
    {
        case motion_control_type::TORQUE:
        case motion_control_type::VELOCITY:
        case motion_control_type::ANGLE:
            break;
        default:
            controller = motion_control_type::TORQUE;
            break;
    }

    uint32_t interrupt_state = __get_PRIMASK();
    __disable_irq();
    command_.controller = controller;
    command_.target_id_a = 0.0f;
    command_.target_iq_a = 0.0f;
    command_.target_velocity_rad_s = 0.0f;
    command_.target_position_rad = 0.0;
    __DMB();
    __set_PRIMASK(interrupt_state);
}

/**
 * @brief 原子地提交当前模式下的单一业务目标
 *
 * @param target 力矩电流、机械角速度或机械位置目标
 */
void sguan_foc_wrapper::move(float target)
{
    if(!initialized_ || !isfinite(target))
    {
        return;
    }

    uint32_t interrupt_state = __get_PRIMASK();
    __disable_irq();
    motion_control_type controller = command_.controller;
    command_.target_id_a = 0.0f;
    command_.target_iq_a = 0.0f;
    command_.target_velocity_rad_s = 0.0f;
    command_.target_position_rad = 0.0;

    switch(controller)
    {
        case motion_control_type::TORQUE:
            command_.target_iq_a = target;
            if(command_.target_iq_a > config_.limits.max_iq_a)
            {
                command_.target_iq_a = config_.limits.max_iq_a;
            }
            if(command_.target_iq_a < -config_.limits.max_iq_a)
            {
                command_.target_iq_a = -config_.limits.max_iq_a;
            }
            break;

        case motion_control_type::VELOCITY:
            command_.target_velocity_rad_s = target;
            break;

        case motion_control_type::ANGLE:
            command_.target_position_rad = (double)target;
            break;

        default:
            command_.controller = motion_control_type::TORQUE;
            command_.target_iq_a = target;
            if(command_.target_iq_a > config_.limits.max_iq_a)
            {
                command_.target_iq_a = config_.limits.max_iq_a;
            }
            if(command_.target_iq_a < -config_.limits.max_iq_a)
            {
                command_.target_iq_a = -config_.limits.max_iq_a;
            }
            break;
    }

    __DMB();
    __set_PRIMASK(interrupt_state);
}

/**
 * @brief 原子地提交 D/Q 电流目标并切换到力矩模式
 *
 * @param id_a D轴目标电流，单位安培
 * @param iq_a Q轴目标电流，单位安培
 */
void sguan_foc_wrapper::set_current_dq(float id_a, float iq_a)
{
    if(!initialized_ || !isfinite(id_a) || !isfinite(iq_a))
    {
        return;
    }

    if(id_a > config_.limits.max_id_a)
    {
        id_a = config_.limits.max_id_a;
    }
    if(id_a < -config_.limits.max_id_a)
    {
        id_a = -config_.limits.max_id_a;
    }
    if(iq_a > config_.limits.max_iq_a)
    {
        iq_a = config_.limits.max_iq_a;
    }
    if(iq_a < -config_.limits.max_iq_a)
    {
        iq_a = -config_.limits.max_iq_a;
    }

    uint32_t interrupt_state = __get_PRIMASK();
    __disable_irq();
    command_.controller = motion_control_type::TORQUE;
    command_.target_id_a = id_a;
    command_.target_iq_a = iq_a;
    command_.target_velocity_rad_s = 0.0f;
    command_.target_position_rad = 0.0;
    __DMB();
    __set_PRIMASK(interrupt_state);
}

/**
 * @brief 允许已完成 FOC 初始化的 wrapper 输出业务目标
 */
void sguan_foc_wrapper::enable()
{
    if(!initialized_ || !ready())
    {
        return;
    }

    uint32_t interrupt_state = __get_PRIMASK();
    __disable_irq();
    output_enabled_ = true;
    __DMB();
    __set_PRIMASK(interrupt_state);
}

/**
 * @brief 禁止业务目标输出并保持 backend 的校准状态
 *
 * @note 这里只提交零目标，不写 MOTOR_STATUS_DISABLED，也不关闭 MOE，
 *       因而不会触发第三方状态机清空编码器和电流校准数据。
 */
void sguan_foc_wrapper::disable()
{
    if(!initialized_)
    {
        return;
    }

    uint32_t interrupt_state = __get_PRIMASK();
    __disable_irq();
    command_.target_id_a = 0.0f;
    command_.target_iq_a = 0.0f;
    command_.target_velocity_rad_s = 0.0f;
    command_.target_position_rad = 0.0;
    output_enabled_ = false;
    __DMB();
    __set_PRIMASK(interrupt_state);
}

/**
 * @brief 查询 wrapper 是否已经占用 backend
 *
 * @return 已完成依赖初始化和 backend 占用时返回 true
 */
bool sguan_foc_wrapper::initialized() const
{
    return initialized_;
}

/**
 * @brief 查询 FOC 校准和 backend 初始化是否完成
 *
 * @return 已进入可控制状态且没有当前故障时返回 true
 */
bool sguan_foc_wrapper::ready() const
{
    if(!initialized_ || state_ != sguan_foc_state::READY ||
        faulted())
    {
        return false;
    }

    return Sguan.status >= MOTOR_STATUS_IDLE &&
        Sguan.status < MOTOR_STATUS_OVERVOLTAGE;
}

/**
 * @brief 查询业务目标当前是否允许输出
 *
 * @return 已 ready 且未被 wrapper 禁止输出时返回 true
 */
bool sguan_foc_wrapper::enabled() const
{
    return output_enabled_ && ready();
}

/**
 * @brief 查询 wrapper 或第三方 backend 是否存在运行故障
 *
 * @return 存在传感器、采样或 backend 故障时返回 true
 */
bool sguan_foc_wrapper::faulted() const
{
    return state_ == sguan_foc_state::FAULT || sensor_fault_ ||
        current_fault_ || backend_faulted();
}

/**
 * @brief 获取当前业务控制器模式
 *
 * @return 当前 motion control 模式
 */
motion_control_type sguan_foc_wrapper::controller() const
{
    return command_.controller;
}

/**
 * @brief 读取高速环发布的一致运行快照
 *
 * @return 通过 sequence 校验的一致快照
 */
sguan_foc_snapshot sguan_foc_wrapper::snapshot() const
{
    sguan_foc_snapshot result{};

    while(true)
    {
        uint32_t sequence_begin = snapshot_guard_sequence_;
        if(sequence_begin & 1)
        {
            continue;
        }

        __DMB();
        result.sequence = snapshot_cache_.sequence;
        result.initialized = snapshot_cache_.initialized;
        result.ready = snapshot_cache_.ready;
        result.enabled = snapshot_cache_.enabled;
        result.faulted = snapshot_cache_.faulted;
        result.state = snapshot_cache_.state;
        result.controller = snapshot_cache_.controller;
        result.target = snapshot_cache_.target;
        result.angle_rad = snapshot_cache_.angle_rad;
        result.full_angle_rad = snapshot_cache_.full_angle_rad;
        result.velocity_rad_s = snapshot_cache_.velocity_rad_s;
        result.electrical_angle_rad =
            snapshot_cache_.electrical_angle_rad;
        result.electrical_velocity_rad_s =
            snapshot_cache_.electrical_velocity_rad_s;
        result.ia_a = snapshot_cache_.ia_a;
        result.ib_a = snapshot_cache_.ib_a;
        result.ic_a = snapshot_cache_.ic_a;
        result.id_a = snapshot_cache_.id_a;
        result.iq_a = snapshot_cache_.iq_a;
        result.target_id_a = snapshot_cache_.target_id_a;
        result.target_iq_a = snapshot_cache_.target_iq_a;
        result.ud_v = snapshot_cache_.ud_v;
        result.uq_v = snapshot_cache_.uq_v;
        result.bus_voltage_v = snapshot_cache_.bus_voltage_v;
        result.duty_u = snapshot_cache_.duty_u;
        result.duty_v = snapshot_cache_.duty_v;
        result.duty_w = snapshot_cache_.duty_w;
        result.raw_status = snapshot_cache_.raw_status;
        __DMB();

        uint32_t sequence_end = snapshot_guard_sequence_;
        if(sequence_begin == sequence_end)
        {
            return result;
        }
    }
}

/**
 * @brief 返回当前工程已验证的 SguanFOC 默认配置
 *
 * @return 与当前实机电流环参数一致的默认配置
 */
sguan_foc_config sguan_foc_wrapper::default_config()
{
    sguan_foc_config config{};

    config.motor.pole_pairs = 7;
    config.motor.resistance_ohm = 2.55f;
    config.motor.ld_h = 0.00086f;
    config.motor.lq_h = 0.00086f;
    config.motor.ls_h = 0.00086f;
    config.motor.flux_wb = 0.0035f;
    config.motor.motor_direction = 1;
    config.motor.encoder_direction = -1;
    config.motor.pwm_direction = -1;

    config.current_pi.kp = 0.2995f;
    config.current_pi.ki = 300.0f;
    config.current_pi.output_limit_v = 1.5f;
    config.current_pi.integral_limit_v = 0.3f;

    config.limits.max_id_a = 0.5f;
    config.limits.max_iq_a = 0.5f;
    config.limits.min_bus_voltage_v = 10.0f;
    config.limits.max_bus_voltage_v = 14.0f;

    config.nominal_bus_voltage_v = 12.0f;
    config.pwm_period = 4200;
    config.control_period_s = 0.00005f;

    return config;
}

/**
 * @brief 在 ADC 注入转换中断中推进一次 FOC 高速环
 */
void sguan_foc_wrapper::loop_foc()
{
    if(!initialized_)
    {
        return;
    }

    if(state_ == sguan_foc_state::CALIBRATING)
    {
        if(current_sense_ == nullptr)
        {
            current_fault_ = true;
            output_enabled_ = false;
            state_ = sguan_foc_state::FAULT;
            last_result_ = foc_result::NOT_LINKED;
            return;
        }

        phase_current_sample sample{};
        foc_result result = current_sense_->read_conversion_from_isr(
            sys_time::get_us_tick(),
            sample);
        if(result != foc_result::CALIBRATING &&
            result != foc_result::OK)
        {
            current_fault_ = true;
            output_enabled_ = false;
            state_ = sguan_foc_state::FAULT;
            last_result_ = result;
        }
        return;
    }

    if(state_ != sguan_foc_state::READY &&
        state_ != sguan_foc_state::FAULT)
    {
        return;
    }

    SguanFOC_High_Loop();
    snapshot_divider_++;
    if(snapshot_divider_ >= SNAPSHOT_DIVIDER)
    {
        snapshot_divider_ = 0;
        publish_snapshot();
    }
}

/**
 * @brief 在板级低频任务中推进初始化、主循环和保护状态机
 */
void sguan_foc_wrapper::service()
{
    if(!initialized_)
    {
        return;
    }

    if(state_ != sguan_foc_state::READY)
    {
        if(state_ != sguan_foc_state::FAULT)
        {
            init_foc();
        }
        return;
    }

    SguanFOC_main_Loop();
    SguanFOC_Low_Loop();
    if(backend_faulted())
    {
        output_enabled_ = false;
        state_ = sguan_foc_state::FAULT;
        last_result_ = foc_result::DRIVER_FAULT;
    }
}

/**
 * @brief 将 wrapper 的电机和 PI 配置写入第三方 backend
 */
void sguan_foc_wrapper::apply_config_to_backend()
{
    Sguan.identify.Ld = config_.motor.ld_h;
    Sguan.identify.Lq = config_.motor.lq_h;
    Sguan.identify.Ls = config_.motor.ls_h;
    Sguan.identify.Rs = config_.motor.resistance_ohm;
    Sguan.identify.Flux = config_.motor.flux_wb;

    Sguan.motor.Poles = config_.motor.pole_pairs;
    Sguan.motor.VBUS = config_.nominal_bus_voltage_v;
    Sguan.motor.Motor_Dir = config_.motor.motor_direction;
    Sguan.motor.PWM_Dir = config_.motor.pwm_direction;
    Sguan.motor.Encoder_Dir = config_.motor.encoder_direction;
    Sguan.motor.Duty = config_.pwm_period;

    Sguan.safe.VBUS_MAX = config_.limits.max_bus_voltage_v;
    Sguan.safe.VBUS_MIM = config_.limits.min_bus_voltage_v;
    Sguan.safe.Dcur_MAX = config_.limits.max_id_a;
    Sguan.safe.Qcur_MAX = config_.limits.max_iq_a;

    Sguan.control.Current_D.Kp = config_.current_pi.kp;
    Sguan.control.Current_D.Ki = config_.current_pi.ki;
    Sguan.control.Current_D.OutMax =
        config_.current_pi.output_limit_v;
    Sguan.control.Current_D.OutMin =
        -config_.current_pi.output_limit_v;
    Sguan.control.Current_D.IntMax =
        config_.current_pi.integral_limit_v;
    Sguan.control.Current_D.IntMin =
        -config_.current_pi.integral_limit_v;

    Sguan.control.Current_Q.Kp = config_.current_pi.kp;
    Sguan.control.Current_Q.Ki = config_.current_pi.ki;
    Sguan.control.Current_Q.OutMax =
        config_.current_pi.output_limit_v;
    Sguan.control.Current_Q.OutMin =
        -config_.current_pi.output_limit_v;
    Sguan.control.Current_Q.IntMax =
        config_.current_pi.integral_limit_v;
    Sguan.control.Current_Q.IntMin =
        -config_.current_pi.integral_limit_v;

    Sguan.PMSM_RUN_T = config_.control_period_s;
}

/**
 * @brief 将完整业务命令事务映射到 SguanFOC 实时目标字段
 */
void sguan_foc_wrapper::apply_command_to_backend()
{
    motion_control_type controller = command_.controller;
    uint8_t backend_mode = Current_SINGLE_MODE;
    switch(controller)
    {
        case motion_control_type::TORQUE:
            backend_mode = Current_SINGLE_MODE;
            break;
        case motion_control_type::VELOCITY:
            backend_mode = VelCur_DOUBLE_MODE;
            break;
        case motion_control_type::ANGLE:
            backend_mode = PosVelCur_THREE_MODE;
            break;
        default:
            controller = motion_control_type::TORQUE;
            backend_mode = Current_SINGLE_MODE;
            break;
    }

    bool output_allowed = output_enabled_ &&
        state_ == sguan_foc_state::READY && !faulted();
    Sguan.mode = backend_mode;
    if(output_allowed)
    {
        Sguan.foc.Target_Id = command_.target_id_a;
        Sguan.foc.Target_Iq = command_.target_iq_a;
        Sguan.foc.Target_Speed = command_.target_velocity_rad_s;
        Sguan.foc.Target_Pos = command_.target_position_rad;
    }
    else
    {
        Sguan.foc.Target_Id = 0.0f;
        Sguan.foc.Target_Iq = 0.0f;
        Sguan.foc.Target_Speed = 0.0f;
        Sguan.foc.Target_Pos = 0.0;
    }
}

/**
 * @brief 判断第三方状态是否属于锁定型硬件故障
 *
 * @return backend 处于过压至急停状态时返回 true
 */
bool sguan_foc_wrapper::backend_faulted() const
{
    return Sguan.status >= MOTOR_STATUS_OVERVOLTAGE &&
        Sguan.status <= MOTOR_STATUS_EMERGENCY_STOP;
}

/**
 * @brief 从已绑定转子传感器读取角度并执行短期速度外推
 *
 * @return 归一化机械角度，单位弧度
 */
float sguan_foc_wrapper::read_encoder_from_isr()
{
    if(sensor_ == nullptr)
    {
        sensor_fault_ = true;
        output_enabled_ = false;
        state_ = sguan_foc_state::FAULT;
        last_result_ = foc_result::NOT_LINKED;
        return 0.0f;
    }

    rotor_sample sample{};
    foc_result result;
    if(__get_IPSR() == 0)
    {
        result = sensor_->read_task(sample);
    }
    else
    {
        result = sensor_->read_from_isr(sample);
    }

    if(result == foc_result::OK && sample.valid)
    {
        last_rotor_sample_ = sample;
        last_rotor_sample_valid_ = true;
    }
    else if(result != foc_result::SAMPLE_NOT_READY)
    {
        sensor_fault_ = true;
        output_enabled_ = false;
        state_ = sguan_foc_state::FAULT;
        last_result_ = result;
    }

    if(!last_rotor_sample_valid_)
    {
        sensor_fault_ = true;
        output_enabled_ = false;
        state_ = sguan_foc_state::FAULT;
        last_result_ = result;
        return 0.0f;
    }

    uint32_t elapsed_us = sys_time::get_us_tick() -
        last_rotor_sample_.timestamp_us;
    if(elapsed_us > ROTOR_HARD_TIMEOUT_US)
    {
        sensor_fault_ = true;
        output_enabled_ = false;
        state_ = sguan_foc_state::FAULT;
        last_result_ = foc_result::SAMPLE_STALE;
    }

    uint32_t extrapolation_us = elapsed_us;
    if(extrapolation_us > ROTOR_EXTRAPOLATION_LIMIT_US)
    {
        extrapolation_us = ROTOR_EXTRAPOLATION_LIMIT_US;
    }

    float angle_rad = last_rotor_sample_.mechanical_angle_rad +
        last_rotor_sample_.mechanical_velocity_rad_s *
        (float)extrapolation_us * 0.000001f;
    return Value_normalize(angle_rad);
}

/**
 * @brief 从已绑定相电流传感器读取本周期逻辑 A/B/C 相电流
 *
 * @param ia 输出 A 相电流，单位安培
 * @param ib 输出 B 相电流，单位安培
 * @param ic 输出 C 相电流，单位安培
 *
 * @return 样本有效时返回 1，否则返回 0
 */
uint8_t sguan_foc_wrapper::read_phase_currents_from_isr(
    float *ia,
    float *ib,
    float *ic)
{
    if(ia == nullptr || ib == nullptr || ic == nullptr ||
        current_sense_ == nullptr)
    {
        if(ia != nullptr)
        {
            *ia = 0.0f;
        }
        if(ib != nullptr)
        {
            *ib = 0.0f;
        }
        if(ic != nullptr)
        {
            *ic = 0.0f;
        }
        current_fault_ = true;
        output_enabled_ = false;
        state_ = sguan_foc_state::FAULT;
        last_result_ = current_sense_ == nullptr ?
            foc_result::NOT_LINKED : foc_result::INVALID_ARGUMENT;
        return 0;
    }

    phase_current_sample sample{};
    foc_result result = current_sense_->read_conversion_from_isr(
        sys_time::get_us_tick(),
        sample);
    if(result != foc_result::OK || !sample.valid ||
        !isfinite(sample.current_a) ||
        !isfinite(sample.current_b) ||
        !isfinite(sample.current_c))
    {
        *ia = 0.0f;
        *ib = 0.0f;
        *ic = 0.0f;
        current_fault_ = true;
        output_enabled_ = false;
        state_ = sguan_foc_state::FAULT;
        last_result_ = result;
        return 0;
    }

    *ia = sample.current_a;
    *ib = sample.current_b;
    *ic = sample.current_c;
    return 1;
}

/**
 * @brief 按约定的低频频率发布一次运行快照
 */
void sguan_foc_wrapper::publish_snapshot()
{
    snapshot_guard_sequence_++;
    __DMB();

    uint8_t status = Sguan.status;
    bool backend_fault = status >= MOTOR_STATUS_OVERVOLTAGE &&
        status <= MOTOR_STATUS_EMERGENCY_STOP;
    bool wrapper_fault = state_ == sguan_foc_state::FAULT ||
        sensor_fault_ || current_fault_ || backend_fault;
    bool snapshot_ready = initialized_ &&
        state_ == sguan_foc_state::READY && !wrapper_fault &&
        status >= MOTOR_STATUS_IDLE &&
        status < MOTOR_STATUS_OVERVOLTAGE;

    motion_control_type controller = command_.controller;
    switch(controller)
    {
        case motion_control_type::TORQUE:
        case motion_control_type::VELOCITY:
        case motion_control_type::ANGLE:
            break;
        default:
            controller = motion_control_type::TORQUE;
            break;
    }

    snapshot_cache_.sequence = ++snapshot_publish_sequence_;
    snapshot_cache_.initialized = initialized_;
    snapshot_cache_.ready = snapshot_ready;
    snapshot_cache_.enabled = output_enabled_ && snapshot_ready;
    snapshot_cache_.faulted = wrapper_fault;
    snapshot_cache_.state = wrapper_fault ?
        sguan_foc_state::FAULT : state_;
    snapshot_cache_.controller = controller;
    switch(controller)
    {
        case motion_control_type::TORQUE:
            snapshot_cache_.target = Sguan.foc.Target_Iq;
            break;
        case motion_control_type::VELOCITY:
            snapshot_cache_.target = Sguan.foc.Target_Speed;
            break;
        case motion_control_type::ANGLE:
            snapshot_cache_.target = (float)Sguan.foc.Target_Pos;
            break;
        default:
            snapshot_cache_.target = 0.0f;
            break;
    }

    snapshot_cache_.angle_rad = Sguan.encoder.Real_Rad;
    snapshot_cache_.full_angle_rad = Sguan.encoder.Real_Pos;
    snapshot_cache_.velocity_rad_s = Sguan.encoder.Real_Speed;
    snapshot_cache_.electrical_angle_rad = Sguan.encoder.Real_Erad;
    snapshot_cache_.electrical_velocity_rad_s = Sguan.encoder.Real_Espeed;
    snapshot_cache_.ia_a = Sguan.current.Real_Ia;
    snapshot_cache_.ib_a = Sguan.current.Real_Ib;
    snapshot_cache_.ic_a = Sguan.current.Real_Ic;
    snapshot_cache_.id_a = Sguan.current.Real_Id;
    snapshot_cache_.iq_a = Sguan.current.Real_Iq;
    snapshot_cache_.target_id_a = Sguan.foc.Target_Id;
    snapshot_cache_.target_iq_a = Sguan.foc.Target_Iq;
    snapshot_cache_.ud_v = Sguan.foc.Ud_in;
    snapshot_cache_.uq_v = Sguan.foc.Uq_in;
    snapshot_cache_.bus_voltage_v = Sguan.foc.Real_VBUS;
    snapshot_cache_.duty_u = Sguan.foc.Duty_u;
    snapshot_cache_.duty_v = Sguan.foc.Duty_v;
    snapshot_cache_.duty_w = Sguan.foc.Duty_w;
    snapshot_cache_.raw_status = status;

    __DMB();
    snapshot_guard_sequence_++;
}

/**
 * @brief 供 C 适配层在初始化参数完成后应用 wrapper 配置
 */
extern "C" void sguan_foc_wrapper_apply_config(void)
{
    if(sguan_foc_wrapper::active_instance_ != nullptr)
    {
        sguan_foc_wrapper::active_instance_->apply_config_to_backend();
    }
}

/**
 * @brief 供 C 高速适配层读取 wrapper 当前控制命令
 */
extern "C" void sguan_foc_wrapper_apply_command(void)
{
    if(sguan_foc_wrapper::active_instance_ != nullptr)
    {
        sguan_foc_wrapper::active_instance_->apply_command_to_backend();
    }
}

/**
 * @brief 供 SguanFOC 编码器 hook 读取 linked rotor sensor 的角度
 *
 * @return 归一化机械角度，单位弧度
 */
extern "C" float sguan_foc_wrapper_read_encoder_rad(void)
{
    if(sguan_foc_wrapper::active_instance_ == nullptr)
    {
        return 0.0f;
    }

    return sguan_foc_wrapper::active_instance_->
        read_encoder_from_isr();
}

/**
 * @brief 供 SguanFOC current hook 读取 linked current sensor 的相电流
 *
 * @param ia 输出 A 相电流，单位安培
 * @param ib 输出 B 相电流，单位安培
 * @param ic 输出 C 相电流，单位安培
 *
 * @return 样本有效时返回 1，否则返回 0
 */
extern "C" uint8_t sguan_foc_wrapper_read_phase_currents(
    float *ia,
    float *ib,
    float *ic)
{
    if(sguan_foc_wrapper::active_instance_ == nullptr)
    {
        if(ia != nullptr)
        {
            *ia = 0.0f;
        }
        if(ib != nullptr)
        {
            *ib = 0.0f;
        }
        if(ic != nullptr)
        {
            *ic = 0.0f;
        }
        return 0;
    }

    return sguan_foc_wrapper::active_instance_->
        read_phase_currents_from_isr(ia, ib, ic);
}

/**
 * @brief 供 SguanFOC 初始化 hook 确认 CurrentSense 零偏已准备完成
 *
 * @return CurrentSense 已完成校准时返回 1
 */
extern "C" uint8_t sguan_foc_wrapper_current_offset_prepared(void)
{
    if(sguan_foc_wrapper::active_instance_ == nullptr)
    {
        return 0;
    }

    return sguan_foc_wrapper::active_instance_->
        current_calibration_done_ ? 1 : 0;
}

#include "sguan_foc_wrapper.h"

#include "main.h"

extern "C"
{
#include "../SguanFOC.h"
}

SguanFOCWrapper *SguanFOCWrapper::active_instance_ = nullptr;

/**
 * @brief 返回当前工程已验证的 SguanFOC 默认配置
 *
 * @return 与当前实机电流环参数一致的默认配置
 */
SguanFOCConfig SguanFOCWrapper::default_config()
{
    SguanFOCConfig config{};

    config.motor.pole_pairs = 7;
    config.motor.resistance_ohm = 2.55f;
    config.motor.ld_h = 0.00086f;
    config.motor.lq_h = 0.00086f;
    config.motor.ls_h = 0.00086f;
    config.motor.flux_wb = 0.0035f;
    config.motor.motor_direction = 1;
    config.motor.encoder_direction = -1;
    config.motor.pwm_direction = -1;

    config.current_sense.shunt_resistance_ohm = 0.01f;
    config.current_sense.amplifier_gain = 50.0f;
    config.current_sense.adc_reference_v = 3.3f;
    config.current_sense.adc_full_scale = 4096;
    config.current_sense.dir0 = 1;
    config.current_sense.dir1 = -1;
    config.current_sense.phase_mapping = 1;

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
 * @brief 绑定 SguanFOC backend 并设置初始化入口状态
 *
 * @param config 工程侧电机和电流环配置
 *
 * @return 配置有效且 backend 未被其他实例占用时返回 true
 */
bool SguanFOCWrapper::init(const SguanFOCConfig &config)
{
    if(config.motor.pole_pairs == 0 ||
        !(config.motor.resistance_ohm > 0.0f) ||
        !(config.motor.ld_h > 0.0f) ||
        !(config.motor.lq_h > 0.0f) ||
        !(config.motor.ls_h > 0.0f) ||
        !(config.motor.flux_wb >= 0.0f) ||
        (config.motor.motor_direction != 1 &&
            config.motor.motor_direction != -1) ||
        (config.motor.encoder_direction != 1 &&
            config.motor.encoder_direction != -1) ||
        (config.motor.pwm_direction != 1 &&
            config.motor.pwm_direction != -1) ||
        !(config.current_sense.shunt_resistance_ohm > 0.0f) ||
        !(config.current_sense.amplifier_gain > 0.0f) ||
        !(config.current_sense.adc_reference_v > 0.0f) ||
        config.current_sense.adc_full_scale == 0 ||
        (config.current_sense.dir0 != 1 &&
            config.current_sense.dir0 != -1) ||
        (config.current_sense.dir1 != 1 &&
            config.current_sense.dir1 != -1) ||
        config.current_sense.phase_mapping > 2 ||
        !(config.current_pi.kp >= 0.0f) ||
        !(config.current_pi.ki >= 0.0f) ||
        !(config.current_pi.output_limit_v > 0.0f) ||
        !(config.current_pi.integral_limit_v > 0.0f) ||
        !(config.limits.max_id_a > 0.0f) ||
        !(config.limits.max_iq_a > 0.0f) ||
        !(config.limits.min_bus_voltage_v <
            config.limits.max_bus_voltage_v) ||
        !(config.nominal_bus_voltage_v > 0.0f) ||
        config.nominal_bus_voltage_v < config.limits.min_bus_voltage_v ||
        config.nominal_bus_voltage_v > config.limits.max_bus_voltage_v ||
        config.pwm_period == 0 ||
        !(config.control_period_s > 0.0f))
    {
        last_error_ = SguanFOCInitError::InvalidConfig;
        return false;
    }

    if(active_instance_ != nullptr && active_instance_ != this)
    {
        last_error_ = SguanFOCInitError::BackendAlreadyInUse;
        return false;
    }

    if(initialized_)
    {
        last_error_ = SguanFOCInitError::None;
        return true;
    }

    config_ = config;
    active_instance_ = this;
    initialized_ = true;
    enabled_ = true;
    last_error_ = SguanFOCInitError::None;

    // 保持原有生命周期：由 service_loop() 在传感器就绪后执行完整初始化。
    Sguan.status = MOTOR_STATUS_UNINITIALIZED;
    return true;
}

/**
 * @brief 从 ADC 注入转换完成中断推进一次 SguanFOC 高速环
 */
void SguanFOCWrapper::high_freq_loop()
{
    SguanFOC_High_Loop();
    publish_snapshot();
}

/**
 * @brief 执行 SguanFOC 原有主循环入口
 */
void SguanFOCWrapper::service_loop()
{
    if(!initialized_)
    {
        return;
    }

    SguanFOC_main_Loop();
}

/**
 * @brief 执行 SguanFOC 原有低频保护和状态机入口
 */
void SguanFOCWrapper::low_freq_loop()
{
    if(!initialized_)
    {
        return;
    }

    SguanFOC_Low_Loop();
}

/**
 * @brief 原子地更新电流目标命令
 *
 * @param id_a D轴目标电流，单位安培
 * @param iq_a Q轴目标电流，单位安培
 */
void SguanFOCWrapper::set_current(float id_a, float iq_a)
{
    uint32_t interrupt_state = __get_PRIMASK();
    __disable_irq();
    command_.target_id_a = id_a;
    command_.target_iq_a = iq_a;
    __DMB();
    __set_PRIMASK(interrupt_state);
}

/**
 * @brief 原子地更新速度目标命令
 *
 * @param velocity_rad_s 目标机械角速度，单位弧度每秒
 */
void SguanFOCWrapper::set_velocity(float velocity_rad_s)
{
    uint32_t interrupt_state = __get_PRIMASK();
    __disable_irq();
    command_.target_velocity_rad_s = velocity_rad_s;
    __DMB();
    __set_PRIMASK(interrupt_state);
}

/**
 * @brief 原子地更新位置目标命令
 *
 * @param position_rad 目标机械位置，单位弧度
 */
void SguanFOCWrapper::set_position(double position_rad)
{
    uint32_t interrupt_state = __get_PRIMASK();
    __disable_irq();
    command_.target_position_rad = position_rad;
    __DMB();
    __set_PRIMASK(interrupt_state);
}

/**
 * @brief 原子地更新工程侧控制模式
 *
 * @param mode 工程侧控制模式
 */
void SguanFOCWrapper::set_mode(SguanFOCMode mode)
{
    uint32_t interrupt_state = __get_PRIMASK();
    __disable_irq();
    command_.mode = mode;
    __DMB();
    __set_PRIMASK(interrupt_state);
}

/**
 * @brief 请求第三方状态机离开失能状态
 */
void SguanFOCWrapper::enable()
{
    if(!initialized_)
    {
        return;
    }

    uint32_t interrupt_state = __get_PRIMASK();
    __disable_irq();
    if(Sguan.status == MOTOR_STATUS_DISABLED)
    {
        Sguan.status = MOTOR_STATUS_IDLE;
    }
    enabled_ = true;
    __DMB();
    __set_PRIMASK(interrupt_state);
}

/**
 * @brief 请求第三方状态机进入失能状态
 */
void SguanFOCWrapper::disable()
{
    if(!initialized_)
    {
        return;
    }

    uint32_t interrupt_state = __get_PRIMASK();
    __disable_irq();
    Sguan.status = MOTOR_STATUS_DISABLED;
    enabled_ = false;
    __DMB();
    __set_PRIMASK(interrupt_state);
}

/**
 * @brief 查询 wrapper 是否已经绑定 backend
 *
 * @return 已完成 wrapper 初始化时返回 true
 */
bool SguanFOCWrapper::initialized() const
{
    return initialized_;
}

/**
 * @brief 查询 backend 是否处于可运行的非故障状态
 *
 * @return 已请求使能且第三方状态位于运行区间时返回 true
 */
bool SguanFOCWrapper::enabled() const
{
    if(!initialized_ || !enabled_)
    {
        return false;
    }

    return Sguan.status >= MOTOR_STATUS_IDLE &&
        Sguan.status < MOTOR_STATUS_OVERVOLTAGE;
}

/**
 * @brief 读取高速环发布的完整运行快照
 *
 * @return 通过 sequence 校验的一致快照
 */
SguanFOCSnapshot SguanFOCWrapper::snapshot() const
{
    SguanFOCSnapshot result{};

    while(true)
    {
        uint32_t sequence_begin = snapshot_sequence_;
        if(sequence_begin & 1)
        {
            continue;
        }

        __DMB();
        result.initialized = snapshot_cache_.initialized;
        result.enabled = snapshot_cache_.enabled;
        result.mode = snapshot_cache_.mode;
        result.raw_status = snapshot_cache_.raw_status;
        result.mechanical_angle_rad =
            snapshot_cache_.mechanical_angle_rad;
        result.mechanical_position_rad =
            snapshot_cache_.mechanical_position_rad;
        result.mechanical_velocity_rad_s =
            snapshot_cache_.mechanical_velocity_rad_s;
        result.electrical_angle_rad = snapshot_cache_.electrical_angle_rad;
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
        __DMB();

        uint32_t sequence_end = snapshot_sequence_;
        if(sequence_begin == sequence_end)
        {
            return result;
        }
    }
}

/**
 * @brief 查询最近一次初始化失败原因
 *
 * @return 初始化错误枚举
 */
SguanFOCInitError SguanFOCWrapper::last_init_error() const
{
    return last_error_;
}

/**
 * @brief 将 wrapper 配置映射到 SguanFOC 的全局 backend
 */
void SguanFOCWrapper::apply_config_to_backend()
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

    Sguan.motor.Current_Dir0 = config_.current_sense.dir0;
    Sguan.motor.Current_Dir1 = config_.current_sense.dir1;
    Sguan.motor.Current_Num = config_.current_sense.phase_mapping;
    Sguan.motor.ADC_Precision = config_.current_sense.adc_full_scale;
    Sguan.motor.Amplifier = config_.current_sense.amplifier_gain;
    Sguan.motor.MCU_Voltage = config_.current_sense.adc_reference_v;
    Sguan.motor.Sampling_Rs =
        config_.current_sense.shunt_resistance_ohm;

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
 * @brief 将工程命令映射到 SguanFOC 的实时目标字段
 */
void SguanFOCWrapper::apply_command_to_backend()
{
    uint8_t backend_mode = Current_SINGLE_MODE;
    switch(command_.mode)
    {
        case SguanFOCMode::OpenVoltage:
            backend_mode = Velocity_OPEN_MODE;
            break;
        case SguanFOCMode::Current:
            backend_mode = Current_SINGLE_MODE;
            break;
        case SguanFOCMode::Velocity:
            backend_mode = VelCur_DOUBLE_MODE;
            break;
        case SguanFOCMode::Position:
            backend_mode = PosVelCur_THREE_MODE;
            break;
        default:
            backend_mode = Current_SINGLE_MODE;
            break;
    }

    Sguan.mode = backend_mode;
    Sguan.foc.Target_Id = command_.target_id_a;
    Sguan.foc.Target_Iq = command_.target_iq_a;
    Sguan.foc.Target_Speed = command_.target_velocity_rad_s;
    Sguan.foc.Target_Pos = command_.target_position_rad;
}

/**
 * @brief 从 backend 复制少量字段并发布一次一致快照
 */
void SguanFOCWrapper::publish_snapshot()
{
    snapshot_sequence_++;
    __DMB();

    const uint8_t status = Sguan.status;
    snapshot_cache_.initialized = initialized_;
    snapshot_cache_.enabled = enabled_ &&
        status >= MOTOR_STATUS_IDLE &&
        status < MOTOR_STATUS_OVERVOLTAGE;
    snapshot_cache_.raw_status = status;
    switch(Sguan.mode)
    {
        case Velocity_OPEN_MODE:
            snapshot_cache_.mode = SguanFOCMode::OpenVoltage;
            break;
        case Current_SINGLE_MODE:
            snapshot_cache_.mode = SguanFOCMode::Current;
            break;
        case VelCur_DOUBLE_MODE:
            snapshot_cache_.mode = SguanFOCMode::Velocity;
            break;
        case PosVelCur_THREE_MODE:
            snapshot_cache_.mode = SguanFOCMode::Position;
            break;
        default:
            snapshot_cache_.mode = SguanFOCMode::Current;
            break;
    }

    snapshot_cache_.mechanical_angle_rad = Sguan.encoder.Real_Rad;
    snapshot_cache_.mechanical_position_rad = Sguan.encoder.Real_Pos;
    snapshot_cache_.mechanical_velocity_rad_s = Sguan.encoder.Real_Speed;
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

    __DMB();
    snapshot_sequence_++;
}

/**
 * @brief 供 C 适配层在初始化参数完成后应用 wrapper 配置
 */
extern "C" void sguan_foc_wrapper_apply_config(void)
{
    if(SguanFOCWrapper::active_instance_ != nullptr)
    {
        SguanFOCWrapper::active_instance_->apply_config_to_backend();
    }
}

/**
 * @brief 供 C 高速适配层读取 wrapper 当前控制命令
 */
extern "C" void sguan_foc_wrapper_apply_command(void)
{
    if(SguanFOCWrapper::active_instance_ != nullptr)
    {
        SguanFOCWrapper::active_instance_->apply_command_to_backend();
    }
}

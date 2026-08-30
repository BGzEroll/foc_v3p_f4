#ifndef SGUAN_FOC_WRAPPER_H
#define SGUAN_FOC_WRAPPER_H

#include <stdint.h>

#include "sguan_foc_bridge.h"

enum class SguanFOCMode : uint8_t
{
    OpenVoltage = 0,
    Current,
    Velocity,
    Position,
};

enum class SguanFOCInitError : uint8_t
{
    None = 0,
    BackendAlreadyInUse,
    InvalidConfig,
};

struct SguanFOCConfig
{
    struct Motor
    {
        uint8_t pole_pairs = 0;
        float resistance_ohm = 0.0f;
        float ld_h = 0.0f;
        float lq_h = 0.0f;
        float ls_h = 0.0f;
        float flux_wb = 0.0f;
        int8_t motor_direction = 0;
        int8_t encoder_direction = 0;
        int8_t pwm_direction = 0;
    } motor;

    struct CurrentSense
    {
        float shunt_resistance_ohm = 0.0f;
        float amplifier_gain = 0.0f;
        float adc_reference_v = 0.0f;
        uint32_t adc_full_scale = 0;
        int8_t dir0 = 0;
        int8_t dir1 = 0;
        uint8_t phase_mapping = 0;
    } current_sense;

    struct CurrentPI
    {
        float kp = 0.0f;
        float ki = 0.0f;
        float output_limit_v = 0.0f;
        float integral_limit_v = 0.0f;
    } current_pi;

    struct Limits
    {
        float max_id_a = 0.0f;
        float max_iq_a = 0.0f;
        float min_bus_voltage_v = 0.0f;
        float max_bus_voltage_v = 0.0f;
    } limits;

    float nominal_bus_voltage_v = 0.0f;
    uint16_t pwm_period = 0;
    float control_period_s = 0.0f;
};

struct SguanFOCCommand
{
    float target_id_a = 0.0f;
    float target_iq_a = 0.10f;
    float target_velocity_rad_s = 0.0f;
    double target_position_rad = 0.0;
    SguanFOCMode mode = SguanFOCMode::Current;
};

struct SguanFOCSnapshot
{
    bool initialized = false;
    bool enabled = false;
    SguanFOCMode mode = SguanFOCMode::Current;
    uint8_t raw_status = 0;

    float mechanical_angle_rad = 0.0f;
    double mechanical_position_rad = 0.0;
    float mechanical_velocity_rad_s = 0.0f;
    float electrical_angle_rad = 0.0f;
    float electrical_velocity_rad_s = 0.0f;

    float ia_a = 0.0f;
    float ib_a = 0.0f;
    float ic_a = 0.0f;
    float id_a = 0.0f;
    float iq_a = 0.0f;

    float target_id_a = 0.0f;
    float target_iq_a = 0.0f;
    float ud_v = 0.0f;
    float uq_v = 0.0f;
    float bus_voltage_v = 0.0f;

    uint16_t duty_u = 0;
    uint16_t duty_v = 0;
    uint16_t duty_w = 0;
};

// 当前 wrapper 保留对象形式，但 SguanFOC 3.0.0 backend 只允许单实例占用。
// backend 没有 deinit，guard 在首次成功 init 后保持占用到系统复位。
class SguanFOCWrapper
{
    public:
        SguanFOCWrapper() = default;
        SguanFOCWrapper(const SguanFOCWrapper &) = delete;
        SguanFOCWrapper &operator=(const SguanFOCWrapper &) = delete;
        SguanFOCWrapper(SguanFOCWrapper &&) = delete;
        SguanFOCWrapper &operator=(SguanFOCWrapper &&) = delete;

    public:
        bool init(const SguanFOCConfig &config);

    public:
        // ISR-safe，ADC2 注入转换完成回调在20 kHz调用。
        void high_freq_loop();

        // Task context，保持现有 SguanFOC 主循环调用频率。
        void service_loop();

        // Task context，约1 kHz调用保护和状态机。
        void low_freq_loop();

        // Task context，写入下一次高速环使用的电流目标。
        void set_current(float id_a, float iq_a);

        // Task context，写入速度环目标。
        void set_velocity(float velocity_rad_s);

        // Task context，写入位置环目标。
        void set_position(double position_rad);

        // Task context，切换工程侧控制模式。
        void set_mode(SguanFOCMode mode);

        // Task context，请求进入已使能状态。
        void enable();

        // Task context，请求进入第三方失能状态。
        void disable();

        // Task context，返回 wrapper 是否已绑定 backend。
        bool initialized() const;

        // Task context，返回当前是否处于可运行状态。
        bool enabled() const;

        // Task context，读取一致的只读运行快照。
        SguanFOCSnapshot snapshot() const;

        // Task context，读取最近一次 init 失败原因。
        SguanFOCInitError last_init_error() const;

        static SguanFOCConfig default_config();

    private:
        friend void sguan_foc_wrapper_apply_config(void);
        friend void sguan_foc_wrapper_apply_command(void);

        void apply_config_to_backend();
        void apply_command_to_backend();
        void publish_snapshot();

        static SguanFOCWrapper *active_instance_;

        SguanFOCConfig config_{};
        volatile SguanFOCCommand command_{};
        volatile SguanFOCSnapshot snapshot_cache_{};
        volatile uint32_t snapshot_sequence_ = 0;

        bool initialized_ = false;
        volatile bool enabled_ = false;
        SguanFOCInitError last_error_ = SguanFOCInitError::None;
};

#endif

#ifndef SGUAN_FOC_WRAPPER_H
#define SGUAN_FOC_WRAPPER_H

#include <stdint.h>
#include "sguan_foc_bridge.h"

enum class sguan_foc_mode : uint8_t
{
    OPEN_VOLTAGE = 0,
    CURRENT,
    VELOCITY,
    POSITION,
};

enum class sguan_foc_init_error : uint8_t
{
    NONE = 0,
    BACKEND_ALREADY_IN_USE,
    INVALID_CONFIG,
};

struct sguan_foc_config
{
    struct motor
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

    struct current_sense
    {
        float shunt_resistance_ohm = 0.0f;
        float amplifier_gain = 0.0f;
        float adc_reference_v = 0.0f;
        uint32_t adc_full_scale = 0;
        int8_t dir0 = 0;
        int8_t dir1 = 0;
        uint8_t phase_mapping = 0;
    } current_sense;

    struct current_pi
    {
        float kp = 0.0f;
        float ki = 0.0f;
        float output_limit_v = 0.0f;
        float integral_limit_v = 0.0f;
    } current_pi;

    struct limits
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

struct sguan_foc_command
{
    float target_id_a = 0.0f;
    float target_iq_a = 0.10f;
    float target_velocity_rad_s = 0.0f;
    double target_position_rad = 0.0;
    sguan_foc_mode mode = sguan_foc_mode::CURRENT;
};

struct sguan_foc_snapshot
{
    bool initialized = false;
    bool enabled = false;
    sguan_foc_mode mode = sguan_foc_mode::CURRENT;
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
class sguan_foc_wrapper
{
    public:
        sguan_foc_wrapper() = default;
        sguan_foc_wrapper(const sguan_foc_wrapper &) = delete;
        sguan_foc_wrapper &operator=(const sguan_foc_wrapper &) = delete;
        sguan_foc_wrapper(sguan_foc_wrapper &&) = delete;
        sguan_foc_wrapper &operator=(sguan_foc_wrapper &&) = delete;

    public:
        bool init(const sguan_foc_config &config);

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
        void set_mode(sguan_foc_mode mode);

        // Task context，请求进入已使能状态。
        void enable();

        // Task context，请求进入第三方失能状态。
        void disable();

        // Task context，返回 wrapper 是否已绑定 backend。
        bool initialized() const;

        // Task context，返回当前是否处于可运行状态。
        bool enabled() const;

        // Task context，读取一致的只读运行快照。
        sguan_foc_snapshot snapshot() const;

        // Task context，读取最近一次 init 失败原因。
        sguan_foc_init_error last_init_error() const;

        static sguan_foc_config default_config();

    private:
        friend void sguan_foc_wrapper_apply_config(void);
        friend void sguan_foc_wrapper_apply_command(void);

        void apply_config_to_backend();
        void apply_command_to_backend();
        void publish_snapshot();

        static sguan_foc_wrapper *active_instance_;

        sguan_foc_config config_{};
        volatile sguan_foc_command command_{};
        volatile sguan_foc_snapshot snapshot_cache_{};
        volatile uint32_t snapshot_sequence_ = 0;

        bool initialized_ = false;
        volatile bool enabled_ = false;
        sguan_foc_init_error last_error_ = sguan_foc_init_error::NONE;
};

#endif

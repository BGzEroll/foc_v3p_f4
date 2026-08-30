#ifndef SGUAN_FOC_WRAPPER_H
#define SGUAN_FOC_WRAPPER_H

#include <stdint.h>
#include "drivers/foc/sensors/current_sensor.h"
#include "drivers/foc/sensors/rotor_sensor.h"
#include "sguan_foc_bridge.h"

enum class motion_control_type : uint8_t
{
    TORQUE = 0,
    VELOCITY,
    ANGLE,
};

enum class sguan_foc_state : uint8_t
{
    UNINITIALIZED = 0,
    INITIALIZING,
    CALIBRATING,
    READY,
    FAULT,
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

struct sguan_foc_snapshot
{
    uint32_t sequence = 0;
    bool initialized = false;
    bool ready = false;
    bool enabled = false;
    bool faulted = false;
    sguan_foc_state state = sguan_foc_state::UNINITIALIZED;
    motion_control_type controller = motion_control_type::TORQUE;
    float target = 0.0f;

    float angle_rad = 0.0f;
    double full_angle_rad = 0.0;
    float velocity_rad_s = 0.0f;
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
    uint8_t raw_status = 0;
};

// SguanFOC 3.0.0 backend 仍由全局 Sguan 提供，只允许一个 wrapper 占用。
class sguan_foc_wrapper
{
    public:
        sguan_foc_wrapper() = default;
        sguan_foc_wrapper(const sguan_foc_wrapper &) = delete;
        sguan_foc_wrapper &operator=(const sguan_foc_wrapper &) = delete;
        sguan_foc_wrapper(sguan_foc_wrapper &&) = delete;
        sguan_foc_wrapper &operator=(sguan_foc_wrapper &&) = delete;

    public:
        foc_result link_sensor(rotor_sensor &sensor);
        foc_result link_current_sense(current_sensor &current_sense);
        foc_result init(const sguan_foc_config &config);
        foc_result init_foc();

    public:
        void set_controller(motion_control_type controller);
        void move(float target);
        void set_current_dq(float id_a, float iq_a);
        void enable();
        void disable();

        bool initialized() const;
        bool ready() const;
        bool enabled() const;
        bool faulted() const;
        motion_control_type controller() const;
        sguan_foc_snapshot snapshot() const;

        static sguan_foc_config default_config();

    public:
        // 仅由 ADC2 注入转换完成中断在20 kHz调用。
        void loop_foc();

        // 仅由板级低频任务调用，保持主循环先于低频状态机执行。
        void service();

    private:
        friend void sguan_foc_wrapper_apply_config(void);
        friend void sguan_foc_wrapper_apply_command(void);
        friend float sguan_foc_wrapper_read_encoder_rad(void);
        friend uint8_t sguan_foc_wrapper_read_phase_currents(
            float *ia,
            float *ib,
            float *ic);
        friend uint8_t sguan_foc_wrapper_current_offset_prepared(void);

        struct command
        {
            motion_control_type controller = motion_control_type::TORQUE;
            float target_id_a = 0.0f;
            float target_iq_a = 0.0f;
            float target_velocity_rad_s = 0.0f;
            double target_position_rad = 0.0;
        };

        void apply_config_to_backend();
        void apply_command_to_backend();
        void publish_snapshot();
        float read_encoder_from_isr();
        uint8_t read_phase_currents_from_isr(
            float *ia,
            float *ib,
            float *ic);
        bool backend_faulted() const;

        static sguan_foc_wrapper *active_instance_;

        rotor_sensor *sensor_ = nullptr;
        current_sensor *current_sense_ = nullptr;
        sguan_foc_config config_{};
        volatile command command_{};
        rotor_sample last_rotor_sample_{};
        bool last_rotor_sample_valid_ = false;
        volatile bool sensor_fault_ = false;
        volatile bool current_fault_ = false;
        volatile bool current_calibration_done_ = false;
        volatile bool initialized_ = false;
        volatile bool output_enabled_ = false;
        volatile sguan_foc_state state_ =
            sguan_foc_state::UNINITIALIZED;
        volatile foc_result last_result_ = foc_result::OK;

        volatile sguan_foc_snapshot snapshot_cache_{};
        volatile uint32_t snapshot_guard_sequence_ = 0;
        uint32_t snapshot_publish_sequence_ = 0;
        uint32_t snapshot_divider_ = 0;
};

#endif

#ifndef FOC_TYPES_H
#define FOC_TYPES_H

#include <stdint.h>

enum class foc_result : uint8_t
{
    OK = 0,
    INVALID_ARGUMENT,
    INVALID_STATE,
    INVALID_CONFIG,
    NOT_LINKED,
    NOT_INITIALIZED,
    NOT_READY,
    DISABLED,
    TOPIC_ERROR,
    SENSOR_ERROR,
    SAMPLE_NOT_READY,
    SAMPLE_STALE,
    CALIBRATING,
    DRIVER_FAULT,
    OUTPUT_FAULT,
    ROTOR_ALIGNMENT_FAILED
};

enum class foc_state : uint8_t
{
    UNINITIALIZED = 0,
    MONITORING,
    READY,
    RUNNING,
    FAULT
};

enum class foc_control_mode : uint8_t
{
    DISABLED = 0,
    OPEN_LOOP_VOLTAGE,
    VOLTAGE,
    CURRENT
};

enum class foc_fault : uint32_t
{
    NONE = 0U,
    ROTOR_NOT_READY = 1UL << 0,
    ROTOR_STALE = 1UL << 1,
    ROTOR_COMMUNICATION = 1UL << 2,
    CURRENT_NOT_READY = 1UL << 3,
    OVER_CURRENT = 1UL << 4,
    DRIVER = 1UL << 5,
    INVALID_NUMBER = 1UL << 6,
    OUTPUT_RANGE = 1UL << 7,
    COMMAND_TIMEOUT = 1UL << 8,
    INTERNAL = 1UL << 9
};

struct pi_config
{
    float proportional_gain = 0.0f;
    float integral_gain = 0.0f;
    float integral_limit = 0.0f;
};

struct rotor_sample
{
    uint32_t sequence = 0U;
    uint32_t timestamp_us = 0U;
    uint16_t raw_count = 0U;
    float mechanical_angle_rad = 0.0f;
    float full_angle_rad = 0.0f;
    float mechanical_velocity_rad_s = 0.0f;
    bool valid = false;
};

struct phase_current_sample
{
    uint32_t sequence = 0U;
    uint32_t timestamp_us = 0U;
    uint16_t raw_count_a = 0U;
    uint16_t raw_count_b = 0U;
    float offset_count_a = 0.0f;
    float offset_count_b = 0.0f;
    float current_a = 0.0f;
    float current_b = 0.0f;
    float current_c = 0.0f;
    bool valid = false;
};

struct phase_duty
{
    float phase_a = 0.5f;
    float phase_b = 0.5f;
    float phase_c = 0.5f;
};

struct foc_target
{
    uint32_t sequence = 0U;
    uint32_t timestamp_ms = 0U;
    uint32_t electrical_angle_timestamp_us = 0U;
    foc_control_mode mode = foc_control_mode::DISABLED;
    float d_axis_current_a = 0.0f;
    float q_axis_current_a = 0.0f;
    float d_axis_voltage_v = 0.0f;
    float q_axis_voltage_v = 0.0f;
    float electrical_angle_rad = 0.0f;
    float electrical_velocity_rad_s = 0.0f;
};

struct foc_config
{
    bool monitor_only = true;
    bool open_loop_control_from_task = false;
    uint8_t pole_pairs = 0U;
    int8_t rotor_direction = 1;
    float electrical_zero_offset_rad = 0.0f;
    float control_period_s = 0.00005f;
    float bus_voltage_v = 0.0f;
    float voltage_limit_v = 0.0f;
    float max_phase_current_a = 0.0f;
    uint32_t rotor_extrapolation_limit_us = 2000U;
    uint32_t rotor_hard_timeout_us = 5000U;
    uint32_t rotor_slow_timeout_us = 50000U;
    uint32_t command_timeout_ms = 100U;
    uint32_t communication_error_limit = 10U;
    uint32_t telemetry_divider = 20U;
    uint16_t control_isr_divider = 1U;
    pi_config d_axis_pi{};
    pi_config q_axis_pi{};
};

struct foc_snapshot
{
    uint32_t sequence = 0U;
    uint32_t timestamp_us = 0U;
    foc_state state = foc_state::UNINITIALIZED;
    uint32_t fault_flags = 0U;
    bool monitor_only = true;
    bool output_active = false;
    rotor_sample rotor{};
    phase_current_sample current{};
    foc_target target{};
    float rotor_sample_age_ms = 0.0f;
    float d_axis_current_a = 0.0f;
    float q_axis_current_a = 0.0f;
    float d_axis_voltage_v = 0.0f;
    float q_axis_voltage_v = 0.0f;
    phase_duty duty{};
    uint32_t control_sequence = 0U;
    uint32_t bus_update_error_count = 0U;
    uint32_t consecutive_bus_error_count = 0U;
};

constexpr uint32_t foc_fault_mask(foc_fault fault)
{
    return (uint32_t)fault;
}

#endif

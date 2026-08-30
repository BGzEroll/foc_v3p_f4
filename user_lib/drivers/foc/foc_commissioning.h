#ifndef FOC_COMMISSIONING_H
#define FOC_COMMISSIONING_H

#include "foc_types.h"

enum class foc_commissioning_stage : uint8_t
{
    WAIT_CALIBRATION = 0,
    WAIT_ROTOR,
    OPEN_LOOP_FORWARD,
    OPEN_LOOP_PAUSE,
    OPEN_LOOP_REVERSE,
    ALIGN_FIRST,
    DIAGNOSE_PHASE_B,
    DIAGNOSE_PHASE_C,
    VERIFY_CURRENT_POLARITY,
    ALIGN_SECOND,
    VERIFY_D_AXIS_CURRENT,
    VERIFY_Q_AXIS_CURRENT,
    COMPLETE,
    FAILED
};

struct foc_commissioning_config
{
    uint32_t current_sensor_settle_time_ms = 500;
    uint32_t current_calibration_sample_count = 4096;
    float alignment_voltage_v = 3.0f;
    float open_loop_voltage_v = 3.0f;
    float open_loop_mechanical_velocity_rad_s = 10.0f;
    float minimum_bus_voltage_v = 9.0f;
    float maximum_bus_voltage_v = 15.0f;
    float d_axis_verify_current_a = 0.05f;
    float q_axis_verify_current_a = 0.03f;
};

struct foc_commissioning_status
{
    uint32_t sequence = 0;
    foc_commissioning_stage stage =
        foc_commissioning_stage::WAIT_CALIBRATION;
    foc_result result = foc_result::NOT_READY;
    int8_t current_direction_a = 1;
    int8_t current_direction_b = -1;
    int8_t rotor_direction = 1;
    float electrical_zero_offset_rad = 0.0f;
    float first_mechanical_angle_rad = 0.0f;
    float second_mechanical_angle_rad = 0.0f;
    float phase_vector_current_a[3]{};
    float phase_vector_current_b[3]{};
    float phase_vector_current_c[3]{};
    float phase_vector_mechanical_angle_rad[3]{};
    float bus_voltage_v = 0.0f;
    float minimum_bus_voltage_v = 0.0f;
    bool phase_vector_check_passed = false;
    float measured_d_axis_current_a = 0.0f;
    float measured_q_axis_current_a = 0.0f;
    float open_loop_voltage_v = 0.0f;
    float open_loop_mechanical_velocity_rad_s = 0.0f;
    float open_loop_electrical_velocity_rad_s = 0.0f;
    float open_loop_forward_delta_rad = 0.0f;
    float open_loop_reverse_delta_rad = 0.0f;
    bool open_loop_motion_detected = false;
};

namespace foc
{
    namespace commissioning
    {
        foc_result init(const foc_commissioning_config &config,
            uint32_t timestamp_ms);
        void update(uint32_t timestamp_ms, float bus_voltage_v);
        bool peek_status(foc_commissioning_status &status);
    }
}

#endif

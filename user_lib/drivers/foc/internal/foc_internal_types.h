#ifndef FOC_INTERNAL_TYPES_H
#define FOC_INTERNAL_TYPES_H

#include "../foc_types.h"

// 控制命令包含公共目标之外的运行时元数据，仅在 FOC 内部 Topic 中传递。
struct foc_command
{
    foc_target target{};
    foc_runtime_mode runtime_mode = foc_runtime_mode::DISABLED;
    uint32_t sequence = 0;
    uint32_t timestamp_ms = 0;
    uint32_t electrical_angle_timestamp_us = 0;
    float electrical_angle_rad = 0.0f;
    float electrical_velocity_rad_s = 0.0f;
};

// 控制 ISR 发布给安全任务的实例级遥测数据。
struct foc_control_telemetry
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

// 低频安全任务发送给控制 ISR 的实例级故障请求。
struct foc_fault_request
{
    uint32_t sequence = 0;
    uint32_t fault_flags = 0;
};

#endif

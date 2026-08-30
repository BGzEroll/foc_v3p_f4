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

#endif

#ifndef FOC_INTERNAL_TYPES_H
#define FOC_INTERNAL_TYPES_H

#include "../foc_types.h"
#include "../phase_driver/phase_driver.h"
#include "../sensors/current_sensor.h"
#include "../sensors/rotor_sensor.h"

// 硬件对象由设备层一次性绑定，普通业务不需要管理这些接口。
struct foc_hardware
{
    rotor_sensor *rotor = nullptr;
    current_sensor *current = nullptr;
    phase_driver *driver = nullptr;
};

// 控制命令包含公共目标之外的运行时元数据，仅在 FOC 内部 Topic 中传递。
struct foc_command
{
    foc_target target{};
    uint32_t sequence = 0;
    uint32_t timestamp_ms = 0;
    uint32_t electrical_angle_timestamp_us = 0;
    float electrical_angle_rad = 0.0f;
    float electrical_velocity_rad_s = 0.0f;
};

#endif

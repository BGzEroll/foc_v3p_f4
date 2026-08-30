#ifndef FOC_HARDWARE_H
#define FOC_HARDWARE_H

#include "phase_driver/phase_driver.h"
#include "sensors/current_sensor.h"
#include "sensors/rotor_sensor.h"

// 由设备装配层注入 FOC 使用的硬件抽象对象。
struct foc_hardware
{
    rotor_sensor *rotor = nullptr;
    current_sensor *current = nullptr;
    phase_driver *driver = nullptr;
};

#endif

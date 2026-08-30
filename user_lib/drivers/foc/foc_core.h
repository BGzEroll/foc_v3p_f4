#ifndef FOC_CORE_H
#define FOC_CORE_H

#include "foc_types.h"
#include "phase_driver/phase_driver.h"
#include "sensors/current_sensor.h"
#include "sensors/rotor_sensor.h"

namespace foc_core
{
    foc_result link_rotor_sensor(rotor_sensor &sensor);
    foc_result link_current_sensor(current_sensor &sensor);
    foc_result link_phase_driver(phase_driver &driver);
    foc_result set_target(const foc_target &target);
    foc_result set_rotor_alignment(int8_t rotor_direction,
        float electrical_zero_offset_rad);
    foc_result begin_current_calibration(uint32_t sample_count);
    foc_result finish_current_calibration();
    bool current_calibration_complete();
    foc_result enable();
    void disable();
    foc_result clear_fault();
    foc_result run_control_from_isr(uint32_t timestamp_us);
    foc_result update_bus_sensors();
    foc_result update_safety(uint32_t timestamp_ms);
    bool peek_snapshot(foc_snapshot &snapshot);
    foc_result init(const foc_config &config);
}

#endif

#ifndef FOC_RUNTIME_H
#define FOC_RUNTIME_H

#include "foc_internal_types.h"

namespace foc
{
    namespace internal
    {
        // 以下接口只供设备层、任务和 ADC 同步 ISR 调度使用。
        foc_result bind_hardware(const foc_hardware &hardware);
        foc_result set_command(const foc_command &command);
        foc_result set_rotor_alignment(int8_t rotor_direction,
            float electrical_zero_offset_rad);
        foc_result calibrate_current_task(uint32_t sample_count);
        foc_result run_control_from_isr(uint32_t timestamp_us);
        foc_result update_sensors();
        foc_result update_safety(uint32_t timestamp_ms);
    }
}

#endif

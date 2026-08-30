#ifndef FOC_RUNTIME_H
#define FOC_RUNTIME_H

#include "foc_types.h"

namespace foc
{
    namespace runtime
    {
        // 以下接口由 RTOS 任务和 ADC 同步 ISR 调用，不属于业务控制面。
        foc_result update_sensors();
        foc_result update_safety(uint32_t timestamp_ms);
        foc_result run_control_from_isr(uint32_t timestamp_us);
    }
}

#endif

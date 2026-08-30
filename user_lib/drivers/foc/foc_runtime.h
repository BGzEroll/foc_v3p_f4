#ifndef FOC_RUNTIME_H
#define FOC_RUNTIME_H

#include "foc_types.h"

namespace foc
{
    class instance;

    namespace runtime
    {
        // 以下接口由 RTOS 任务和 ADC 同步 ISR 调用，不属于业务控制面。
        foc_result update_sensors(instance &motor);
        foc_result update_safety(instance &motor,
            uint32_t timestampMs);
        foc_result run_control_from_isr(instance &motor,
            uint32_t timestampUs);
    }
}

#endif

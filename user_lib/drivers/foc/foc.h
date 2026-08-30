#ifndef FOC_H
#define FOC_H

#include "foc_hardware.h"
#include "foc_types.h"

namespace foc
{
    // 普通业务只提交闭环电流/电压目标；开环 commissioning 由 internal API 驱动。
    foc_result set_target(const foc_target &target);
    foc_result enable();
    void disable();
    foc_result clear_fault();
    bool peek_snapshot(foc_snapshot &snapshot);
    foc_result init(const foc_config &config,
        const foc_hardware &hardware);
}

#endif

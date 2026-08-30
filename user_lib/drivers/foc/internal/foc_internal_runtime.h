#ifndef FOC_INTERNAL_RUNTIME_H
#define FOC_INTERNAL_RUNTIME_H

#include "foc_internal_types.h"

namespace foc
{
    class instance;

    namespace internal
    {
        foc_result set_command(instance &motor,
            const foc_command &command);
        foc_result get_pole_pairs(const instance &motor,
            uint8_t &polePairs);
        foc_result set_rotor_alignment(instance &motor,
            int8_t rotorDirection,
            float electricalZeroOffsetRad);
        foc_result calibrate_current_task(instance &motor,
            uint32_t sampleCount);
        foc_result set_current_directions_task(instance &motor,
            int8_t directionA,
            int8_t directionB);
    }
}

#endif

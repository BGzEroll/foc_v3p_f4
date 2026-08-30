#ifndef FOC_INTERNAL_RUNTIME_H
#define FOC_INTERNAL_RUNTIME_H

#include "foc_internal_types.h"

namespace foc
{
    namespace internal
    {
        foc_result set_command(const foc_command &command);
        foc_result get_pole_pairs(uint8_t &pole_pairs);
        foc_result set_rotor_alignment(int8_t rotor_direction,
            float electrical_zero_offset_rad);
        foc_result calibrate_current_task(uint32_t sample_count);
        foc_result set_current_directions_task(int8_t direction_a,
            int8_t direction_b);
    }
}

#endif

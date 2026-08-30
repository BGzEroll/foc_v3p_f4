#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H

#include "../foc_types.h"

class current_sensor
{
    public:
        virtual ~current_sensor() = default;

    public:
        virtual foc_result init() = 0;

    public:
        virtual foc_result calibrate_task(uint32_t sample_count) = 0;
        virtual foc_result read_conversion_from_isr(
            uint32_t timestamp_us,
            phase_current_sample &sample) = 0;
};

#endif

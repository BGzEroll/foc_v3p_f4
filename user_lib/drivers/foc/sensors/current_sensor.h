#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H

#include "drivers/foc/foc_types.h"

class current_sensor
{
    public:
        virtual ~current_sensor() = default;

    public:
        virtual foc_result init() = 0;

    public:
        virtual foc_result begin_calibration_task(
            uint32_t sample_count) = 0;
        virtual foc_result finish_calibration_task() = 0;
        virtual bool calibration_complete_task() const = 0;
        virtual foc_result read_conversion_from_isr(
            uint32_t timestamp_us,
            phase_current_sample &sample) = 0;
};

#endif

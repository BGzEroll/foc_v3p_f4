#ifndef ROTOR_SENSOR_H
#define ROTOR_SENSOR_H

#include "../foc_types.h"

class rotor_sensor
{
    public:
        virtual ~rotor_sensor() = default;

    public:
        virtual foc_result init() = 0;

    public:
        virtual foc_result update_task() = 0;
        virtual foc_result read_task(rotor_sample &sample) = 0;
        virtual foc_result read_from_isr(rotor_sample &sample) = 0;
        virtual uint32_t communication_error_count() const = 0;
};

#endif

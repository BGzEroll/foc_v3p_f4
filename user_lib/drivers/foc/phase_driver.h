#ifndef PHASE_DRIVER_H
#define PHASE_DRIVER_H

#include "foc_types.h"

class phase_driver
{
    public:
        virtual ~phase_driver() = default;

    public:
        virtual foc_result init() = 0;

    public:
        virtual foc_result enable_output_task() = 0;
        virtual void disable_output() = 0;
        virtual foc_result write_duty_from_isr(
            const phase_duty &duty) = 0;
        virtual bool fault_active_from_isr() const = 0;
};

#endif

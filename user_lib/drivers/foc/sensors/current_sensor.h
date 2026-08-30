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
        // 非所有电流传感器都支持运行时极性调整，默认返回不支持。
        virtual foc_result set_directions_task(int8_t direction_a,
            int8_t direction_b)
        {
            return foc_result::INVALID_STATE;
        }
        virtual foc_result read_conversion_from_isr(
            uint32_t timestamp_us,
            phase_current_sample &sample) = 0;
};

#endif

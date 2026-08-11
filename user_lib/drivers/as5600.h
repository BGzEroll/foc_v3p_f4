#ifndef AS5600_H
#define AS5600_H

#include "bus/i2c_bus.h"
#include <stdint.h>

struct as5600_sample
{
    uint32_t sequence = 0U;
    uint32_t timestamp_ms = 0U;
    uint16_t raw_count = 0U;
    float angle_rad = 0.0f;
    float full_angle_rad = 0.0f;
    float angular_velocity_rad_s = 0.0f;
};

class as5600
{
    public:
        as5600(uint8_t i2c_bus_id, uint8_t device_address);

    public:
        i2c_result init();

    public:
        i2c_result update();
        const as5600_sample &sample() const;

    private:
        void process_raw_angle(uint16_t raw_count, uint32_t update_tick);

    private:
        i2c_bus i2c;
        uint8_t device_address;
        as5600_sample current_sample{};
        int32_t accumulated_count = 0;
        uint16_t previous_count = 0U;
        uint32_t previous_update_tick = 0U;
        bool initialized = false;
        bool first_sample = true;
};

#endif

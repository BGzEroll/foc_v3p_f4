#ifndef AS5600_ROTOR_SENSOR_H
#define AS5600_ROTOR_SENSOR_H

#include "../../../bus/i2c_bus.h"
#include "../rotor_sensor.h"
#include "system/topic.h"

class as5600_rotor_sensor : public rotor_sensor
{
    public:
        as5600_rotor_sensor(uint8_t i2c_bus_id,
            uint8_t device_address);

    public:
        foc_result init() override;

    public:
        foc_result update_task() override;
        foc_result read_task(rotor_sample &sample) override;
        foc_result read_from_isr(rotor_sample &sample) override;
        uint32_t communication_error_count() const override;

    private:
        foc_result read_and_publish_sample();
        void process_raw_angle(uint16_t raw_count,
            uint32_t timestamp_us,
            rotor_sample &sample);

    private:
        i2c_bus i2c;
        uint8_t device_address;
        topic::latest_topic<rotor_sample> sample_topic;
        int32_t accumulated_count = 0;
        uint16_t previous_count = 0U;
        uint32_t previous_timestamp_us = 0U;
        uint32_t sequence = 0U;
        uint32_t error_count = 0U;
        bool initialized = false;
        bool first_sample = true;
};

#endif

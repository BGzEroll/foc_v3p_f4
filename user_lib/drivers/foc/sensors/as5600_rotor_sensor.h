#ifndef AS5600_ROTOR_SENSOR_H
#define AS5600_ROTOR_SENSOR_H

#include "drivers/as5600.h"
#include "drivers/foc/sensors/rotor_sensor.h"
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
        foc_result publish_current_sample();

    private:
        as5600 encoder;
        topic::latest_topic<rotor_sample> sample_topic;
        uint32_t error_count = 0U;
        bool initialized = false;
};

#endif

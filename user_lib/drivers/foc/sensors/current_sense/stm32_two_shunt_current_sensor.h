#ifndef STM32_TWO_SHUNT_CURRENT_SENSOR_H
#define STM32_TWO_SHUNT_CURRENT_SENSOR_H

#include "../current_sensor.h"
#include "stm32f4xx_hal.h"

struct stm32_two_shunt_current_config
{
    ADC_HandleTypeDef *adc = nullptr;
    float ampere_per_count_a = 0.0f;
    float ampere_per_count_b = 0.0f;
    int8_t direction_a = 1;
    int8_t direction_b = 1;
};

class stm32_two_shunt_current_sensor : public current_sensor
{
    public:
        explicit stm32_two_shunt_current_sensor(
            const stm32_two_shunt_current_config &config);

    public:
        foc_result init() override;

    public:
        foc_result calibrate_task(uint32_t sample_count) override;
        foc_result set_directions_task(int8_t direction_a,
            int8_t direction_b) override;
        foc_result read_conversion_from_isr(uint32_t timestamp_us,
            phase_current_sample &sample) override;

    private:
        stm32_two_shunt_current_config config;
        uint64_t calibration_sum_a = 0;
        uint64_t calibration_sum_b = 0;
        uint32_t calibration_target = 0;
        volatile uint32_t calibration_collected = 0;
        float offset_a = 0.0f;
        float offset_b = 0.0f;
        uint32_t sequence = 0;
        bool initialized = false;
        volatile bool calibrating = false;
        bool calibrated = false;
};

#endif

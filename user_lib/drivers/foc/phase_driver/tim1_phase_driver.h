#ifndef TIM1_PHASE_DRIVER_H
#define TIM1_PHASE_DRIVER_H

#include "phase_driver.h"
#include "stm32f4xx_hal.h"

struct tim1_phase_driver_config
{
    TIM_HandleTypeDef *timer = nullptr;
    GPIO_TypeDef *enable_port = nullptr;
    uint16_t enable_pin = 0;
    uint8_t phase_a_channel = 1;
    uint8_t phase_b_channel = 2;
    uint8_t phase_c_channel = 3;
    bool enable_active_high = true;
    bool allow_output = false;
};

class tim1_phase_driver : public phase_driver
{
    public:
        explicit tim1_phase_driver(
            const tim1_phase_driver_config &config);

    public:
        foc_result init() override;

    public:
        foc_result enable_output_task() override;
        void disable_output() override;
        foc_result write_duty_from_isr(const phase_duty &duty) override;
        bool fault_active_from_isr() const override;

    private:
        bool channels_valid() const;
        void write_compare(uint8_t channel, uint32_t compare);
        void write_enable_pin(bool enabled);
        void write_neutral_duty();

    private:
        tim1_phase_driver_config config;
        bool initialized = false;
        bool pwm_started = false;
        bool output_enabled = false;
};

#endif

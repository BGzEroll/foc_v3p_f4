#ifndef STM32_TIM1_PHASE_DRIVER_H
#define STM32_TIM1_PHASE_DRIVER_H

#include "drivers/foc/phase_driver.h"
#include "stm32f4xx_hal.h"

struct stm32_tim1_phase_driver_config
{
    TIM_HandleTypeDef *timer = nullptr;
    GPIO_TypeDef *enable_port = nullptr;
    uint16_t enable_pin = 0U;
    bool enable_active_high = true;
    bool allow_output = false;
};

class stm32_tim1_phase_driver : public phase_driver
{
    public:
        explicit stm32_tim1_phase_driver(
            const stm32_tim1_phase_driver_config &config);

    public:
        foc_result init() override;

    public:
        foc_result enable_output_task() override;
        void disable_output() override;
        foc_result write_duty_from_isr(const phase_duty &duty) override;
        bool fault_active_from_isr() const override;

    private:
        void write_enable_pin(bool enabled);
        void write_neutral_duty();

    private:
        stm32_tim1_phase_driver_config config;
        bool initialized = false;
        bool output_enabled = false;
};

#endif

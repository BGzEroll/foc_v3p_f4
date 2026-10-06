#ifndef PLL_EXPERIMENT_H
#define PLL_EXPERIMENT_H
#include "bemf_pll.h"
#ifdef __cplusplus
extern "C" {
#endif
/* SWD-visible experiment; NEVER set active directly. request=1 asks a guarded
 * encoder->sensorless trial. request=2 stops output. Active trials last 10 s.
 * capture_request=1 records 512 rows at 2 kHz without UART in the ISR. */
typedef struct {
    volatile uint32_t request, active, fault, capture_request, capture_count;
    volatile uint32_t samples, last_period_us, max_step_cycles, max_loop_cycles;
    volatile uint32_t encoder_fault_age_us, encoder_fault_sample_us, encoder_fault_now_us;
    volatile uint32_t current_mapping;
    volatile uint32_t missed_periods, max_period_us;
    uint32_t previous_mapping;
    uint32_t previous_us, capture_divider, switch_good, active_samples;
    uint32_t max_switch_good;
    float reference_angle, reference_speed, angle_error;
    float startup_encoder_delta_rad, startup_max_phase_a;
    float startup_reverse_delta_rad, startup_zero_spread_rad;
    float encoder_target_speed_rad_s, encoder_current_limit_a, encoder_iq_command_a;
    bemf_pll estimator;
    float capture[512][16];
} pll_experiment_state;
extern pll_experiment_state pll_experiment;
void pll_experiment_init(void);
void pll_experiment_step(float va, float vb, float ia, float ib,
                         float reference_angle, float reference_speed,
                         float bus_voltage, uint32_t timestamp_us);
/* Next-cycle predicted electrical feedback. No encoder is used here. */
int pll_experiment_feedback(float *angle, float *speed);
#ifdef __cplusplus
}
#endif
#endif

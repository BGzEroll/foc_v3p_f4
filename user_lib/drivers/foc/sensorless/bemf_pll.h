#ifndef BEMF_PLL_H
#define BEMF_PLL_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
/* SPMSM, amplitude-invariant Clarke; SI units; electrical radians/second.
 * v[n] is the actual average voltage for interval [n,n+1), i[n] its start.
 * Direction is supplied by startup (+1/-1); it is not inferred at standstill.
 * Single owner: call reset/init and step from the same execution context. */
typedef struct {
    float dt_s, resistance_ohm, inductance_h;
    float observer_wn_rad_s, observer_zeta;
    float pll_wn_rad_s, pll_zeta;
    float min_emf_v, max_speed_rad_s, lock_error_rad, lock_time_s;
    float min_speed_rad_s;
} bemf_pll_config;
typedef struct {
    bemf_pll_config config;
    float current_hat_a, current_hat_b, emf_state_a, emf_state_b;
    float emf_alpha_v, emf_beta_v, emf_magnitude_v;
    float angle_rad, speed_rad_s, phase_error_rad;
    float phase_error_energy; // 10 ms EMA of squared detector error, rad^2.
    float observer_k_i, observer_k_e, pll_kp, pll_ki;
    uint32_t good_samples, required_samples, invalid_samples;
    int8_t direction;
    uint8_t configured, seeded, valid, locked;
} bemf_pll;
bemf_pll_config bemf_pll_default_config(void);
int bemf_pll_init(bemf_pll *s, const bemf_pll_config *config, int direction);
void bemf_pll_reset(bemf_pll *s, int direction);
/* Returns 1 for a finite accepted sample (not a claim of lock), 0 otherwise.
 * On rejected samples all dynamic state is reset and lock is cleared. */
int bemf_pll_step(bemf_pll *s, float v_alpha, float v_beta,
                  float i_alpha, float i_beta);
float bemf_pll_wrap(float angle);
#ifdef __cplusplus
}
#endif
#endif

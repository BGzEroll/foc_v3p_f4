#include "bemf_pll.h"
#include <math.h>
#include <string.h>
#define PI_F 3.14159265358979323846f
#define TAU_F (2.0f * PI_F)
float bemf_pll_wrap(float a) {
    a = fmodf(a + PI_F, TAU_F);
    if (a < 0.0f) a += TAU_F;
    return a - PI_F;
}
static float clamp(float a, float lo, float hi) {
    return a < lo ? lo : (a > hi ? hi : a);
}
bemf_pll_config bemf_pll_default_config(void) {
    bemf_pll_config c = {0.00005f, 2.55f, 0.00086f, 6000.0f, 1.0f,
                        200.0f, 0.70710678f, 0.08f, 2000.0f, 0.15f, 0.05f, 30.0f};
    return c;
}
void bemf_pll_reset(bemf_pll *s, int direction) {
    if (!s) return;
    bemf_pll_config c = s->config;
    uint8_t configured = s->configured;
    uint32_t invalid = s->invalid_samples;
    memset(s, 0, sizeof(*s));
    s->config = c; s->configured = configured; s->invalid_samples = invalid;
    s->direction = direction == -1 ? -1 : (direction == 1 ? 1 : 0);
    if (!configured) return;
    s->observer_k_i = 2.0f*c.observer_zeta*c.observer_wn_rad_s - c.resistance_ohm/c.inductance_h;
    s->observer_k_e = c.inductance_h*c.observer_wn_rad_s*c.observer_wn_rad_s;
    s->pll_kp = 2.0f*c.pll_zeta*c.pll_wn_rad_s;
    s->pll_ki = c.pll_wn_rad_s*c.pll_wn_rad_s;
    s->required_samples = configured ? (uint32_t)ceilf(c.lock_time_s/c.dt_s) : 0;
}
int bemf_pll_init(bemf_pll *s, const bemf_pll_config *c, int direction) {
    if (!s) return 0;
    memset(s, 0, sizeof(*s));
    if (!c || (direction != 1 && direction != -1)) return 0;
    if (!isfinite(c->dt_s) || !isfinite(c->resistance_ohm) || !isfinite(c->inductance_h) ||
        !isfinite(c->observer_wn_rad_s) || !isfinite(c->observer_zeta) ||
        !isfinite(c->pll_wn_rad_s) || !isfinite(c->pll_zeta) || !isfinite(c->min_emf_v) ||
        !isfinite(c->max_speed_rad_s) || !isfinite(c->lock_error_rad) || !isfinite(c->lock_time_s) ||
        !isfinite(c->min_speed_rad_s) || c->min_speed_rad_s <= 0 || c->min_speed_rad_s >= c->max_speed_rad_s ||
        c->dt_s < 1e-7f || c->resistance_ohm <= 0 || c->inductance_h <= 0 ||
        c->observer_wn_rad_s <= 0 || c->observer_zeta < 0.5f || c->observer_zeta > 1.0f ||
        c->pll_wn_rad_s <= 0 || c->pll_zeta < 0.5f || c->pll_zeta > 1.0f ||
        c->dt_s*c->observer_wn_rad_s >= 0.5f || c->dt_s*c->pll_wn_rad_s >= 0.1f ||
        c->dt_s*c->resistance_ohm/c->inductance_h >= 0.5f ||
        c->min_emf_v <= 0 || c->max_speed_rad_s <= 0 ||
        c->dt_s*c->max_speed_rad_s >= 0.2f || c->lock_error_rad <= 0 ||
        c->lock_error_rad >= PI_F/2 || c->lock_time_s < c->dt_s || c->lock_time_s > 10.0f) return 0;
    s->config = *c; s->configured = 1;
    bemf_pll_reset(s, direction);
    return 1;
}
/* Forward Euler Luenberger observer. No numerical differentiation of current.
 * Each axis: di_hat=(v-R*i_hat-e_hat)/L+k_i*(i-i_hat);
 *           de_hat=-k_e*(i-i_hat).
 * Error characteristic: s^2+2*zeta*wn*s+wn^2. */
int bemf_pll_step(bemf_pll *s, float va, float vb, float ia, float ib) {
    if (!s || !s->configured) return 0;
    if (s->direction == 0 || !isfinite(va) || !isfinite(vb) || !isfinite(ia) || !isfinite(ib)) {
        s->invalid_samples++; bemf_pll_reset(s, s->direction); return 0;
    }
    const bemf_pll_config *c = &s->config;
    s->emf_alpha_v = s->emf_state_a; s->emf_beta_v = s->emf_state_b;
    s->emf_magnitude_v = hypotf(s->emf_alpha_v, s->emf_beta_v);
    if (s->emf_magnitude_v >= c->min_emf_v) {
        /* e=omega*flux*[-sin(theta),cos(theta)]. Reverse needs a pi correction. */
        float phase = atan2f(-s->direction*s->emf_alpha_v, s->direction*s->emf_beta_v);
        /* Exact phase compensation of this Euler observer's steady sine response:
         * H(z)=wn^2/(sd^2+2*zeta*wn*sd+wn^2), sd=(z-1)/dt. */
        float wd = s->speed_rad_s*c->dt_s;
        float sr = (cosf(wd)-1.0f)/c->dt_s, si = sinf(wd)/c->dt_s;
        float a = 2.0f*c->observer_zeta*c->observer_wn_rad_s;
        float b = c->observer_wn_rad_s*c->observer_wn_rad_s;
        phase = bemf_pll_wrap(phase + atan2f(2.0f*sr*si+a*si, sr*sr-si*si+a*sr+b));
        if (!s->seeded) { s->angle_rad = phase; s->seeded = 1; }
        float predicted = bemf_pll_wrap(s->angle_rad+s->speed_rad_s*c->dt_s);
        s->phase_error_rad = bemf_pll_wrap(phase-predicted);
        float candidate = s->speed_rad_s+s->pll_ki*c->dt_s*s->phase_error_rad;
        s->speed_rad_s = clamp(candidate, -c->max_speed_rad_s, c->max_speed_rad_s);
        s->angle_rad = bemf_pll_wrap(predicted+s->pll_kp*c->dt_s*s->phase_error_rad);
        s->valid = 1;
        if (fabsf(s->phase_error_rad) < c->lock_error_rad &&
            s->direction*s->speed_rad_s >= c->min_speed_rad_s && fabsf(candidate) < c->max_speed_rad_s) {
            if (s->good_samples < s->required_samples) s->good_samples++;
        } else s->good_samples = 0;
        s->locked = s->good_samples >= s->required_samples;
    } else {
        /* Do not advertise a stale speed at zero/back-EMF dropout. */
        s->valid = s->locked = s->seeded = 0; s->good_samples = 0;
        s->speed_rad_s = 0; s->phase_error_rad = 0;
    }
    float ra = ia-s->current_hat_a, rb = ib-s->current_hat_b;
    float next_a = s->current_hat_a+c->dt_s*((va-c->resistance_ohm*s->current_hat_a-s->emf_state_a)/c->inductance_h+s->observer_k_i*ra);
    float next_b = s->current_hat_b+c->dt_s*((vb-c->resistance_ohm*s->current_hat_b-s->emf_state_b)/c->inductance_h+s->observer_k_i*rb);
    s->emf_state_a -= c->dt_s*s->observer_k_e*ra;
    s->emf_state_b -= c->dt_s*s->observer_k_e*rb;
    s->current_hat_a = next_a; s->current_hat_b = next_b;
    if (!isfinite(next_a) || !isfinite(next_b) || !isfinite(s->emf_state_a) || !isfinite(s->emf_state_b)) {
        s->invalid_samples++; bemf_pll_reset(s, s->direction); return 0;
    }
    return 1;
}

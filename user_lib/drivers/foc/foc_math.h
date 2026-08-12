#ifndef FOC_MATH_H
#define FOC_MATH_H

#include "drivers/foc/foc_types.h"

struct alpha_beta_value
{
    float alpha = 0.0f;
    float beta = 0.0f;
};

struct d_q_value
{
    float d = 0.0f;
    float q = 0.0f;
};

struct pi_state
{
    float integral = 0.0f;
};

namespace foc_math
{
    float normalize_angle(float angle_rad);
    alpha_beta_value clarke_transform(
        const phase_current_sample &current);
    d_q_value park_transform(const alpha_beta_value &stationary,
        float electrical_angle_rad);
    alpha_beta_value inverse_park_transform(const d_q_value &rotating,
        float electrical_angle_rad);
    float run_pi(float error,
        float period_s,
        const pi_config &config,
        float output_limit,
        pi_state &state);
    void reset_pi(pi_state &state);
    void limit_vector(d_q_value &vector, float magnitude_limit);
    bool calculate_svpwm(const alpha_beta_value &voltage,
        float bus_voltage_v,
        phase_duty &duty);
}

#endif

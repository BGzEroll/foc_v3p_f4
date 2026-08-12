#include "drivers/foc/foc_math.h"

#include <math.h>
#include <stdio.h>

static constexpr float TEST_TOLERANCE = 1.0e-5f;

/**
 * @brief 检查两个浮点值是否在测试误差范围内
 *
 * @param actual 实际结果
 * @param expected 期望结果
 *
 * @return 误差满足要求时返回 true
 */
static bool near(float actual, float expected)
{
    return fabsf(actual - expected) <= TEST_TOLERANCE;
}

/**
 * @brief 报告失败的测试条件
 *
 * @param condition 测试条件
 * @param name 测试名称
 *
 * @return 条件成立时返回零，否则返回一
 */
static uint32_t expect(bool condition, const char *name)
{
    if(condition){return 0U;}
    printf("FAIL: %s\n", name);
    return 1U;
}

/**
 * @brief 运行 FOC 纯数学主机测试
 *
 * @return 所有测试通过时返回零
 */
int main()
{
    uint32_t failure_count = 0U;

    failure_count += expect(near(foc_math::normalize_angle(-0.5f),
        5.783185307179586f), "normalize negative angle");

    phase_current_sample current{};
    current.current_a = 1.0f;
    current.current_b = -0.5f;
    current.current_c = -0.5f;
    alpha_beta_value stationary = foc_math::clarke_transform(current);
    failure_count += expect(near(stationary.alpha, 1.0f),
        "clarke alpha");
    failure_count += expect(near(stationary.beta, 0.0f),
        "clarke beta");

    d_q_value rotating = foc_math::park_transform(stationary, 0.7f);
    alpha_beta_value reconstructed = foc_math::inverse_park_transform(
        rotating,
        0.7f);
    failure_count += expect(near(reconstructed.alpha, stationary.alpha),
        "park round trip alpha");
    failure_count += expect(near(reconstructed.beta, stationary.beta),
        "park round trip beta");

    pi_config controller_config{};
    controller_config.proportional_gain = 2.0f;
    controller_config.integral_gain = 5.0f;
    controller_config.integral_limit = 0.25f;
    pi_state controller_state{};
    float output = foc_math::run_pi(10.0f,
        0.001f,
        controller_config,
        1.0f,
        controller_state);
    failure_count += expect(near(output, 1.0f), "pi output saturation");
    failure_count += expect(near(controller_state.integral, 0.0f),
        "pi anti windup");

    phase_duty duty{};
    alpha_beta_value zero_voltage{};
    failure_count += expect(foc_math::calculate_svpwm(zero_voltage,
        24.0f,
        duty), "zero voltage svpwm valid");
    failure_count += expect(near(duty.phase_a, 0.5f) &&
        near(duty.phase_b, 0.5f) && near(duty.phase_c, 0.5f),
        "zero voltage neutral duty");

    alpha_beta_value boundary_voltage{};
    boundary_voltage.alpha = 12.0f;
    failure_count += expect(foc_math::calculate_svpwm(boundary_voltage,
        24.0f,
        duty), "boundary svpwm valid");
    failure_count += expect(duty.phase_a >= 0.0f && duty.phase_a <= 1.0f &&
        duty.phase_b >= 0.0f && duty.phase_b <= 1.0f &&
        duty.phase_c >= 0.0f && duty.phase_c <= 1.0f,
        "boundary duty range");

    if(failure_count == 0U)
    {
        printf("FOC math tests passed\n");
    }

    return failure_count == 0U ? 0 : 1;
}

#include "foc_math.h"

#include <math.h>

static constexpr float TWO_PI = 6.28318530717958647692f;
static constexpr float ONE_OVER_SQRT_THREE = 0.57735026918962576451f;
static constexpr float SQRT_THREE_OVER_TWO = 0.86602540378443864676f;

/**
 * @brief 把浮点值限制在对称区间内
 *
 * @param value 待限制的数值
 * @param limit 对称限幅绝对值
 *
 * @return 限幅后的数值
 */
static float clamp_symmetric(float value, float limit)
{
    if(value > limit){return limit;}
    if(value < -limit){return -limit;}
    return value;
}

/**
 * @brief 把角度归一化到零至二倍圆周率
 *
 * @param angle_rad 输入角度，单位弧度
 *
 * @return 归一化角度，单位弧度
 */
float foc_math::normalize_angle(float angle_rad)
{
    if(!isfinite(angle_rad)){return 0.0f;}

    float normalized = fmodf(angle_rad, TWO_PI);
    if(normalized < 0.0f)
    {
        normalized += TWO_PI;
    }

    return normalized;
}

/**
 * @brief 把三相电流变换到静止 Alpha-Beta 坐标系
 *
 * @param current 三相电流样本
 *
 * @return 静止坐标系电流
 */
alpha_beta_value foc_math::clarke_transform(
    const phase_current_sample &current)
{
    alpha_beta_value result{};
    result.alpha = current.current_a;
    result.beta = (current.current_a + 2.0f * current.current_b) *
        ONE_OVER_SQRT_THREE;
    return result;
}

/**
 * @brief 把静止坐标系矢量变换到旋转 D-Q 坐标系
 *
 * @param stationary 静止坐标系矢量
 * @param electrical_angle_rad 电角度，单位弧度
 *
 * @return D-Q 坐标系矢量
 */
d_q_value foc_math::park_transform(const alpha_beta_value &stationary,
    float electrical_angle_rad)
{
    float sine = sinf(electrical_angle_rad);
    float cosine = cosf(electrical_angle_rad);

    d_q_value result{};
    result.d = stationary.alpha * cosine + stationary.beta * sine;
    result.q = -stationary.alpha * sine + stationary.beta * cosine;
    return result;
}

/**
 * @brief 把 D-Q 坐标系矢量变换到静止 Alpha-Beta 坐标系
 *
 * @param rotating D-Q 坐标系矢量
 * @param electrical_angle_rad 电角度，单位弧度
 *
 * @return 静止坐标系矢量
 */
alpha_beta_value foc_math::inverse_park_transform(
    const d_q_value &rotating,
    float electrical_angle_rad)
{
    float sine = sinf(electrical_angle_rad);
    float cosine = cosf(electrical_angle_rad);

    alpha_beta_value result{};
    result.alpha = rotating.d * cosine - rotating.q * sine;
    result.beta = rotating.d * sine + rotating.q * cosine;
    return result;
}

/**
 * @brief 运行一次带积分限幅和条件积分的 PI 控制器
 *
 * @param error 本周期控制误差
 * @param period_s 控制周期，单位秒
 * @param config PI 参数
 * @param output_limit 输出绝对值限制
 * @param state PI 持续状态
 *
 * @return 本周期 PI 输出
 */
float foc_math::run_pi(float error,
    float period_s,
    const pi_config &config,
    float output_limit,
    pi_state &state)
{
    if(!isfinite(error) || !isfinite(period_s) || period_s <= 0.0f ||
        !isfinite(output_limit) || output_limit <= 0.0f)
    {
        return 0.0f;
    }

    float integral_candidate = state.integral +
        config.integral_gain * error * period_s;
    integral_candidate = clamp_symmetric(integral_candidate,
        config.integral_limit);

    float proportional = config.proportional_gain * error;
    float unsaturated = proportional + integral_candidate;
    float output = clamp_symmetric(unsaturated, output_limit);
    bool saturated_high = unsaturated > output_limit;
    bool saturated_low = unsaturated < -output_limit;
    bool drives_out_of_saturation =
        (saturated_high && error < 0.0f) ||
        (saturated_low && error > 0.0f);

    if((!saturated_high && !saturated_low) || drives_out_of_saturation)
    {
        state.integral = integral_candidate;
    }

    return output;
}

/**
 * @brief 清零 PI 积分状态
 *
 * @param state PI 持续状态
 */
void foc_math::reset_pi(pi_state &state)
{
    state.integral = 0.0f;
}

/**
 * @brief 按矢量幅值限制 D-Q 分量
 *
 * @param vector 待限制的 D-Q 矢量
 * @param magnitude_limit 幅值上限
 */
void foc_math::limit_vector(d_q_value &vector, float magnitude_limit)
{
    if(!isfinite(vector.d) || !isfinite(vector.q) ||
        !isfinite(magnitude_limit) || magnitude_limit <= 0.0f)
    {
        vector = {};
        return;
    }

    float magnitude_squared = vector.d * vector.d + vector.q * vector.q;
    float limit_squared = magnitude_limit * magnitude_limit;
    if(magnitude_squared <= limit_squared){return;}

    float scale = magnitude_limit / sqrtf(magnitude_squared);
    vector.d *= scale;
    vector.q *= scale;
}

/**
 * @brief 使用零序注入计算三相中心对齐 PWM 占空比
 *
 * @param voltage 静止坐标系电压矢量
 * @param bus_voltage_v 母线电压，单位伏特
 * @param duty 用于接收三相占空比
 *
 * @return 输入有效且占空比位于零至一时返回 true
 */
bool foc_math::calculate_svpwm(const alpha_beta_value &voltage,
    float bus_voltage_v,
    phase_duty &duty)
{
    if(!isfinite(voltage.alpha) || !isfinite(voltage.beta) ||
        !isfinite(bus_voltage_v) || bus_voltage_v <= 0.0f)
    {
        duty = {};
        return false;
    }

    float phase_a_voltage = voltage.alpha;
    float phase_b_voltage = -0.5f * voltage.alpha +
        SQRT_THREE_OVER_TWO * voltage.beta;
    float phase_c_voltage = -0.5f * voltage.alpha -
        SQRT_THREE_OVER_TWO * voltage.beta;
    float maximum = fmaxf(phase_a_voltage,
        fmaxf(phase_b_voltage, phase_c_voltage));
    float minimum = fminf(phase_a_voltage,
        fminf(phase_b_voltage, phase_c_voltage));
    float common_mode = -0.5f * (maximum + minimum);

    duty.phase_a = 0.5f + (phase_a_voltage + common_mode) /
        bus_voltage_v;
    duty.phase_b = 0.5f + (phase_b_voltage + common_mode) /
        bus_voltage_v;
    duty.phase_c = 0.5f + (phase_c_voltage + common_mode) /
        bus_voltage_v;

    return isfinite(duty.phase_a) && isfinite(duty.phase_b) &&
        isfinite(duty.phase_c) && duty.phase_a >= 0.0f &&
        duty.phase_a <= 1.0f && duty.phase_b >= 0.0f &&
        duty.phase_b <= 1.0f && duty.phase_c >= 0.0f &&
        duty.phase_c <= 1.0f;
}

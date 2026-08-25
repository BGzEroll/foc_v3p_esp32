#include "foc_math.h"

#include <math.h>

static constexpr float TWO_PI = 6.28318530717958647692f;
static constexpr float TWO_OVER_THREE = 0.66666666666666666667f;
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
 * @brief 把浮点值限制在零至一之间
 *
 * @param value 待限制的数值
 *
 * @return 限幅后的数值
 */
static float clamp_duty(float value)
{
    if(value > 1.0f){return 1.0f;}
    if(value < 0.0f){return 0.0f;}
    return value;
}

/**
 * @brief 把角度归一化到零至二倍圆周率
 *
 * @param angle_rad 输入角度，单位弧度
 *
 * @return 归一化角度；输入无效时返回零
 */
float foc_math::normalize_angle(float angle_rad)
{
    if(!isfinite(angle_rad)){return 0.0f;}

    float normalized_angle_rad = fmodf(angle_rad, TWO_PI);
    if(normalized_angle_rad < 0.0f)
    {
        normalized_angle_rad += TWO_PI;
    }

    return normalized_angle_rad;
}

/**
 * @brief 把三相电流变换到静止 Alpha-Beta 坐标系
 *
 * @param current 三相电流样本，单位安培
 *
 * @return 静止坐标系电流，单位安培
 */
alpha_beta_current foc_math::clarke(const phase_current_sample &current)
{
    alpha_beta_current result{};
    result.alpha_a = TWO_OVER_THREE *
        (current.phase_a_a - 0.5f * current.phase_b_a -
            0.5f * current.phase_c_a);
    result.beta_a = TWO_OVER_THREE * SQRT_THREE_OVER_TWO *
        (current.phase_b_a - current.phase_c_a);
    return result;
}

/**
 * @brief 把静止坐标系电流变换到旋转 D-Q 坐标系
 *
 * @param current 静止坐标系电流，单位安培
 * @param electrical_angle_rad 电角度，单位弧度
 *
 * @return D-Q 坐标系电流，单位安培
 */
d_q_current foc_math::park(const alpha_beta_current &current,
    float electrical_angle_rad)
{
    float sine = sinf(electrical_angle_rad);
    float cosine = cosf(electrical_angle_rad);

    d_q_current result{};
    result.d_a = current.alpha_a * cosine + current.beta_a * sine;
    result.q_a = -current.alpha_a * sine + current.beta_a * cosine;
    return result;
}

/**
 * @brief 把 D-Q 坐标系电压变换到静止 Alpha-Beta 坐标系
 *
 * @param voltage D-Q 坐标系电压，单位伏特
 * @param electrical_angle_rad 电角度，单位弧度
 *
 * @return 静止坐标系电压，单位伏特
 */
alpha_beta_voltage foc_math::inverse_park(const d_q_voltage &voltage,
    float electrical_angle_rad)
{
    float sine = sinf(electrical_angle_rad);
    float cosine = cosf(electrical_angle_rad);

    alpha_beta_voltage result{};
    result.alpha_v = voltage.d_v * cosine - voltage.q_v * sine;
    result.beta_v = voltage.d_v * sine + voltage.q_v * cosine;
    return result;
}

/**
 * @brief 运行一次带积分限幅和输出限幅的 PI 控制器
 *
 * @param target 控制目标
 * @param measured 实际测量值
 * @param period_s 控制周期，单位秒
 * @param config PI 参数
 * @param output_limit_v 输出绝对值限制，单位伏特
 * @param integral PI 积分状态
 * @param output 用于接收本周期输出
 *
 * @return 输入和计算结果均有效时返回 true
 */
bool foc_math::run_pi(float target,
    float measured,
    float period_s,
    const pi_config &config,
    float output_limit_v,
    float &integral,
    float &output)
{
    if(!isfinite(target) || !isfinite(measured) ||
        !isfinite(period_s) || period_s <= 0.0f ||
        !isfinite(config.kp) || config.kp < 0.0f ||
        !isfinite(config.ki) || config.ki < 0.0f ||
        !isfinite(config.integral_limit) || config.integral_limit < 0.0f ||
        !isfinite(output_limit_v) || output_limit_v <= 0.0f ||
        !isfinite(integral))
    {
        integral = 0.0f;
        output = 0.0f;
        return false;
    }

    float error = target - measured;
    float integral_candidate = integral + config.ki * error * period_s;
    integral = clamp_symmetric(integral_candidate, config.integral_limit);
    output = clamp_symmetric(config.kp * error + integral,
        output_limit_v);

    if(!isfinite(integral) || !isfinite(output))
    {
        integral = 0.0f;
        output = 0.0f;
        return false;
    }

    return true;
}

/**
 * @brief 按矢量幅值限制 D-Q 电压
 *
 * @param voltage 待限制的 D-Q 电压，单位伏特
 * @param magnitude_limit_v 电压矢量幅值上限，单位伏特
 *
 * @return 输入和计算结果均有效时返回 true
 */
bool foc_math::limit_voltage(d_q_voltage &voltage,
    float magnitude_limit_v)
{
    if(!isfinite(voltage.d_v) || !isfinite(voltage.q_v) ||
        !isfinite(magnitude_limit_v) || magnitude_limit_v <= 0.0f)
    {
        voltage = {};
        return false;
    }

    float magnitude_squared_v2 = voltage.d_v * voltage.d_v +
        voltage.q_v * voltage.q_v;
    float limit_squared_v2 = magnitude_limit_v * magnitude_limit_v;
    if(magnitude_squared_v2 <= limit_squared_v2){return true;}

    float scale = magnitude_limit_v / sqrtf(magnitude_squared_v2);
    voltage.d_v *= scale;
    voltage.q_v *= scale;
    return isfinite(voltage.d_v) && isfinite(voltage.q_v);
}

/**
 * @brief 使用零序注入计算三相中心对齐 PWM 占空比
 *
 * @param voltage 静止坐标系电压，单位伏特
 * @param bus_voltage_v 母线电压，单位伏特
 * @param duty 用于接收三相占空比
 *
 * @return 输入有效且未发生占空比限幅时返回 true
 */
bool foc_math::svpwm(const alpha_beta_voltage &voltage,
    float bus_voltage_v,
    phase_duty &duty)
{
    if(!isfinite(voltage.alpha_v) || !isfinite(voltage.beta_v) ||
        !isfinite(bus_voltage_v) || bus_voltage_v <= 0.0f)
    {
        duty = {};
        return false;
    }

    float phase_a_voltage_v = voltage.alpha_v;
    float phase_b_voltage_v = -0.5f * voltage.alpha_v +
        SQRT_THREE_OVER_TWO * voltage.beta_v;
    float phase_c_voltage_v = -0.5f * voltage.alpha_v -
        SQRT_THREE_OVER_TWO * voltage.beta_v;
    float maximum_voltage_v = fmaxf(phase_a_voltage_v,
        fmaxf(phase_b_voltage_v, phase_c_voltage_v));
    float minimum_voltage_v = fminf(phase_a_voltage_v,
        fminf(phase_b_voltage_v, phase_c_voltage_v));
    float common_mode_voltage_v = -0.5f *
        (maximum_voltage_v + minimum_voltage_v);
    float phase_a_duty = 0.5f +
        (phase_a_voltage_v + common_mode_voltage_v) / bus_voltage_v;
    float phase_b_duty = 0.5f +
        (phase_b_voltage_v + common_mode_voltage_v) / bus_voltage_v;
    float phase_c_duty = 0.5f +
        (phase_c_voltage_v + common_mode_voltage_v) / bus_voltage_v;
    if(!isfinite(phase_a_duty) || !isfinite(phase_b_duty) ||
        !isfinite(phase_c_duty))
    {
        duty = {};
        return false;
    }

    bool output_in_range = phase_a_duty >= 0.0f && phase_a_duty <= 1.0f &&
        phase_b_duty >= 0.0f && phase_b_duty <= 1.0f &&
        phase_c_duty >= 0.0f && phase_c_duty <= 1.0f;

    duty.phase_a = clamp_duty(phase_a_duty);
    duty.phase_b = clamp_duty(phase_b_duty);
    duty.phase_c = clamp_duty(phase_c_duty);
    return output_in_range;
}

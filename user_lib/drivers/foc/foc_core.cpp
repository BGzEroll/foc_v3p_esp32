#include "foc_core.h"

#include "esp_attr.h"

/* ---- FOC 内部数学类型和常量 ---- */

static constexpr float PI = 3.14159265358979323846f;
static constexpr float HALF_PI = 1.57079632679489661923f;
static constexpr float THREE_HALF_PI = 4.71238898038468985769f;
static constexpr float TWO_PI = 6.28318530717958647692f;
static constexpr float ONE_OVER_TWO_PI = 0.15915494309189533577f;
static constexpr float TWO_OVER_THREE = 0.66666666666666666667f;
static constexpr float SQRT_THREE_OVER_TWO = 0.86602540378443864676f;
static constexpr float MAX_TURN_COUNT = 2147483000.0f;
static constexpr uint32_t FLOAT_EXPONENT_MASK = 0x7f800000;
static constexpr uint32_t FLOAT_MANTISSA_MASK = 0x007fffff;
static constexpr uint32_t FLOAT_SIGN_MASK = 0x80000000;

struct alpha_beta_current
{
    float alpha_a = 0.0f;
    float beta_a = 0.0f;
};

struct d_q_current
{
    float d_a = 0.0f;
    float q_a = 0.0f;
};

struct d_q_voltage
{
    float d_v = 0.0f;
    float q_v = 0.0f;
};

struct alpha_beta_voltage
{
    float alpha_v = 0.0f;
    float beta_v = 0.0f;
};

/**
 * @brief 在不依赖运行时库的情况下读取浮点数位模式
 *
 * @param value 待读取的浮点数
 *
 * @return IEEE 754 单精度位模式
 */
static uint32_t IRAM_ATTR float_to_bits(float value)
{
    uint32_t bits = 0;
    __builtin_memcpy(&bits, &value, sizeof(bits));
    return bits;
}

/**
 * @brief 在不依赖运行时库的情况下构造浮点数
 *
 * @param bits IEEE 754 单精度位模式
 *
 * @return 构造出的浮点数
 */
static float IRAM_ATTR bits_to_float(uint32_t bits)
{
    float value = 0.0f;
    __builtin_memcpy(&value, &bits, sizeof(value));
    return value;
}

/**
 * @brief 判断浮点数是否为有限值
 *
 * @param value 待检查的浮点数
 *
 * @return 为有限值时返回 true
 */
static bool IRAM_ATTR is_finite_number(float value)
{
    return (float_to_bits(value) & FLOAT_EXPONENT_MASK) !=
        FLOAT_EXPONENT_MASK;
}

/**
 * @brief 计算浮点数绝对值
 *
 * @param value 输入浮点数
 *
 * @return 绝对值
 */
static float IRAM_ATTR absolute_value(float value)
{
    return bits_to_float(float_to_bits(value) & ~FLOAT_SIGN_MASK);
}

/**
 * @brief 返回两个浮点数中的较大值
 *
 * @param left 左操作数
 * @param right 右操作数
 *
 * @return 较大值
 */
static float IRAM_ATTR maximum_value(float left, float right)
{
    return left > right ? left : right;
}

/**
 * @brief 返回两个浮点数中的较小值
 *
 * @param left 左操作数
 * @param right 右操作数
 *
 * @return 较小值
 */
static float IRAM_ATTR minimum_value(float left, float right)
{
    return left < right ? left : right;
}

/**
 * @brief 把浮点值限制在对称区间内
 *
 * @param value 待限制的数值
 * @param limit 对称限幅绝对值
 *
 * @return 限幅后的数值
 */
static float IRAM_ATTR clamp_symmetric(float value, float limit)
{
    if(value > limit){return limit;}
    if(value < -limit){return -limit;}
    return value;
}

/**
 * @brief 把占空比限制在零至一之间
 *
 * @param value 待限制的占空比
 *
 * @return 限制后的占空比
 */
static float IRAM_ATTR clamp_duty(float value)
{
    if(value > 1.0f){return 1.0f;}
    if(value < 0.0f){return 0.0f;}
    return value;
}

/**
 * @brief 把角度归一化到零至二倍圆周率
 *
 * @param angle_rad 输入角度，单位弧度
 * @param normalized_angle_rad 用于接收归一化角度
 *
 * @return 输入有效时返回 true
 */
static bool IRAM_ATTR normalize_angle(float angle_rad,
    float &normalized_angle_rad)
{
    if(!is_finite_number(angle_rad))
    {
        normalized_angle_rad = 0.0f;
        return false;
    }

    float turn_count = angle_rad * ONE_OVER_TWO_PI;
    if(!is_finite_number(turn_count) ||
        turn_count > MAX_TURN_COUNT || turn_count < -MAX_TURN_COUNT)
    {
        normalized_angle_rad = 0.0f;
        return false;
    }

    int32_t complete_turns = static_cast<int32_t>(turn_count);
    normalized_angle_rad = angle_rad -
        static_cast<float>(complete_turns) * TWO_PI;
    if(normalized_angle_rad < 0.0f)
    {
        normalized_angle_rad += TWO_PI;
    }
    if(normalized_angle_rad >= TWO_PI)
    {
        normalized_angle_rad -= TWO_PI;
    }

    return is_finite_number(normalized_angle_rad);
}

/**
 * @brief 使用固定阶数多项式计算正弦和余弦
 *
 * @param angle_rad 输入角度，单位弧度
 * @param sine 用于接收正弦值
 * @param cosine 用于接收余弦值
 *
 * @return 输入有效时返回 true
 */
static bool IRAM_ATTR calculate_sin_cos(float angle_rad,
    float &sine,
    float &cosine)
{
    float normalized_angle_rad = 0.0f;
    if(!normalize_angle(angle_rad, normalized_angle_rad))
    {
        sine = 0.0f;
        cosine = 1.0f;
        return false;
    }

    float reduced_angle_rad = normalized_angle_rad;
    float sine_sign = 1.0f;
    float cosine_sign = 1.0f;
    if(normalized_angle_rad < HALF_PI)
    {
        reduced_angle_rad = normalized_angle_rad;
    }
    else if(normalized_angle_rad < PI)
    {
        reduced_angle_rad = PI - normalized_angle_rad;
        cosine_sign = -1.0f;
    }
    else if(normalized_angle_rad < THREE_HALF_PI)
    {
        reduced_angle_rad = normalized_angle_rad - PI;
        sine_sign = -1.0f;
        cosine_sign = -1.0f;
    }
    else
    {
        reduced_angle_rad = TWO_PI - normalized_angle_rad;
        sine_sign = -1.0f;
    }

    float angle_squared = reduced_angle_rad * reduced_angle_rad;
    float angle_fourth = angle_squared * angle_squared;
    float angle_sixth = angle_fourth * angle_squared;
    float angle_eighth = angle_fourth * angle_fourth;
    float angle_tenth = angle_eighth * angle_squared;
    float sine_value = reduced_angle_rad *
        (1.0f - angle_squared * 0.16666666666666666667f +
            angle_fourth * 0.00833333333333333333f -
            angle_sixth * 0.00019841269841269841f +
            angle_eighth * 0.00000275573192239859f -
            angle_tenth * 0.00000002505210838544f);
    float cosine_value = 1.0f - angle_squared * 0.5f +
        angle_fourth * 0.04166666666666666667f -
        angle_sixth * 0.00138888888888888889f +
        angle_eighth * 0.00002480158730158730f -
        angle_tenth * 0.00000027557319223986f;
    sine = sine_sign * sine_value;
    cosine = cosine_sign * cosine_value;
    return is_finite_number(sine) && is_finite_number(cosine);
}

/**
 * @brief 使用固定次数 Newton 迭代计算正平方根倒数
 *
 * @param value 输入正数
 * @param inverse_sqrt 用于接收平方根倒数
 *
 * @return 输入有效时返回 true
 */
static bool IRAM_ATTR calculate_inverse_sqrt(float value,
    float &inverse_sqrt)
{
    if(!is_finite_number(value) || value <= 0.0f)
    {
        inverse_sqrt = 0.0f;
        return false;
    }

    uint32_t estimate_bits = float_to_bits(value);
    estimate_bits = 0x5f3759df - (estimate_bits >> 1);
    float estimate = bits_to_float(estimate_bits);
    float half_value = 0.5f * value;
    for(uint32_t iteration = 0; iteration < 3; iteration++)
    {
        estimate = estimate * (1.5f - half_value * estimate * estimate);
    }

    inverse_sqrt = estimate;
    return is_finite_number(inverse_sqrt) && inverse_sqrt > 0.0f;
}

/**
 * @brief 使用 Newton 迭代计算正数倒数
 *
 * @param value 输入正数
 * @param reciprocal 用于接收倒数
 *
 * @return 输入和计算结果均有效时返回 true
 */
static bool IRAM_ATTR calculate_reciprocal(float value,
    float &reciprocal)
{
    if(!is_finite_number(value) || value <= 0.0f)
    {
        reciprocal = 0.0f;
        return false;
    }

    uint32_t estimate_bits = 0x7f000000 - float_to_bits(value);
    float estimate = bits_to_float(estimate_bits);
    for(uint32_t iteration = 0; iteration < 3; iteration++)
    {
        estimate = estimate * (2.0f - value * estimate);
    }

    reciprocal = estimate;
    return is_finite_number(reciprocal) && reciprocal > 0.0f;
}

/**
 * @brief 执行 Clarke 变换
 *
 * @param current 三相电流样本
 *
 * @return Alpha-Beta 静止坐标系电流
 */
static alpha_beta_current IRAM_ATTR clarke(
    const phase_current_sample &current)
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
 * @brief 执行 Park 变换
 *
 * @param current Alpha-Beta 静止坐标系电流
 * @param electrical_angle_rad 电角度，单位弧度
 *
 * @return D-Q 旋转坐标系电流
 */
static d_q_current IRAM_ATTR park(
    const alpha_beta_current &current,
    float electrical_angle_rad)
{
    float sine = 0.0f;
    float cosine = 1.0f;
    calculate_sin_cos(electrical_angle_rad, sine, cosine);

    d_q_current result{};
    result.d_a = current.alpha_a * cosine + current.beta_a * sine;
    result.q_a = -current.alpha_a * sine + current.beta_a * cosine;
    return result;
}

/**
 * @brief 执行反 Park 变换
 *
 * @param voltage D-Q 旋转坐标系电压
 * @param electrical_angle_rad 电角度，单位弧度
 *
 * @return Alpha-Beta 静止坐标系电压
 */
static alpha_beta_voltage IRAM_ATTR inverse_park(
    const d_q_voltage &voltage,
    float electrical_angle_rad)
{
    float sine = 0.0f;
    float cosine = 1.0f;
    calculate_sin_cos(electrical_angle_rad, sine, cosine);

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
static bool IRAM_ATTR run_pi(float target,
    float measured,
    float period_s,
    const pi_config &config,
    float output_limit_v,
    float &integral,
    float &output)
{
    if(!is_finite_number(target) || !is_finite_number(measured) ||
        !is_finite_number(period_s) || period_s <= 0.0f ||
        !is_finite_number(config.kp) || config.kp < 0.0f ||
        !is_finite_number(config.ki) || config.ki < 0.0f ||
        !is_finite_number(config.integral_limit) ||
            config.integral_limit < 0.0f ||
        !is_finite_number(output_limit_v) || output_limit_v <= 0.0f ||
        !is_finite_number(integral))
    {
        integral = 0.0f;
        output = 0.0f;
        return false;
    }

    float error = target - measured;
    float integral_candidate = integral + config.ki * error * period_s;
    integral = clamp_symmetric(integral_candidate,
        config.integral_limit);
    output = clamp_symmetric(config.kp * error + integral,
        output_limit_v);

    if(!is_finite_number(integral) || !is_finite_number(output))
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
 * @param voltage 待限制的 D-Q 电压
 * @param magnitude_limit_v 电压矢量幅值上限，单位伏特
 *
 * @return 输入和计算结果均有效时返回 true
 */
static bool IRAM_ATTR limit_voltage(d_q_voltage &voltage,
    float magnitude_limit_v)
{
    if(!is_finite_number(voltage.d_v) || !is_finite_number(voltage.q_v) ||
        !is_finite_number(magnitude_limit_v) || magnitude_limit_v <= 0.0f)
    {
        voltage = {};
        return false;
    }

    float magnitude_squared_v2 = voltage.d_v * voltage.d_v +
        voltage.q_v * voltage.q_v;
    float limit_squared_v2 = magnitude_limit_v * magnitude_limit_v;
    if(!is_finite_number(magnitude_squared_v2) ||
        !is_finite_number(limit_squared_v2))
    {
        voltage = {};
        return false;
    }
    if(magnitude_squared_v2 <= limit_squared_v2){return true;}

    float inverse_magnitude = 0.0f;
    if(!calculate_inverse_sqrt(magnitude_squared_v2,
        inverse_magnitude))
    {
        voltage = {};
        return false;
    }

    float scale = magnitude_limit_v * inverse_magnitude;
    voltage.d_v *= scale;
    voltage.q_v *= scale;
    return is_finite_number(voltage.d_v) &&
        is_finite_number(voltage.q_v);
}

/**
 * @brief 使用零序注入计算三相中心对齐 PWM 占空比
 *
 * @param voltage Alpha-Beta 静止坐标系电压
 * @param bus_voltage_v 母线电压，单位伏特
 * @param duty 用于接收三相占空比
 *
 * @return 输入有效且未发生占空比限幅时返回 true
 */
static bool IRAM_ATTR svpwm(const alpha_beta_voltage &voltage,
    float bus_voltage_v,
    phase_duty &duty)
{
    if(!is_finite_number(voltage.alpha_v) ||
        !is_finite_number(voltage.beta_v) ||
        !is_finite_number(bus_voltage_v) || bus_voltage_v <= 0.0f)
    {
        duty = {};
        return false;
    }

    float phase_a_voltage_v = voltage.alpha_v;
    float phase_b_voltage_v = -0.5f * voltage.alpha_v +
        SQRT_THREE_OVER_TWO * voltage.beta_v;
    float phase_c_voltage_v = -0.5f * voltage.alpha_v -
        SQRT_THREE_OVER_TWO * voltage.beta_v;
    float maximum_voltage_v = maximum_value(phase_a_voltage_v,
        maximum_value(phase_b_voltage_v, phase_c_voltage_v));
    float minimum_voltage_v = minimum_value(phase_a_voltage_v,
        minimum_value(phase_b_voltage_v, phase_c_voltage_v));
    float common_mode_voltage_v = -0.5f *
        (maximum_voltage_v + minimum_voltage_v);
    float reciprocal_bus_voltage = 0.0f;
    if(!calculate_reciprocal(bus_voltage_v, reciprocal_bus_voltage))
    {
        duty = {};
        return false;
    }
    float phase_a_duty = 0.5f +
        (phase_a_voltage_v + common_mode_voltage_v) *
            reciprocal_bus_voltage;
    float phase_b_duty = 0.5f +
        (phase_b_voltage_v + common_mode_voltage_v) *
            reciprocal_bus_voltage;
    float phase_c_duty = 0.5f +
        (phase_c_voltage_v + common_mode_voltage_v) *
            reciprocal_bus_voltage;
    if(!is_finite_number(phase_a_duty) ||
        !is_finite_number(phase_b_duty) ||
        !is_finite_number(phase_c_duty))
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

/* ---- foc_core 内部状态和校验 ---- */

/**
 * @brief 检查 FOC 配置是否满足第一阶段电流环约束
 *
 * @param config 待检查配置
 *
 * @return 配置有效时返回 true
 */
bool foc_core::valid_config(const foc_config &config) const
{
    bool d_axis_pi_valid = is_finite_number(config.d_axis_pi.kp) &&
        config.d_axis_pi.kp >= 0.0f &&
        is_finite_number(config.d_axis_pi.ki) &&
        config.d_axis_pi.ki >= 0.0f &&
        is_finite_number(config.d_axis_pi.integral_limit) &&
        config.d_axis_pi.integral_limit >= 0.0f;
    bool q_axis_pi_valid = is_finite_number(config.q_axis_pi.kp) &&
        config.q_axis_pi.kp >= 0.0f &&
        is_finite_number(config.q_axis_pi.ki) &&
        config.q_axis_pi.ki >= 0.0f &&
        is_finite_number(config.q_axis_pi.integral_limit) &&
        config.q_axis_pi.integral_limit >= 0.0f;
    float maximum_svpwm_voltage_v = config.bus_voltage_v *
        MAX_SVPWM_VOLTAGE_RATIO;

    return config.pole_pairs > 0 &&
        (config.rotor_direction == 1 || config.rotor_direction == -1) &&
        is_finite_number(config.electrical_zero_offset_rad) &&
        is_finite_number(config.control_period_s) &&
            config.control_period_s > 0.0f &&
        is_finite_number(config.bus_voltage_v) &&
            config.bus_voltage_v > 0.0f &&
        is_finite_number(config.voltage_limit_v) &&
            config.voltage_limit_v > 0.0f &&
        config.voltage_limit_v <= maximum_svpwm_voltage_v &&
        is_finite_number(config.max_phase_current_a) &&
            config.max_phase_current_a > 0.0f &&
        d_axis_pi_valid && q_axis_pi_valid;
}

/**
 * @brief 检查 Target 是否满足电流环输入约束
 *
 * @param target 待检查的目标
 *
 * @return 目标有效时返回 true
 */
bool IRAM_ATTR foc_core::valid_target(const foc_target &target) const
{
    if(target.mode != foc_control_mode::DISABLED &&
        target.mode != foc_control_mode::CURRENT)
    {
        return false;
    }
    if(!is_finite_number(target.d_axis_current_a) ||
        !is_finite_number(target.q_axis_current_a))
    {
        return false;
    }
    if(target.mode == foc_control_mode::DISABLED){return true;}

    float target_magnitude_squared_a2 =
        target.d_axis_current_a * target.d_axis_current_a +
        target.q_axis_current_a * target.q_axis_current_a;
    float current_limit_squared_a2 = config.max_phase_current_a *
        config.max_phase_current_a;
    return is_finite_number(target_magnitude_squared_a2) &&
        is_finite_number(current_limit_squared_a2) &&
        target_magnitude_squared_a2 <= current_limit_squared_a2;
}

/**
 * @brief 检查转子样本有效性和时间新鲜度
 *
 * @param sample 待检查的转子样本
 * @param timestamp_us 当前控制周期时间戳，单位微秒
 *
 * @return 样本可用于控制时返回 true
 */
bool IRAM_ATTR foc_core::valid_rotor_sample(
    const rotor_sample &sample,
    uint32_t timestamp_us) const
{
    uint32_t sample_age_us = timestamp_us - sample.timestamp_us;
    return sample.valid && sample_age_us <= SENSOR_TIMEOUT_US &&
        is_finite_number(sample.mechanical_angle_rad) &&
        is_finite_number(sample.mechanical_velocity_rad_s);
}

/**
 * @brief 检查电流样本有效性和时间新鲜度
 *
 * @param sample 待检查的电流样本
 * @param timestamp_us 当前控制周期时间戳，单位微秒
 *
 * @return 样本可用于控制时返回 true
 */
bool IRAM_ATTR foc_core::valid_current_sample(
    const phase_current_sample &sample,
    uint32_t timestamp_us) const
{
    uint32_t sample_age_us = timestamp_us - sample.timestamp_us;
    return sample.valid && sample_age_us <= SENSOR_TIMEOUT_US &&
        is_finite_number(sample.phase_a_a) &&
        is_finite_number(sample.phase_b_a) &&
        is_finite_number(sample.phase_c_a);
}

/**
 * @brief 获取运行时中最近的样本时间戳
 *
 * @return 最近样本时间戳，单位微秒
 */
uint32_t foc_core::latest_timestamp_us() const
{
    return runtime.rotor.timestamp_us > runtime.current.timestamp_us ?
        runtime.rotor.timestamp_us : runtime.current.timestamp_us;
}

/**
 * @brief 发布 Disabled Target 并同步当前活动目标
 */
void foc_core::publish_disabled_target()
{
    foc_target disabled_target{};
    disabled_target.sequence = ++target_sequence;
    target_topic.publish(disabled_target);
    active_target = disabled_target;
}

/**
 * @brief 清零 PI、控制电压并恢复中性占空比
 */
void IRAM_ATTR foc_core::reset_control_output()
{
    runtime.u_d_v = 0.0f;
    runtime.u_q_v = 0.0f;
    runtime.u_alpha_v = 0.0f;
    runtime.u_beta_v = 0.0f;
    runtime.d_integral = 0.0f;
    runtime.q_integral = 0.0f;
    runtime.duty = {};
}

/**
 * @brief 关闭功率输出并进入故障状态
 *
 * @param fault 需要记录的故障位
 */
void IRAM_ATTR foc_core::enter_fault(foc_fault fault)
{
    fault_flags |= foc_fault_mask(fault);
    if(output.disable)
    {
        output.disable(output.context);
    }
    output_active = false;
    active_target = {};
    reset_control_output();
    state = foc_state::FAULT;
}

/**
 * @brief 读取一个转子样本并计算电角度
 *
 * @param sample 转子样本
 *
 * @return 转子更新结果
 */
foc_result IRAM_ATTR foc_core::update_rotor(
    const rotor_sample &sample)
{
    runtime.rotor = sample;
    float direction = static_cast<float>(config.rotor_direction);
    float pole_pairs = static_cast<float>(config.pole_pairs);
    float electrical_angle_rad = direction * pole_pairs *
        sample.mechanical_angle_rad - config.electrical_zero_offset_rad;
    if(!normalize_angle(electrical_angle_rad,
        runtime.electrical_angle_rad))
    {
        return foc_result::INVALID_NUMBER;
    }
    runtime.electrical_velocity_rad_s = direction * pole_pairs *
        sample.mechanical_velocity_rad_s;
    if(!is_finite_number(runtime.electrical_angle_rad) ||
        !is_finite_number(runtime.electrical_velocity_rad_s))
    {
        return foc_result::INVALID_NUMBER;
    }

    return foc_result::OK;
}

/**
 * @brief 读取三相电流并执行基础数值和过流检查
 *
 * @param sample 三相电流样本
 * @param timestamp_us 当前控制周期时间戳，单位微秒
 *
 * @return 电流更新结果
 */
foc_result IRAM_ATTR foc_core::update_current(
    const phase_current_sample &sample,
    uint32_t timestamp_us)
{
    runtime.current = sample;
    if(!valid_current_sample(sample, timestamp_us))
    {
        return foc_result::SENSOR_ERROR;
    }

    if(absolute_value(sample.phase_a_a) > config.max_phase_current_a ||
        absolute_value(sample.phase_b_a) > config.max_phase_current_a ||
        absolute_value(sample.phase_c_a) > config.max_phase_current_a)
    {
        return foc_result::OUTPUT_RANGE;
    }

    return foc_result::OK;
}

/**
 * @brief 检查功率级是否报告硬件故障
 *
 * @return 功率级故障有效时返回 true
 */
bool IRAM_ATTR foc_core::output_fault_active() const
{
    return output.fault_active && output.fault_active(output.context);
}

/**
 * @brief 执行 Clarke、Park 和 D-Q 电流 PI 运算
 *
 * @return 电流控制计算结果
 */
foc_result IRAM_ATTR foc_core::run_current_control()
{
    alpha_beta_current stationary_current = clarke(runtime.current);
    d_q_current rotating_current = park(stationary_current,
        runtime.electrical_angle_rad);
    runtime.i_alpha_a = stationary_current.alpha_a;
    runtime.i_beta_a = stationary_current.beta_a;
    runtime.i_d_a = rotating_current.d_a;
    runtime.i_q_a = rotating_current.q_a;

    if(!is_finite_number(runtime.i_alpha_a) ||
        !is_finite_number(runtime.i_beta_a) ||
        !is_finite_number(runtime.i_d_a) ||
        !is_finite_number(runtime.i_q_a))
    {
        return foc_result::INVALID_NUMBER;
    }

    if(active_target.mode == foc_control_mode::DISABLED)
    {
        runtime.d_integral = 0.0f;
        runtime.q_integral = 0.0f;
        runtime.u_d_v = 0.0f;
        runtime.u_q_v = 0.0f;
        return foc_result::OK;
    }
    if(active_target.mode != foc_control_mode::CURRENT)
    {
        return foc_result::INVALID_STATE;
    }

    bool d_axis_valid = run_pi(active_target.d_axis_current_a,
        runtime.i_d_a,
        config.control_period_s,
        config.d_axis_pi,
        config.voltage_limit_v,
        runtime.d_integral,
        runtime.u_d_v);
    bool q_axis_valid = run_pi(active_target.q_axis_current_a,
        runtime.i_q_a,
        config.control_period_s,
        config.q_axis_pi,
        config.voltage_limit_v,
        runtime.q_integral,
        runtime.u_q_v);
    if(!d_axis_valid || !q_axis_valid)
    {
        return foc_result::INVALID_NUMBER;
    }

    d_q_voltage voltage{};
    voltage.d_v = runtime.u_d_v;
    voltage.q_v = runtime.u_q_v;
    if(!limit_voltage(voltage, config.voltage_limit_v))
    {
        return foc_result::INVALID_NUMBER;
    }
    runtime.u_d_v = voltage.d_v;
    runtime.u_q_v = voltage.q_v;
    return foc_result::OK;
}

/**
 * @brief 执行反 Park 和 SVPWM 计算
 *
 * @return 输出计算结果
 */
foc_result IRAM_ATTR foc_core::calculate_output()
{
    d_q_voltage rotating_voltage{};
    rotating_voltage.d_v = runtime.u_d_v;
    rotating_voltage.q_v = runtime.u_q_v;
    alpha_beta_voltage stationary_voltage = inverse_park(rotating_voltage,
        runtime.electrical_angle_rad);
    runtime.u_alpha_v = stationary_voltage.alpha_v;
    runtime.u_beta_v = stationary_voltage.beta_v;

    if(!is_finite_number(runtime.u_alpha_v) ||
        !is_finite_number(runtime.u_beta_v))
    {
        runtime.duty = {};
        return foc_result::INVALID_NUMBER;
    }
    if(!svpwm(stationary_voltage, config.bus_voltage_v, runtime.duty))
    {
        return foc_result::OUTPUT_RANGE;
    }
    return foc_result::OK;
}

/* ---- foc_core Topic 和控制循环 ---- */

/**
 * @brief 从 Target Topic 读取最新控制目标
 *
 * @param target 用于接收目标
 *
 * @return 成功读取时返回 true
 */
template<bool FROM_ISR>
bool IRAM_ATTR foc_core::load_target(foc_target &target)
{
    if constexpr(FROM_ISR)
    {
        return target_topic.peek_from_isr(target);
    }
    else
    {
        return target_topic.peek(target, 0);
    }
}

/**
 * @brief 从转子 Topic 读取最新转子样本
 *
 * @param sample 用于接收转子样本
 *
 * @return 成功读取时返回 true
 */
template<bool FROM_ISR>
bool IRAM_ATTR foc_core::load_rotor(rotor_sample &sample)
{
    if constexpr(FROM_ISR)
    {
        return rotor_topic.peek_from_isr(sample);
    }
    else
    {
        return rotor_topic.peek(sample, 0);
    }
}

/**
 * @brief 从电流 Topic 读取最新三相电流样本
 *
 * @param sample 用于接收三相电流样本
 *
 * @return 成功读取时返回 true
 */
template<bool FROM_ISR>
bool IRAM_ATTR foc_core::load_current(
    phase_current_sample &sample)
{
    if constexpr(FROM_ISR)
    {
        return current_topic.peek_from_isr(sample);
    }
    else
    {
        return current_topic.peek(sample, 0);
    }
}

/**
 * @brief 将本周期占空比提交给功率输出
 *
 * @return 输出结果
 */
template<bool FROM_ISR>
foc_result IRAM_ATTR foc_core::apply_output()
{
    if(output_fault_active())
    {
        return foc_result::DRIVER_FAULT;
    }
    if(!is_finite_number(runtime.duty.phase_a) ||
        !is_finite_number(runtime.duty.phase_b) ||
        !is_finite_number(runtime.duty.phase_c) ||
        runtime.duty.phase_a < 0.0f || runtime.duty.phase_a > 1.0f ||
        runtime.duty.phase_b < 0.0f || runtime.duty.phase_b > 1.0f ||
        runtime.duty.phase_c < 0.0f || runtime.duty.phase_c > 1.0f)
    {
        return foc_result::OUTPUT_RANGE;
    }

    foc_result result = foc_result::DRIVER_FAULT;
    if constexpr(FROM_ISR)
    {
        result = output.apply_duty_from_isr(output.context,
            runtime.duty);
    }
    else
    {
        result = output.apply_duty(output.context, runtime.duty);
    }
    return result == foc_result::OK ? foc_result::OK :
        foc_result::DRIVER_FAULT;
}

/**
 * @brief 发布当前完整运行状态到 Snapshot Topic
 *
 * @param timestamp_us 快照时间戳，单位微秒
 * @param higher_priority_task_woken ISR 唤醒标记
 * @param force 是否忽略降频间隔立即发布
 */
template<bool FROM_ISR>
void IRAM_ATTR foc_core::publish_snapshot(uint32_t timestamp_us,
    BaseType_t *higher_priority_task_woken,
    bool force)
{
    if(!force && snapshot_has_timestamp &&
        static_cast<uint32_t>(timestamp_us - last_snapshot_timestamp_us) <
            SNAPSHOT_PERIOD_US)
    {
        return;
    }

    foc_snapshot snapshot{};
    snapshot.sequence = snapshot_sequence + 1;
    snapshot.timestamp_us = timestamp_us;
    snapshot.state = state;
    snapshot.fault_flags = fault_flags;
    snapshot.output_active = output_active &&
        state == foc_state::RUNNING;
    snapshot.mechanical_angle_rad = runtime.rotor.mechanical_angle_rad;
    snapshot.mechanical_velocity_rad_s =
        runtime.rotor.mechanical_velocity_rad_s;
    snapshot.electrical_angle_rad = runtime.electrical_angle_rad;
    snapshot.electrical_velocity_rad_s =
        runtime.electrical_velocity_rad_s;
    snapshot.phase_a_current_a = runtime.current.phase_a_a;
    snapshot.phase_b_current_a = runtime.current.phase_b_a;
    snapshot.phase_c_current_a = runtime.current.phase_c_a;
    snapshot.i_alpha_a = runtime.i_alpha_a;
    snapshot.i_beta_a = runtime.i_beta_a;
    snapshot.i_d_a = runtime.i_d_a;
    snapshot.i_q_a = runtime.i_q_a;
    snapshot.target_i_d_a = active_target.d_axis_current_a;
    snapshot.target_i_q_a = active_target.q_axis_current_a;
    snapshot.u_d_v = runtime.u_d_v;
    snapshot.u_q_v = runtime.u_q_v;
    snapshot.duty = runtime.duty;
    snapshot.bus_voltage_v = config.bus_voltage_v;
    snapshot.bus_current_a = 0.0f;

    bool published = false;
    if constexpr(FROM_ISR)
    {
        if(!higher_priority_task_woken){return;}
        published = snapshot_topic.publish_from_isr(snapshot,
            *higher_priority_task_woken);
    }
    else
    {
        published = snapshot_topic.publish(snapshot);
    }
    if(!published){return;}

    snapshot_sequence = snapshot.sequence;
    last_snapshot_timestamp_us = timestamp_us;
    snapshot_has_timestamp = true;
}

/**
 * @brief 结束发生严重错误的控制周期
 *
 * @param fault 本周期故障位
 * @param result 返回给调用方的错误结果
 * @param timestamp_us 本控制周期时间戳，单位微秒
 * @param higher_priority_task_woken ISR 唤醒标记
 *
 * @return 传入的错误结果
 */
template<bool FROM_ISR>
foc_result IRAM_ATTR foc_core::fail_control_cycle(
    foc_fault fault,
    foc_result result,
    uint32_t timestamp_us,
    BaseType_t *higher_priority_task_woken)
{
    enter_fault(fault);
    publish_snapshot<FROM_ISR>(timestamp_us,
        higher_priority_task_woken,
        true);
    return result;
}

/**
 * @brief 执行一次固定顺序的 FOC 控制周期
 *
 * @param timestamp_us 本控制周期时间戳，单位微秒
 * @param higher_priority_task_woken ISR 唤醒标记
 *
 * @return 本控制周期结果
 */
template<bool FROM_ISR>
foc_result IRAM_ATTR foc_core::run_control_loop(
    uint32_t timestamp_us,
    BaseType_t *higher_priority_task_woken)
{
    if(!initialized || state == foc_state::UNINITIALIZED)
    {
        return foc_result::NOT_INITIALIZED;
    }
    if(state == foc_state::READY){return foc_result::NOT_READY;}
    if(state != foc_state::RUNNING)
    {
        return foc_result::INVALID_STATE;
    }
    if(output_fault_active())
    {
        return fail_control_cycle<FROM_ISR>(foc_fault::DRIVER,
            foc_result::DRIVER_FAULT,
            timestamp_us,
            higher_priority_task_woken);
    }

    foc_target target{};
    if(!load_target<FROM_ISR>(target) || !valid_target(target))
    {
        return fail_control_cycle<FROM_ISR>(foc_fault::INTERNAL,
            foc_result::INTERNAL_ERROR,
            timestamp_us,
            higher_priority_task_woken);
    }
    active_target = target;

    rotor_sample rotor{};
    if(!load_rotor<FROM_ISR>(rotor) ||
        !valid_rotor_sample(rotor, timestamp_us))
    {
        return fail_control_cycle<FROM_ISR>(foc_fault::ROTOR_SENSOR,
            foc_result::SENSOR_ERROR,
            timestamp_us,
            higher_priority_task_woken);
    }
    foc_result result = update_rotor(rotor);
    if(result != foc_result::OK)
    {
        return fail_control_cycle<FROM_ISR>(foc_fault::INVALID_NUMBER,
            result,
            timestamp_us,
            higher_priority_task_woken);
    }

    phase_current_sample current{};
    if(!load_current<FROM_ISR>(current) ||
        !valid_current_sample(current, timestamp_us))
    {
        return fail_control_cycle<FROM_ISR>(foc_fault::CURRENT_SENSOR,
            foc_result::SENSOR_ERROR,
            timestamp_us,
            higher_priority_task_woken);
    }
    result = update_current(current, timestamp_us);
    if(result == foc_result::OUTPUT_RANGE)
    {
        return fail_control_cycle<FROM_ISR>(foc_fault::OVER_CURRENT,
            result,
            timestamp_us,
            higher_priority_task_woken);
    }
    if(result != foc_result::OK)
    {
        return fail_control_cycle<FROM_ISR>(foc_fault::INVALID_NUMBER,
            result,
            timestamp_us,
            higher_priority_task_woken);
    }

    result = run_current_control();
    if(result != foc_result::OK)
    {
        foc_fault fault = result == foc_result::INVALID_STATE ?
            foc_fault::INTERNAL : foc_fault::INVALID_NUMBER;
        return fail_control_cycle<FROM_ISR>(fault,
            result,
            timestamp_us,
            higher_priority_task_woken);
    }

    result = calculate_output();
    if(result != foc_result::OK)
    {
        foc_fault fault = result == foc_result::OUTPUT_RANGE ?
            foc_fault::OUTPUT_RANGE : foc_fault::INVALID_NUMBER;
        return fail_control_cycle<FROM_ISR>(fault,
            result,
            timestamp_us,
            higher_priority_task_woken);
    }

    result = apply_output<FROM_ISR>();
    if(result != foc_result::OK)
    {
        foc_fault fault = result == foc_result::OUTPUT_RANGE ?
            foc_fault::OUTPUT_RANGE : foc_fault::DRIVER;
        return fail_control_cycle<FROM_ISR>(fault,
            result,
            timestamp_us,
            higher_priority_task_woken);
    }
    if(output_fault_active())
    {
        return fail_control_cycle<FROM_ISR>(foc_fault::DRIVER,
            foc_result::DRIVER_FAULT,
            timestamp_us,
            higher_priority_task_woken);
    }

    publish_snapshot<FROM_ISR>(timestamp_us,
        higher_priority_task_woken,
        false);
    return active_target.mode == foc_control_mode::DISABLED ?
        foc_result::DISABLED : foc_result::OK;
}

/* ---- foc_core 公共 API ---- */

/**
 * @brief 初始化单个 FOC 实例及其 Topic 和功率输出
 *
 * @param config 电机和电流环配置
 * @param output 功率输出回调
 *
 * @return 初始化结果
 */
foc_result foc_core::init(const foc_config &config,
    const foc_output &output)
{
    if(state != foc_state::UNINITIALIZED)
    {
        return foc_result::INVALID_STATE;
    }
    if(!valid_config(config))
    {
        return foc_result::INVALID_CONFIG;
    }
    if(!output.init || !output.enable || !output.disable ||
        !output.apply_duty || !output.apply_duty_from_isr ||
        !output.fault_active)
    {
        return foc_result::INVALID_ARGUMENT;
    }

    this->config = config;
    this->output = output;
    runtime = {};
    active_target = {};
    fault_flags = 0;
    output_active = false;
    initialized = false;
    target_sequence = 0;
    snapshot_sequence = 0;
    last_snapshot_timestamp_us = 0;
    snapshot_has_timestamp = false;

    bool topics_ready = target_topic.init() && rotor_topic.init() &&
        current_topic.init() && snapshot_topic.init();
    if(!topics_ready)
    {
        return foc_result::INTERNAL_ERROR;
    }
    initialized = true;

    foc_target disabled_target{};
    if(!target_topic.publish(disabled_target) ||
        !rotor_topic.publish(rotor_sample{}) ||
        !current_topic.publish(phase_current_sample{}))
    {
        initialized = false;
        return foc_result::INTERNAL_ERROR;
    }

    foc_result result = this->output.init(this->output.context);
    if(result != foc_result::OK)
    {
        fault_flags = foc_fault_mask(foc_fault::DRIVER);
        state = foc_state::FAULT;
        this->output.disable(this->output.context);
        publish_snapshot<false>(0, nullptr, true);
        initialized = false;
        state = foc_state::UNINITIALIZED;
        return foc_result::DRIVER_FAULT;
    }

    this->output.disable(this->output.context);
    state = foc_state::READY;
    publish_snapshot<false>(0, nullptr, true);
    return foc_result::OK;
}

/**
 * @brief 以中性占空比安全开启功率级
 *
 * @return 使能结果
 *
 * @note 调用前必须停止本实例控制 ISR，并确认 ISR 已经执行结束。
 */
foc_result foc_core::enable()
{
    if(!initialized || state == foc_state::UNINITIALIZED)
    {
        return foc_result::NOT_INITIALIZED;
    }
    if(state != foc_state::READY){return foc_result::INVALID_STATE;}

    foc_target target{};
    if(!target_topic.peek(target, 0)){return foc_result::NOT_READY;}
    if(target.mode != foc_control_mode::CURRENT)
    {
        return foc_result::DISABLED;
    }
    if(!valid_target(target))
    {
        return foc_result::INVALID_ARGUMENT;
    }
    if(output_fault_active())
    {
        enter_fault(foc_fault::DRIVER);
        publish_snapshot<false>(latest_timestamp_us(), nullptr, true);
        return foc_result::DRIVER_FAULT;
    }

    active_target = target;
    reset_control_output();
    foc_result result = output.apply_duty(output.context,
        runtime.duty);
    if(result != foc_result::OK)
    {
        enter_fault(foc_fault::DRIVER);
        publish_snapshot<false>(latest_timestamp_us(), nullptr, true);
        return foc_result::DRIVER_FAULT;
    }

    result = output.enable(output.context);
    if(result != foc_result::OK || output_fault_active())
    {
        enter_fault(foc_fault::DRIVER);
        publish_snapshot<false>(latest_timestamp_us(), nullptr, true);
        return foc_result::DRIVER_FAULT;
    }

    output_active = true;
    state = foc_state::RUNNING;
    publish_snapshot<false>(latest_timestamp_us(), nullptr, true);
    return foc_result::OK;
}

/**
 * @brief 立即关闭功率输出并清除控制输出状态
 *
 * @note 调用前必须停止本实例控制 ISR，并确认 ISR 已经执行结束。
 */
void foc_core::disable()
{
    if(!initialized || state == foc_state::UNINITIALIZED){return;}

    output.disable(output.context);
    output_active = false;
    if(state == foc_state::RUNNING)
    {
        state = foc_state::READY;
    }
    publish_disabled_target();
    reset_control_output();
    publish_snapshot<false>(latest_timestamp_us(), nullptr, true);
}

/**
 * @brief 向 Target Topic 发布一条经过校验的控制目标
 *
 * @param target 待发布控制目标
 *
 * @return 目标检查和发布结果
 *
 * @note 本函数使用任务上下文 Queue API，不能从 ISR 调用。FAULT 状态下必须
 *       先调用 clear_fault()。
 */
foc_result foc_core::set_target(const foc_target &target)
{
    if(!initialized || state == foc_state::UNINITIALIZED)
    {
        return foc_result::NOT_INITIALIZED;
    }
    if(state == foc_state::FAULT){return foc_result::INVALID_STATE;}
    if(target.mode != foc_control_mode::DISABLED &&
        target.mode != foc_control_mode::CURRENT)
    {
        return foc_result::INVALID_ARGUMENT;
    }
    if(!is_finite_number(target.d_axis_current_a) ||
        !is_finite_number(target.q_axis_current_a))
    {
        return foc_result::INVALID_NUMBER;
    }

    foc_target checked_target = target;
    if(checked_target.mode == foc_control_mode::DISABLED)
    {
        checked_target.d_axis_current_a = 0.0f;
        checked_target.q_axis_current_a = 0.0f;
    }
    else if(!valid_target(checked_target))
    {
        return foc_result::OUTPUT_RANGE;
    }

    checked_target.sequence = ++target_sequence;
    return target_topic.publish(checked_target) ? foc_result::OK :
        foc_result::NOT_READY;
}

/**
 * @brief 执行一次任务上下文 FOC 控制周期
 *
 * @param timestamp_us 本控制周期时间戳，单位微秒
 *
 * @return 本控制周期结果
 */
foc_result foc_core::core_loop(uint32_t timestamp_us)
{
    return run_control_loop<false>(timestamp_us, nullptr);
}

/**
 * @brief 执行一次 ISR 上下文 FOC 控制周期
 *
 * @param timestamp_us 本控制周期时间戳，单位微秒
 * @param higher_priority_task_woken ISR 唤醒标记
 *
 * @return 本控制周期结果
 *
 * @note 本函数不调用 portYIELD_FROM_ISR()；调用方应在 ISR 结束前统一处理
 *       higher_priority_task_woken。调用链不得执行阻塞、分配或任务上下文操作。
 */
foc_result IRAM_ATTR foc_core::core_loop_from_isr(
    uint32_t timestamp_us,
    BaseType_t &higher_priority_task_woken)
{
    return run_control_loop<true>(timestamp_us,
        &higher_priority_task_woken);
}

/**
 * @brief 获取本实例对外开放的 Topic
 *
 * @return 转子、电流和 Snapshot Topic 引用
 */
foc_topic_access foc_core::topics()
{
    return foc_topic_access{rotor_topic, current_topic, snapshot_topic};
}

/**
 * @brief 在功率输出关闭且硬件故障解除后清除故障
 *
 * @return 故障清除结果
 *
 * @note 调用前必须停止本实例控制 ISR，并确认 ISR 已经执行结束。
 */
foc_result foc_core::clear_fault()
{
    if(!initialized || state == foc_state::UNINITIALIZED)
    {
        return foc_result::NOT_INITIALIZED;
    }
    if(state != foc_state::FAULT){return foc_result::INVALID_STATE;}
    if(output_active){return foc_result::INVALID_STATE;}
    if(output_fault_active()){return foc_result::DRIVER_FAULT;}

    output.disable(output.context);
    publish_disabled_target();
    reset_control_output();
    fault_flags = 0;
    state = foc_state::READY;
    publish_snapshot<false>(latest_timestamp_us(), nullptr, true);
    return foc_result::OK;
}

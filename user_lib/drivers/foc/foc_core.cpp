#include "foc_core.h"

#include "foc_math.h"
#include <math.h>

static constexpr float MAX_SVPWM_VOLTAGE_RATIO =
    0.57735026918962576451f;

static_assert(std::atomic<uint32_t>::is_always_lock_free,
    "FOC requires lock-free 32-bit atomics");
static_assert(std::atomic<bool>::is_always_lock_free,
    "FOC requires lock-free boolean atomics");

/**
 * @brief 检查 FOC 配置是否满足第一阶段电流环约束
 *
 * @param config 待检查配置
 *
 * @return 配置有效时返回 true
 */
bool foc_core::valid_config(const foc_config &config) const
{
    bool d_axis_pi_valid = isfinite(config.d_axis_pi.kp) &&
        config.d_axis_pi.kp >= 0.0f &&
        isfinite(config.d_axis_pi.ki) &&
        config.d_axis_pi.ki >= 0.0f &&
        isfinite(config.d_axis_pi.integral_limit) &&
        config.d_axis_pi.integral_limit >= 0.0f;
    bool q_axis_pi_valid = isfinite(config.q_axis_pi.kp) &&
        config.q_axis_pi.kp >= 0.0f &&
        isfinite(config.q_axis_pi.ki) &&
        config.q_axis_pi.ki >= 0.0f &&
        isfinite(config.q_axis_pi.integral_limit) &&
        config.q_axis_pi.integral_limit >= 0.0f;
    float maximum_svpwm_voltage_v = config.bus_voltage_v *
        MAX_SVPWM_VOLTAGE_RATIO;

    return config.pole_pairs > 0U &&
        (config.rotor_direction == 1 || config.rotor_direction == -1) &&
        isfinite(config.electrical_zero_offset_rad) &&
        isfinite(config.control_period_s) && config.control_period_s > 0.0f &&
        isfinite(config.bus_voltage_v) && config.bus_voltage_v > 0.0f &&
        isfinite(config.voltage_limit_v) && config.voltage_limit_v > 0.0f &&
        config.voltage_limit_v <= maximum_svpwm_voltage_v &&
        isfinite(config.max_phase_current_a) &&
        config.max_phase_current_a > 0.0f &&
        d_axis_pi_valid && q_axis_pi_valid;
}

/**
 * @brief 将一条完整目标命令发布到三槽无锁缓冲区
 *
 * @param target 待发布目标
 *
 * @return 发布结果
 */
foc_result foc_core::publish_target(const foc_target &target)
{
    if(target_writer_busy_.test_and_set(std::memory_order_acquire))
    {
        return foc_result::NOT_READY;
    }

    uint32_t published_index = published_target_index_.load(
        std::memory_order_acquire);
    uint32_t reader_index = target_reader_index_.load(
        std::memory_order_acquire);
    uint32_t write_index = INVALID_BUFFER_INDEX;

    for(uint32_t index = 0U; index < BUFFER_COUNT; index++)
    {
        if(index != published_index && index != reader_index)
        {
            write_index = index;
            break;
        }
    }

    if(write_index == INVALID_BUFFER_INDEX)
    {
        target_writer_busy_.clear(std::memory_order_release);
        return foc_result::NOT_READY;
    }

    foc_target published_target = target;
    published_target.sequence = target_command_sequence_.fetch_add(1U,
        std::memory_order_relaxed) + 1U;
    target_buffers_[write_index] = published_target;
    published_target_index_.store(write_index, std::memory_order_release);
    target_writer_busy_.clear(std::memory_order_release);
    return foc_result::OK;
}

/**
 * @brief 从三槽无锁缓冲区读取一条完整目标命令
 *
 * @param target 用于接收目标命令
 *
 * @return 在有限次数内读到当前发布版本时返回 true
 */
bool foc_core::load_target(foc_target &target)
{
    for(uint32_t attempt = 0U; attempt < READ_ATTEMPT_COUNT; attempt++)
    {
        uint32_t published_index = published_target_index_.load(
            std::memory_order_acquire);
        target_reader_index_.store(published_index,
            std::memory_order_release);

        if(published_index == published_target_index_.load(
            std::memory_order_acquire))
        {
            target = target_buffers_[published_index];
            target_reader_index_.store(INVALID_BUFFER_INDEX,
                std::memory_order_release);
            return true;
        }

        target_reader_index_.store(INVALID_BUFFER_INDEX,
            std::memory_order_release);
    }

    target = active_target_;
    return false;
}

/**
 * @brief 读取转子样本并计算电角度和电角速度
 *
 * @param timestamp_us 本控制周期时间戳，单位微秒
 *
 * @return 转子更新结果
 */
foc_result foc_core::update_rotor(uint32_t timestamp_us)
{
    foc_result result = hardware_.rotor_sensor->read(timestamp_us,
        runtime_.rotor);
    if(result != foc_result::OK || !runtime_.rotor.valid)
    {
        return foc_result::SENSOR_ERROR;
    }

    if(!isfinite(runtime_.rotor.mechanical_angle_rad) ||
        !isfinite(runtime_.rotor.mechanical_velocity_rad_s))
    {
        return foc_result::INVALID_NUMBER;
    }

    float direction = static_cast<float>(config_.rotor_direction);
    float pole_pairs = static_cast<float>(config_.pole_pairs);
    float electrical_angle_rad = direction * pole_pairs *
        runtime_.rotor.mechanical_angle_rad -
        config_.electrical_zero_offset_rad;
    runtime_.electrical_angle_rad = foc_math::normalize_angle(
        electrical_angle_rad);
    runtime_.electrical_velocity_rad_s = direction * pole_pairs *
        runtime_.rotor.mechanical_velocity_rad_s;

    if(!isfinite(runtime_.electrical_angle_rad) ||
        !isfinite(runtime_.electrical_velocity_rad_s))
    {
        return foc_result::INVALID_NUMBER;
    }

    return foc_result::OK;
}

/**
 * @brief 读取三相电流并执行基础数值和过流检查
 *
 * @param timestamp_us 本控制周期时间戳，单位微秒
 *
 * @return 电流更新结果
 */
foc_result foc_core::update_current(uint32_t timestamp_us)
{
    foc_result result = hardware_.current_sensor->read(timestamp_us,
        runtime_.current);
    if(result != foc_result::OK || !runtime_.current.valid)
    {
        return foc_result::SENSOR_ERROR;
    }

    if(!isfinite(runtime_.current.phase_a_a) ||
        !isfinite(runtime_.current.phase_b_a) ||
        !isfinite(runtime_.current.phase_c_a))
    {
        return foc_result::INVALID_NUMBER;
    }

    if(fabsf(runtime_.current.phase_a_a) > config_.max_phase_current_a ||
        fabsf(runtime_.current.phase_b_a) > config_.max_phase_current_a ||
        fabsf(runtime_.current.phase_c_a) > config_.max_phase_current_a)
    {
        return foc_result::OUTPUT_RANGE;
    }

    return foc_result::OK;
}

/**
 * @brief 执行 Clarke、Park 和 D-Q 电流 PI 运算
 *
 * @return 电流控制计算结果
 */
foc_result foc_core::run_current_control()
{
    alpha_beta_current stationary_current = foc_math::clarke(
        runtime_.current);
    d_q_current rotating_current = foc_math::park(stationary_current,
        runtime_.electrical_angle_rad);
    runtime_.i_alpha_a = stationary_current.alpha_a;
    runtime_.i_beta_a = stationary_current.beta_a;
    runtime_.i_d_a = rotating_current.d_a;
    runtime_.i_q_a = rotating_current.q_a;

    if(!isfinite(runtime_.i_alpha_a) || !isfinite(runtime_.i_beta_a) ||
        !isfinite(runtime_.i_d_a) || !isfinite(runtime_.i_q_a))
    {
        return foc_result::INVALID_NUMBER;
    }

    if(active_target_.mode == foc_control_mode::DISABLED)
    {
        runtime_.d_integral = 0.0f;
        runtime_.q_integral = 0.0f;
        runtime_.u_d_v = 0.0f;
        runtime_.u_q_v = 0.0f;
        return foc_result::OK;
    }

    if(active_target_.mode != foc_control_mode::CURRENT)
    {
        return foc_result::INVALID_STATE;
    }

    bool d_axis_valid = foc_math::run_pi(
        active_target_.d_axis_current_a,
        runtime_.i_d_a,
        config_.control_period_s,
        config_.d_axis_pi,
        config_.voltage_limit_v,
        runtime_.d_integral,
        runtime_.u_d_v);
    bool q_axis_valid = foc_math::run_pi(
        active_target_.q_axis_current_a,
        runtime_.i_q_a,
        config_.control_period_s,
        config_.q_axis_pi,
        config_.voltage_limit_v,
        runtime_.q_integral,
        runtime_.u_q_v);
    if(!d_axis_valid || !q_axis_valid)
    {
        return foc_result::INVALID_NUMBER;
    }

    d_q_voltage voltage{};
    voltage.d_v = runtime_.u_d_v;
    voltage.q_v = runtime_.u_q_v;
    if(!foc_math::limit_voltage(voltage, config_.voltage_limit_v))
    {
        return foc_result::INVALID_NUMBER;
    }

    runtime_.u_d_v = voltage.d_v;
    runtime_.u_q_v = voltage.q_v;
    return foc_result::OK;
}

/**
 * @brief 执行反 Park 和 SVPWM 计算
 *
 * @return 输出计算结果
 */
foc_result foc_core::calculate_output()
{
    d_q_voltage rotating_voltage{};
    rotating_voltage.d_v = runtime_.u_d_v;
    rotating_voltage.q_v = runtime_.u_q_v;
    alpha_beta_voltage stationary_voltage = foc_math::inverse_park(
        rotating_voltage,
        runtime_.electrical_angle_rad);
    runtime_.u_alpha_v = stationary_voltage.alpha_v;
    runtime_.u_beta_v = stationary_voltage.beta_v;

    if(!isfinite(runtime_.u_alpha_v) || !isfinite(runtime_.u_beta_v))
    {
        runtime_.duty = {};
        return foc_result::INVALID_NUMBER;
    }

    if(!foc_math::svpwm(stationary_voltage,
        config_.bus_voltage_v,
        runtime_.duty))
    {
        return foc_result::OUTPUT_RANGE;
    }

    return foc_result::OK;
}

/**
 * @brief 将本周期占空比提交给三相驱动
 *
 * @return 驱动输出结果
 */
foc_result foc_core::apply_output()
{
    if(hardware_.phase_driver->fault_active())
    {
        return foc_result::DRIVER_FAULT;
    }

    if(!isfinite(runtime_.duty.phase_a) ||
        !isfinite(runtime_.duty.phase_b) ||
        !isfinite(runtime_.duty.phase_c) ||
        runtime_.duty.phase_a < 0.0f || runtime_.duty.phase_a > 1.0f ||
        runtime_.duty.phase_b < 0.0f || runtime_.duty.phase_b > 1.0f ||
        runtime_.duty.phase_c < 0.0f || runtime_.duty.phase_c > 1.0f)
    {
        return foc_result::OUTPUT_RANGE;
    }

    foc_result result = hardware_.phase_driver->set_duty(runtime_.duty);
    return result == foc_result::OK ? foc_result::OK :
        foc_result::DRIVER_FAULT;
}

/**
 * @brief 结束发生严重错误的控制周期
 *
 * @param fault 本周期故障位
 * @param result 返回给调用方的错误结果
 * @param timestamp_us 本控制周期时间戳，单位微秒
 *
 * @return 传入的错误结果
 */
foc_result foc_core::fail_control_cycle(foc_fault fault,
    foc_result result,
    uint32_t timestamp_us)
{
    enter_fault(fault);
    publish_snapshot(timestamp_us);
    runtime_.control_sequence++;
    return result;
}

/**
 * @brief 关闭功率输出并进入故障状态
 *
 * @param fault 需要记录的故障位
 */
void foc_core::enter_fault(foc_fault fault)
{
    fault_flags_.fetch_or(foc_fault_mask(fault),
        std::memory_order_relaxed);
    if(hardware_.phase_driver)
    {
        hardware_.phase_driver->disable();
    }
    output_active_.store(false, std::memory_order_release);
    reset_control_output();
    state_value_.store(static_cast<uint32_t>(foc_state::FAULT),
        std::memory_order_release);
}

/**
 * @brief 清零 PI、控制电压并恢复中性占空比
 */
void foc_core::reset_control_output()
{
    runtime_.u_d_v = 0.0f;
    runtime_.u_q_v = 0.0f;
    runtime_.u_alpha_v = 0.0f;
    runtime_.u_beta_v = 0.0f;
    runtime_.d_integral = 0.0f;
    runtime_.q_integral = 0.0f;
    runtime_.duty = {};
}

/**
 * @brief 将当前完整运行状态发布到三槽无锁快照缓冲区
 *
 * @param timestamp_us 快照时间戳，单位微秒
 */
void foc_core::publish_snapshot(uint32_t timestamp_us)
{
    uint32_t published_index = published_snapshot_index_.load(
        std::memory_order_acquire);
    uint32_t write_index = INVALID_BUFFER_INDEX;

    for(uint32_t index = 0U; index < BUFFER_COUNT; index++)
    {
        if(index != published_index &&
            snapshot_reader_counts_[index].load(
                std::memory_order_acquire) == 0U)
        {
            write_index = index;
            break;
        }
    }

    if(write_index == INVALID_BUFFER_INDEX){return;}

    foc_snapshot next_snapshot{};
    next_snapshot.sequence = ++snapshot_publish_sequence_;
    next_snapshot.timestamp_us = timestamp_us;
    next_snapshot.state = state();
    next_snapshot.fault_flags = faults();
    next_snapshot.output_active = output_active_.load(
        std::memory_order_acquire) &&
        next_snapshot.state == foc_state::RUNNING;
    next_snapshot.mechanical_angle_rad =
        runtime_.rotor.mechanical_angle_rad;
    next_snapshot.mechanical_velocity_rad_s =
        runtime_.rotor.mechanical_velocity_rad_s;
    next_snapshot.electrical_angle_rad = runtime_.electrical_angle_rad;
    next_snapshot.electrical_velocity_rad_s =
        runtime_.electrical_velocity_rad_s;
    next_snapshot.phase_a_current_a = runtime_.current.phase_a_a;
    next_snapshot.phase_b_current_a = runtime_.current.phase_b_a;
    next_snapshot.phase_c_current_a = runtime_.current.phase_c_a;
    next_snapshot.i_alpha_a = runtime_.i_alpha_a;
    next_snapshot.i_beta_a = runtime_.i_beta_a;
    next_snapshot.i_d_a = runtime_.i_d_a;
    next_snapshot.i_q_a = runtime_.i_q_a;
    next_snapshot.target_i_d_a = active_target_.d_axis_current_a;
    next_snapshot.target_i_q_a = active_target_.q_axis_current_a;
    next_snapshot.u_d_v = runtime_.u_d_v;
    next_snapshot.u_q_v = runtime_.u_q_v;
    next_snapshot.duty = runtime_.duty;
    next_snapshot.bus_voltage_v = config_.bus_voltage_v;
    next_snapshot.bus_current_a = 0.0f;

    snapshot_buffers_[write_index] = next_snapshot;
    published_snapshot_index_.store(write_index,
        std::memory_order_release);
    snapshot_ready_.store(true, std::memory_order_release);
}

/**
 * @brief 获取当前运行时中最近的样本时间戳
 *
 * @return 最近样本时间戳，单位微秒
 */
uint32_t foc_core::latest_timestamp_us() const
{
    return runtime_.rotor.timestamp_us > runtime_.current.timestamp_us ?
        runtime_.rotor.timestamp_us : runtime_.current.timestamp_us;
}

/**
 * @brief 初始化单个 FOC 实例及其硬件抽象
 *
 * @param config 电机和电流环配置
 * @param hardware 该实例独占使用的硬件接口
 *
 * @return 初始化结果
 */
foc_result foc_core::init(const foc_config &config,
    const foc_hardware &hardware)
{
    if(state() != foc_state::UNINITIALIZED)
    {
        return foc_result::INVALID_STATE;
    }

    if(!hardware.rotor_sensor || !hardware.current_sensor ||
        !hardware.phase_driver)
    {
        return foc_result::INVALID_ARGUMENT;
    }

    if(!valid_config(config)){return foc_result::INVALID_CONFIG;}

    config_ = config;
    hardware_ = hardware;
    runtime_ = {};
    active_target_ = {};
    snapshot_publish_sequence_ = 0U;
    fault_flags_.store(0U, std::memory_order_relaxed);
    output_active_.store(false, std::memory_order_relaxed);
    target_command_sequence_.store(0U, std::memory_order_relaxed);
    published_target_index_.store(0U, std::memory_order_relaxed);
    target_reader_index_.store(INVALID_BUFFER_INDEX,
        std::memory_order_relaxed);
    target_writer_busy_.clear(std::memory_order_relaxed);
    target_force_disabled_.store(true, std::memory_order_relaxed);
    published_snapshot_index_.store(0U, std::memory_order_relaxed);
    snapshot_ready_.store(false, std::memory_order_relaxed);

    for(uint32_t index = 0U; index < BUFFER_COUNT; index++)
    {
        target_buffers_[index] = {};
        snapshot_buffers_[index] = {};
        snapshot_reader_counts_[index].store(0U,
            std::memory_order_relaxed);
    }

    foc_result result = hardware_.rotor_sensor->init();
    if(result != foc_result::OK)
    {
        fault_flags_.store(foc_fault_mask(foc_fault::ROTOR_SENSOR),
            std::memory_order_relaxed);
        publish_snapshot(0U);
        return foc_result::SENSOR_ERROR;
    }

    result = hardware_.current_sensor->init();
    if(result != foc_result::OK)
    {
        fault_flags_.store(foc_fault_mask(foc_fault::CURRENT_SENSOR),
            std::memory_order_relaxed);
        publish_snapshot(0U);
        return foc_result::SENSOR_ERROR;
    }

    result = hardware_.phase_driver->init();
    if(result != foc_result::OK)
    {
        fault_flags_.store(foc_fault_mask(foc_fault::DRIVER),
            std::memory_order_relaxed);
        hardware_.phase_driver->disable();
        publish_snapshot(0U);
        return foc_result::DRIVER_FAULT;
    }

    hardware_.phase_driver->disable();
    state_value_.store(static_cast<uint32_t>(foc_state::READY),
        std::memory_order_release);
    publish_snapshot(0U);
    return foc_result::OK;
}

/**
 * @brief 以中性占空比安全开启功率级
 *
 * @return 使能结果
 */
foc_result foc_core::enable()
{
    if(state() == foc_state::UNINITIALIZED)
    {
        return foc_result::NOT_INITIALIZED;
    }
    if(state() != foc_state::READY){return foc_result::INVALID_STATE;}
    if(target_force_disabled_.load(std::memory_order_acquire))
    {
        return foc_result::DISABLED;
    }

    foc_target target{};
    if(!load_target(target)){return foc_result::NOT_READY;}
    if(target.mode != foc_control_mode::CURRENT)
    {
        return foc_result::DISABLED;
    }
    active_target_ = target;

    if(hardware_.phase_driver->fault_active())
    {
        enter_fault(foc_fault::DRIVER);
        publish_snapshot(latest_timestamp_us());
        return foc_result::DRIVER_FAULT;
    }

    reset_control_output();
    foc_result result = hardware_.phase_driver->set_duty(runtime_.duty);
    if(result != foc_result::OK)
    {
        enter_fault(foc_fault::DRIVER);
        publish_snapshot(latest_timestamp_us());
        return foc_result::DRIVER_FAULT;
    }

    result = hardware_.phase_driver->enable();
    if(result != foc_result::OK || hardware_.phase_driver->fault_active())
    {
        enter_fault(foc_fault::DRIVER);
        publish_snapshot(latest_timestamp_us());
        return foc_result::DRIVER_FAULT;
    }

    output_active_.store(true, std::memory_order_release);
    state_value_.store(static_cast<uint32_t>(foc_state::RUNNING),
        std::memory_order_release);
    publish_snapshot(latest_timestamp_us());
    return foc_result::OK;
}

/**
 * @brief 立即关闭功率输出并清除控制输出状态
 */
void foc_core::disable()
{
    foc_state current_state = state();
    if(current_state == foc_state::UNINITIALIZED){return;}

    hardware_.phase_driver->disable();
    output_active_.store(false, std::memory_order_release);
    target_force_disabled_.store(true, std::memory_order_release);

    if(current_state == foc_state::RUNNING)
    {
        state_value_.store(static_cast<uint32_t>(foc_state::READY),
            std::memory_order_release);
    }

    foc_target disabled_target{};
    publish_target(disabled_target);
    active_target_ = disabled_target;
    reset_control_output();
    publish_snapshot(latest_timestamp_us());
}

/**
 * @brief 向高频环无锁发布一条完整控制目标
 *
 * @param target 待发布控制目标
 *
 * @return 目标检查和发布结果
 */
foc_result foc_core::set_target(const foc_target &target)
{
    foc_state current_state = state();
    if(current_state == foc_state::UNINITIALIZED)
    {
        return foc_result::NOT_INITIALIZED;
    }
    if(current_state == foc_state::FAULT)
    {
        return foc_result::INVALID_STATE;
    }
    if(target.mode != foc_control_mode::DISABLED &&
        target.mode != foc_control_mode::CURRENT)
    {
        return foc_result::INVALID_ARGUMENT;
    }
    if(!isfinite(target.d_axis_current_a) ||
        !isfinite(target.q_axis_current_a))
    {
        return foc_result::INVALID_NUMBER;
    }

    foc_target checked_target = target;
    if(checked_target.mode == foc_control_mode::DISABLED)
    {
        checked_target.d_axis_current_a = 0.0f;
        checked_target.q_axis_current_a = 0.0f;
        target_force_disabled_.store(true, std::memory_order_release);
    }
    else
    {
        float target_magnitude_squared_a2 =
            checked_target.d_axis_current_a *
                checked_target.d_axis_current_a +
            checked_target.q_axis_current_a *
                checked_target.q_axis_current_a;
        float current_limit_squared_a2 = config_.max_phase_current_a *
            config_.max_phase_current_a;
        if(!isfinite(target_magnitude_squared_a2) ||
            target_magnitude_squared_a2 > current_limit_squared_a2)
        {
            return foc_result::OUTPUT_RANGE;
        }
    }

    foc_result result = publish_target(checked_target);
    if(result == foc_result::OK &&
        checked_target.mode == foc_control_mode::CURRENT)
    {
        target_force_disabled_.store(false, std::memory_order_release);
    }
    return result;
}

/**
 * @brief 执行一次固定顺序的高频电流 FOC 控制周期
 *
 * @param timestamp_us 本控制周期时间戳，单位微秒
 *
 * @return 本控制周期结果
 */
foc_result foc_core::high_freq_loop(uint32_t timestamp_us)
{
    foc_state current_state = state();
    if(current_state == foc_state::UNINITIALIZED)
    {
        return foc_result::NOT_INITIALIZED;
    }
    if(current_state == foc_state::READY){return foc_result::NOT_READY;}
    if(current_state != foc_state::RUNNING)
    {
        return foc_result::INVALID_STATE;
    }

    if(!hardware_.rotor_sensor || !hardware_.current_sensor ||
        !hardware_.phase_driver)
    {
        return fail_control_cycle(foc_fault::INTERNAL,
            foc_result::INTERNAL_ERROR,
            timestamp_us);
    }
    if(hardware_.phase_driver->fault_active())
    {
        return fail_control_cycle(foc_fault::DRIVER,
            foc_result::DRIVER_FAULT,
            timestamp_us);
    }

    foc_target target{};
    load_target(target);
    if(target_force_disabled_.load(std::memory_order_acquire))
    {
        target = {};
    }
    active_target_ = target;

    foc_result result = update_rotor(timestamp_us);
    if(result == foc_result::SENSOR_ERROR)
    {
        return fail_control_cycle(foc_fault::ROTOR_SENSOR,
            result,
            timestamp_us);
    }
    if(result != foc_result::OK)
    {
        return fail_control_cycle(foc_fault::INVALID_NUMBER,
            result,
            timestamp_us);
    }

    result = update_current(timestamp_us);
    if(result == foc_result::SENSOR_ERROR)
    {
        return fail_control_cycle(foc_fault::CURRENT_SENSOR,
            result,
            timestamp_us);
    }
    if(result == foc_result::OUTPUT_RANGE)
    {
        return fail_control_cycle(foc_fault::OVER_CURRENT,
            result,
            timestamp_us);
    }
    if(result != foc_result::OK)
    {
        return fail_control_cycle(foc_fault::INVALID_NUMBER,
            result,
            timestamp_us);
    }

    result = run_current_control();
    if(result != foc_result::OK)
    {
        foc_fault fault = result == foc_result::INVALID_STATE ?
            foc_fault::INTERNAL : foc_fault::INVALID_NUMBER;
        return fail_control_cycle(fault, result, timestamp_us);
    }

    result = calculate_output();
    if(result != foc_result::OK)
    {
        foc_fault fault = result == foc_result::OUTPUT_RANGE ?
            foc_fault::OUTPUT_RANGE : foc_fault::INVALID_NUMBER;
        return fail_control_cycle(fault, result, timestamp_us);
    }

    result = apply_output();
    if(result != foc_result::OK)
    {
        foc_fault fault = result == foc_result::OUTPUT_RANGE ?
            foc_fault::OUTPUT_RANGE : foc_fault::DRIVER;
        return fail_control_cycle(fault, result, timestamp_us);
    }

    if(hardware_.phase_driver->fault_active())
    {
        return fail_control_cycle(foc_fault::DRIVER,
            foc_result::DRIVER_FAULT,
            timestamp_us);
    }

    publish_snapshot(timestamp_us);
    runtime_.control_sequence++;
    return active_target_.mode == foc_control_mode::DISABLED ?
        foc_result::DISABLED : foc_result::OK;
}

/**
 * @brief 无锁复制最近一次完整控制快照
 *
 * @param output 用于接收快照
 *
 * @return 成功取得完整快照时返回 true
 */
bool foc_core::snapshot(foc_snapshot &output) const
{
    if(!snapshot_ready_.load(std::memory_order_acquire)){return false;}

    for(uint32_t attempt = 0U; attempt < READ_ATTEMPT_COUNT; attempt++)
    {
        uint32_t published_index = published_snapshot_index_.load(
            std::memory_order_acquire);
        snapshot_reader_counts_[published_index].fetch_add(1U,
            std::memory_order_acq_rel);

        if(published_index == published_snapshot_index_.load(
            std::memory_order_acquire))
        {
            output = snapshot_buffers_[published_index];
            snapshot_reader_counts_[published_index].fetch_sub(1U,
                std::memory_order_release);
            return true;
        }

        snapshot_reader_counts_[published_index].fetch_sub(1U,
            std::memory_order_release);
    }

    return false;
}

/**
 * @brief 获取当前 FOC 生命周期状态
 *
 * @return 当前状态
 */
foc_state foc_core::state() const
{
    return static_cast<foc_state>(state_value_.load(
        std::memory_order_acquire));
}

/**
 * @brief 获取当前故障位集合
 *
 * @return foc_fault 位掩码
 */
uint32_t foc_core::faults() const
{
    return fault_flags_.load(std::memory_order_acquire);
}

/**
 * @brief 在功率输出关闭且驱动故障解除后清除故障
 *
 * @return 故障清除结果
 */
foc_result foc_core::clear_fault()
{
    if(state() == foc_state::UNINITIALIZED)
    {
        return foc_result::NOT_INITIALIZED;
    }
    if(state() != foc_state::FAULT){return foc_result::INVALID_STATE;}
    if(output_active_.load(std::memory_order_acquire))
    {
        return foc_result::INVALID_STATE;
    }
    if(!hardware_.rotor_sensor || !hardware_.current_sensor ||
        !hardware_.phase_driver)
    {
        return foc_result::INTERNAL_ERROR;
    }
    if(hardware_.phase_driver->fault_active())
    {
        return foc_result::DRIVER_FAULT;
    }

    hardware_.phase_driver->disable();
    target_force_disabled_.store(true, std::memory_order_release);
    foc_target disabled_target{};
    publish_target(disabled_target);
    active_target_ = disabled_target;
    reset_control_output();
    fault_flags_.store(0U, std::memory_order_release);
    state_value_.store(static_cast<uint32_t>(foc_state::READY),
        std::memory_order_release);
    publish_snapshot(latest_timestamp_us());
    return foc_result::OK;
}

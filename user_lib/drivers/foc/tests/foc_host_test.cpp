#include "foc_host_test.h"

#include "../foc_core.h"
#include "freertos/queue.h"
#include <assert.h>
#include <math.h>
#include <atomic>
#include <limits>
#include <thread>

static constexpr float TEST_TOLERANCE = 1.0e-4f;
static constexpr float HALF_PI = 1.57079632679489661923f;

struct mock_output_state
{
    bool initialized = false;
    bool enabled = false;
    bool driver_fault = false;
    bool isr_apply_called = false;
    phase_duty last_duty{};
    foc_result init_result = foc_result::OK;
    foc_result enable_result = foc_result::OK;
    foc_result write_result = foc_result::OK;
};

/**
 * @brief 把输出回调上下文转换为主机测试输出状态
 *
 * @param context 输出回调上下文
 *
 * @return 主机测试输出状态
 */
static mock_output_state *get_output_state(void *context)
{
    return static_cast<mock_output_state *>(context);
}

/**
 * @brief 初始化主机测试功率输出
 *
 * @param context 输出回调上下文
 *
 * @return 配置的初始化结果
 */
static foc_result mock_output_init(void *context)
{
    mock_output_state *output = get_output_state(context);
    output->initialized = output->init_result == foc_result::OK;
    return output->init_result;
}

/**
 * @brief 使能主机测试功率输出
 *
 * @param context 输出回调上下文
 *
 * @return 配置的使能结果
 */
static foc_result mock_output_enable(void *context)
{
    mock_output_state *output = get_output_state(context);
    if(output->enable_result == foc_result::OK)
    {
        output->enabled = true;
    }
    return output->enable_result;
}

/**
 * @brief 关闭主机测试功率输出
 *
 * @param context 输出回调上下文
 */
static void mock_output_disable(void *context)
{
    get_output_state(context)->enabled = false;
}

/**
 * @brief 记录任务上下文占空比写入
 *
 * @param context 输出回调上下文
 * @param duty 待写入的占空比
 *
 * @return 配置的写入结果
 */
static foc_result mock_output_apply_duty(void *context,
    const phase_duty &duty)
{
    mock_output_state *output = get_output_state(context);
    output->last_duty = duty;
    return output->write_result;
}

/**
 * @brief 记录 ISR 上下文占空比写入
 *
 * @param context 输出回调上下文
 * @param duty 待写入的占空比
 *
 * @return 配置的写入结果
 */
static foc_result mock_output_apply_duty_from_isr(void *context,
    const phase_duty &duty)
{
    mock_output_state *output = get_output_state(context);
    output->isr_apply_called = true;
    output->last_duty = duty;
    return output->write_result;
}

/**
 * @brief 返回主机测试功率级故障状态
 *
 * @param context 输出回调上下文
 *
 * @return 功率级故障有效时返回 true
 */
static bool mock_output_fault_active(void *context)
{
    return get_output_state(context)->driver_fault;
}

/**
 * @brief 创建主机测试功率输出回调表
 *
 * @param output 主机测试输出状态
 *
 * @return 输出回调表
 */
static foc_output make_output(mock_output_state &output)
{
    foc_output result{};
    result.context = &output;
    result.init = mock_output_init;
    result.enable = mock_output_enable;
    result.disable = mock_output_disable;
    result.apply_duty = mock_output_apply_duty;
    result.apply_duty_from_isr = mock_output_apply_duty_from_isr;
    result.fault_active = mock_output_fault_active;
    return result;
}

/**
 * @brief 创建主机测试使用的合法 FOC 配置
 *
 * @return 测试配置
 */
static foc_config make_config()
{
    foc_config config{};
    config.pole_pairs = 7U;
    config.rotor_direction = 1;
    config.control_period_s = 0.00005f;
    config.bus_voltage_v = 12.0f;
    config.voltage_limit_v = 6.0f;
    config.max_phase_current_a = 5.0f;
    config.d_axis_pi.kp = 1.0f;
    config.d_axis_pi.ki = 10.0f;
    config.d_axis_pi.integral_limit = 3.0f;
    config.q_axis_pi = config.d_axis_pi;
    return config;
}

/**
 * @brief 初始化主机测试 FOC 实例
 *
 * @param motor FOC 实例
 * @param output 输出状态
 */
static void initialize_motor(foc_core &motor,
    mock_output_state &output)
{
    assert(motor.init(make_config(), make_output(output)) ==
        foc_result::OK);
}

/**
 * @brief 发布一条任务上下文转子样本
 *
 * @param motor FOC 实例
 * @param timestamp_us 样本时间戳，单位微秒
 * @param mechanical_angle_rad 机械角度，单位弧度
 * @param valid 样本有效标记
 */
static void publish_rotor(foc_core &motor,
    uint32_t timestamp_us,
    float mechanical_angle_rad,
    bool valid = true)
{
    rotor_sample sample{};
    sample.sequence = timestamp_us;
    sample.timestamp_us = timestamp_us;
    sample.mechanical_angle_rad = mechanical_angle_rad;
    sample.mechanical_velocity_rad_s = 2.0f;
    sample.valid = valid;
    foc_topic_access access = motor.topics();
    assert(access.rotor.publish(sample));
}

/**
 * @brief 发布一条任务上下文三相电流样本
 *
 * @param motor FOC 实例
 * @param timestamp_us 样本时间戳，单位微秒
 * @param phase_a_a A 相电流，单位安培
 * @param phase_b_a B 相电流，单位安培
 * @param phase_c_a C 相电流，单位安培
 * @param valid 样本有效标记
 */
static void publish_current(foc_core &motor,
    uint32_t timestamp_us,
    float phase_a_a,
    float phase_b_a,
    float phase_c_a,
    bool valid = true)
{
    phase_current_sample sample{};
    sample.sequence = timestamp_us;
    sample.timestamp_us = timestamp_us;
    sample.phase_a_a = phase_a_a;
    sample.phase_b_a = phase_b_a;
    sample.phase_c_a = phase_c_a;
    sample.valid = valid;
    foc_topic_access access = motor.topics();
    assert(access.current.publish(sample));
}

/**
 * @brief 发布一组任务上下文转子和电流样本
 *
 * @param motor FOC 实例
 * @param timestamp_us 样本时间戳，单位微秒
 * @param mechanical_angle_rad 机械角度，单位弧度
 * @param phase_a_a A 相电流，单位安培
 * @param phase_b_a B 相电流，单位安培
 * @param phase_c_a C 相电流，单位安培
 */
static void publish_samples(foc_core &motor,
    uint32_t timestamp_us,
    float mechanical_angle_rad,
    float phase_a_a,
    float phase_b_a,
    float phase_c_a)
{
    publish_rotor(motor, timestamp_us, mechanical_angle_rad);
    publish_current(motor, timestamp_us, phase_a_a, phase_b_a,
        phase_c_a);
}

/**
 * @brief 发布一组 ISR 上下文转子和电流样本
 *
 * @param motor FOC 实例
 * @param timestamp_us 样本时间戳，单位微秒
 * @param mechanical_angle_rad 机械角度，单位弧度
 * @param higher_priority_task_woken ISR 唤醒标记
 */
static void publish_samples_from_isr(foc_core &motor,
    uint32_t timestamp_us,
    float mechanical_angle_rad,
    BaseType_t &higher_priority_task_woken)
{
    rotor_sample rotor{};
    rotor.sequence = timestamp_us;
    rotor.timestamp_us = timestamp_us;
    rotor.mechanical_angle_rad = mechanical_angle_rad;
    rotor.mechanical_velocity_rad_s = 2.0f;
    rotor.valid = true;

    phase_current_sample current{};
    current.sequence = timestamp_us;
    current.timestamp_us = timestamp_us;
    current.valid = true;

    foc_topic_access access = motor.topics();
    assert(access.rotor.publish_from_isr(rotor,
        higher_priority_task_woken));
    assert(access.current.publish_from_isr(current,
        higher_priority_task_woken));
}

/**
 * @brief 判断两个浮点数是否在测试容差内相等
 *
 * @param left 左操作数
 * @param right 右操作数
 *
 * @return 近似相等时返回 true
 */
static bool nearly_equal(float left, float right)
{
    return fabsf(left - right) <= TEST_TOLERANCE;
}

/**
 * @brief 读取 Snapshot Topic 中的最新快照
 *
 * @param motor FOC 实例
 *
 * @return 最新快照
 */
static foc_snapshot read_snapshot(foc_core &motor)
{
    foc_snapshot snapshot{};
    foc_topic_access access = motor.topics();
    assert(access.snapshot.peek(snapshot, 0U));
    return snapshot;
}

/**
 * @brief 检查三相占空比均处于合法范围
 *
 * @param duty 待检查占空比
 */
static void assert_duty_in_range(const phase_duty &duty)
{
    assert(duty.phase_a >= 0.0f && duty.phase_a <= 1.0f);
    assert(duty.phase_b >= 0.0f && duty.phase_b <= 1.0f);
    assert(duty.phase_c >= 0.0f && duty.phase_c <= 1.0f);
}

/**
 * @brief 验证坐标变换、PI 和 SVPWM 的端到端结果
 */
static void test_control_calculation()
{
    mock_output_state output;
    foc_core motor;
    initialize_motor(motor, output);

    foc_target target{};
    target.mode = foc_control_mode::CURRENT;
    target.q_axis_current_a = 0.5f;
    assert(motor.set_target(target) == foc_result::OK);
    assert(motor.enable() == foc_result::OK);

    publish_samples(motor, 1000U, 0.0f, 1.0f, -0.5f, -0.5f);
    assert(motor.core_loop(1000U) == foc_result::OK);
    foc_snapshot snapshot = read_snapshot(motor);
    assert(nearly_equal(snapshot.i_alpha_a, 1.0f));
    assert(nearly_equal(snapshot.i_beta_a, 0.0f));
    assert(nearly_equal(snapshot.i_d_a, 1.0f));
    assert(nearly_equal(snapshot.i_q_a, 0.0f));
    assert(snapshot.state == foc_state::RUNNING);
    assert(snapshot.fault_flags == 0U);
    assert_duty_in_range(snapshot.duty);
    uint32_t first_snapshot_sequence = snapshot.sequence;

    publish_samples(motor, 1500U, 0.0f, 1.0f, -0.5f, -0.5f);
    assert(motor.core_loop(1500U) == foc_result::OK);
    snapshot = read_snapshot(motor);
    assert(snapshot.sequence == first_snapshot_sequence);

    publish_samples(motor, 2000U, HALF_PI / 7.0f,
        1.0f, -0.5f, -0.5f);
    assert(motor.core_loop(2000U) == foc_result::OK);
    snapshot = read_snapshot(motor);
    assert(snapshot.sequence > first_snapshot_sequence);
    assert(nearly_equal(snapshot.i_d_a, 0.0f));
    assert(nearly_equal(snapshot.i_q_a, -1.0f));
    assert(snapshot.u_q_v > 0.0f);
    assert(snapshot.u_q_v <= 6.0f);
    assert_duty_in_range(snapshot.duty);
}

/**
 * @brief 验证两个核心实例的数据隔离和故障恢复
 */
static void test_instances_and_fault_recovery()
{
    mock_output_state left_output;
    mock_output_state right_output;
    foc_core left_motor;
    foc_core right_motor;
    initialize_motor(left_motor, left_output);
    initialize_motor(right_motor, right_output);

    foc_target left_target{};
    left_target.mode = foc_control_mode::CURRENT;
    left_target.q_axis_current_a = 0.5f;
    foc_target right_target{};
    right_target.mode = foc_control_mode::CURRENT;
    right_target.q_axis_current_a = -0.25f;
    assert(left_motor.set_target(left_target) == foc_result::OK);
    assert(right_motor.set_target(right_target) == foc_result::OK);
    assert(left_motor.enable() == foc_result::OK);
    assert(right_motor.enable() == foc_result::OK);

    publish_samples(left_motor, 1000U, 0.0f, 0.0f, 0.0f, 0.0f);
    publish_samples(right_motor, 1000U, 0.0f, 0.0f, 0.0f, 0.0f);
    assert(left_motor.core_loop(1000U) == foc_result::OK);
    assert(right_motor.core_loop(1000U) == foc_result::OK);
    foc_snapshot left_snapshot = read_snapshot(left_motor);
    foc_snapshot right_snapshot = read_snapshot(right_motor);
    assert(nearly_equal(left_snapshot.target_i_q_a, 0.5f));
    assert(nearly_equal(right_snapshot.target_i_q_a, -0.25f));
    assert(left_snapshot.output_active);
    assert(right_snapshot.output_active);

    publish_samples(left_motor, 2000U, 0.0f, 6.0f, 0.0f, 0.0f);
    assert(left_motor.core_loop(2000U) == foc_result::OUTPUT_RANGE);
    left_snapshot = read_snapshot(left_motor);
    right_snapshot = read_snapshot(right_motor);
    assert(left_snapshot.state == foc_state::FAULT);
    assert(!left_snapshot.output_active);
    assert((left_snapshot.fault_flags &
        foc_fault_mask(foc_fault::OVER_CURRENT)) != 0U);
    assert(!left_output.enabled);
    assert(right_snapshot.state == foc_state::RUNNING);

    assert(left_motor.clear_fault() == foc_result::OK);
    left_snapshot = read_snapshot(left_motor);
    assert(left_snapshot.state == foc_state::READY);
    assert(left_snapshot.fault_flags == 0U);
    assert(left_motor.enable() == foc_result::DISABLED);
    assert(left_motor.set_target(left_target) == foc_result::OK);
    assert(left_motor.enable() == foc_result::OK);
    left_motor.disable();
    left_snapshot = read_snapshot(left_motor);
    assert(left_snapshot.state == foc_state::READY);
    assert(!left_snapshot.output_active);
}

/**
 * @brief 验证传感器样本的 5 ms 新鲜度边界
 */
static void test_sensor_freshness()
{
    mock_output_state rotor_output;
    foc_core rotor_motor;
    initialize_motor(rotor_motor, rotor_output);
    foc_target target{};
    target.mode = foc_control_mode::CURRENT;
    assert(rotor_motor.set_target(target) == foc_result::OK);
    assert(rotor_motor.enable() == foc_result::OK);
    publish_samples(rotor_motor, 1000U, 0.0f, 0.0f, 0.0f, 0.0f);
    assert(rotor_motor.core_loop(1000U) == foc_result::OK);
    assert(rotor_motor.core_loop(6000U) == foc_result::OK);
    assert(rotor_motor.core_loop(6001U) == foc_result::SENSOR_ERROR);
    foc_snapshot snapshot = read_snapshot(rotor_motor);
    assert(snapshot.state == foc_state::FAULT);
    assert((snapshot.fault_flags &
        foc_fault_mask(foc_fault::ROTOR_SENSOR)) != 0U);

    mock_output_state current_output;
    foc_core current_motor;
    initialize_motor(current_motor, current_output);
    assert(current_motor.set_target(target) == foc_result::OK);
    assert(current_motor.enable() == foc_result::OK);
    publish_samples(current_motor, 1000U, 0.0f, 0.0f, 0.0f, 0.0f);
    assert(current_motor.core_loop(1000U) == foc_result::OK);
    publish_rotor(current_motor, 6001U, 0.0f);
    assert(current_motor.core_loop(6001U) == foc_result::SENSOR_ERROR);
    snapshot = read_snapshot(current_motor);
    assert(snapshot.state == foc_state::FAULT);
    assert((snapshot.fault_flags &
        foc_fault_mask(foc_fault::CURRENT_SENSOR)) != 0U);
}

/**
 * @brief 验证普通循环与 ISR 循环使用各自的 API
 */
static void test_isr_loop()
{
    mock_output_state task_output;
    mock_output_state isr_output;
    foc_core task_motor;
    foc_core isr_motor;
    initialize_motor(task_motor, task_output);
    initialize_motor(isr_motor, isr_output);

    foc_target target{};
    target.mode = foc_control_mode::CURRENT;
    target.q_axis_current_a = 0.5f;
    assert(task_motor.set_target(target) == foc_result::OK);
    assert(isr_motor.set_target(target) == foc_result::OK);
    assert(task_motor.enable() == foc_result::OK);
    assert(isr_motor.enable() == foc_result::OK);

    publish_samples(task_motor, 1000U, 0.0f, 0.0f, 0.0f, 0.0f);
    assert(task_motor.core_loop(1000U) == foc_result::OK);

    BaseType_t higher_priority_task_woken = pdFALSE;
    publish_samples_from_isr(isr_motor, 1000U, 0.0f,
        higher_priority_task_woken);
    foc_host_queue::task_peek_calls.store(0U);
    foc_host_queue::isr_peek_calls.store(0U);
    foc_host_queue::isr_overwrite_calls.store(0U);
    assert(isr_motor.core_loop_from_isr(1000U,
        higher_priority_task_woken) == foc_result::OK);
    assert(task_output.isr_apply_called == false);
    assert(isr_output.isr_apply_called);
    assert(foc_host_queue::task_peek_calls.load() == 0U);
    assert(foc_host_queue::isr_peek_calls.load() >= 3U);
    assert(foc_host_queue::isr_overwrite_calls.load() >= 1U);

    foc_snapshot task_snapshot = read_snapshot(task_motor);
    foc_snapshot isr_snapshot = read_snapshot(isr_motor);
    assert(nearly_equal(task_snapshot.i_d_a, isr_snapshot.i_d_a));
    assert(nearly_equal(task_snapshot.i_q_a, isr_snapshot.i_q_a));
    assert(nearly_equal(task_snapshot.duty.phase_a,
        isr_snapshot.duty.phase_a));
    assert(nearly_equal(task_snapshot.duty.phase_b,
        isr_snapshot.duty.phase_b));
    assert(nearly_equal(task_snapshot.duty.phase_c,
        isr_snapshot.duty.phase_c));
}

/**
 * @brief 验证非法目标和功率级故障处理
 */
static void test_invalid_inputs_and_driver_fault()
{
    mock_output_state output;
    foc_core motor;
    initialize_motor(motor, output);

    foc_target invalid_number{};
    invalid_number.mode = foc_control_mode::CURRENT;
    invalid_number.q_axis_current_a =
        std::numeric_limits<float>::quiet_NaN();
    assert(motor.set_target(invalid_number) == foc_result::INVALID_NUMBER);

    foc_target out_of_range{};
    out_of_range.mode = foc_control_mode::CURRENT;
    out_of_range.d_axis_current_a = 4.0f;
    out_of_range.q_axis_current_a = 4.0f;
    assert(motor.set_target(out_of_range) == foc_result::OUTPUT_RANGE);

    foc_target target{};
    target.mode = foc_control_mode::CURRENT;
    assert(motor.set_target(target) == foc_result::OK);
    output.driver_fault = true;
    assert(motor.enable() == foc_result::DRIVER_FAULT);
    foc_snapshot snapshot = read_snapshot(motor);
    assert(snapshot.state == foc_state::FAULT);
    assert((snapshot.fault_flags & foc_fault_mask(foc_fault::DRIVER)) !=
        0U);
    assert(motor.clear_fault() == foc_result::DRIVER_FAULT);
    output.driver_fault = false;
    assert(motor.clear_fault() == foc_result::OK);
}

/**
 * @brief 验证 Target 和 Snapshot Topic 的并发完整性
 */
static void test_concurrent_topics()
{
    mock_output_state output;
    foc_core motor;
    initialize_motor(motor, output);

    foc_target initial_target{};
    initial_target.mode = foc_control_mode::CURRENT;
    initial_target.d_axis_current_a = 0.25f;
    initial_target.q_axis_current_a = -0.25f;
    assert(motor.set_target(initial_target) == foc_result::OK);
    assert(motor.enable() == foc_result::OK);
    publish_samples(motor, 1000U, 0.0f, 0.0f, 0.0f, 0.0f);

    std::atomic<bool> writer_complete{false};
    std::atomic<bool> readers_stop{false};
    std::thread target_writer(
        [&motor, &writer_complete]()
        {
            for(uint32_t index = 1U; index <= 20000U; index++)
            {
                float target_current_a = static_cast<float>(index % 1000U) *
                    0.001f;
                foc_target target{};
                target.mode = foc_control_mode::CURRENT;
                target.d_axis_current_a = target_current_a;
                target.q_axis_current_a = -target_current_a;
                assert(motor.set_target(target) == foc_result::OK);
            }
            writer_complete.store(true, std::memory_order_release);
        });
    auto snapshot_reader = [&motor, &readers_stop]()
    {
        while(!readers_stop.load(std::memory_order_acquire))
        {
            foc_snapshot snapshot = read_snapshot(motor);
            assert(nearly_equal(snapshot.target_i_d_a,
                -snapshot.target_i_q_a));
        }
    };
    std::thread first_snapshot_reader(snapshot_reader);
    std::thread second_snapshot_reader(snapshot_reader);

    for(uint32_t index = 0U; index < 20000U ||
        !writer_complete.load(std::memory_order_acquire); index++)
    {
        uint32_t timestamp_us = 1000U + index * 1000U;
        publish_samples(motor, timestamp_us, 0.0f, 0.0f, 0.0f,
            0.0f);
        assert(motor.core_loop(timestamp_us) == foc_result::OK);
    }

    target_writer.join();
    readers_stop.store(true, std::memory_order_release);
    first_snapshot_reader.join();
    second_snapshot_reader.join();
}

/**
 * @brief 运行 FOC 数学层、Topic 和核心控制测试
 */
void run_foc_host_tests()
{
    test_control_calculation();
    test_instances_and_fault_recovery();
    test_sensor_freshness();
    test_isr_loop();
    test_invalid_inputs_and_driver_fault();
    test_concurrent_topics();
}

/**
 * @brief 运行 FOC 主机测试程序
 *
 * @return 全部断言通过时返回零
 */
int main()
{
    run_foc_host_tests();
    return 0;
}

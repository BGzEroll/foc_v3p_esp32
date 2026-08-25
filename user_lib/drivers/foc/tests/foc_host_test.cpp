#include "foc_host_test.h"

#include "../foc_core.h"
#include "../foc_math.h"
#include <assert.h>
#include <math.h>
#include <atomic>
#include <limits>
#include <thread>

static constexpr float TEST_TOLERANCE = 1.0e-5f;
static constexpr float HALF_PI = 1.57079632679489661923f;

// 提供可控的主机侧转子样本。
class mock_rotor_sensor : public rotor_sensor
{
    public:
        foc_result init() override
        {
            initialized = true;
            return foc_result::OK;
        }

    public:
        foc_result read(uint32_t timestamp_us,
            rotor_sample &sample) override
        {
            sample.sequence++;
            sample.timestamp_us = timestamp_us;
            sample.mechanical_angle_rad = mechanical_angle_rad;
            sample.mechanical_velocity_rad_s =
                mechanical_velocity_rad_s;
            sample.valid = sample_valid;
            return read_result;
        }

    public:
        bool initialized = false;
        bool sample_valid = true;
        float mechanical_angle_rad = 0.1f;
        float mechanical_velocity_rad_s = 2.0f;
        foc_result read_result = foc_result::OK;
};

// 提供可控的主机侧三相电流样本。
class mock_current_sensor : public current_sensor
{
    public:
        foc_result init() override
        {
            initialized = true;
            return foc_result::OK;
        }

    public:
        foc_result read(uint32_t timestamp_us,
            phase_current_sample &sample) override
        {
            sample.sequence++;
            sample.timestamp_us = timestamp_us;
            sample.phase_a_a = phase_a_a;
            sample.phase_b_a = phase_b_a;
            sample.phase_c_a = phase_c_a;
            sample.valid = sample_valid;
            return read_result;
        }

    public:
        bool initialized = false;
        bool sample_valid = true;
        float phase_a_a = 0.0f;
        float phase_b_a = 0.0f;
        float phase_c_a = 0.0f;
        foc_result read_result = foc_result::OK;
};

// 记录核心写入的占空比和功率级状态。
class mock_phase_driver : public phase_driver
{
    public:
        foc_result init() override
        {
            initialized = true;
            return foc_result::OK;
        }

    public:
        foc_result enable() override
        {
            if(driver_fault){return foc_result::DRIVER_FAULT;}
            enabled = true;
            return foc_result::OK;
        }

        void disable() override
        {
            enabled = false;
        }

        foc_result set_duty(const phase_duty &duty) override
        {
            last_duty = duty;
            return write_result;
        }

        bool fault_active() const override
        {
            return driver_fault;
        }

    public:
        bool initialized = false;
        bool enabled = false;
        bool driver_fault = false;
        phase_duty last_duty{};
        foc_result write_result = foc_result::OK;
};

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
 * @brief 验证角度归一化和坐标变换
 */
static void test_coordinate_transforms()
{
    float normalized_angle_rad = foc_math::normalize_angle(-HALF_PI);
    assert(nearly_equal(normalized_angle_rad, 3.0f * HALF_PI));

    phase_current_sample phase_current{};
    phase_current.phase_a_a = 1.0f;
    phase_current.phase_b_a = -0.5f;
    phase_current.phase_c_a = -0.5f;
    alpha_beta_current stationary_current = foc_math::clarke(
        phase_current);
    assert(nearly_equal(stationary_current.alpha_a, 1.0f));
    assert(nearly_equal(stationary_current.beta_a, 0.0f));

    d_q_current rotating_current = foc_math::park(stationary_current,
        HALF_PI);
    assert(nearly_equal(rotating_current.d_a, 0.0f));
    assert(nearly_equal(rotating_current.q_a, -1.0f));

    d_q_voltage rotating_voltage{};
    rotating_voltage.d_v = 0.0f;
    rotating_voltage.q_v = -1.0f;
    alpha_beta_voltage stationary_voltage = foc_math::inverse_park(
        rotating_voltage,
        HALF_PI);
    assert(nearly_equal(stationary_voltage.alpha_v, 1.0f));
    assert(nearly_equal(stationary_voltage.beta_v, 0.0f));
}

/**
 * @brief 验证 PI 限幅和无效浮点防护
 */
static void test_pi_controller()
{
    pi_config config{};
    config.kp = 1.0f;
    config.ki = 2.0f;
    config.integral_limit = 0.5f;
    float integral = 0.0f;
    float output = 0.0f;

    assert(foc_math::run_pi(1.0f,
        0.0f,
        0.1f,
        config,
        2.0f,
        integral,
        output));
    assert(nearly_equal(integral, 0.2f));
    assert(nearly_equal(output, 1.2f));

    float invalid_number = std::numeric_limits<float>::quiet_NaN();
    assert(!foc_math::run_pi(invalid_number,
        0.0f,
        0.1f,
        config,
        2.0f,
        integral,
        output));
    assert(nearly_equal(integral, 0.0f));
    assert(nearly_equal(output, 0.0f));
}

/**
 * @brief 验证 SVPWM 始终输出合法占空比
 */
static void test_svpwm_range()
{
    alpha_beta_voltage voltage{};
    voltage.alpha_v = 2.0f;
    voltage.beta_v = -1.0f;
    phase_duty duty{};
    assert(foc_math::svpwm(voltage, 12.0f, duty));
    assert(duty.phase_a >= 0.0f && duty.phase_a <= 1.0f);
    assert(duty.phase_b >= 0.0f && duty.phase_b <= 1.0f);
    assert(duty.phase_c >= 0.0f && duty.phase_c <= 1.0f);

    voltage.alpha_v = 100.0f;
    assert(!foc_math::svpwm(voltage, 12.0f, duty));
    assert(duty.phase_a >= 0.0f && duty.phase_a <= 1.0f);
    assert(duty.phase_b >= 0.0f && duty.phase_b <= 1.0f);
    assert(duty.phase_c >= 0.0f && duty.phase_c <= 1.0f);
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
 * @brief 验证两个核心实例的数据隔离和基础故障恢复
 */
static void test_foc_core_instances()
{
    mock_rotor_sensor left_rotor;
    mock_current_sensor left_current;
    mock_phase_driver left_phase;
    mock_rotor_sensor right_rotor;
    mock_current_sensor right_current;
    mock_phase_driver right_phase;
    foc_hardware left_hardware{};
    left_hardware.rotor_sensor = &left_rotor;
    left_hardware.current_sensor = &left_current;
    left_hardware.phase_driver = &left_phase;
    foc_hardware right_hardware{};
    right_hardware.rotor_sensor = &right_rotor;
    right_hardware.current_sensor = &right_current;
    right_hardware.phase_driver = &right_phase;
    foc_core left_motor;
    foc_core right_motor;

    assert(left_motor.init(make_config(), left_hardware) == foc_result::OK);
    assert(right_motor.init(make_config(), right_hardware) ==
        foc_result::OK);

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
    assert(left_motor.high_freq_loop(100U) == foc_result::OK);
    assert(right_motor.high_freq_loop(100U) == foc_result::OK);

    foc_snapshot left_snapshot{};
    foc_snapshot right_snapshot{};
    assert(left_motor.snapshot(left_snapshot));
    assert(right_motor.snapshot(right_snapshot));
    assert(nearly_equal(left_snapshot.target_i_q_a, 0.5f));
    assert(nearly_equal(right_snapshot.target_i_q_a, -0.25f));
    assert(left_snapshot.output_active);
    assert(right_snapshot.output_active);

    left_current.phase_a_a = 6.0f;
    assert(left_motor.high_freq_loop(150U) == foc_result::OUTPUT_RANGE);
    assert(left_motor.state() == foc_state::FAULT);
    assert(!left_phase.enabled);
    assert((left_motor.faults() &
        foc_fault_mask(foc_fault::OVER_CURRENT)) != 0U);
    assert(right_motor.state() == foc_state::RUNNING);
    assert(left_motor.clear_fault() == foc_result::OK);
    assert(left_motor.state() == foc_state::READY);
    assert(left_motor.enable() == foc_result::DISABLED);
}

/**
 * @brief 并发验证 Target 和 Snapshot 不会出现字段撕裂
 */
static void test_concurrent_exchange()
{
    mock_rotor_sensor rotor;
    mock_current_sensor current;
    mock_phase_driver phase;
    foc_hardware hardware{};
    hardware.rotor_sensor = &rotor;
    hardware.current_sensor = &current;
    hardware.phase_driver = &phase;
    foc_core motor;
    assert(motor.init(make_config(), hardware) == foc_result::OK);

    foc_target initial_target{};
    initial_target.mode = foc_control_mode::CURRENT;
    initial_target.d_axis_current_a = 0.25f;
    initial_target.q_axis_current_a = -0.25f;
    assert(motor.set_target(initial_target) == foc_result::OK);
    assert(motor.enable() == foc_result::OK);

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
                target.timestamp_us = index;
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
            foc_snapshot current_snapshot{};
            if(motor.snapshot(current_snapshot))
            {
                assert(nearly_equal(current_snapshot.target_i_d_a,
                    -current_snapshot.target_i_q_a));
            }
        }
    };
    std::thread first_snapshot_reader(snapshot_reader);
    std::thread second_snapshot_reader(snapshot_reader);

    uint32_t timestamp_us = 1U;
    for(uint32_t index = 0U; index < 20000U ||
        !writer_complete.load(std::memory_order_acquire); index++)
    {
        assert(motor.high_freq_loop(timestamp_us++) == foc_result::OK);
    }

    target_writer.join();
    readers_stop.store(true, std::memory_order_release);
    first_snapshot_reader.join();
    second_snapshot_reader.join();
}

/**
 * @brief 运行 FOC 数学层和核心的全部断言
 */
void run_foc_host_tests()
{
    test_coordinate_transforms();
    test_pi_controller();
    test_svpwm_range();
    test_foc_core_instances();
    test_concurrent_exchange();
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

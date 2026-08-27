#ifndef FOC_CORE_H
#define FOC_CORE_H

#include "foc_types.h"
#include "system/topic.h"
#include <cstdint>

struct foc_output
{
    void *context = nullptr;

    foc_result (*init)(void *context) = nullptr;
    foc_result (*enable)(void *context) = nullptr;
    void (*disable)(void *context) = nullptr;

    foc_result (*apply_duty)(void *context,
        const phase_duty &duty) = nullptr;
    foc_result (*apply_duty_from_isr)(void *context,
        const phase_duty &duty) = nullptr;

    bool (*fault_active)(void *context) = nullptr;
};

struct foc_topic_access
{
    topic::latest_topic<rotor_sample> &rotor;
    topic::latest_topic<phase_current_sample> &current;
    const topic::latest_topic<foc_snapshot> &snapshot;
};

/**
 * @brief 单个 BLDC 或 PMSM 电机的实时电流 FOC 控制实例
 *
 * @note enable()、disable() 和 clear_fault() 只能在任务上下文调用，且调用
 *       前必须停止本实例使用的控制 ISR，并确认 ISR 已经执行结束。
 *       一个实例只能选择 core_loop() 或 core_loop_from_isr() 其中一种循环。
 */
class foc_core
{
    public:
        foc_core() = default;
        foc_core(const foc_core &) = delete;
        foc_core &operator=(const foc_core &) = delete;

    public:
        foc_result init(const foc_config &config,
            const foc_output &output);

    public:
        foc_result enable();
        void disable();
        foc_result set_target(const foc_target &target);
        foc_result core_loop(uint32_t timestamp_us);
        foc_result core_loop_from_isr(uint32_t timestamp_us,
            BaseType_t &higher_priority_task_woken);
        foc_result clear_fault();
        foc_topic_access topics();

    private:
        struct foc_runtime
        {
            rotor_sample rotor{};
            phase_current_sample current{};
            float electrical_angle_rad = 0.0f;
            float electrical_velocity_rad_s = 0.0f;
            float i_alpha_a = 0.0f;
            float i_beta_a = 0.0f;
            float i_d_a = 0.0f;
            float i_q_a = 0.0f;
            float u_d_v = 0.0f;
            float u_q_v = 0.0f;
            float u_alpha_v = 0.0f;
            float u_beta_v = 0.0f;
            phase_duty duty{};
            float d_integral = 0.0f;
            float q_integral = 0.0f;
        };

    private:
        bool valid_config(const foc_config &config) const;
        bool valid_target(const foc_target &target) const;
        bool valid_rotor_sample(const rotor_sample &sample,
            uint32_t timestamp_us) const;
        bool valid_current_sample(const phase_current_sample &sample,
            uint32_t timestamp_us) const;
        void publish_disabled_target();
        void reset_control_output();
        void enter_fault(foc_fault fault);
        uint32_t latest_timestamp_us() const;
        foc_result update_rotor(const rotor_sample &sample);
        foc_result update_current(const phase_current_sample &sample,
            uint32_t timestamp_us);
        foc_result run_current_control();
        foc_result calculate_output();

        template<bool FROM_ISR>
        bool load_target(foc_target &target);

        template<bool FROM_ISR>
        bool load_rotor(rotor_sample &sample);

        template<bool FROM_ISR>
        bool load_current(phase_current_sample &sample);

        bool output_fault_active() const;

        template<bool FROM_ISR>
        foc_result apply_output();

        template<bool FROM_ISR>
        void publish_snapshot(uint32_t timestamp_us,
            BaseType_t *higher_priority_task_woken,
            bool force);

        template<bool FROM_ISR>
        foc_result fail_control_cycle(foc_fault fault,
            foc_result result,
            uint32_t timestamp_us,
            BaseType_t *higher_priority_task_woken);

        template<bool FROM_ISR>
        foc_result run_control_loop(uint32_t timestamp_us,
            BaseType_t *higher_priority_task_woken);

    private:
        static constexpr uint32_t SENSOR_TIMEOUT_US = 5000U;
        static constexpr uint32_t SNAPSHOT_PERIOD_US = 1000U;
        static constexpr float MAX_SVPWM_VOLTAGE_RATIO =
            0.57735026918962576451f;

        foc_config config_{};
        foc_output output_{};
        topic::latest_topic<foc_target> target_topic_;
        topic::latest_topic<rotor_sample> rotor_topic_;
        topic::latest_topic<phase_current_sample> current_topic_;
        topic::latest_topic<foc_snapshot> snapshot_topic_;
        foc_target active_target_{};
        foc_runtime runtime_{};
        foc_state state_ = foc_state::UNINITIALIZED;
        uint32_t fault_flags_ = 0U;
        bool output_active_ = false;
        bool initialized_ = false;
        uint32_t target_sequence_ = 0U;
        uint32_t snapshot_sequence_ = 0U;
        uint32_t last_snapshot_timestamp_us_ = 0U;
        bool snapshot_has_timestamp_ = false;
};

#endif

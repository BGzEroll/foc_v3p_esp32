#ifndef FOC_CORE_H
#define FOC_CORE_H

#include "foc_types.h"
#include "phase_driver/phase_driver.h"
#include "sensors/current_sensor.h"
#include "sensors/rotor_sensor.h"
#include <atomic>
#include <cstdint>

struct foc_hardware
{
    ::rotor_sensor *rotor_sensor = nullptr;
    ::current_sensor *current_sensor = nullptr;
    ::phase_driver *phase_driver = nullptr;
};

/**
 * @brief 单个 BLDC 或 PMSM 电机的实时电流 FOC 控制实例
 *
 * @note 生命周期 API 与 high_freq_loop() 应由应用层串行调度；set_target()
 * 和 snapshot() 已提供跨上下文的无锁一致性传递。
 */
class foc_core
{
    public:
        foc_core() = default;
        foc_core(const foc_core &) = delete;
        foc_core &operator=(const foc_core &) = delete;

    public:
        foc_result init(const foc_config &config,
            const foc_hardware &hardware);

    public:
        foc_result enable();
        void disable();
        foc_result set_target(const foc_target &target);
        foc_result high_freq_loop(uint32_t timestamp_us);
        bool snapshot(foc_snapshot &output) const;
        foc_state state() const;
        uint32_t faults() const;
        foc_result clear_fault();

    private:
        struct foc_runtime
        {
            uint32_t control_sequence = 0U;
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
        foc_result publish_target(const foc_target &target);
        bool load_target(foc_target &target);
        foc_result update_rotor(uint32_t timestamp_us);
        foc_result update_current(uint32_t timestamp_us);
        foc_result run_current_control();
        foc_result calculate_output();
        foc_result apply_output();
        foc_result fail_control_cycle(foc_fault fault,
            foc_result result,
            uint32_t timestamp_us);
        void enter_fault(foc_fault fault);
        void reset_control_output();
        void publish_snapshot(uint32_t timestamp_us);
        uint32_t latest_timestamp_us() const;

    private:
        static constexpr uint32_t BUFFER_COUNT = 3U;
        static constexpr uint32_t INVALID_BUFFER_INDEX = BUFFER_COUNT;
        static constexpr uint32_t READ_ATTEMPT_COUNT = 3U;

        foc_config config_{};
        foc_hardware hardware_{};
        foc_target target_buffers_[BUFFER_COUNT]{};
        std::atomic<uint32_t> published_target_index_{0U};
        std::atomic<uint32_t> target_reader_index_{INVALID_BUFFER_INDEX};
        std::atomic_flag target_writer_busy_ = ATOMIC_FLAG_INIT;
        std::atomic<uint32_t> target_command_sequence_{0U};
        std::atomic<bool> target_force_disabled_{true};
        foc_target active_target_{};
        foc_runtime runtime_{};
        uint32_t snapshot_publish_sequence_ = 0U;
        foc_snapshot snapshot_buffers_[BUFFER_COUNT]{};
        std::atomic<uint32_t> published_snapshot_index_{0U};
        mutable std::atomic<uint32_t>
            snapshot_reader_counts_[BUFFER_COUNT]{};
        std::atomic<bool> snapshot_ready_{false};
        std::atomic<uint32_t> state_value_{
            static_cast<uint32_t>(foc_state::UNINITIALIZED)};
        std::atomic<uint32_t> fault_flags_{0U};
        std::atomic<bool> output_active_{false};
};

#endif

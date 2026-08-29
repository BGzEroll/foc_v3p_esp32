#ifndef FOC_TYPES_H
#define FOC_TYPES_H

#include <cstdint>

enum class foc_result : uint8_t
{
    OK = 0,
    INVALID_ARGUMENT,
    INVALID_CONFIG,
    INVALID_STATE,
    NOT_INITIALIZED,
    NOT_READY,
    DISABLED,
    SENSOR_ERROR,
    DRIVER_FAULT,
    INVALID_NUMBER,
    OUTPUT_RANGE,
    INTERNAL_ERROR
};

enum class foc_state : uint8_t
{
    UNINITIALIZED = 0,
    READY,
    RUNNING,
    FAULT
};

enum class foc_control_mode : uint8_t
{
    DISABLED = 0,
    CURRENT
};

enum class foc_fault : uint32_t
{
    NONE = 0,
    ROTOR_SENSOR = 1U << 0,
    CURRENT_SENSOR = 1U << 1,
    OVER_CURRENT = 1U << 2,
    DRIVER = 1U << 3,
    INVALID_NUMBER = 1U << 4,
    OUTPUT_RANGE = 1U << 5,
    INTERNAL = 1U << 6
};

struct pi_config
{
    float kp = 0.0f;
    float ki = 0.0f;
    float integral_limit = 0.0f;
};

struct rotor_sample
{
    uint32_t sequence = 0;
    uint32_t timestamp_us = 0;
    float mechanical_angle_rad = 0.0f;
    float mechanical_velocity_rad_s = 0.0f;
    bool valid = false;
};

struct phase_current_sample
{
    uint32_t sequence = 0;
    uint32_t timestamp_us = 0;
    float phase_a_a = 0.0f;
    float phase_b_a = 0.0f;
    float phase_c_a = 0.0f;
    bool valid = false;
};

struct phase_duty
{
    float phase_a = 0.5f;
    float phase_b = 0.5f;
    float phase_c = 0.5f;
};

struct foc_config
{
    uint8_t pole_pairs = 0;
    int8_t rotor_direction = 1;
    float electrical_zero_offset_rad = 0.0f;
    float control_period_s = 0.00005f;
    float bus_voltage_v = 0.0f;
    float voltage_limit_v = 0.0f;
    float max_phase_current_a = 0.0f;
    pi_config d_axis_pi{};
    pi_config q_axis_pi{};
};

struct foc_target
{
    uint32_t sequence = 0;
    uint32_t timestamp_us = 0;
    foc_control_mode mode = foc_control_mode::DISABLED;
    float d_axis_current_a = 0.0f;
    float q_axis_current_a = 0.0f;
};

struct foc_snapshot
{
    uint32_t sequence = 0;
    uint32_t timestamp_us = 0;
    foc_state state = foc_state::UNINITIALIZED;
    uint32_t fault_flags = 0;
    bool output_active = false;
    float mechanical_angle_rad = 0.0f;
    float mechanical_velocity_rad_s = 0.0f;
    float electrical_angle_rad = 0.0f;
    float electrical_velocity_rad_s = 0.0f;
    float phase_a_current_a = 0.0f;
    float phase_b_current_a = 0.0f;
    float phase_c_current_a = 0.0f;
    float i_alpha_a = 0.0f;
    float i_beta_a = 0.0f;
    float i_d_a = 0.0f;
    float i_q_a = 0.0f;
    float target_i_d_a = 0.0f;
    float target_i_q_a = 0.0f;
    float u_d_v = 0.0f;
    float u_q_v = 0.0f;
    phase_duty duty{};
    float bus_voltage_v = 0.0f;
    float bus_current_a = 0.0f;
};

constexpr uint32_t foc_fault_mask(foc_fault fault)
{
    return static_cast<uint32_t>(fault);
}

#endif

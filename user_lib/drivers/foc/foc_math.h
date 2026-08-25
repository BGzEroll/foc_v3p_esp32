#ifndef FOC_MATH_H
#define FOC_MATH_H

#include "foc_types.h"

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

namespace foc_math
{
    float normalize_angle(float angle_rad);
    alpha_beta_current clarke(const phase_current_sample &current);
    d_q_current park(const alpha_beta_current &current,
        float electrical_angle_rad);
    alpha_beta_voltage inverse_park(const d_q_voltage &voltage,
        float electrical_angle_rad);
    bool run_pi(float target,
        float measured,
        float period_s,
        const pi_config &config,
        float output_limit_v,
        float &integral,
        float &output);
    bool limit_voltage(d_q_voltage &voltage, float magnitude_limit_v);
    bool svpwm(const alpha_beta_voltage &voltage,
        float bus_voltage_v,
        phase_duty &duty);
}

#endif

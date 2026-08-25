#ifndef ROTOR_SENSOR_H
#define ROTOR_SENSOR_H

#include "../foc_types.h"

/**
 * @brief 为 FOC 核心提供机械角度和机械角速度样本
 *
 * @note 用于高频环的 read() 实现必须是非阻塞的，并返回已经准备好的样本。
 */
class rotor_sensor
{
    public:
        virtual ~rotor_sensor() = default;

    public:
        virtual foc_result init() = 0;

    public:
        virtual foc_result read(uint32_t timestamp_us,
            rotor_sample &sample) = 0;
};

#endif

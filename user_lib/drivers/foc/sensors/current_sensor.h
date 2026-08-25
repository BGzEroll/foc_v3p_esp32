#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H

#include "../foc_types.h"

/**
 * @brief 为 FOC 核心提供同步后的三相电流样本
 *
 * @note 用于高频环的 read() 实现必须是非阻塞的，并返回已经准备好的样本。
 */
class current_sensor
{
    public:
        virtual ~current_sensor() = default;

    public:
        virtual foc_result init() = 0;

    public:
        virtual foc_result read(uint32_t timestamp_us,
            phase_current_sample &sample) = 0;
};

#endif

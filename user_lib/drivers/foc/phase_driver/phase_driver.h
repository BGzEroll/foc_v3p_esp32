#ifndef PHASE_DRIVER_H
#define PHASE_DRIVER_H

#include "../foc_types.h"

/**
 * @brief 接收三相占空比并控制具体的功率级
 *
 * @note 用于高频环的 set_duty() 和 fault_active() 实现必须是非阻塞的。
 */
class phase_driver
{
    public:
        virtual ~phase_driver() = default;

    public:
        virtual foc_result init() = 0;

    public:
        virtual foc_result enable() = 0;
        virtual void disable() = 0;
        virtual foc_result set_duty(const phase_duty &duty) = 0;
        virtual bool fault_active() const = 0;
};

#endif

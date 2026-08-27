#ifndef ROTOR_SENSOR_H
#define ROTOR_SENSOR_H

#include "../foc_types.h"

#if defined(ESP_PLATFORM)
#include "esp_attr.h"
#define FOC_SENSOR_IRAM_ATTR IRAM_ATTR
#else
#define FOC_SENSOR_IRAM_ATTR
#endif

/**
 * @brief 可选的转子传感器适配接口
 *
 * @note foc_core 不持有或调用此接口。应用层负责读取样本并发布到 rotor
 *       Topic。
 */
class rotor_sensor
{
    public:
        virtual ~rotor_sensor() = default;

    public:
        /**
         * @brief 初始化转子传感器
         *
         * @return 初始化结果
         */
        virtual foc_result init() = 0;

    public:
        /**
         * @brief 在任务上下文读取转子样本
         *
         * @param timestamp_us 当前采样时间戳，单位微秒
         * @param sample 用于接收转子样本
         *
         * @return 读取结果
         *
         * @note 允许执行 I2C、UART 等任务上下文操作。调用方读取成功后负责
         *       发布样本到 rotor Topic。
         */
        virtual foc_result read(uint32_t timestamp_us,
            rotor_sample &sample) = 0;

    public:
        /**
         * @brief 在 ISR 上下文读取已经准备好的转子样本
         *
         * @param timestamp_us 当前采样时间戳，单位微秒
         * @param sample 用于接收转子样本
         *
         * @return 读取结果
         *
         * @note 实现必须位于 IRAM，且只能执行非阻塞、无锁、无分配的操作；
         *       不得访问 I2C、UART、日志或延时。实现对象、虚表和上下文也必
         *       须位于 ISR 可访问的内部 RAM。foc_core 不会直接调用此接口。
         */
        virtual foc_result FOC_SENSOR_IRAM_ATTR read_from_isr(
            uint32_t timestamp_us,
            rotor_sample &sample) = 0;
};

#undef FOC_SENSOR_IRAM_ATTR

#endif

#ifndef TOPIC_H
#define TOPIC_H

#include "FreeRTOS.h"
#include "queue.h"
#include <stdint.h>
#include <type_traits>

namespace topic
{
    /**
     * @brief 保存最新完整快照的单槽话题
     *
     * @tparam item_type 需要按值传播的数据类型
     *
     * @note 生产者使用 publish() 覆盖旧快照，消费者使用 peek()
     *       读取但不移除快照。对象必须在所有使用者停止后才能销毁。
     */
    template<typename item_type>
    class latest_topic
    {
        static_assert(std::is_trivially_copyable<item_type>::value,
            "latest_topic item_type must be trivially copyable");

        public:
            latest_topic() = default;
            latest_topic(const latest_topic &) = delete;
            latest_topic &operator=(const latest_topic &) = delete;
            latest_topic(latest_topic &&) = delete;
            latest_topic &operator=(latest_topic &&) = delete;

        public:
            /**
             * @brief 使用对象内部静态存储初始化单槽 Queue
             *
             * @return 初始化成功时返回 true
             *
             * @note 应在创建生产者和消费者任务前调用。重复调用是安全的。
             */
            bool init()
            {
                if(queue_handle){return true;}

                queue_handle = xQueueCreateStatic(1U,
                    sizeof(item_type),
                    queue_storage,
                    &queue_control);
                return queue_handle != nullptr;
            }

        public:
            /**
             * @brief 在任务上下文发布最新快照
             *
             * @param item 待发布的完整快照
             *
             * @return 发布成功时返回 true
             */
            bool publish(const item_type &item)
            {
                if(!queue_handle){return false;}
                return xQueueOverwrite(queue_handle, &item) == pdPASS;
            }

            /**
             * @brief 在任务上下文读取最新快照但不移除数据
             *
             * @param item 用于接收快照的对象
             * @param wait_ticks Queue 尚无首个快照时的最大等待 tick 数
             *
             * @return 成功取得快照时返回 true
             *
             * @note Queue 收到首个快照后会一直保持非空，wait_ticks 不能用于
             *       等待下一次更新。消费者应使用 sequence 判断是否出现新样本。
             */
            bool peek(item_type &item, TickType_t wait_ticks = 0U) const
            {
                if(!queue_handle){return false;}
                return xQueuePeek(queue_handle,
                    &item,
                    wait_ticks) == pdPASS;
            }

            /**
             * @brief 在中断上下文发布最新快照
             *
             * @param item 待发布的完整快照
             * @param higher_priority_task_woken 高优先级任务唤醒标记
             *
             * @return 发布成功时返回 true
             *
             * @note 调用方必须在首次使用前将 higher_priority_task_woken 初始化为
             *       pdFALSE，并在退出 ISR 前调用 portYIELD_FROM_ISR()。
             */
            bool publish_from_isr(const item_type &item,
                BaseType_t &higher_priority_task_woken)
            {
                if(!queue_handle){return false;}
                return xQueueOverwriteFromISR(queue_handle,
                    &item,
                    &higher_priority_task_woken) == pdPASS;
            }

            /**
             * @brief 在中断上下文读取最新快照但不移除数据
             *
             * @param item 用于接收快照的对象
             *
             * @return 成功取得快照时返回 true
             */
            bool peek_from_isr(item_type &item) const
            {
                if(!queue_handle){return false;}
                return xQueuePeekFromISR(queue_handle, &item) == pdPASS;
            }

            /**
             * @brief 查询话题是否已经完成初始化
             *
             * @return 已初始化时返回 true
             */
            bool initialized() const
            {
                return queue_handle != nullptr;
            }

        private:
            StaticQueue_t queue_control{};
            alignas(item_type) uint8_t queue_storage[sizeof(item_type)]{};
            QueueHandle_t queue_handle = nullptr;
    };

    /**
     * @brief 按 FIFO 顺序传递消息的多槽话题
     *
     * @tparam item_type 需要按值传播的消息类型
     * @tparam QUEUE_LENGTH Queue 可保存的消息数量
     *
     * @note publish() 在队尾写入，receive() 读取并移除队首消息。
     *       多个消费者调用 receive() 时是竞争消费，不是广播。Queue
     *       已满时是否等待、重试或丢弃由调用方决定。
     */
    template<typename item_type, UBaseType_t QUEUE_LENGTH>
    class fifo_topic
    {
        static_assert(QUEUE_LENGTH > 0U,
            "fifo_topic QUEUE_LENGTH must be greater than zero");
        static_assert(std::is_trivially_copyable<item_type>::value,
            "fifo_topic item_type must be trivially copyable");

        public:
            fifo_topic() = default;
            fifo_topic(const fifo_topic &) = delete;
            fifo_topic &operator=(const fifo_topic &) = delete;
            fifo_topic(fifo_topic &&) = delete;
            fifo_topic &operator=(fifo_topic &&) = delete;

        public:
            /**
             * @brief 使用对象内部静态存储初始化 FIFO Queue
             *
             * @return 初始化成功时返回 true
             *
             * @note 应在创建生产者和消费者任务前调用。重复调用是安全的。
             */
            bool init()
            {
                if(queue_handle){return true;}

                queue_handle = xQueueCreateStatic(QUEUE_LENGTH,
                    sizeof(item_type),
                    queue_storage,
                    &queue_control);
                return queue_handle != nullptr;
            }

        public:
            /**
             * @brief 在任务上下文向 FIFO 队尾发布消息
             *
             * @param item 待发布消息
             * @param wait_ticks Queue 已满时的最大等待 tick 数
             *
             * @return 成功写入消息时返回 true
             */
            bool publish(const item_type &item,
                TickType_t wait_ticks = 0U)
            {
                if(!queue_handle){return false;}
                return xQueueSendToBack(queue_handle,
                    &item,
                    wait_ticks) == pdPASS;
            }

            /**
             * @brief 在任务上下文读取并移除 FIFO 队首消息
             *
             * @param item 用于接收消息的对象
             * @param wait_ticks Queue 为空时的最大等待 tick 数
             *
             * @return 成功取得消息时返回 true
             */
            bool receive(item_type &item,
                TickType_t wait_ticks = 0U)
            {
                if(!queue_handle){return false;}
                return xQueueReceive(queue_handle,
                    &item,
                    wait_ticks) == pdPASS;
            }

            /**
             * @brief 在中断上下文向 FIFO 队尾发布消息
             *
             * @param item 待发布消息
             * @param higher_priority_task_woken 高优先级任务唤醒标记
             *
             * @return 成功写入消息时返回 true
             */
            bool publish_from_isr(const item_type &item,
                BaseType_t &higher_priority_task_woken)
            {
                if(!queue_handle){return false;}
                return xQueueSendToBackFromISR(queue_handle,
                    &item,
                    &higher_priority_task_woken) == pdPASS;
            }

            /**
             * @brief 在中断上下文读取并移除 FIFO 队首消息
             *
             * @param item 用于接收消息的对象
             * @param higher_priority_task_woken 高优先级任务唤醒标记
             *
             * @return 成功取得消息时返回 true
             */
            bool receive_from_isr(item_type &item,
                BaseType_t &higher_priority_task_woken)
            {
                if(!queue_handle){return false;}
                return xQueueReceiveFromISR(queue_handle,
                    &item,
                    &higher_priority_task_woken) == pdPASS;
            }

            /**
             * @brief 查询 Queue 中等待消费的消息数量
             *
             * @return 当前消息数量，尚未初始化时返回 0
             */
            UBaseType_t waiting_count() const
            {
                if(!queue_handle){return 0U;}
                return uxQueueMessagesWaiting(queue_handle);
            }

            /**
             * @brief 查询话题是否已经完成初始化
             *
             * @return 已初始化时返回 true
             */
            bool initialized() const
            {
                return queue_handle != nullptr;
            }

        private:
            StaticQueue_t queue_control{};
            alignas(item_type) uint8_t queue_storage[
                sizeof(item_type) * QUEUE_LENGTH]{};
            QueueHandle_t queue_handle = nullptr;
    };
}

#endif

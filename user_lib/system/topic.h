#ifndef TOPIC_H
#define TOPIC_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdint.h>
#include <type_traits>
#include "esp_attr.h"

namespace topic
{
    /**
     * @brief 保存最新完整快照的无锁话题
     *
     * @tparam item_type 需要按值传播的数据类型
     *
     * @note 该话题适合一个生产者和多个消费者。生产者使用 publish()
     *       覆盖旧快照，消费者使用 peek() 读取但不移除快照。数据通过
     *       版本号提交，任务和 ISR 之间不使用 Queue 自旋锁。
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
             * @brief 初始化对象内部快照存储
             *
             * @return 初始化成功时返回 true
             *
             * @note 应在创建生产者和消费者任务前调用。重复调用是安全的。
             */
            bool init()
            {
                initialized_flag = true;
                return true;
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
                if(!initialized_flag){return false;}
                return write_snapshot(item);
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
            bool peek(item_type &item, TickType_t wait_ticks = 0) const
            {
                if(!initialized_flag){return false;}
                if(read_snapshot(item)){return true;}
                if(wait_ticks == 0){return false;}

                TickType_t start_tick = xTaskGetTickCount();
                while(static_cast<TickType_t>(
                    xTaskGetTickCount() - start_tick) < wait_ticks)
                {
                    vTaskDelay(1);
                    if(read_snapshot(item)){return true;}
                }
                return read_snapshot(item);
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
            bool IRAM_ATTR publish_from_isr(const item_type &item,
                BaseType_t &higher_priority_task_woken)
            {
                (void)higher_priority_task_woken;
                if(!initialized_flag){return false;}
                return write_snapshot_from_isr(item);
            }

            /**
             * @brief 在中断上下文读取最新快照但不移除数据
             *
             * @param item 用于接收快照的对象
             *
             * @return 成功取得快照时返回 true
             */
            bool IRAM_ATTR peek_from_isr(item_type &item) const
            {
                if(!initialized_flag){return false;}
                return read_snapshot(item);
            }

            /**
             * @brief 查询话题是否已经完成初始化
             *
             * @return 已初始化时返回 true
             */
            bool initialized() const
            {
                return initialized_flag;
            }

        private:
            static void IRAM_ATTR copy_to_storage(
                volatile uint8_t *storage,
                const item_type &item)
            {
                const uint8_t *source = reinterpret_cast<const uint8_t *>(
                    &item);
                for(uint32_t index = 0; index < sizeof(item_type); index++)
                {
                    storage[index] = source[index];
                }
            }

            static void IRAM_ATTR copy_from_storage(
                item_type &item,
                const volatile uint8_t *storage)
            {
                uint8_t *destination = reinterpret_cast<uint8_t *>(&item);
                for(uint32_t index = 0; index < sizeof(item_type); index++)
                {
                    destination[index] = storage[index];
                }
            }

            bool IRAM_ATTR write_snapshot(const item_type &item)
            {
                uint32_t current_version = __atomic_load_n(&version,
                    __ATOMIC_RELAXED);
                if((current_version & 1U) != 0U){return false;}

                __atomic_store_n(&version,
                    current_version + 1U,
                    __ATOMIC_RELEASE);
                copy_to_storage(storage, item);
                __atomic_store_n(&version,
                    current_version + 2U,
                    __ATOMIC_RELEASE);
                return true;
            }

            bool IRAM_ATTR write_snapshot_from_isr(const item_type &item)
            {
                return write_snapshot(item);
            }

            bool IRAM_ATTR read_snapshot(item_type &item) const
            {
                for(uint32_t attempt = 0; attempt < 2; attempt++)
                {
                    uint32_t version_before = __atomic_load_n(&version,
                        __ATOMIC_ACQUIRE);
                    if((version_before & 1U) != 0U){continue;}

                    copy_from_storage(item, storage);
                    uint32_t version_after = __atomic_load_n(&version,
                        __ATOMIC_ACQUIRE);
                    if(version_before == version_after)
                    {
                        return version_before != 0U;
                    }
                }
                return false;
            }

            alignas(item_type) volatile uint8_t storage[sizeof(item_type)]{};
            volatile uint32_t version = 0;
            bool initialized_flag = false;
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
        static_assert(QUEUE_LENGTH > 0,
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
                TickType_t wait_ticks = 0)
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
                TickType_t wait_ticks = 0)
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
            bool IRAM_ATTR publish_from_isr(const item_type &item,
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
             *
             * @return 成功取得消息时返回 true
             */
            bool IRAM_ATTR receive_from_isr(item_type &item,
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
                if(!queue_handle){return 0;}
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

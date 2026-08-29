#ifndef FOC_HOST_QUEUE_H
#define FOC_HOST_QUEUE_H

#include "freertos/FreeRTOS.h"
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <mutex>

struct host_static_queue
{
    std::mutex mutex;
    uint8_t *storage = nullptr;
    UBaseType_t queue_length = 0;
    UBaseType_t item_size = 0;
    bool has_item = false;
};

using StaticQueue_t = host_static_queue;
using QueueHandle_t = host_static_queue *;

namespace foc_host_queue
{
    inline std::atomic<uint32_t> task_overwrite_calls{0};
    inline std::atomic<uint32_t> task_peek_calls{0};
    inline std::atomic<uint32_t> isr_overwrite_calls{0};
    inline std::atomic<uint32_t> isr_peek_calls{0};
}

inline QueueHandle_t xQueueCreateStatic(UBaseType_t queue_length,
    UBaseType_t item_size,
    uint8_t *queue_storage,
    StaticQueue_t *queue_control)
{
    if(queue_length != 1 || item_size == 0 || !queue_storage ||
        !queue_control)
    {
        return nullptr;
    }

    queue_control->storage = queue_storage;
    queue_control->queue_length = queue_length;
    queue_control->item_size = item_size;
    queue_control->has_item = false;
    return queue_control;
}

inline BaseType_t xQueueSendToBack(QueueHandle_t queue,
    const void *item,
    TickType_t)
{
    if(!queue || !item){return 0;}

    std::lock_guard<std::mutex> lock(queue->mutex);
    if(queue->has_item && queue->queue_length == 1){return 0;}
    std::memcpy(queue->storage, item, queue->item_size);
    queue->has_item = true;
    return pdPASS;
}

inline BaseType_t xQueueReceive(QueueHandle_t queue,
    void *item,
    TickType_t)
{
    if(!queue || !item){return 0;}

    std::lock_guard<std::mutex> lock(queue->mutex);
    if(!queue->has_item){return 0;}
    std::memcpy(item, queue->storage, queue->item_size);
    queue->has_item = false;
    return pdPASS;
}

inline BaseType_t xQueueSendToBackFromISR(QueueHandle_t queue,
    const void *item,
    BaseType_t *)
{
    if(!queue || !item){return 0;}

    std::lock_guard<std::mutex> lock(queue->mutex);
    if(queue->has_item && queue->queue_length == 1){return 0;}
    std::memcpy(queue->storage, item, queue->item_size);
    queue->has_item = true;
    return pdPASS;
}

inline BaseType_t xQueueReceiveFromISR(QueueHandle_t queue,
    void *item,
    BaseType_t *)
{
    if(!queue || !item){return 0;}

    std::lock_guard<std::mutex> lock(queue->mutex);
    if(!queue->has_item){return 0;}
    std::memcpy(item, queue->storage, queue->item_size);
    queue->has_item = false;
    return pdPASS;
}

inline UBaseType_t uxQueueMessagesWaiting(QueueHandle_t queue)
{
    if(!queue){return 0;}

    std::lock_guard<std::mutex> lock(queue->mutex);
    return queue->has_item ? 1 : 0;
}

inline BaseType_t xQueueOverwrite(QueueHandle_t queue,
    const void *item)
{
    if(!queue || !item){return 0;}

    foc_host_queue::task_overwrite_calls.fetch_add(1);
    std::lock_guard<std::mutex> lock(queue->mutex);
    std::memcpy(queue->storage, item, queue->item_size);
    queue->has_item = true;
    return pdPASS;
}

inline BaseType_t xQueuePeek(QueueHandle_t queue,
    void *item,
    TickType_t)
{
    if(!queue || !item){return 0;}

    foc_host_queue::task_peek_calls.fetch_add(1);
    std::lock_guard<std::mutex> lock(queue->mutex);
    if(!queue->has_item){return 0;}
    std::memcpy(item, queue->storage, queue->item_size);
    return pdPASS;
}

inline BaseType_t xQueueOverwriteFromISR(QueueHandle_t queue,
    const void *item,
    BaseType_t *)
{
    if(!queue || !item){return 0;}

    foc_host_queue::isr_overwrite_calls.fetch_add(1);
    std::lock_guard<std::mutex> lock(queue->mutex);
    std::memcpy(queue->storage, item, queue->item_size);
    queue->has_item = true;
    return pdPASS;
}

inline BaseType_t xQueuePeekFromISR(QueueHandle_t queue,
    void *item)
{
    if(!queue || !item){return 0;}

    foc_host_queue::isr_peek_calls.fetch_add(1);
    std::lock_guard<std::mutex> lock(queue->mutex);
    if(!queue->has_item){return 0;}
    std::memcpy(item, queue->storage, queue->item_size);
    return pdPASS;
}

#endif

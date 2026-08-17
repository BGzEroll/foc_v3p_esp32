#include "uart_bus.h"

#include "usart.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "stream_buffer.h"
#include "task.h"

static constexpr uint16_t UART_RX_DMA_BUFFER_SIZE = 256;
static constexpr uint16_t UART_RX_STREAM_STORAGE_SIZE = 1025;

// 管理一条物理 UART 的 DMA 收发、任务同步和接收缓存。
class uart_dev
{
    public:
        explicit uart_dev(UART_HandleTypeDef *handle);

    public:
        uart_result init();
        uart_result read_bytes(uint8_t *data,
            uint16_t max_size,
            uint16_t &received_size,
            uint32_t read_timeout_ms,
            uint32_t lock_timeout_ms);
        uart_result write_bytes(const uint8_t *data,
            uint16_t size,
            uint32_t lock_timeout_ms,
            uint32_t transfer_timeout_ms);
        uint32_t rx_dropped_bytes() const;
        uint32_t rx_error_count() const;
        uart_result last_rx_error() const;
        bool matches_handle(UART_HandleTypeDef *target_handle) const;
        void receive_event_from_isr(uint16_t position);
        void complete_tx_from_isr(uart_result result);
        void handle_error_from_isr(uart_result result);

    private:
        uart_result ensure_receive_active();
        bool restart_receive_from_isr();
        bool recover_transmit();
        void cancel_active_transmit();
        void send_received_chunk_from_isr(const uint8_t *data,
            uint16_t size,
            BaseType_t &higher_priority_task_woken);

    private:
        UART_HandleTypeDef *handle;
        SemaphoreHandle_t tx_mutex = nullptr;
        StaticSemaphore_t tx_mutex_storage{};
        SemaphoreHandle_t rx_mutex = nullptr;
        StaticSemaphore_t rx_mutex_storage{};
        SemaphoreHandle_t tx_completion_semaphore = nullptr;
        StaticSemaphore_t tx_completion_semaphore_storage{};
        StreamBufferHandle_t rx_stream = nullptr;
        StaticStreamBuffer_t rx_stream_control{};
        uint8_t rx_stream_storage[UART_RX_STREAM_STORAGE_SIZE]{};
        uint8_t rx_dma_buffer[UART_RX_DMA_BUFFER_SIZE]{};
        bool initialized = false;
        volatile bool tx_active = false;
        volatile bool rx_active = false;
        volatile uint16_t rx_dma_position = 0;
        volatile uart_result tx_result = uart_result::NOT_INITIALIZED;
        volatile uart_result receive_error = uart_result::OK;
        volatile uint32_t receive_error_count = 0;
        volatile uint32_t receive_dropped_bytes = 0;
};

static uart_dev uart_devs[] =
{
    uart_dev(&huart1)
};

static constexpr uint8_t UART_DEV_COUNT =
    (uint8_t)(sizeof(uart_devs) / sizeof(uart_devs[0]));

/**
 * @brief 将毫秒超时转换为 FreeRTOS tick
 *
 * @param timeout_ms 超时时间，单位毫秒
 *
 * @return FreeRTOS tick 数
 */
static TickType_t milliseconds_to_ticks(uint32_t timeout_ms)
{
    TickType_t ticks = pdMS_TO_TICKS(timeout_ms);

    if(timeout_ms > 0U && ticks == 0U)
    {
        ticks = 1U;
    }

    return ticks;
}

/**
 * @brief 根据 HAL UART 错误码生成总线结果
 *
 * @param handle HAL UART 句柄
 *
 * @return UART 总线结果
 */
static uart_result map_hal_error(UART_HandleTypeDef *handle)
{
    uint32_t error = HAL_UART_GetError(handle);

    if((error & HAL_UART_ERROR_DMA) != 0U)
    {
        return uart_result::DMA_ERROR;
    }

    if((error & HAL_UART_ERROR_ORE) != 0U)
    {
        return uart_result::OVERRUN_ERROR;
    }

    if((error & HAL_UART_ERROR_NE) != 0U)
    {
        return uart_result::NOISE_ERROR;
    }

    if((error & HAL_UART_ERROR_FE) != 0U)
    {
        return uart_result::FRAME_ERROR;
    }

    if((error & HAL_UART_ERROR_PE) != 0U)
    {
        return uart_result::PARITY_ERROR;
    }

    return uart_result::BUS_ERROR;
}

/**
 * @brief 将 HAL 状态转换为 UART 总线结果
 *
 * @param handle HAL UART 句柄
 * @param status HAL 调用状态
 *
 * @return UART 总线结果
 */
static uart_result map_hal_status(UART_HandleTypeDef *handle,
    HAL_StatusTypeDef status)
{
    switch(status)
    {
        case HAL_OK:
            return uart_result::OK;

        case HAL_BUSY:
            return uart_result::BUSY;

        case HAL_TIMEOUT:
            return uart_result::TRANSFER_TIMEOUT;

        case HAL_ERROR:
        default:
            return map_hal_error(handle);
    }
}

/**
 * @brief 根据总线编号获取物理 UART 设备
 *
 * @param bus_id UART 总线编号
 *
 * @return 有效编号对应的设备指针，无效编号返回 nullptr
 */
static uart_dev *get_dev(uint8_t bus_id)
{
    if(bus_id >= UART_DEV_COUNT)
    {
        return nullptr;
    }

    return &uart_devs[bus_id];
}

/**
 * @brief 根据 HAL 句柄获取物理 UART 设备
 *
 * @param handle HAL UART 句柄
 *
 * @return 匹配的设备指针，未匹配时返回 nullptr
 */
static uart_dev *get_dev(UART_HandleTypeDef *handle)
{
    for(uint8_t index = 0; index < UART_DEV_COUNT; index++)
    {
        if(uart_devs[index].matches_handle(handle))
        {
            return &uart_devs[index];
        }
    }

    return nullptr;
}

/**
 * @brief 创建物理 UART 管理对象
 *
 * @param handle HAL UART 句柄
 */
uart_dev::uart_dev(UART_HandleTypeDef *handle)
    : handle(handle)
{
}

/**
 * @brief 初始化 UART 的 FreeRTOS 同步资源和循环 RX DMA
 *
 * @return UART 总线结果
 */
uart_result uart_dev::init()
{
    if(initialized)
    {
        return uart_result::OK;
    }

    if(!handle || !handle->Instance || !handle->hdmarx || !handle->hdmatx ||
        handle->hdmarx->Init.Mode != DMA_CIRCULAR)
    {
        return uart_result::INIT_FAILED;
    }

    tx_mutex = xSemaphoreCreateMutexStatic(&tx_mutex_storage);
    rx_mutex = xSemaphoreCreateMutexStatic(&rx_mutex_storage);
    tx_completion_semaphore =
        xSemaphoreCreateBinaryStatic(&tx_completion_semaphore_storage);
    rx_stream = xStreamBufferCreateStatic(UART_RX_STREAM_STORAGE_SIZE,
        1U,
        rx_stream_storage,
        &rx_stream_control);

    if(!tx_mutex || !rx_mutex || !tx_completion_semaphore || !rx_stream)
    {
        tx_mutex = nullptr;
        rx_mutex = nullptr;
        tx_completion_semaphore = nullptr;
        rx_stream = nullptr;
        return uart_result::INIT_FAILED;
    }

    rx_dma_position = 0U;
    receive_error = uart_result::OK;
    receive_error_count = 0U;
    receive_dropped_bytes = 0U;
    tx_result = uart_result::OK;
    initialized = true;

    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(handle,
        rx_dma_buffer,
        UART_RX_DMA_BUFFER_SIZE);
    if(status != HAL_OK)
    {
        initialized = false;
        return uart_result::INIT_FAILED;
    }

    rx_active = true;
    return uart_result::OK;
}

/**
 * @brief 从接收流缓冲区读取 UART 数据
 *
 * @param data 接收缓冲区
 * @param max_size 最大读取长度
 * @param received_size 实际读取长度
 * @param read_timeout_ms 等待首批数据的超时时间，单位毫秒
 * @param lock_timeout_ms 等待读取互斥锁的超时时间，单位毫秒
 *
 * @return UART 总线结果
 */
uart_result uart_dev::read_bytes(uint8_t *data,
    uint16_t max_size,
    uint16_t &received_size,
    uint32_t read_timeout_ms,
    uint32_t lock_timeout_ms)
{
    received_size = 0U;

    if(!initialized)
    {
        return uart_result::NOT_INITIALIZED;
    }

    if(!data || max_size == 0U)
    {
        return uart_result::INVALID_ARGUMENT;
    }

    if(__get_IPSR() != 0U ||
        xTaskGetSchedulerState() != taskSCHEDULER_RUNNING)
    {
        return uart_result::INVALID_CONTEXT;
    }

    TickType_t lock_timeout = milliseconds_to_ticks(lock_timeout_ms);
    if(xSemaphoreTake(rx_mutex, lock_timeout) != pdTRUE)
    {
        return uart_result::LOCK_TIMEOUT;
    }

    uart_result receive_status = ensure_receive_active();
    if(receive_status != uart_result::OK)
    {
        xSemaphoreGive(rx_mutex);
        return receive_status;
    }

    TickType_t read_timeout = milliseconds_to_ticks(read_timeout_ms);
    size_t read_size = xStreamBufferReceive(rx_stream,
        data,
        max_size,
        read_timeout);
    received_size = (uint16_t)read_size;

    xSemaphoreGive(rx_mutex);

    if(read_size == 0U && read_timeout_ms > 0U)
    {
        return uart_result::READ_TIMEOUT;
    }

    return uart_result::OK;
}

/**
 * @brief 使用 DMA 发送 UART 数据并等待传输完成
 *
 * @param data 发送缓冲区
 * @param size 发送长度
 * @param lock_timeout_ms 等待发送互斥锁的超时时间，单位毫秒
 * @param transfer_timeout_ms 等待 DMA 发送完成的超时时间，单位毫秒
 *
 * @return UART 总线结果
 */
uart_result uart_dev::write_bytes(const uint8_t *data,
    uint16_t size,
    uint32_t lock_timeout_ms,
    uint32_t transfer_timeout_ms)
{
    if(!initialized)
    {
        return uart_result::NOT_INITIALIZED;
    }

    if(!data || size == 0U)
    {
        return uart_result::INVALID_ARGUMENT;
    }

    if(__get_IPSR() != 0U ||
        xTaskGetSchedulerState() != taskSCHEDULER_RUNNING)
    {
        return uart_result::INVALID_CONTEXT;
    }

    TickType_t lock_timeout = milliseconds_to_ticks(lock_timeout_ms);
    if(xSemaphoreTake(tx_mutex, lock_timeout) != pdTRUE)
    {
        return uart_result::LOCK_TIMEOUT;
    }

    while(xSemaphoreTake(tx_completion_semaphore, 0U) == pdTRUE)
    {
    }

    tx_result = uart_result::BUSY;
    tx_active = true;

    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(handle,
        const_cast<uint8_t *>(data),
        size);
    if(status != HAL_OK)
    {
        uart_result result = map_hal_status(handle, status);
        cancel_active_transmit();
        bool recovered = recover_transmit();
        xSemaphoreGive(tx_mutex);
        return recovered ? result : uart_result::RECOVERY_FAILED;
    }

    TickType_t transfer_timeout =
        milliseconds_to_ticks(transfer_timeout_ms);
    if(xSemaphoreTake(tx_completion_semaphore, transfer_timeout) != pdTRUE)
    {
        cancel_active_transmit();
        bool recovered = recover_transmit();
        xSemaphoreGive(tx_mutex);
        return recovered ? uart_result::TRANSFER_TIMEOUT :
            uart_result::RECOVERY_FAILED;
    }

    uart_result result = tx_result;
    xSemaphoreGive(tx_mutex);
    return result;
}

/**
 * @brief 获取因软件接收缓存已满而丢弃的字节数
 *
 * @return 累计丢弃字节数
 */
uint32_t uart_dev::rx_dropped_bytes() const
{
    return receive_dropped_bytes;
}

/**
 * @brief 获取 UART 接收错误次数
 *
 * @return 累计错误次数
 */
uint32_t uart_dev::rx_error_count() const
{
    return receive_error_count;
}

/**
 * @brief 获取最近一次 UART 接收错误
 *
 * @return 最近一次接收错误，尚未发生错误时返回 OK
 */
uart_result uart_dev::last_rx_error() const
{
    return receive_error;
}

/**
 * @brief 判断 HAL UART 句柄是否属于当前物理总线
 *
 * @param target_handle 待匹配的 HAL UART 句柄
 *
 * @return 句柄匹配时返回 true
 */
bool uart_dev::matches_handle(UART_HandleTypeDef *target_handle) const
{
    return handle == target_handle;
}

/**
 * @brief 在任务上下文中确保循环 RX DMA 正在运行
 *
 * @return UART 总线结果
 */
uart_result uart_dev::ensure_receive_active()
{
    if(rx_active)
    {
        return uart_result::OK;
    }

    if(HAL_UART_AbortReceive(handle) != HAL_OK)
    {
        return uart_result::RECOVERY_FAILED;
    }

    rx_dma_position = 0U;
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(handle,
        rx_dma_buffer,
        UART_RX_DMA_BUFFER_SIZE);
    if(status != HAL_OK)
    {
        return uart_result::RECOVERY_FAILED;
    }

    rx_active = true;
    return uart_result::OK;
}

/**
 * @brief 在 UART 错误回调中重新启动循环 RX DMA
 *
 * @return 重启成功时返回 true
 */
bool uart_dev::restart_receive_from_isr()
{
    rx_dma_position = 0U;
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(handle,
        rx_dma_buffer,
        UART_RX_DMA_BUFFER_SIZE);
    rx_active = status == HAL_OK;
    return rx_active;
}

/**
 * @brief 在异常发送后停止 UART TX DMA
 *
 * @return 恢复成功时返回 true
 */
bool uart_dev::recover_transmit()
{
    bool recovered = HAL_UART_AbortTransmit(handle) == HAL_OK;

    while(xSemaphoreTake(tx_completion_semaphore, 0U) == pdTRUE)
    {
    }

    return recovered;
}

/**
 * @brief 在任务上下文中取消当前活动发送标记
 */
void uart_dev::cancel_active_transmit()
{
    taskENTER_CRITICAL();
    tx_active = false;
    taskEXIT_CRITICAL();
}

/**
 * @brief 从中断向 UART 接收流缓冲区写入一段数据
 *
 * @param data 接收数据起始地址
 * @param size 接收数据长度
 * @param higher_priority_task_woken 高优先级任务唤醒标志
 */
void uart_dev::send_received_chunk_from_isr(const uint8_t *data,
    uint16_t size,
    BaseType_t &higher_priority_task_woken)
{
    if(size == 0U)
    {
        return;
    }

    size_t sent_size = xStreamBufferSendFromISR(rx_stream,
        data,
        size,
        &higher_priority_task_woken);
    receive_dropped_bytes += (uint32_t)(size - sent_size);
}

/**
 * @brief 处理循环 RX DMA 的半传输、整传输和 IDLE 事件
 *
 * @param position DMA 缓冲区中的当前写入位置
 */
void uart_dev::receive_event_from_isr(uint16_t position)
{
    if(!initialized || !rx_active || position > UART_RX_DMA_BUFFER_SIZE)
    {
        return;
    }

    HAL_UART_RxEventTypeTypeDef event_type =
        HAL_UARTEx_GetRxEventType(handle);

    // 整缓冲区完成后 HAL 还可能报告一次 position == size 的 IDLE 事件。
    if(event_type == HAL_UART_RXEVENT_IDLE &&
        position == UART_RX_DMA_BUFFER_SIZE &&
        rx_dma_position == 0U)
    {
        return;
    }

    BaseType_t higher_priority_task_woken = pdFALSE;
    uint16_t previous_position = rx_dma_position;

    if(position > previous_position)
    {
        send_received_chunk_from_isr(&rx_dma_buffer[previous_position],
            (uint16_t)(position - previous_position),
            higher_priority_task_woken);
    }
    else if(position < previous_position)
    {
        send_received_chunk_from_isr(&rx_dma_buffer[previous_position],
            (uint16_t)(UART_RX_DMA_BUFFER_SIZE - previous_position),
            higher_priority_task_woken);
        send_received_chunk_from_isr(rx_dma_buffer,
            position,
            higher_priority_task_woken);
    }

    rx_dma_position = position == UART_RX_DMA_BUFFER_SIZE ? 0U : position;
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

/**
 * @brief 在中断中完成当前 UART TX DMA 发送
 *
 * @param result DMA 发送结果
 */
void uart_dev::complete_tx_from_isr(uart_result result)
{
    if(!initialized || !tx_active)
    {
        return;
    }

    tx_result = result;
    tx_active = false;

    BaseType_t higher_priority_task_woken = pdFALSE;
    xSemaphoreGiveFromISR(tx_completion_semaphore,
        &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

/**
 * @brief 处理 UART DMA 或线路错误并尝试恢复 RX DMA
 *
 * @param result UART 错误结果
 */
void uart_dev::handle_error_from_isr(uart_result result)
{
    if(!initialized)
    {
        return;
    }

    receive_error = result;
    receive_error_count++;
    complete_tx_from_isr(result);

    if(handle->RxState == HAL_UART_STATE_READY)
    {
        rx_active = false;
        restart_receive_from_isr();
    }
}

/**
 * @brief 创建 UART 总线访问对象
 *
 * @param bus_id UART 总线编号
 */
uart_bus::uart_bus(uint8_t bus_id)
    : bus_id(bus_id)
{
}

/**
 * @brief 初始化 UART 总线同步资源和循环 RX DMA
 *
 * @return UART 总线结果
 */
uart_result uart_bus::init()
{
    uart_dev *device = get_dev(bus_id);
    if(!device)
    {
        return uart_result::INVALID_BUS;
    }

    return device->init();
}

/**
 * @brief 从 UART 接收流缓冲区读取数据
 *
 * @param data 接收缓冲区
 * @param max_size 最大读取长度
 * @param received_size 实际读取长度
 * @param read_timeout_ms 等待首批数据的超时时间，单位毫秒
 * @param lock_timeout_ms 等待读取互斥锁的超时时间，单位毫秒
 *
 * @return UART 总线结果
 */
uart_result uart_bus::read_bytes(uint8_t *data,
    uint16_t max_size,
    uint16_t &received_size,
    uint32_t read_timeout_ms,
    uint32_t lock_timeout_ms)
{
    uart_dev *device = get_dev(bus_id);
    if(!device)
    {
        received_size = 0U;
        return uart_result::INVALID_BUS;
    }

    return device->read_bytes(data,
        max_size,
        received_size,
        read_timeout_ms,
        lock_timeout_ms);
}

/**
 * @brief 使用 DMA 发送 UART 数据并等待完成
 *
 * @param data 发送缓冲区
 * @param size 发送长度
 * @param lock_timeout_ms 等待发送互斥锁的超时时间，单位毫秒
 * @param transfer_timeout_ms 等待 DMA 发送完成的超时时间，单位毫秒
 *
 * @return UART 总线结果
 */
uart_result uart_bus::write_bytes(const uint8_t *data,
    uint16_t size,
    uint32_t lock_timeout_ms,
    uint32_t transfer_timeout_ms)
{
    uart_dev *device = get_dev(bus_id);
    if(!device)
    {
        return uart_result::INVALID_BUS;
    }

    return device->write_bytes(data,
        size,
        lock_timeout_ms,
        transfer_timeout_ms);
}

/**
 * @brief 获取因软件接收缓存已满而丢弃的字节数
 *
 * @return 累计丢弃字节数，无效总线返回 0
 */
uint32_t uart_bus::rx_dropped_bytes() const
{
    uart_dev *device = get_dev(bus_id);
    return device ? device->rx_dropped_bytes() : 0U;
}

/**
 * @brief 获取 UART 接收错误次数
 *
 * @return 累计错误次数，无效总线返回 0
 */
uint32_t uart_bus::rx_error_count() const
{
    uart_dev *device = get_dev(bus_id);
    return device ? device->rx_error_count() : 0U;
}

/**
 * @brief 获取最近一次 UART 接收错误
 *
 * @return 最近一次接收错误，无效总线返回 INVALID_BUS
 */
uart_result uart_bus::last_rx_error() const
{
    uart_dev *device = get_dev(bus_id);
    return device ? device->last_rx_error() : uart_result::INVALID_BUS;
}

/**
 * @brief 处理 UART TX DMA 发送完成事件
 *
 * @param handle HAL UART 句柄
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *handle)
{
    uart_dev *device = get_dev(handle);
    if(device)
    {
        device->complete_tx_from_isr(uart_result::OK);
    }
}

/**
 * @brief 处理 UART 循环 RX DMA 和 IDLE 接收事件
 *
 * @param handle HAL UART 句柄
 * @param size DMA 缓冲区中的当前写入位置
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *handle, uint16_t size)
{
    uart_dev *device = get_dev(handle);
    if(device)
    {
        device->receive_event_from_isr(size);
    }
}

/**
 * @brief 处理 UART DMA 或线路错误事件
 *
 * @param handle HAL UART 句柄
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *handle)
{
    uart_dev *device = get_dev(handle);
    if(device)
    {
        device->handle_error_from_isr(map_hal_error(handle));
    }
}

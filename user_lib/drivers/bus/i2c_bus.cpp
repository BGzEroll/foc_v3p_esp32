#include "i2c_bus.h"

#include "i2c.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

enum class i2c_transfer_direction : uint8_t
{
    READ = 0,
    WRITE
};

// 管理一条物理 I2C 总线的互斥访问、DMA 状态和完成同步。
class i2c_dev
{
    public:
        explicit i2c_dev(I2C_HandleTypeDef *handle);

    public:
        i2c_result init();
        i2c_result read_bytes(uint8_t device_address,
            uint8_t register_address,
            uint8_t *data,
            uint16_t size,
            uint32_t lock_timeout_ms,
            uint32_t transfer_timeout_ms);
        i2c_result write_bytes(uint8_t device_address,
            uint8_t register_address,
            const uint8_t *data,
            uint16_t size,
            uint32_t lock_timeout_ms,
            uint32_t transfer_timeout_ms);
        bool matches_handle(I2C_HandleTypeDef *target_handle) const;
        void complete_from_isr(i2c_result result);

    private:
        i2c_result transfer_bytes(i2c_transfer_direction direction,
            uint8_t device_address,
            uint8_t register_address,
            uint8_t *data,
            uint16_t size,
            uint32_t lock_timeout_ms,
            uint32_t transfer_timeout_ms);
        bool recover_bus();
        void cancel_active_transfer();

    private:
        I2C_HandleTypeDef *handle;
        SemaphoreHandle_t mutex = nullptr;
        StaticSemaphore_t mutex_storage{};
        SemaphoreHandle_t completion_semaphore = nullptr;
        StaticSemaphore_t completion_semaphore_storage{};
        bool initialized = false;
        volatile bool transfer_active = false;
        volatile i2c_result transfer_result = i2c_result::NOT_INITIALIZED;
};

static i2c_dev i2c_devs[] =
{
    i2c_dev(&hi2c1),
    i2c_dev(&hi2c2)
};

static constexpr uint8_t I2C_DEV_COUNT =
    (uint8_t)(sizeof(i2c_devs) / sizeof(i2c_devs[0]));

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
 * @brief 根据 HAL I2C 错误码生成总线结果
 *
 * @param handle HAL I2C 句柄
 *
 * @return I2C 总线结果
 */
static i2c_result map_hal_error(I2C_HandleTypeDef *handle)
{
    uint32_t error = HAL_I2C_GetError(handle);

    if((error & HAL_I2C_ERROR_AF) != 0U)
    {
        return i2c_result::NACK;
    }

    if((error & (HAL_I2C_ERROR_DMA | HAL_I2C_ERROR_DMA_PARAM)) != 0U)
    {
        return i2c_result::DMA_ERROR;
    }

    if((error & HAL_I2C_ERROR_TIMEOUT) != 0U)
    {
        return i2c_result::TRANSFER_TIMEOUT;
    }

    return i2c_result::BUS_ERROR;
}

/**
 * @brief 将 HAL 状态转换为 I2C 总线结果
 *
 * @param handle HAL I2C 句柄
 * @param status HAL 调用状态
 *
 * @return I2C 总线结果
 */
static i2c_result map_hal_status(I2C_HandleTypeDef *handle,
    HAL_StatusTypeDef status)
{
    switch(status)
    {
        case HAL_OK:
            return i2c_result::OK;

        case HAL_BUSY:
            return i2c_result::BUSY;

        case HAL_TIMEOUT:
            return i2c_result::TRANSFER_TIMEOUT;

        case HAL_ERROR:
        default:
            return map_hal_error(handle);
    }
}

/**
 * @brief 根据总线编号获取物理 I2C 设备
 *
 * @param bus_id I2C 总线编号
 *
 * @return 有效编号对应的设备指针，无效编号返回 nullptr
 */
static i2c_dev *get_dev(uint8_t bus_id)
{
    if(bus_id >= I2C_DEV_COUNT)
    {
        return nullptr;
    }

    return &i2c_devs[bus_id];
}

/**
 * @brief 根据 HAL 句柄获取物理 I2C 设备
 *
 * @param handle HAL I2C 句柄
 *
 * @return 匹配的设备指针，未匹配时返回 nullptr
 */
static i2c_dev *get_dev(I2C_HandleTypeDef *handle)
{
    for(uint8_t index = 0; index < I2C_DEV_COUNT; index++)
    {
        if(i2c_devs[index].matches_handle(handle))
        {
            return &i2c_devs[index];
        }
    }

    return nullptr;
}

/**
 * @brief 创建物理 I2C 总线管理对象
 *
 * @param handle HAL I2C 句柄
 */
i2c_dev::i2c_dev(I2C_HandleTypeDef *handle)
    : handle(handle)
{
}

/**
 * @brief 初始化 I2C 总线的 FreeRTOS 同步对象
 *
 * @return I2C 总线结果
 */
i2c_result i2c_dev::init()
{
    if(initialized)
    {
        return i2c_result::OK;
    }

    if(!handle || !handle->Instance || !handle->hdmarx || !handle->hdmatx)
    {
        return i2c_result::INIT_FAILED;
    }

    mutex = xSemaphoreCreateMutexStatic(&mutex_storage);
    completion_semaphore =
        xSemaphoreCreateBinaryStatic(&completion_semaphore_storage);

    if(!mutex || !completion_semaphore)
    {
        mutex = nullptr;
        completion_semaphore = nullptr;
        return i2c_result::INIT_FAILED;
    }

    transfer_result = i2c_result::OK;
    initialized = true;
    return i2c_result::OK;
}

/**
 * @brief 使用 DMA 读取 I2C 设备寄存器
 *
 * @param device_address 7 位设备地址
 * @param register_address 起始寄存器地址
 * @param data 接收缓冲区
 * @param size 接收长度
 * @param lock_timeout_ms 等待总线互斥锁的超时时间，单位毫秒
 * @param transfer_timeout_ms 等待 DMA 完成的超时时间，单位毫秒
 *
 * @return I2C 总线结果
 */
i2c_result i2c_dev::read_bytes(uint8_t device_address,
    uint8_t register_address,
    uint8_t *data,
    uint16_t size,
    uint32_t lock_timeout_ms,
    uint32_t transfer_timeout_ms)
{
    return transfer_bytes(i2c_transfer_direction::READ,
        device_address,
        register_address,
        data,
        size,
        lock_timeout_ms,
        transfer_timeout_ms);
}

/**
 * @brief 使用 DMA 写入 I2C 设备寄存器
 *
 * @param device_address 7 位设备地址
 * @param register_address 起始寄存器地址
 * @param data 发送缓冲区
 * @param size 发送长度
 * @param lock_timeout_ms 等待总线互斥锁的超时时间，单位毫秒
 * @param transfer_timeout_ms 等待 DMA 完成的超时时间，单位毫秒
 *
 * @return I2C 总线结果
 */
i2c_result i2c_dev::write_bytes(uint8_t device_address,
    uint8_t register_address,
    const uint8_t *data,
    uint16_t size,
    uint32_t lock_timeout_ms,
    uint32_t transfer_timeout_ms)
{
    return transfer_bytes(i2c_transfer_direction::WRITE,
        device_address,
        register_address,
        const_cast<uint8_t *>(data),
        size,
        lock_timeout_ms,
        transfer_timeout_ms);
}

/**
 * @brief 判断 HAL I2C 句柄是否属于当前物理总线
 *
 * @param target_handle 待匹配的 HAL I2C 句柄
 *
 * @return 句柄匹配时返回 true
 */
bool i2c_dev::matches_handle(I2C_HandleTypeDef *target_handle) const
{
    return handle == target_handle;
}

/**
 * @brief 执行一次受互斥锁保护的 I2C DMA 传输
 *
 * @param direction DMA 传输方向
 * @param device_address 7 位设备地址
 * @param register_address 起始寄存器地址
 * @param data 数据缓冲区
 * @param size 数据长度
 * @param lock_timeout_ms 等待总线互斥锁的超时时间，单位毫秒
 * @param transfer_timeout_ms 等待 DMA 完成的超时时间，单位毫秒
 *
 * @return I2C 总线结果
 */
i2c_result i2c_dev::transfer_bytes(i2c_transfer_direction direction,
    uint8_t device_address,
    uint8_t register_address,
    uint8_t *data,
    uint16_t size,
    uint32_t lock_timeout_ms,
    uint32_t transfer_timeout_ms)
{
    if(!initialized)
    {
        return i2c_result::NOT_INITIALIZED;
    }

    if(device_address > 0x7FU || !data || size == 0U)
    {
        return i2c_result::INVALID_ARGUMENT;
    }
    if(__get_IPSR() != 0U ||
        xTaskGetSchedulerState() != taskSCHEDULER_RUNNING)
    {
        return i2c_result::INVALID_CONTEXT;
    }

    TickType_t lock_timeout = milliseconds_to_ticks(lock_timeout_ms);
    if(xSemaphoreTake(mutex, lock_timeout) != pdTRUE)
    {
        return i2c_result::LOCK_TIMEOUT;
    }

    while(xSemaphoreTake(completion_semaphore, 0U) == pdTRUE)
    {
    }

    transfer_result = i2c_result::BUSY;
    transfer_active = true;

    uint16_t hal_device_address = (uint16_t)(device_address << 1);
    HAL_StatusTypeDef hal_status;

    if(direction == i2c_transfer_direction::READ)
    {
        hal_status = HAL_I2C_Mem_Read_DMA(handle,
            hal_device_address,
            register_address,
            I2C_MEMADD_SIZE_8BIT,
            data,
            size);
    }
    else
    {
        hal_status = HAL_I2C_Mem_Write_DMA(handle,
            hal_device_address,
            register_address,
            I2C_MEMADD_SIZE_8BIT,
            data,
            size);
    }

    if(hal_status != HAL_OK)
    {
        i2c_result result = map_hal_status(handle, hal_status);
        cancel_active_transfer();
        bool recovered = recover_bus();
        xSemaphoreGive(mutex);
        return recovered ? result : i2c_result::RECOVERY_FAILED;
    }

    TickType_t transfer_timeout =
        milliseconds_to_ticks(transfer_timeout_ms);
    if(xSemaphoreTake(completion_semaphore, transfer_timeout) != pdTRUE)
    {
        cancel_active_transfer();
        bool recovered = recover_bus();
        xSemaphoreGive(mutex);
        return recovered ? i2c_result::TRANSFER_TIMEOUT :
            i2c_result::RECOVERY_FAILED;
    }

    i2c_result result = transfer_result;
    xSemaphoreGive(mutex);
    return result;
}

/**
 * @brief 在异常传输后重新初始化 I2C 外设及其 DMA
 *
 * @return 恢复成功时返回 true
 */
bool i2c_dev::recover_bus()
{
    bool recovered = HAL_I2C_DeInit(handle) == HAL_OK &&
        HAL_I2C_Init(handle) == HAL_OK;

    while(xSemaphoreTake(completion_semaphore, 0U) == pdTRUE)
    {
    }

    if(!recovered)
    {
        initialized = false;
    }

    return recovered;
}

/**
 * @brief 在任务上下文中取消当前活动传输标记
 */
void i2c_dev::cancel_active_transfer()
{
    taskENTER_CRITICAL();
    transfer_active = false;
    taskEXIT_CRITICAL();
}

/**
 * @brief 在中断中完成当前 I2C DMA 传输
 *
 * @param result DMA 传输结果
 */
void i2c_dev::complete_from_isr(i2c_result result)
{
    if(!initialized || !transfer_active)
    {
        return;
    }

    transfer_result = result;
    transfer_active = false;

    BaseType_t higher_priority_task_woken = pdFALSE;
    xSemaphoreGiveFromISR(completion_semaphore,
        &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

/**
 * @brief 创建 I2C 总线访问对象
 *
 * @param bus_id I2C 总线编号
 */
i2c_bus::i2c_bus(uint8_t bus_id)
    : bus_id(bus_id)
{
}

/**
 * @brief 初始化 I2C 总线的 FreeRTOS 同步资源
 *
 * @return I2C 总线结果
 */
i2c_result i2c_bus::init()
{
    i2c_dev *device = get_dev(bus_id);
    if(!device)
    {
        return i2c_result::INVALID_BUS;
    }

    return device->init();
}

/**
 * @brief 使用 DMA 连续读取 I2C 设备寄存器
 *
 * @param device_address 7 位设备地址
 * @param register_address 起始寄存器地址
 * @param data 接收缓冲区
 * @param size 接收长度
 * @param lock_timeout_ms 等待总线互斥锁的超时时间，单位毫秒
 * @param transfer_timeout_ms 等待 DMA 完成的超时时间，单位毫秒
 *
 * @return I2C 总线结果
 */
i2c_result i2c_bus::read_bytes(uint8_t device_address,
    uint8_t register_address,
    uint8_t *data,
    uint16_t size,
    uint32_t lock_timeout_ms,
    uint32_t transfer_timeout_ms)
{
    i2c_dev *device = get_dev(bus_id);
    if(!device)
    {
        return i2c_result::INVALID_BUS;
    }

    return device->read_bytes(device_address,
        register_address,
        data,
        size,
        lock_timeout_ms,
        transfer_timeout_ms);
}

/**
 * @brief 使用 DMA 连续写入 I2C 设备寄存器
 *
 * @param device_address 7 位设备地址
 * @param register_address 起始寄存器地址
 * @param data 发送缓冲区
 * @param size 发送长度
 * @param lock_timeout_ms 等待总线互斥锁的超时时间，单位毫秒
 * @param transfer_timeout_ms 等待 DMA 完成的超时时间，单位毫秒
 *
 * @return I2C 总线结果
 */
i2c_result i2c_bus::write_bytes(uint8_t device_address,
    uint8_t register_address,
    const uint8_t *data,
    uint16_t size,
    uint32_t lock_timeout_ms,
    uint32_t transfer_timeout_ms)
{
    i2c_dev *device = get_dev(bus_id);
    if(!device)
    {
        return i2c_result::INVALID_BUS;
    }

    return device->write_bytes(device_address,
        register_address,
        data,
        size,
        lock_timeout_ms,
        transfer_timeout_ms);
}

/**
 * @brief 处理 I2C DMA 寄存器读取完成事件
 *
 * @param handle HAL I2C 句柄
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *handle)
{
    i2c_dev *device = get_dev(handle);
    if(device)
    {
        device->complete_from_isr(i2c_result::OK);
    }
}

/**
 * @brief 处理 I2C DMA 寄存器写入完成事件
 *
 * @param handle HAL I2C 句柄
 */
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *handle)
{
    i2c_dev *device = get_dev(handle);
    if(device)
    {
        device->complete_from_isr(i2c_result::OK);
    }
}

/**
 * @brief 处理 I2C DMA 或总线错误事件
 *
 * @param handle HAL I2C 句柄
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *handle)
{
    i2c_dev *device = get_dev(handle);
    if(device)
    {
        device->complete_from_isr(map_hal_error(handle));
    }
}

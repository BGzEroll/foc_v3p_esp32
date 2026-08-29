#include "uart_bus.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <algorithm>
#include <climits>

/**
 * @brief 将毫秒超时转换为 FreeRTOS tick
 *
 * @param timeout_ms 超时时间，单位毫秒
 *
 * @return FreeRTOS tick 数，非零毫秒至少转换为一个 tick
 */
static TickType_t milliseconds_to_ticks(uint32_t timeout_ms)
{
    TickType_t ticks = pdMS_TO_TICKS(timeout_ms);
    if(timeout_ms > 0 && ticks == 0){ticks = 1;}
    return ticks;
}

/**
 * @brief 判断当前上下文是否允许调用阻塞式 UART 驱动 API
 *
 * @return 处于任务上下文且调度器已运行时返回 true
 */
static bool task_context_ready()
{
    return xPortInIsrContext() == pdFALSE &&
        xTaskGetSchedulerState() == taskSCHEDULER_RUNNING;
}

/**
 * @brief 创建 ESP-IDF UART 总线对象
 *
 * @param port UART 控制器编号
 * @param tx_pin TX GPIO
 * @param rx_pin RX GPIO
 * @param baud_rate 波特率
 * @param rx_buffer_size 驱动接收环形缓冲区大小，单位字节
 */
uart_bus::uart_bus(uart_port_t port,
    gpio_num_t tx_pin,
    gpio_num_t rx_pin,
    uint32_t baud_rate,
    uint16_t rx_buffer_size)
    : port(port),
      tx_pin(tx_pin),
      rx_pin(rx_pin),
      baud_rate(baud_rate),
      rx_buffer_size(rx_buffer_size)
{
}

/**
 * @brief 配置 UART 参数、引脚和 ESP-IDF 驱动接收缓冲区
 *
 * @return UART 初始化结果
 *
 * @note 当前封装固定使用 8 数据位、无校验、1 停止位且不启用硬件流控。
 */
uart_result uart_bus::init()
{
    if(initialized){return uart_result::OK;}

    if(port < UART_NUM_0 || port >= UART_NUM_MAX)
    {
        return uart_result::INVALID_BUS;
    }

    if(!GPIO_IS_VALID_OUTPUT_GPIO(tx_pin) ||
        !GPIO_IS_VALID_GPIO(rx_pin) ||
        tx_pin == rx_pin ||
        baud_rate == 0 ||
        baud_rate > UART_BITRATE_MAX ||
        baud_rate > INT_MAX ||
        rx_buffer_size <= UART_HW_FIFO_LEN(port))
    {
        return uart_result::INVALID_ARGUMENT;
    }

    if(uart_is_driver_installed(port))
    {
        return uart_result::INIT_FAILED;
    }

    esp_err_t error = uart_driver_install(port,
        static_cast<int>(rx_buffer_size),
        0,
        0,
        nullptr,
        0);
    if(error != ESP_OK)
    {
        return uart_result::INIT_FAILED;
    }

    uart_config_t config = {};
    config.baud_rate = static_cast<int>(baud_rate);
    config.data_bits = UART_DATA_8_BITS;
    config.parity = UART_PARITY_DISABLE;
    config.stop_bits = UART_STOP_BITS_1;
    config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    config.rx_flow_ctrl_thresh = 0;
    config.source_clk = UART_SCLK_DEFAULT;
    config.flags.allow_pd = false;

    error = uart_param_config(port, &config);
    if(error == ESP_OK)
    {
        error = uart_set_pin(port,
            static_cast<int>(tx_pin),
            static_cast<int>(rx_pin),
            UART_PIN_NO_CHANGE,
            UART_PIN_NO_CHANGE);
    }

    if(error != ESP_OK)
    {
        uart_driver_delete(port);
        return uart_result::INIT_FAILED;
    }

    initialized = true;
    return uart_result::OK;
}

/**
 * @brief 等待首字节后读取当前已经缓存的 UART 数据
 *
 * @param data 接收缓冲区
 * @param max_size 最大读取长度
 * @param received_size 实际读取长度
 * @param read_timeout_ms 等待首字节的超时时间，单位毫秒
 *
 * @return UART 接收结果
 *
 * @note 读取到首字节后只提取驱动中已经缓存的数据，不继续等待缓冲区填满。
 */
uart_result uart_bus::read_bytes(uint8_t *data,
    uint16_t max_size,
    uint16_t &received_size,
    uint32_t read_timeout_ms)
{
    received_size = 0;

    if(!initialized){return uart_result::NOT_INITIALIZED;}
    if(!data || max_size == 0){return uart_result::INVALID_ARGUMENT;}
    if(!task_context_ready()){return uart_result::INVALID_CONTEXT;}

    int first_read_size = uart_read_bytes(port,
        data,
        1,
        milliseconds_to_ticks(read_timeout_ms));
    if(first_read_size < 0){return uart_result::BUS_ERROR;}
    if(first_read_size == 0)
    {
        return read_timeout_ms > 0 ? uart_result::READ_TIMEOUT :
            uart_result::OK;
    }

    received_size = 1;
    if(max_size == 1){return uart_result::OK;}

    size_t buffered_size = 0;
    if(uart_get_buffered_data_len(port, &buffered_size) != ESP_OK)
    {
        return uart_result::BUS_ERROR;
    }

    size_t remaining_capacity = (size_t)max_size - received_size;
    uint32_t drain_size = static_cast<uint32_t>(
        std::min(buffered_size, remaining_capacity));
    if(drain_size == 0){return uart_result::OK;}

    int drained_size = uart_read_bytes(port,
        &data[received_size],
        drain_size,
        0);
    if(drained_size < 0){return uart_result::BUS_ERROR;}

    received_size += static_cast<uint16_t>(drained_size);
    return uart_result::OK;
}

/**
 * @brief 发送 UART 数据并等待硬件完成传输
 *
 * @param data 发送缓冲区
 * @param size 发送长度
 * @param transfer_timeout_ms 等待硬件发送完成的超时时间，单位毫秒
 *
 * @return UART 发送结果
 */
uart_result uart_bus::write_bytes(const uint8_t *data,
    uint16_t size,
    uint32_t transfer_timeout_ms)
{
    if(!initialized){return uart_result::NOT_INITIALIZED;}
    if(!data || size == 0){return uart_result::INVALID_ARGUMENT;}
    if(!task_context_ready()){return uart_result::INVALID_CONTEXT;}

    int written_size = uart_write_bytes(port, data, size);
    if(written_size < 0 || written_size != size)
    {
        return uart_result::BUS_ERROR;
    }

    esp_err_t error = uart_wait_tx_done(port,
        milliseconds_to_ticks(transfer_timeout_ms));
    if(error == ESP_ERR_TIMEOUT){return uart_result::TRANSFER_TIMEOUT;}
    return error == ESP_OK ? uart_result::OK : uart_result::BUS_ERROR;
}

/**
 * @brief 丢弃 UART 驱动接收缓冲区中的全部数据
 *
 * @return UART 接收缓冲区清理结果
 */
uart_result uart_bus::flush_rx()
{
    if(!initialized){return uart_result::NOT_INITIALIZED;}
    if(!task_context_ready()){return uart_result::INVALID_CONTEXT;}

    return uart_flush_input(port) == ESP_OK ? uart_result::OK :
        uart_result::BUS_ERROR;
}

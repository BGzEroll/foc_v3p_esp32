#include "i2c_bus.h"

#include <climits>

static constexpr uint8_t I2C_GLITCH_IGNORE_COUNT = 7U;

/**
 * @brief 将 ESP-IDF I2C 传输结果转换为总线结果
 *
 * @param error ESP-IDF 错误码
 *
 * @return I2C 总线结果
 */
static i2c_result map_transfer_error(esp_err_t error)
{
    switch(error)
    {
        case ESP_OK:
            return i2c_result::OK;

        case ESP_ERR_INVALID_ARG:
            return i2c_result::INVALID_ARGUMENT;

        case ESP_ERR_INVALID_STATE:
            return i2c_result::NOT_INITIALIZED;

        case ESP_ERR_INVALID_RESPONSE:
            return i2c_result::NACK;

        case ESP_ERR_TIMEOUT:
            return i2c_result::TRANSFER_TIMEOUT;

        default:
            return i2c_result::BUS_ERROR;
    }
}

/**
 * @brief 创建 I2C 主机总线对象
 *
 * @param port ESP32 I2C 控制器编号
 * @param sda_pin SDA GPIO
 * @param scl_pin SCL GPIO
 * @param enable_internal_pullup 是否启用内部上拉
 */
i2c_bus::i2c_bus(i2c_port_num_t port,
    gpio_num_t sda_pin,
    gpio_num_t scl_pin,
    bool enable_internal_pullup)
    : port(port),
      sda_pin(sda_pin),
      scl_pin(scl_pin),
      enable_internal_pullup(enable_internal_pullup)
{
}

/**
 * @brief 初始化 ESP-IDF I2C 主机总线
 *
 * @return I2C 总线结果
 */
i2c_result i2c_bus::init()
{
    if(bus_handle){return i2c_result::OK;}

    if(port < I2C_NUM_0 || port >= I2C_NUM_MAX)
    {
        return i2c_result::INVALID_BUS;
    }

    if(!GPIO_IS_VALID_OUTPUT_GPIO(sda_pin) ||
        !GPIO_IS_VALID_OUTPUT_GPIO(scl_pin) ||
        sda_pin == scl_pin)
    {
        return i2c_result::INVALID_ARGUMENT;
    }

    i2c_master_bus_config_t config = {};
    config.i2c_port = port;
    config.sda_io_num = sda_pin;
    config.scl_io_num = scl_pin;
    config.clk_source = I2C_CLK_SRC_DEFAULT;
    config.glitch_ignore_cnt = I2C_GLITCH_IGNORE_COUNT;
    config.intr_priority = 0;
    config.trans_queue_depth = 0U;
    config.flags.enable_internal_pullup = enable_internal_pullup;
    config.flags.allow_pd = false;

    esp_err_t error = i2c_new_master_bus(&config, &bus_handle);
    if(error == ESP_ERR_INVALID_ARG){return i2c_result::INVALID_ARGUMENT;}
    if(error != ESP_OK)
    {
        bus_handle = nullptr;
        return i2c_result::INIT_FAILED;
    }

    return i2c_result::OK;
}

/**
 * @brief 复位 I2C 主机总线硬件状态
 *
 * @return I2C 总线结果
 */
i2c_result i2c_bus::reset()
{
    if(!bus_handle){return i2c_result::NOT_INITIALIZED;}

    return map_transfer_error(i2c_master_bus_reset(bus_handle));
}

/**
 * @brief 创建固定地址的 I2C 从设备对象
 *
 * @param bus 从设备所属的物理总线
 * @param device_address 7 位从设备地址
 * @param scl_speed_hz SCL 时钟频率，单位 Hz
 */
i2c_device::i2c_device(i2c_bus &bus,
    uint8_t device_address,
    uint32_t scl_speed_hz)
    : bus(bus),
      device_address(device_address),
      scl_speed_hz(scl_speed_hz)
{
}

/**
 * @brief 初始化 I2C 从设备句柄
 *
 * @return I2C 总线结果
 */
i2c_result i2c_device::init()
{
    if(device_handle){return i2c_result::OK;}

    if(device_address > 0x7FU || scl_speed_hz == 0U)
    {
        return i2c_result::INVALID_ARGUMENT;
    }

    i2c_result result = bus.init();
    if(result != i2c_result::OK){return result;}

    i2c_device_config_t config = {};
    config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    config.device_address = device_address;
    config.scl_speed_hz = scl_speed_hz;
    config.scl_wait_us = 0U;
    config.flags.disable_ack_check = false;

    esp_err_t error = i2c_master_bus_add_device(bus.bus_handle,
        &config,
        &device_handle);
    if(error == ESP_ERR_INVALID_ARG){return i2c_result::INVALID_ARGUMENT;}
    if(error != ESP_OK)
    {
        device_handle = nullptr;
        return i2c_result::INIT_FAILED;
    }

    return i2c_result::OK;
}

/**
 * @brief 连续读取 I2C 从设备寄存器
 *
 * @param register_address 起始寄存器地址
 * @param data 接收缓冲区
 * @param size 接收长度
 * @param transfer_timeout_ms 传输超时时间，单位毫秒
 *
 * @return I2C 总线结果
 */
i2c_result i2c_device::read_bytes(uint8_t register_address,
    uint8_t *data,
    uint16_t size,
    uint32_t transfer_timeout_ms)
{
    if(!device_handle){return i2c_result::NOT_INITIALIZED;}
    if(!data || size == 0U || transfer_timeout_ms > INT_MAX)
    {
        return i2c_result::INVALID_ARGUMENT;
    }

    esp_err_t error = i2c_master_transmit_receive(device_handle,
        &register_address,
        1U,
        data,
        size,
        static_cast<int>(transfer_timeout_ms));
    return map_transfer_error(error);
}

/**
 * @brief 连续写入 I2C 从设备寄存器
 *
 * @param register_address 起始寄存器地址
 * @param data 发送缓冲区
 * @param size 发送长度
 * @param transfer_timeout_ms 传输超时时间，单位毫秒
 *
 * @return I2C 总线结果
 */
i2c_result i2c_device::write_bytes(uint8_t register_address,
    const uint8_t *data,
    uint16_t size,
    uint32_t transfer_timeout_ms)
{
    if(!device_handle){return i2c_result::NOT_INITIALIZED;}
    if(!data || size == 0U || transfer_timeout_ms > INT_MAX)
    {
        return i2c_result::INVALID_ARGUMENT;
    }

    i2c_master_transmit_multi_buffer_info_t buffers[2]{};
    buffers[0].write_buffer = &register_address;
    buffers[0].buffer_size = 1U;
    buffers[1].write_buffer = data;
    buffers[1].buffer_size = size;

    esp_err_t error = i2c_master_multi_buffer_transmit(device_handle,
        buffers,
        2U,
        static_cast<int>(transfer_timeout_ms));
    return map_transfer_error(error);
}

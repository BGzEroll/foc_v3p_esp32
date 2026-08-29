#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <cstdint>
#include "driver/gpio.h"
#include "driver/i2c_master.h"

enum class i2c_result : uint8_t
{
    OK = 0,
    INVALID_BUS,
    INVALID_ARGUMENT,
    NOT_INITIALIZED,
    INIT_FAILED,
    TRANSFER_TIMEOUT,
    NACK,
    BUS_ERROR,
    CONFIG_CONFLICT
};

class i2c_device;

class i2c_bus
{
    public:
        i2c_bus(i2c_port_num_t port,
            gpio_num_t sda_pin,
            gpio_num_t scl_pin,
            bool enable_internal_pullup = false);
        i2c_bus(const i2c_bus &) = delete;
        i2c_bus &operator=(const i2c_bus &) = delete;

    public:
        i2c_result reset();

    private:
        friend class i2c_device;

        i2c_result ensure_initialized();

        i2c_port_num_t port;
        gpio_num_t sda_pin;
        gpio_num_t scl_pin;
        bool enable_internal_pullup;
        i2c_master_bus_handle_t bus_handle = nullptr;
};

class i2c_device
{
    public:
        static constexpr uint32_t DEFAULT_TRANSFER_TIMEOUT_MS = 50;

        i2c_device(i2c_bus &bus,
            uint8_t device_address,
            uint32_t scl_speed_hz);
        i2c_device(const i2c_device &) = delete;
        i2c_device &operator=(const i2c_device &) = delete;

    public:
        i2c_result init();

    public:
        i2c_result read_bytes(uint8_t register_address,
            uint8_t *data,
            uint16_t size,
            uint32_t transfer_timeout_ms = DEFAULT_TRANSFER_TIMEOUT_MS);
        i2c_result write_bytes(uint8_t register_address,
            const uint8_t *data,
            uint16_t size,
            uint32_t transfer_timeout_ms = DEFAULT_TRANSFER_TIMEOUT_MS);

    private:
        i2c_bus &bus;
        uint8_t device_address;
        uint32_t scl_speed_hz;
        i2c_master_dev_handle_t device_handle = nullptr;
};

#endif

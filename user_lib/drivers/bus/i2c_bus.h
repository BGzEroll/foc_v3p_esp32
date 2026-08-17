#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <stdint.h>

enum class i2c_result : uint8_t
{
    OK = 0,
    INVALID_BUS,
    INVALID_ARGUMENT,
    NOT_INITIALIZED,
    INVALID_CONTEXT,
    INIT_FAILED,
    LOCK_TIMEOUT,
    TRANSFER_TIMEOUT,
    BUSY,
    NACK,
    DMA_ERROR,
    BUS_ERROR,
    RECOVERY_FAILED
};

class i2c_bus
{
    public:
        static constexpr uint32_t DEFAULT_LOCK_TIMEOUT_MS = 10;
        static constexpr uint32_t DEFAULT_TRANSFER_TIMEOUT_MS = 50;

        explicit i2c_bus(uint8_t bus_id = 0);

    public:
        i2c_result init();
        i2c_result read_bytes(uint8_t device_address,
            uint8_t register_address,
            uint8_t *data,
            uint16_t size,
            uint32_t lock_timeout_ms = DEFAULT_LOCK_TIMEOUT_MS,
            uint32_t transfer_timeout_ms = DEFAULT_TRANSFER_TIMEOUT_MS);
        i2c_result write_bytes(uint8_t device_address,
            uint8_t register_address,
            const uint8_t *data,
            uint16_t size,
            uint32_t lock_timeout_ms = DEFAULT_LOCK_TIMEOUT_MS,
            uint32_t transfer_timeout_ms = DEFAULT_TRANSFER_TIMEOUT_MS);

    private:
        uint8_t bus_id;
};

#endif

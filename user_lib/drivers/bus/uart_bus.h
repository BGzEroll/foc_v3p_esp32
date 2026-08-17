#ifndef UART_BUS_H
#define UART_BUS_H

#include <stdint.h>

enum class uart_result : uint8_t
{
    OK = 0,
    INVALID_BUS,
    INVALID_ARGUMENT,
    NOT_INITIALIZED,
    INVALID_CONTEXT,
    INIT_FAILED,
    LOCK_TIMEOUT,
    READ_TIMEOUT,
    TRANSFER_TIMEOUT,
    BUSY,
    DMA_ERROR,
    OVERRUN_ERROR,
    NOISE_ERROR,
    FRAME_ERROR,
    PARITY_ERROR,
    BUS_ERROR,
    RECOVERY_FAILED
};

class uart_bus
{
    public:
        static constexpr uint32_t DEFAULT_LOCK_TIMEOUT_MS = 10;
        static constexpr uint32_t DEFAULT_READ_TIMEOUT_MS = 0;
        static constexpr uint32_t DEFAULT_TRANSFER_TIMEOUT_MS = 50;

        explicit uart_bus(uint8_t bus_id = 0);

    public:
        uart_result init();
        uart_result read_bytes(uint8_t *data,
            uint16_t max_size,
            uint16_t &received_size,
            uint32_t read_timeout_ms = DEFAULT_READ_TIMEOUT_MS,
            uint32_t lock_timeout_ms = DEFAULT_LOCK_TIMEOUT_MS);
        uart_result write_bytes(const uint8_t *data,
            uint16_t size,
            uint32_t lock_timeout_ms = DEFAULT_LOCK_TIMEOUT_MS,
            uint32_t transfer_timeout_ms = DEFAULT_TRANSFER_TIMEOUT_MS);
        uint32_t rx_dropped_bytes() const;
        uint32_t rx_error_count() const;
        uart_result last_rx_error() const;

    private:
        uint8_t bus_id;
};

#endif

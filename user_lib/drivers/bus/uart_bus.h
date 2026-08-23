#ifndef UART_BUS_H
#define UART_BUS_H

#include <cstdint>
#include "driver/gpio.h"
#include "driver/uart.h"

enum class uart_result : uint8_t
{
    OK = 0,
    INVALID_BUS,
    INVALID_ARGUMENT,
    NOT_INITIALIZED,
    INVALID_CONTEXT,
    INIT_FAILED,
    READ_TIMEOUT,
    TRANSFER_TIMEOUT,
    BUS_ERROR
};

class uart_bus
{
    public:
        static constexpr uint32_t DEFAULT_READ_TIMEOUT_MS = 0U;
        static constexpr uint32_t DEFAULT_TRANSFER_TIMEOUT_MS = 50U;

        uart_bus(uart_port_t port,
            gpio_num_t tx_pin,
            gpio_num_t rx_pin,
            uint32_t baud_rate,
            uint16_t rx_buffer_size = 1024U);
        uart_bus(const uart_bus &) = delete;
        uart_bus &operator=(const uart_bus &) = delete;

    public:
        uart_result init();

    public:
        uart_result read_bytes(uint8_t *data,
            uint16_t max_size,
            uint16_t &received_size,
            uint32_t read_timeout_ms = DEFAULT_READ_TIMEOUT_MS);
        uart_result write_bytes(const uint8_t *data,
            uint16_t size,
            uint32_t transfer_timeout_ms = DEFAULT_TRANSFER_TIMEOUT_MS);
        uart_result flush_rx();

    private:
        uart_port_t port;
        gpio_num_t tx_pin;
        gpio_num_t rx_pin;
        uint32_t baud_rate;
        uint16_t rx_buffer_size;
        bool initialized = false;
};

#endif

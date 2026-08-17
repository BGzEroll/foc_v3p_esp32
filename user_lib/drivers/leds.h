#ifndef LEDS_H
#define LEDS_H

#include <cstdint>
#include "driver/gpio.h"
#include "esp_err.h"

class leds
{
    public:
        leds(gpio_num_t pin, uint32_t on_level);

    public:
        esp_err_t init();

    public:
        void on();
        void off();
        void toggle();

    private:
        gpio_num_t pin;
        uint32_t on_level;
        uint32_t current_level;
        bool initialized;
};

#endif

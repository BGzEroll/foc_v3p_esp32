#include "leds.h"

/**
 * @brief 创建 LED GPIO 控制对象
 *
 * @param pin LED 所在 GPIO 引脚
 * @param on_level LED 点亮时的 GPIO 电平
 */
leds::leds(gpio_num_t pin, uint32_t on_level)
    : pin(pin),
      on_level(on_level ? 1U : 0U),
      current_level(on_level ? 0U : 1U),
      initialized(false)
{
}

/**
 * @brief 初始化 LED GPIO 并将其设置为熄灭状态
 *
 * @return 初始化结果
 */
esp_err_t leds::init()
{
    gpio_config_t config = {};
    config.pin_bit_mask = 1ULL << static_cast<uint32_t>(pin);
    config.mode = GPIO_MODE_OUTPUT;
    config.pull_up_en = GPIO_PULLUP_DISABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.intr_type = GPIO_INTR_DISABLE;

    esp_err_t result = gpio_config(&config);
    if(result != ESP_OK){return result;}

    current_level = on_level ? 0U : 1U;
    result = gpio_set_level(pin, current_level);
    if(result != ESP_OK){return result;}

    initialized = true;
    return ESP_OK;
}

/**
 * @brief 点亮 LED
 */
void leds::on()
{
    if(!initialized){return;}

    if(gpio_set_level(pin, on_level) == ESP_OK)
    {
        current_level = on_level;
    }
}

/**
 * @brief 熄灭 LED
 */
void leds::off()
{
    if(!initialized){return;}

    uint32_t off_level = on_level ? 0U : 1U;
    if(gpio_set_level(pin, off_level) == ESP_OK)
    {
        current_level = off_level;
    }
}

/**
 * @brief 切换 LED 状态
 */
void leds::toggle()
{
    if(!initialized){return;}

    uint32_t next_level = current_level ? 0U : 1U;
    if(gpio_set_level(pin, next_level) == ESP_OK)
    {
        current_level = next_level;
    }
}

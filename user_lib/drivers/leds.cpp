#include "leds.h"

/**
 * @brief 创建 LED GPIO 控制对象
 *
 * @param port LED 所在 GPIO 端口
 * @param pin LED 所在 GPIO 引脚
 * @param on_level LED 点亮时的 GPIO 电平
 */
leds::leds(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState on_level)
    : port(port), pin(pin), on_level(on_level)
{
}

/**
 * @brief 点亮 LED
 */
void leds::on()
{
    HAL_GPIO_WritePin(port, pin, on_level);
}

/**
 * @brief 熄灭 LED
 */
void leds::off()
{
    GPIO_PinState level = (on_level == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    HAL_GPIO_WritePin(port, pin, level);
}

/**
 * @brief 切换 LED 状态
 */
void leds::toggle()
{
    HAL_GPIO_TogglePin(port, pin);
}

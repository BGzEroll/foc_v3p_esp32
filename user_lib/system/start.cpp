#include "devices/led_dev.h"
#include "drivers/foc/esp32_hardware_test.h"

/**
 * @brief 初始化所有用户模块
 */
extern "C" void start_init_all(void)
{
    led_dev::init();
    esp32_hardware_test::init();
}

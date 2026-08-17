#include "devices/led_dev.h"

/**
 * @brief 初始化所有用户模块
 */
extern "C" void start_init_all(void)
{
    led_dev::init();
}

#include "debug/foc_open_loop_debug.h"
#include "devices/led_dev.h"

/**
 * @brief 初始化所有用户模块
 */
extern "C" void start_init_all(void)
{
    foc_open_loop_debug::init();
    led_dev::init();
}

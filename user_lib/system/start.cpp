#include "devices/foc_dev.h"
#include "devices/led_dev.h"

/**
 * @brief 初始化所有用户模块
 */
extern "C" void start_init_all(void)
{
    foc_dev::init();
    led_dev::init();
}

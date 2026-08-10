#include "devices/led_dev.h"

#ifdef DEBUG_MODE

#endif

/**
 * @brief 初始化所有用户模块
 */
extern "C" void start_init_all(void)
{
#ifdef DEBUG_MODE

#endif

    led_dev::init();
}

#include "devices/as5600_dev.h"
#include "devices/led_dev.h"
#include "devices/mpu6050_dev.h"
#include "devices/ssd1306_dev.h"

/**
 * @brief 初始化所有用户模块
 */
extern "C" void start_init_all(void)
{
    as5600_dev::init();
    led_dev::init();
    mpu6050_dev::init();
    // ssd1306_dev::init();
}

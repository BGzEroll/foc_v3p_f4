#include "ssd1306_dev.h"

#include "main.h"

ssd1306 oled(1,
    OLED_CS_GPIO_Port, OLED_CS_Pin,
    OLED_RES_GPIO_Port, OLED_RES_Pin,
    OLED_DC_GPIO_Port, OLED_DC_Pin);

#include "ssd1306_dev.h"

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

static constexpr uint16_t SSD1306_TASK_STACK_DEPTH = 256;
static constexpr UBaseType_t SSD1306_TASK_PRIORITY = tskIDLE_PRIORITY + 2;
static constexpr uint32_t SSD1306_POWER_ON_DELAY_MS = 1000;
static constexpr uint32_t SSD1306_UPDATE_PERIOD_MS = 1000;

ssd1306 ssd1306_dev::oled(0,
    OLED_CS_GPIO_Port, OLED_CS_Pin,
    OLED_RES_GPIO_Port, OLED_RES_Pin,
    OLED_DC_GPIO_Port, OLED_DC_Pin);

/**
 * @brief 初始化 SSD1306 并周期刷新测试计数
 *
 * @param argument FreeRTOS 任务参数
 */
static void ssd1306_task_entry(void *argument)
{
    // 上电等待，避免 OLED 电源尚未稳定就接收复位和命令。
    vTaskDelay(pdMS_TO_TICKS(SSD1306_POWER_ON_DELAY_MS));

    spi_result result = ssd1306_dev::oled.init();
    if(result != spi_result::OK)
    {
        Error_Handler();
    }

    ssd1306_dev::oled.show_string(1, 1, "FOC V3P");
    ssd1306_dev::oled.show_string(2, 1, "SPI DMA OK");
    ssd1306_dev::oled.show_string(4, 1, "COUNT:");
    result = ssd1306_dev::oled.flush();
    if(result != spi_result::OK)
    {
        Error_Handler();
    }

    uint32_t update_count = 0;
    TickType_t last_wake_time = xTaskGetTickCount();

    while(true)
    {
        update_count++;
        ssd1306_dev::oled.show_num(4, 7, update_count, 6);
        result = ssd1306_dev::oled.flush();
        if(result != spi_result::OK)
        {
            Error_Handler();
        }

        vTaskDelayUntil(&last_wake_time,
            pdMS_TO_TICKS(SSD1306_UPDATE_PERIOD_MS));
    }
}

/**
 * @brief 创建 SSD1306 显示测试任务
 */
void ssd1306_dev::init()
{
    BaseType_t result = xTaskCreate(ssd1306_task_entry,
        "ssd1306",
        SSD1306_TASK_STACK_DEPTH,
        nullptr,
        SSD1306_TASK_PRIORITY,
        nullptr);

    if(result != pdPASS)
    {
        Error_Handler();
    }
}

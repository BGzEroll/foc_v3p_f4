#include "as5600_dev.h"

#include "main.h"
#include "system/topic.h"
#include "FreeRTOS.h"
#include "task.h"

static constexpr uint8_t AS5600_I2C_BUS_ID = 0U;
static constexpr uint8_t AS5600_I2C_ADDRESS = 0x36U;
static constexpr uint16_t AS5600_TASK_STACK_DEPTH = 512U;
static constexpr UBaseType_t AS5600_TASK_PRIORITY = tskIDLE_PRIORITY + 3U;
static constexpr uint32_t AS5600_UPDATE_PERIOD_MS = 1U;
static constexpr uint32_t AS5600_RETRY_DELAY_MS = 1000U;

static as5600 encoder(AS5600_I2C_BUS_ID, AS5600_I2C_ADDRESS);
static topic::latest_topic<as5600_sample> sample_topic;

/**
 * @brief 初始化 AS5600 并周期发布最新样本
 *
 * @param argument FreeRTOS 任务参数
 */
static void as5600_task_entry(void *argument)
{
    i2c_result init_result;
    do
    {
        init_result = encoder.init();

        if(init_result != i2c_result::OK)
        {
            vTaskDelay(pdMS_TO_TICKS(AS5600_RETRY_DELAY_MS));
        }
    }
    while(init_result != i2c_result::OK);

    if(!sample_topic.publish(encoder.sample()))
    {
        Error_Handler();
    }

    TickType_t last_wake_time = xTaskGetTickCount();

    while(true)
    {
        if(encoder.update() == i2c_result::OK &&
            !sample_topic.publish(encoder.sample()))
        {
            Error_Handler();
        }

        vTaskDelayUntil(&last_wake_time,
            pdMS_TO_TICKS(AS5600_UPDATE_PERIOD_MS));
    }
}

/**
 * @brief 读取最新 AS5600 样本但不移除话题数据
 *
 * @param sample 用于接收快照的对象
 *
 * @return 已存在样本时返回 true
 */
bool as5600_dev::peek_latest(as5600_sample &sample)
{
    return sample_topic.peek(sample);
}

/**
 * @brief 初始化 AS5600 设备、话题并创建样本生产任务
 */
void as5600_dev::init()
{
    if(!sample_topic.init())
    {
        Error_Handler();
    }

    BaseType_t result = xTaskCreate(as5600_task_entry,
        "as5600",
        AS5600_TASK_STACK_DEPTH,
        nullptr,
        AS5600_TASK_PRIORITY,
        nullptr);

    if(result != pdPASS)
    {
        Error_Handler();
    }
}

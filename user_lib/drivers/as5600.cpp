#include "as5600.h"

#include "FreeRTOS.h"
#include "task.h"

static constexpr uint8_t AS5600_RAW_ANGLE_REGISTER = 0x0CU;
static constexpr uint16_t AS5600_RESOLUTION_COUNTS = 4096U;
static constexpr int16_t AS5600_HALF_RESOLUTION_COUNTS = 2048;
static constexpr float TWO_PI = 6.28318530717958647692f;
static constexpr float COUNT_TO_RADIAN =
    TWO_PI / (float)AS5600_RESOLUTION_COUNTS;

/**
 * @brief 创建 AS5600 驱动对象
 *
 * @param i2c_bus_id I2C 总线编号
 * @param device_address AS5600 的 7 位 I2C 地址
 */
as5600::as5600(uint8_t i2c_bus_id, uint8_t device_address)
    : i2c(i2c_bus_id),
      device_address(device_address)
{
}

/**
 * @brief 初始化 AS5600 总线并读取首个角度样本
 *
 * @return I2C 操作结果
 */
i2c_result as5600::init()
{
    initialized = false;
    first_sample = true;
    accumulated_count = 0;
    previous_count = 0U;
    previous_update_tick = 0U;
    current_sample = {};

    i2c_result result = i2c.init();
    if(result != i2c_result::OK)
    {
        return result;
    }

    initialized = true;
    result = update();
    if(result != i2c_result::OK)
    {
        initialized = false;
    }

    return result;
}

/**
 * @brief 读取并处理一个 AS5600 原始角度样本
 *
 * @return I2C 操作结果
 */
i2c_result as5600::update()
{
    if(!initialized)
    {
        return i2c_result::NOT_INITIALIZED;
    }

    uint8_t raw_data[2]{};
    i2c_result result = i2c.read_bytes(device_address,
        AS5600_RAW_ANGLE_REGISTER,
        raw_data,
        sizeof(raw_data));
    if(result != i2c_result::OK)
    {
        return result;
    }

    uint16_t raw_count = (uint16_t)(((uint16_t)raw_data[0] << 8) |
        raw_data[1]);
    raw_count &= AS5600_RESOLUTION_COUNTS - 1U;
    process_raw_angle(raw_count, (uint32_t)xTaskGetTickCount());
    return i2c_result::OK;
}

/**
 * @brief 获取最近一次成功读取的 AS5600 样本
 *
 * @return AS5600 样本只读引用
 */
const as5600_sample &as5600::sample() const
{
    return current_sample;
}

/**
 * @brief 将 12 位原始角度转换为单圈角度、累计角度和角速度
 *
 * @param raw_count 12 位原始角度计数
 * @param update_tick 当前 FreeRTOS tick
 */
void as5600::process_raw_angle(uint16_t raw_count, uint32_t update_tick)
{
    current_sample.raw_count = raw_count;
    current_sample.angle_rad = (float)raw_count * COUNT_TO_RADIAN;

    if(first_sample)
    {
        accumulated_count = raw_count;
        current_sample.full_angle_rad = current_sample.angle_rad;
        current_sample.angular_velocity_rad_s = 0.0f;
        first_sample = false;
    }
    else
    {
        int16_t delta_count = (int16_t)(raw_count - previous_count);

        if(delta_count >= AS5600_HALF_RESOLUTION_COUNTS)
        {
            delta_count -= (int16_t)AS5600_RESOLUTION_COUNTS;
        }
        else if(delta_count < -AS5600_HALF_RESOLUTION_COUNTS)
        {
            delta_count += (int16_t)AS5600_RESOLUTION_COUNTS;
        }

        accumulated_count += delta_count;
        current_sample.full_angle_rad =
            (float)accumulated_count * COUNT_TO_RADIAN;

        uint32_t elapsed_ticks = update_tick - previous_update_tick;
        if(elapsed_ticks > 0U)
        {
            float elapsed_seconds =
                (float)elapsed_ticks / (float)configTICK_RATE_HZ;
            current_sample.angular_velocity_rad_s =
                (float)delta_count * COUNT_TO_RADIAN / elapsed_seconds;
        }
    }

    previous_count = raw_count;
    previous_update_tick = update_tick;
    current_sample.timestamp_ms =
        update_tick * (uint32_t)portTICK_PERIOD_MS;
    current_sample.sequence++;
}

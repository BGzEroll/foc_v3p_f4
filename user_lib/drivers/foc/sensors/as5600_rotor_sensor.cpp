#include "as5600_rotor_sensor.h"

#include "system/timebase.h"

/**
 * @brief 创建 AS5600 FOC 转子传感器适配器
 *
 * @param i2c_bus_id I2C 总线编号
 * @param device_address AS5600 七位地址
 */
as5600_rotor_sensor::as5600_rotor_sensor(uint8_t i2c_bus_id,
    uint8_t device_address)
    : encoder(i2c_bus_id, device_address)
{
}

/**
 * @brief 初始化内部话题和 AS5600 驱动
 *
 * @return 初始化结果
 */
foc_result as5600_rotor_sensor::init()
{
    initialized = false;
    error_count = 0U;

    if(!sample_topic.init())
    {
        return foc_result::TOPIC_ERROR;
    }

    if(encoder.init() != i2c_result::OK)
    {
        error_count++;
        return foc_result::SENSOR_ERROR;
    }

    initialized = true;
    return publish_current_sample();
}

/**
 * @brief 在任务上下文完成一次 I2C 采样并发布最新快照
 *
 * @return 采样和发布结果
 */
foc_result as5600_rotor_sensor::update_task()
{
    if(!initialized){return foc_result::NOT_INITIALIZED;}

    if(encoder.update() != i2c_result::OK)
    {
        error_count++;
        return foc_result::SENSOR_ERROR;
    }

    return publish_current_sample();
}

/**
 * @brief 在任务上下文读取最新转子样本
 *
 * @param sample 用于接收样本的对象
 *
 * @return 已取得样本时返回 OK
 */
foc_result as5600_rotor_sensor::read_task(rotor_sample &sample)
{
    if(!initialized){return foc_result::NOT_INITIALIZED;}
    if(!sample_topic.peek(sample)){return foc_result::SAMPLE_NOT_READY;}
    return sample.valid ? foc_result::OK : foc_result::SAMPLE_NOT_READY;
}

/**
 * @brief 在中断上下文通过 Topic Peek 读取最新转子样本
 *
 * @param sample 用于接收样本的对象
 *
 * @return 已取得样本时返回 OK
 */
foc_result as5600_rotor_sensor::read_from_isr(rotor_sample &sample)
{
    if(!initialized){return foc_result::NOT_INITIALIZED;}
    if(!sample_topic.peek_from_isr(sample))
    {
        return foc_result::SAMPLE_NOT_READY;
    }

    return sample.valid ? foc_result::OK : foc_result::SAMPLE_NOT_READY;
}

/**
 * @brief 获取累计 I2C 通信或 Topic 发布错误次数
 *
 * @return 累计错误次数
 */
uint32_t as5600_rotor_sensor::communication_error_count() const
{
    return error_count;
}

/**
 * @brief 把底层 AS5600 样本转换并发布为 FOC 转子样本
 *
 * @return 发布结果
 */
foc_result as5600_rotor_sensor::publish_current_sample()
{
    const as5600_sample &source = encoder.sample();
    rotor_sample sample{};
    sample.sequence = source.sequence;
    sample.timestamp_us = timebase::now_us();
    sample.raw_count = source.raw_count;
    sample.mechanical_angle_rad = source.angle_rad;
    sample.full_angle_rad = source.full_angle_rad;
    sample.mechanical_velocity_rad_s = source.angular_velocity_rad_s;
    sample.valid = true;

    if(!sample_topic.publish(sample))
    {
        error_count++;
        return foc_result::TOPIC_ERROR;
    }

    return foc_result::OK;
}

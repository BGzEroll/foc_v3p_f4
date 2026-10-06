#include "as5600_rotor_sensor.h"

#include "system/sys_time.h"

static constexpr uint8_t AS5600_RAW_ANGLE_REGISTER = 0x0C;
static constexpr uint16_t AS5600_RESOLUTION_COUNTS = 4096;
static constexpr uint32_t AS5600_LOCK_TIMEOUT_MS = 1;
static constexpr uint32_t AS5600_TRANSFER_TIMEOUT_MS = 2;
static constexpr int16_t AS5600_HALF_RESOLUTION_COUNTS = 2048;
static constexpr float TWO_PI = 6.28318530717958647692f;
static constexpr float COUNT_TO_RADIAN =
    TWO_PI / (float)AS5600_RESOLUTION_COUNTS;

/**
 * @brief 创建直接拥有 I2C 总线访问的 AS5600 转子传感器
 *
 * @param i2c_bus_id I2C 总线编号
 * @param device_address AS5600 七位地址
 */
as5600_rotor_sensor::as5600_rotor_sensor(uint8_t i2c_bus_id,
    uint8_t device_address, bool low_latency)
    : i2c(i2c_bus_id),
      device_address(device_address), low_latency(low_latency)
{
}

/**
 * @brief 初始化内部状态、Topic 和 I2C 总线并读取首个样本
 *
 * @return 初始化结果
 */
foc_result as5600_rotor_sensor::init()
{
    if(initialized)
    {
        return foc_result::OK;
    }

    initialized = false;
    first_sample = true;
    accumulated_count = 0;
    previous_count = 0;
    previous_timestamp_us = 0;
    sequence = 0;
    // Keep failures visible across retries after an MCU-only reset.

    if(!sample_topic.init())
    {
        error_count++;
        return foc_result::TOPIC_ERROR;
    }

    if(i2c.init() != i2c_result::OK)
    {
        error_count++;
        return foc_result::SENSOR_ERROR;
    }

    if(low_latency) {
        // Volatile CONF only; never issue an OTP BURN command. A powered sensor
        // retains CONF across MCU resets, so read/modify/verify on every boot.
        uint8_t conf[2]{};
        auto read=i2c.read_bytes_blocking(device_address,0x07,conf,2,1,2);
        if(read!=i2c_result::OK) {error_count++;return foc_result::SENSOR_ERROR;}
        config_before=((uint16_t)conf[0]<<8)|conf[1];
        // SF=11 (2x linear filter), FTH=0, watchdog off, normal power mode.
        // Preserve analog/PWM output selection, hysteresis and factory bits.
        uint16_t wanted=(config_before & ~0x3f03u)|0x0300u;
        conf[0]=wanted>>8;conf[1]=wanted&0xff;
        auto write=i2c.write_bytes(device_address,0x07,conf,2,1,2);
        if(write!=i2c_result::OK) {error_count++;return foc_result::SENSOR_ERROR;}
        read=i2c.read_bytes_blocking(device_address,0x07,conf,2,1,2);
        config_after=((uint16_t)conf[0]<<8)|conf[1];
        uint8_t status[2]{};
        auto status_result=i2c.read_bytes_blocking(device_address,0x0b,status,2,1,2);
        magnet_status=status[0];
        uint8_t magnetic[3]{};
        auto magnetic_result=i2c.read_bytes_blocking(device_address,0x1a,magnetic,3,1,2);
        magnet_agc=magnetic[0];
        magnet_magnitude=(((uint16_t)magnetic[1]<<8)|magnetic[2])&0x0fff;
        if(read!=i2c_result::OK || config_after!=wanted || status_result!=i2c_result::OK ||
           magnetic_result!=i2c_result::OK || !(status[0]&0x20u)) {error_count++;return foc_result::SENSOR_ERROR;}
    }
    initialized = true;
    foc_result result = read_and_publish_sample();
    if(result != foc_result::OK)
    {
        initialized = false;
    }

    return result;
}

/**
 * @brief 在任务上下文读取 AS5600 并发布最新转子快照
 *
 * @return 采样和发布结果
 */
foc_result as5600_rotor_sensor::update_task()
{
    if(!initialized){return foc_result::NOT_INITIALIZED;}
    return read_and_publish_sample();
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
 * @brief 读取 AS5600 原始角度、生成物理量并发布完整快照
 *
 * @return I2C 读取和 Topic 发布结果
 */
foc_result as5600_rotor_sensor::read_and_publish_sample()
{
    uint8_t raw_data[2]{};
    uint32_t read_start_us=sys_time::get_us_tick();
    i2c_result read_result = i2c.read_bytes_blocking(device_address,
        AS5600_RAW_ANGLE_REGISTER,
        raw_data,
        sizeof(raw_data),AS5600_LOCK_TIMEOUT_MS,AS5600_TRANSFER_TIMEOUT_MS);
    uint32_t read_end_us=sys_time::get_us_tick();
    last_read_duration_us=read_end_us-read_start_us;
    if(last_read_duration_us>max_read_duration_us)max_read_duration_us=last_read_duration_us;
    last_i2c_result=(uint32_t)read_result;
    if(read_result != i2c_result::OK)
    {
        error_count++;
        consecutive_errors++;
        return foc_result::SENSOR_ERROR;
    }

    uint16_t raw_count = (uint16_t)(((uint16_t)raw_data[0] << 8) |
        raw_data[1]);
    raw_count &= AS5600_RESOLUTION_COUNTS - 1;

    rotor_sample sample{};
    process_raw_angle(raw_count, read_end_us, sample);
    last_success_us=sample.timestamp_us;
    consecutive_errors=0;
    if(!sample_topic.publish(sample))
    {
        error_count++;
        return foc_result::TOPIC_ERROR;
    }

    return foc_result::OK;
}

/**
 * @brief 把 12 位原始角度转换为单圈角度、累计角度和角速度
 *
 * @param raw_count AS5600 原始角度计数
 * @param timestamp_us 本次通信完成时间戳，单位微秒
 * @param sample 用于接收转换结果的对象
 */
void as5600_rotor_sensor::process_raw_angle(uint16_t raw_count,
    uint32_t timestamp_us,
    rotor_sample &sample)
{
    sample.sequence = ++sequence;
    sample.timestamp_us = timestamp_us;
    sample.raw_count = raw_count;
    sample.mechanical_angle_rad = (float)raw_count * COUNT_TO_RADIAN;

    if(first_sample)
    {
        accumulated_count = raw_count;
        sample.full_angle_rad = sample.mechanical_angle_rad;
        sample.mechanical_velocity_rad_s = 0.0f;
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
        sample.full_angle_rad =
            (float)accumulated_count * COUNT_TO_RADIAN;

        uint32_t elapsed_us = timestamp_us - previous_timestamp_us;
        if(elapsed_us > 0)
        {
            sample.mechanical_velocity_rad_s =
                (float)delta_count * COUNT_TO_RADIAN * 1000000.0f /
                (float)elapsed_us;
        }
    }

    previous_count = raw_count;
    previous_timestamp_us = timestamp_us;
    sample.valid = true;
}

#include "stm32_two_shunt_current_sensor.h"

#include <math.h>

/**
 * @brief 创建 STM32 ADC 双分流电阻相电流适配器
 *
 * @param config ADC 句柄、电流换算和方向配置
 */
stm32_two_shunt_current_sensor::stm32_two_shunt_current_sensor(
    const stm32_two_shunt_current_config &config)
    : config(config)
{
}

/**
 * @brief 校验 ADC 和双通道电流换算配置
 *
 * @return 配置有效时返回 OK
 */
foc_result stm32_two_shunt_current_sensor::init()
{
    initialized = false;
    calibrated = false;
    calibrating = false;
    sequence = 0;

    if(!config.adc || !config.adc->Instance ||
        !isfinite(config.ampere_per_count_a) ||
        !isfinite(config.ampere_per_count_b) ||
        config.ampere_per_count_a <= 0.0f ||
        config.ampere_per_count_b <= 0.0f ||
        (config.direction_a != 1 && config.direction_a != -1) ||
        (config.direction_b != 1 && config.direction_b != -1))
    {
        return foc_result::INVALID_CONFIG;
    }

    initialized = true;
    return foc_result::OK;
}

/**
 * @brief 在任务上下文推进一次由 ADC ISR 协作完成的零偏校准
 *
 * @param sample_count 需要累计的无电流样本数量
 *
 * @return 校准未完成时返回 CALIBRATING，完成时返回 OK
 */
foc_result stm32_two_shunt_current_sensor::calibrate_task(
    uint32_t sample_count)
{
    if(!initialized){return foc_result::NOT_INITIALIZED;}
    if(sample_count == 0){return foc_result::INVALID_ARGUMENT;}
    if(calibrated){return foc_result::OK;}

    if(!calibrating)
    {
        calibration_sum_a = 0;
        calibration_sum_b = 0;
        calibration_target = sample_count;
        calibration_collected = 0;
        calibrating = true;
        return foc_result::CALIBRATING;
    }

    if(calibration_target != sample_count)
    {
        return foc_result::INVALID_ARGUMENT;
    }

    if(calibration_collected < calibration_target)
    {
        return foc_result::CALIBRATING;
    }

    calibrating = false;
    offset_a = (float)calibration_sum_a / (float)calibration_target;
    offset_b = (float)calibration_sum_b / (float)calibration_target;
    calibrated = true;
    return foc_result::OK;
}

/**
 * @brief 更新两个相电流通道的测量极性
 *
 * @param direction_a A 相测量方向
 * @param direction_b B 相测量方向
 *
 * @return 两个方向均为正一或负一时返回 OK
 */
foc_result stm32_two_shunt_current_sensor::set_directions_task(
    int8_t direction_a,
    int8_t direction_b)
{
    if(!initialized){return foc_result::NOT_INITIALIZED;}
    if(calibrating){return foc_result::INVALID_STATE;}
    if((direction_a != 1 && direction_a != -1) ||
        (direction_b != 1 && direction_b != -1))
    {
        return foc_result::INVALID_ARGUMENT;
    }

    config.direction_a = direction_a;
    config.direction_b = direction_b;
    return foc_result::OK;
}

/**
 * @brief 从 ADC 注入序列寄存器读取并换算本周期三相电流
 *
 * @param timestamp_us 本次同步采样时间戳
 * @param sample 用于接收三相电流的对象
 *
 * @return 读取、校准或有效样本结果
 */
foc_result stm32_two_shunt_current_sensor::read_conversion_from_isr(
    uint32_t timestamp_us,
    phase_current_sample &sample)
{
    if(!initialized){return foc_result::NOT_INITIALIZED;}

    uint32_t raw_a = config.adc->Instance->JDR1;
    uint32_t raw_b = config.adc->Instance->JDR2;

    if(calibrating)
    {
        if(calibration_collected < calibration_target)
        {
            calibration_sum_a += raw_a;
            calibration_sum_b += raw_b;
            calibration_collected++;
        }

        return foc_result::CALIBRATING;
    }

    if(!calibrated){return foc_result::NOT_READY;}

    sample.sequence = ++sequence;
    sample.timestamp_us = timestamp_us;
    sample.raw_count_a = (uint16_t)raw_a;
    sample.raw_count_b = (uint16_t)raw_b;
    sample.offset_count_a = offset_a;
    sample.offset_count_b = offset_b;
    sample.current_a = ((float)raw_a - offset_a) *
        config.ampere_per_count_a * (float)config.direction_a;
    sample.current_b = ((float)raw_b - offset_b) *
        config.ampere_per_count_b * (float)config.direction_b;
    sample.current_c = -sample.current_a - sample.current_b;
    sample.valid = isfinite(sample.current_a) &&
        isfinite(sample.current_b) && isfinite(sample.current_c);
    return sample.valid ? foc_result::OK : foc_result::SENSOR_ERROR;
}

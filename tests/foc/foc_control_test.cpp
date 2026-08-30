#include "drivers/foc/foc_core.h"

#include <math.h>
#include <stdio.h>

class fake_rotor_sensor : public rotor_sensor
{
    public:
        foc_result init() override
        {
            initialized = true;
            sample.sequence = 1U;
            sample.timestamp_us = 1000U;
            sample.mechanical_angle_rad = 0.2f;
            sample.valid = true;
            return foc_result::OK;
        }

        foc_result update_task() override
        {
            return foc_result::OK;
        }

        foc_result read_task(rotor_sample &output) override
        {
            output = sample;
            return sample.valid ? foc_result::OK :
                foc_result::SAMPLE_NOT_READY;
        }

        foc_result read_from_isr(rotor_sample &output) override
        {
            return read_task(output);
        }

        uint32_t communication_error_count() const override
        {
            return 0U;
        }

    public:
        rotor_sample sample{};
        bool initialized = false;
};

class fake_current_sensor : public current_sensor
{
    public:
        foc_result init() override
        {
            initialized = true;
            return foc_result::OK;
        }

        foc_result calibrate_task(uint32_t) override
        {
            calibration_call_count++;
            return calibration_call_count == 1U ?
                foc_result::CALIBRATING : foc_result::OK;
        }

        foc_result read_conversion_from_isr(uint32_t timestamp_us,
            phase_current_sample &output) override
        {
            output = sample;
            output.sequence++;
            output.timestamp_us = timestamp_us;
            return output.valid ? foc_result::OK :
                foc_result::SAMPLE_NOT_READY;
        }

    public:
        phase_current_sample sample{};
        bool initialized = false;
        uint32_t calibration_call_count = 0U;
};

class fake_phase_driver : public phase_driver
{
    public:
        foc_result init() override
        {
            initialized = true;
            return foc_result::OK;
        }

        foc_result enable_output_task() override
        {
            enabled = true;
            return foc_result::OK;
        }

        void disable_output() override
        {
            enabled = false;
            disable_count++;
        }

        foc_result write_duty_from_isr(const phase_duty &duty) override
        {
            if(fail_write){return foc_result::OUTPUT_FAULT;}
            last_duty = duty;
            write_count++;
            return foc_result::OK;
        }

        bool fault_active_from_isr() const override
        {
            return hardware_fault;
        }

    public:
        phase_duty last_duty{};
        uint32_t write_count = 0U;
        uint32_t disable_count = 0U;
        bool initialized = false;
        bool enabled = false;
        bool fail_write = false;
        bool hardware_fault = false;
};

/**
 * @brief 报告失败的控制路径测试条件
 *
 * @param condition 测试条件
 * @param name 测试名称
 *
 * @return 条件成立时返回零，否则返回一
 */
static uint32_t expect(bool condition, const char *name)
{
    if(condition){return 0U;}
    printf("FAIL: %s\n", name);
    return 1U;
}

/**
 * @brief 清故障并重新进入运行状态
 *
 * @return 操作成功时返回 true
 */
static bool restart_control()
{
    return foc_core::clear_fault() == foc_result::OK &&
        foc_core::enable() == foc_result::OK;
}

/**
 * @brief 验证运行控制、快速故障和硬件禁能路径
 *
 * @return 所有测试通过时返回零
 */
int main()
{
    uint32_t failure_count = 0U;
    fake_rotor_sensor rotor{};
    fake_current_sensor current{};
    fake_phase_driver driver{};
    current.sample.valid = true;
    current.sample.current_a = 0.1f;
    current.sample.current_b = -0.05f;
    current.sample.current_c = -0.05f;

    foc_config config{};
    config.monitor_only = false;
    config.pole_pairs = 7U;
    config.bus_voltage_v = 24.0f;
    config.voltage_limit_v = 12.0f;
    config.max_phase_current_a = 10.0f;
    config.rotor_hard_timeout_us = 5000U;
    config.rotor_slow_timeout_us = 50000U;
    config.telemetry_divider = 1U;
    config.d_axis_pi.proportional_gain = 1.0f;
    config.d_axis_pi.integral_gain = 10.0f;
    config.d_axis_pi.integral_limit = 5.0f;
    config.q_axis_pi = config.d_axis_pi;

    failure_count += expect(foc_core::link_rotor_sensor(rotor) ==
        foc_result::OK, "link rotor");
    failure_count += expect(foc_core::link_current_sensor(current) ==
        foc_result::OK, "link current");
    failure_count += expect(foc_core::link_phase_driver(driver) ==
        foc_result::OK, "link driver");
    failure_count += expect(foc_core::init(config) == foc_result::OK,
        "initialize control mode");
    failure_count += expect(foc_core::update_bus_sensors() ==
        foc_result::OK, "initialize rotor in task");
    failure_count += expect(foc_core::set_rotor_alignment(-1, 0.3f) ==
        foc_result::OK, "set rotor alignment while disabled");
    failure_count += expect(foc_core::calibrate_current_task(16U) ==
        foc_result::CALIBRATING && driver.enabled,
        "calibrate with neutral output enabled");
    failure_count += expect(foc_core::calibrate_current_task(16U) ==
        foc_result::OK && driver.enabled,
        "keep neutral output active after calibration");

    foc_target target{};
    target.mode = foc_control_mode::OPEN_LOOP_VOLTAGE;
    target.d_axis_voltage_v = 1.0f;
    target.q_axis_voltage_v = 2.0f;
    target.electrical_angle_rad = 0.7f;
    target.timestamp_ms = 1U;
    failure_count += expect(foc_core::set_target(target) == foc_result::OK,
        "set voltage target");
    failure_count += expect(foc_core::enable() == foc_result::OK,
        "enable control");
    failure_count += expect(foc_core::set_rotor_alignment(1, 0.0f) ==
        foc_result::INVALID_STATE,
        "reject rotor alignment while running");
    failure_count += expect(foc_core::run_control_from_isr(1100U) ==
        foc_result::OK, "run valid control cycle");
    failure_count += expect(driver.write_count == 1U &&
        driver.last_duty.phase_a >= 0.0f &&
        driver.last_duty.phase_a <= 1.0f,
        "write valid duty");

    failure_count += expect(foc_core::run_control_from_isr(7000U) ==
        foc_result::SAMPLE_STALE && !driver.enabled,
        "stale rotor disables output");

    rotor.sample.timestamp_us = 7100U;
    failure_count += expect(restart_control(), "restart after stale fault");
    rotor.sample.valid = false;
    failure_count += expect(foc_core::run_control_from_isr(7200U) ==
        foc_result::SAMPLE_NOT_READY && !driver.enabled,
        "missing rotor disables output");

    rotor.sample.valid = true;
    rotor.sample.timestamp_us = 7300U;
    failure_count += expect(restart_control(), "restart after rotor fault");
    current.sample.current_a = 20.0f;
    failure_count += expect(foc_core::run_control_from_isr(7400U) ==
        foc_result::SENSOR_ERROR && !driver.enabled,
        "over current disables output");

    current.sample.current_a = NAN;
    failure_count += expect(restart_control(), "restart after over current");
    failure_count += expect(foc_core::run_control_from_isr(7500U) ==
        foc_result::SENSOR_ERROR && !driver.enabled,
        "nan current disables output");

    current.sample.current_a = 0.1f;
    failure_count += expect(restart_control(), "restart after nan fault");
    driver.fail_write = true;
    failure_count += expect(foc_core::run_control_from_isr(7600U) ==
        foc_result::OUTPUT_FAULT && !driver.enabled,
        "driver write failure disables output");
    driver.fail_write = false;
    failure_count += expect(foc_core::run_control_from_isr(7700U) ==
        foc_result::DISABLED,
        "fault remains latched");

    if(failure_count == 0U)
    {
        printf("FOC control tests passed\n");
    }

    return failure_count == 0U ? 0 : 1;
}

#include "drivers/foc/foc_core.h"

#include <stdio.h>

class fake_rotor_sensor : public rotor_sensor
{
    public:
        foc_result init() override
        {
            initialized = true;
            sample.valid = true;
            sample.sequence++;
            sample.timestamp_us = 1000U;
            return foc_result::OK;
        }

        foc_result update_task() override
        {
            if(fail_update)
            {
                error_count++;
                return foc_result::SENSOR_ERROR;
            }

            sample.sequence++;
            sample.timestamp_us += 1000U;
            return foc_result::OK;
        }

        foc_result read_task(rotor_sample &output) override
        {
            if(!initialized){return foc_result::NOT_INITIALIZED;}
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
            return error_count;
        }

    public:
        rotor_sample sample{};
        uint32_t error_count = 0U;
        bool initialized = false;
        bool fail_update = false;
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
            enable_count++;
            return foc_result::OK;
        }

        void disable_output() override
        {
            disable_count++;
        }

        foc_result write_duty_from_isr(const phase_duty &) override
        {
            return foc_result::OK;
        }

        bool fault_active_from_isr() const override
        {
            return false;
        }

    public:
        uint32_t enable_count = 0U;
        uint32_t disable_count = 0U;
        bool initialized = false;
};

/**
 * @brief 报告失败的监视状态测试条件
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
 * @brief 验证监视模式、连续通信故障和故障锁存
 *
 * @return 所有测试通过时返回零
 */
int main()
{
    uint32_t failure_count = 0U;
    fake_rotor_sensor rotor{};
    fake_phase_driver driver{};
    foc_config config{};
    config.monitor_only = true;
    config.communication_error_limit = 10U;
    config.rotor_hard_timeout_us = 5000U;
    config.rotor_slow_timeout_us = 50000U;

    failure_count += expect(foc_core::init(config) ==
        foc_result::NOT_LINKED, "init rejects missing links");
    failure_count += expect(foc_core::link_rotor_sensor(rotor) ==
        foc_result::OK, "link rotor");
    failure_count += expect(foc_core::link_rotor_sensor(rotor) ==
        foc_result::INVALID_STATE, "reject repeated rotor link");
    failure_count += expect(foc_core::link_phase_driver(driver) ==
        foc_result::OK, "link driver");

    foc_config invalid_config = config;
    invalid_config.rotor_direction = 0;
    failure_count += expect(foc_core::init(invalid_config) ==
        foc_result::INVALID_CONFIG, "reject invalid config");
    failure_count += expect(foc_core::init(config) == foc_result::OK,
        "initialize monitor mode");
    failure_count += expect(foc_core::enable() == foc_result::DISABLED,
        "monitor mode rejects enable");
    failure_count += expect(driver.enable_count == 0U,
        "monitor mode never calls hardware enable");

    failure_count += expect(foc_core::update_bus_sensors() ==
        foc_result::OK, "deferred rotor initialization");
    failure_count += expect(foc_core::update_safety(1U) == foc_result::OK,
        "monitor safety update");

    foc_snapshot snapshot{};
    failure_count += expect(foc_core::peek_snapshot(snapshot),
        "monitor snapshot available");
    failure_count += expect(snapshot.state == foc_state::MONITORING &&
        snapshot.rotor.valid && !snapshot.output_active,
        "monitor snapshot state");

    rotor.sample.timestamp_us = 1739U;
    failure_count += expect(foc_core::update_safety(1U) == foc_result::OK,
        "future sample from task preemption is accepted");
    foc_core::peek_snapshot(snapshot);
    failure_count += expect(snapshot.rotor_sample_age_ms == 0.0f &&
        snapshot.state == foc_state::MONITORING,
        "future sample age is clamped to zero");

    rotor.fail_update = true;
    for(uint32_t index = 0U; index < 10U; index++)
    {
        foc_core::update_bus_sensors();
    }
    failure_count += expect(foc_core::update_safety(12U) ==
        foc_result::SENSOR_ERROR, "communication error latches fault");
    foc_core::peek_snapshot(snapshot);
    failure_count += expect(snapshot.state == foc_state::FAULT &&
        (snapshot.fault_flags &
        foc_fault_mask(foc_fault::ROTOR_COMMUNICATION)) != 0U,
        "fault appears in snapshot");

    rotor.fail_update = false;
    foc_core::update_bus_sensors();
    foc_core::update_safety(13U);
    foc_core::peek_snapshot(snapshot);
    failure_count += expect(snapshot.state == foc_state::FAULT,
        "fault does not recover automatically");
    failure_count += expect(foc_core::clear_fault() == foc_result::OK,
        "explicit fault clear");
    foc_core::update_safety(14U);
    foc_core::peek_snapshot(snapshot);
    failure_count += expect(snapshot.state == foc_state::MONITORING,
        "clear returns to monitoring");

    if(failure_count == 0U)
    {
        printf("FOC monitor tests passed\n");
    }

    return failure_count == 0U ? 0 : 1;
}

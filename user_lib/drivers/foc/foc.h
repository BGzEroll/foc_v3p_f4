#ifndef FOC_H
#define FOC_H

#include "foc_hardware.h"
#include "foc_math.h"
#include "foc_runtime.h"
#include "foc_types.h"
#include "internal/foc_internal_runtime.h"
#include "system/topic.h"

namespace foc
{
    class instance
    {
        public:
            instance() = default;
            instance(const instance &) = delete;
            instance &operator=(const instance &) = delete;
            instance(instance &&) = delete;
            instance &operator=(instance &&) = delete;

        public:
            foc_result init(const foc_config &config,
                const foc_hardware &hardware);
            foc_result set_target(const foc_target &target);
            foc_result enable();
            void disable();
            foc_result clear_fault();
            bool peek_snapshot(foc_snapshot &snapshot);

        private:
            friend foc_result runtime::update_sensors(instance &motor);
            friend foc_result runtime::update_safety(instance &motor,
                uint32_t timestampMs);
            friend foc_result runtime::run_control_from_isr(
                instance &motor,
                uint32_t timestampUs);
            friend foc_result internal::set_command(instance &motor,
                const foc_command &command);
            friend foc_result internal::get_pole_pairs(
                const instance &motor,
                uint8_t &polePairs);
            friend foc_result internal::set_rotor_alignment(
                instance &motor,
                int8_t rotorDirection,
                float electricalZeroOffsetRad);
            friend foc_result internal::calibrate_current_task(
                instance &motor,
                uint32_t sampleCount);
            friend foc_result internal::set_current_directions_task(
                instance &motor,
                int8_t directionA,
                int8_t directionB);

            /**
             * @brief 保存一个 FOC 实例的全部可变核心状态和 Topic
             */
            struct coreState
            {
                foc_config config{};
                rotor_sensor *rotor = nullptr;
                current_sensor *current = nullptr;
                phase_driver *driver = nullptr;
                pi_state dAxisPi{};
                pi_state qAxisPi{};
                phase_current_sample faultCurrent{};
                volatile foc_state state = foc_state::UNINITIALIZED;
                volatile uint32_t faultFlags = 0;
                uint32_t commandSequence = 0;
                uint32_t controlIsrPrescaler = 0;
                uint32_t controlSequence = 0;
                uint32_t snapshotSequence = 0;
                uint32_t faultRequestSequence = 0;
                volatile uint32_t busUpdateErrorCount = 0;
                volatile uint32_t consecutiveBusErrorCount = 0;
                bool rotorInitialized = false;
                bool currentInitialized = false;
                bool driverInitialized = false;
                bool initialized = false;
                bool faultCurrentValid = false;
                bool currentCalibrationStarted = false;
                bool currentCalibrationDone = false;
                volatile bool commandValid = false;
                volatile bool calibrationOutputActive = false;
                topic::latest_topic<foc_command> commandTopic;
                topic::latest_topic<foc_control_telemetry> telemetryTopic;
                topic::latest_topic<foc_fault_request> faultRequestTopic;
                topic::latest_topic<phase_current_sample> currentSampleTopic;
                topic::latest_topic<foc_snapshot> snapshotTopic;
            };

            void latchFault(uint32_t faultFlags);
            void publishCurrentSampleFromIsr(
                const phase_current_sample &sample,
                bool forcePublish = false);

            coreState coreContext{};
    };
}

#endif

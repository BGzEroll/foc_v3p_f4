#ifndef FOC_COMMISSIONING_H
#define FOC_COMMISSIONING_H

#include "foc_types.h"
#include "system/topic.h"

enum class foc_commissioning_stage : uint8_t
{
    WAIT_CALIBRATION = 0,
    WAIT_ROTOR,
    OPEN_LOOP_FORWARD,
    OPEN_LOOP_PAUSE,
    OPEN_LOOP_REVERSE,
    ALIGN_FIRST,
    DIAGNOSE_PHASE_B,
    DIAGNOSE_PHASE_C,
    VERIFY_CURRENT_POLARITY,
    ALIGN_SECOND,
    VERIFY_D_AXIS_CURRENT,
    VERIFY_Q_AXIS_CURRENT,
    COMPLETE,
    FAILED
};

struct foc_commissioning_config
{
    uint32_t current_sensor_settle_time_ms = 500;
    uint32_t current_calibration_sample_count = 4096;
    float alignment_voltage_v = 3.0f;
    float open_loop_voltage_v = 3.0f;
    float open_loop_mechanical_velocity_rad_s = 10.0f;
    float minimum_bus_voltage_v = 9.0f;
    float maximum_bus_voltage_v = 15.0f;
    float d_axis_verify_current_a = 0.05f;
    float q_axis_verify_current_a = 0.03f;
};

struct foc_commissioning_status
{
    uint32_t sequence = 0;
    foc_commissioning_stage stage =
        foc_commissioning_stage::WAIT_CALIBRATION;
    foc_result result = foc_result::NOT_READY;
    int8_t current_direction_a = 1;
    int8_t current_direction_b = -1;
    int8_t rotor_direction = 1;
    float electrical_zero_offset_rad = 0.0f;
    float first_mechanical_angle_rad = 0.0f;
    float second_mechanical_angle_rad = 0.0f;
    float phase_vector_current_a[3]{};
    float phase_vector_current_b[3]{};
    float phase_vector_current_c[3]{};
    float phase_vector_mechanical_angle_rad[3]{};
    float bus_voltage_v = 0.0f;
    float minimum_bus_voltage_v = 0.0f;
    bool phase_vector_check_passed = false;
    float measured_d_axis_current_a = 0.0f;
    float measured_q_axis_current_a = 0.0f;
    float open_loop_voltage_v = 0.0f;
    float open_loop_mechanical_velocity_rad_s = 0.0f;
    float open_loop_electrical_velocity_rad_s = 0.0f;
    float open_loop_forward_delta_rad = 0.0f;
    float open_loop_reverse_delta_rad = 0.0f;
    bool open_loop_motion_detected = false;
};

namespace foc
{
    class instance;

    class commissioner
    {
        public:
            commissioner() = default;
            commissioner(const commissioner &) = delete;
            commissioner &operator=(const commissioner &) = delete;
            commissioner(commissioner &&) = delete;
            commissioner &operator=(commissioner &&) = delete;

        public:
            foc_result init(instance &motor,
                const foc_commissioning_config &config,
                uint32_t timestampMs);
            void update(uint32_t timestampMs, float busVoltageV);
            bool peek_status(foc_commissioning_status &status);

        private:
            void publish_commissioning_status();
            void reset_stage_samples();
            bool save_phase_vector_result(uint8_t vectorIndex,
                const foc_snapshot &snapshot);
            bool phase_vector_results_valid();
            void update_bus_voltage_status(float busVoltageV);
            void enter_commissioning_stage(
                foc_commissioning_stage stage,
                uint32_t timestampMs);
            void fail_commissioning(foc_result result);
            foc_result set_alignment_target(uint32_t timestampMs,
                float electricalAngleRad);
            foc_result set_open_loop_target(uint32_t timestampMs,
                float electricalAngleRad,
                float qAxisVoltageV,
                float electricalVelocityRadS);
            bool open_loop_bus_voltage_valid();
            foc_result start_open_loop_stage(
                foc_commissioning_stage stage,
                const foc_snapshot &snapshot,
                uint32_t timestampMs,
                float electricalAngleRad);
            float calculate_open_loop_rotation_rad(
                uint32_t rotationElapsedMs);
            float calculate_open_loop_velocity_rad_s(
                uint32_t rotationElapsedMs);
            foc_result update_open_loop_target(uint32_t timestampMs,
                uint32_t stageElapsedMs,
                int8_t direction);
            foc_result start_alignment_stage(
                foc_commissioning_stage stage,
                uint32_t timestampMs,
                float electricalAngleRad);
            void finish_open_loop_forward(const foc_snapshot &snapshot,
                uint32_t timestampMs);
            void finish_open_loop_reverse(const foc_snapshot &snapshot,
                uint32_t timestampMs);
            foc_result set_current_target(uint32_t timestampMs,
                float dAxisCurrentA,
                float qAxisCurrentA);
            void accumulate_stage_sample(const foc_snapshot &snapshot);
            void finish_first_alignment(const foc_snapshot &snapshot,
                uint32_t timestampMs);
            void finish_second_phase_vector(const foc_snapshot &snapshot,
                uint32_t timestampMs);
            void finish_third_phase_vector(const foc_snapshot &snapshot,
                uint32_t timestampMs);
            void finish_current_polarity_verification(uint32_t timestampMs);
            void finish_second_alignment(const foc_snapshot &snapshot,
                uint32_t timestampMs);
            void finish_d_axis_verification(uint32_t timestampMs);
            void finish_q_axis_verification();
            void update_commissioning(const foc_snapshot &snapshot,
                bool snapshotAvailable,
                uint32_t timestampMs);

            instance *motor = nullptr;
            foc_commissioning_config config{};
            foc_commissioning_status status{};
            topic::latest_topic<foc_commissioning_status> statusTopic;
            bool initialized = false;
            uint8_t polePairs = 0;
            uint32_t commissioningStartMs = 0;
            uint32_t stageStartMs = 0;
            float stageCurrentSumA = 0.0f;
            float stageCurrentSumB = 0.0f;
            float stageDAxisCurrentSum = 0.0f;
            float stageQAxisCurrentSum = 0.0f;
            uint32_t stageSampleCount = 0;
            bool currentCalibrationFinished = false;
            bool calibrationTaskStarted = false;
            float openLoopStageStartMechanicalAngleRad = 0.0f;
            float openLoopStageStartElectricalAngleRad = 0.0f;
    };
}

#endif

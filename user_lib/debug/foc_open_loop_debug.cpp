#include "foc_open_loop_debug.h"

#include "drivers/bus/uart_bus.h"
#include "drivers/foc/sensors/encoder/as5600_rotor_sensor.h"
#include "system/sys_time.h"
#include "main.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>

static constexpr uint16_t DEBUG_TASK_STACK_DEPTH = 384;
static constexpr uint16_t OBSERVER_TASK_STACK_DEPTH = 512;
static constexpr UBaseType_t DEBUG_TASK_PRIORITY = tskIDLE_PRIORITY + 4;
static constexpr UBaseType_t OBSERVER_TASK_PRIORITY = tskIDLE_PRIORITY + 3;
static constexpr uint32_t STARTUP_DELAY_MS = 1000;
static constexpr uint32_t STATIC_VECTOR_HOLD_TIME_MS = 1500;
static constexpr uint32_t ALIGNMENT_TIME_MS = 1000;
static constexpr uint32_t SLOW_RUN_TIME_MS = 8000;
static constexpr uint32_t TEST_PAUSE_MS = 2000;
static constexpr uint32_t TASK_UPDATE_PERIOD_MS = 1;
static constexpr uint32_t OBSERVER_UPDATE_PERIOD_MS = 2;
static constexpr uint32_t OBSERVER_OUTPUT_PERIOD_MS = 100;
static constexpr uint32_t INTERRUPT_EVENT_DIVIDER = 2;
static constexpr uint8_t AS5600_I2C_BUS_ID = 0;
static constexpr uint8_t AS5600_I2C_ADDRESS = 0x36;
static constexpr uint8_t DEBUG_UART_BUS_ID = 0;
static constexpr float BUS_VOLTAGE_V = 12.0f;
static constexpr float TEST_VOLTAGE_V = 3.0f;
static constexpr float TASK_MECHANICAL_VELOCITY_RAD_S = 1.0f;
static constexpr float INTERRUPT_MECHANICAL_VELOCITY_RAD_S = -1.0f;
static constexpr float MOTOR_POLE_PAIRS = 7.0f;
static constexpr float TWO_PI = 6.28318530717958647692f;
static constexpr float STATIC_VECTOR_STEP_RAD = TWO_PI / 6.0f;
static constexpr float SQRT_THREE_OVER_TWO = 0.86602540378443864676f;
static constexpr uint8_t STATIC_VECTOR_COUNT = 6;

enum class debug_stage : uint8_t
{
    STARTUP = 0,
    STATIC_VECTOR_TEST,
    TASK_ALIGNMENT,
    TASK_RUNNING,
    PAUSE,
    INTERRUPT_ALIGNMENT,
    INTERRUPT_RUNNING,
    COMPLETE,
    ERROR
};

static volatile debug_stage current_stage = debug_stage::STARTUP;
static volatile uint8_t static_vector_index = 0;
static volatile uint32_t task_update_count = 0;
static volatile uint32_t interrupt_event_count = 0;
static volatile uint32_t interrupt_update_count = 0;
static volatile uint32_t interrupt_last_timestamp_us = 0;
static volatile float interrupt_electrical_angle_rad = 0.0f;
static volatile float interrupt_electrical_velocity_rad_s = 0.0f;
static volatile bool interrupt_test_active = false;
static volatile bool observer_valid = false;
static volatile uint16_t observer_raw_count = 0;
static volatile int32_t observer_full_angle_millirad = 0;
static volatile uint32_t observer_sequence = 0;
static volatile uint32_t observer_error_count = 0;
static volatile int32_t static_vector_angle_millirad[STATIC_VECTOR_COUNT]{};
static volatile int32_t task_start_angle_millirad = 0;
static volatile int32_t task_end_angle_millirad = 0;
static volatile int32_t interrupt_start_angle_millirad = 0;
static volatile int32_t interrupt_end_angle_millirad = 0;

static as5600_rotor_sensor observer_rotor(AS5600_I2C_BUS_ID,
    AS5600_I2C_ADDRESS);
static uart_bus observer_uart(DEBUG_UART_BUS_ID);

/**
 * @brief 把电角度归一化到零至二倍圆周率
 *
 * @param angle_rad 输入电角度
 *
 * @return 归一化后的电角度
 */
static float normalize_angle(float angle_rad)
{
    float normalized = fmodf(angle_rad, TWO_PI);
    if(normalized < 0.0f)
    {
        normalized += TWO_PI;
    }

    return normalized;
}

/**
 * @brief 写入一个正弦调制三相电压矢量
 *
 * @param electrical_angle_rad 电角度
 */
static void write_voltage_vector(float electrical_angle_rad)
{
    float sine = sinf(electrical_angle_rad);
    float cosine = cosf(electrical_angle_rad);
    float phase_a_voltage = -TEST_VOLTAGE_V * sine;
    float phase_b_voltage = TEST_VOLTAGE_V *
        (0.5f * sine + SQRT_THREE_OVER_TWO * cosine);
    float phase_c_voltage = TEST_VOLTAGE_V *
        (0.5f * sine - SQRT_THREE_OVER_TWO * cosine);
    float phase_a_duty = 0.5f + phase_a_voltage / BUS_VOLTAGE_V;
    float phase_b_duty = 0.5f + phase_b_voltage / BUS_VOLTAGE_V;
    float phase_c_duty = 0.5f + phase_c_voltage / BUS_VOLTAGE_V;
    uint32_t period = htim1.Init.Period + 1;

    TIM1->CCR1 = (uint32_t)(phase_c_duty * (float)period);
    TIM1->CCR2 = (uint32_t)(phase_b_duty * (float)period);
    TIM1->CCR3 = (uint32_t)(phase_a_duty * (float)period);
}

/**
 * @brief 将三相比较值恢复为中性占空比
 */
static void write_neutral_duty()
{
    uint32_t neutral_compare = (htim1.Init.Period + 1) / 2;
    TIM1->CCR1 = neutral_compare;
    TIM1->CCR2 = neutral_compare;
    TIM1->CCR3 = neutral_compare;
}

/**
 * @brief 关闭门极驱动和定时器主输出
 */
static void disable_output()
{
    HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port,
        MOTOR_EN_Pin,
        GPIO_PIN_RESET);
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    write_neutral_duty();
}

/**
 * @brief 在中性占空比下开启门极驱动
 */
static void enable_output()
{
    write_neutral_duty();
    TIM1->BDTR |= TIM_BDTR_MOE;
    HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port,
        MOTOR_EN_Pin,
        GPIO_PIN_SET);
}

/**
 * @brief 初始化三相 PWM 并保持功率输出关闭
 *
 * @return 三个 PWM 通道均启动时返回 true
 */
static bool init_pwm()
{
    disable_output();

    if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
    {
        return false;
    }
    if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
    {
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        return false;
    }
    if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
    {
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        return false;
    }
    if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK)
    {
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        return false;
    }

    disable_output();
    return true;
}

/**
 * @brief 记录当前 AS5600 观测角度
 *
 * @return 最新累计机械角，单位毫弧度
 */
static int32_t read_observed_angle_millirad()
{
    return observer_full_angle_millirad;
}

/**
 * @brief 通过 UART 输出当前测试阶段与转子观测值
 */
static void output_observer_status()
{
    char message[160]{};
    int message_length = snprintf(message,
        sizeof(message),
        "OPEN_LOOP stage=%u vector=%u valid=%u raw=%u "
        "angle_mrad=%ld sequence=%lu errors=%lu\r\n",
        (unsigned int)current_stage,
        (unsigned int)static_vector_index,
        observer_valid ? 1 : 0,
        (unsigned int)observer_raw_count,
        (long)observer_full_angle_millirad,
        (unsigned long)observer_sequence,
        (unsigned long)observer_error_count);

    if(message_length <= 0 ||
        (uint32_t)message_length >= sizeof(message))
    {
        observer_error_count++;
        return;
    }

    uart_result result = observer_uart.write_bytes(
        reinterpret_cast<const uint8_t *>(message),
        (uint16_t)message_length);
    if(result != uart_result::OK)
    {
        observer_error_count++;
    }
}

/**
 * @brief 周期采集 AS5600 并输出独立于控制路径的机械角
 *
 * @param argument FreeRTOS 任务参数
 */
static void observer_task_entry(void *argument)
{
    if(observer_uart.init() != uart_result::OK ||
        observer_rotor.init() != foc_result::OK)
    {
        observer_error_count++;
        vTaskDelete(nullptr);
        return;
    }

    TickType_t last_wake_time = xTaskGetTickCount();
    uint32_t last_output_ms = sys_time::get_ms_tick();

    while(true)
    {
        foc_result update_result = observer_rotor.update_task();
        rotor_sample sample{};
        foc_result read_result = observer_rotor.read_task(sample);
        if(update_result == foc_result::OK &&
            read_result == foc_result::OK)
        {
            observer_raw_count = sample.raw_count;
            observer_full_angle_millirad =
                (int32_t)(sample.full_angle_rad * 1000.0f);
            observer_sequence = sample.sequence;
            observer_valid = true;
        }
        else
        {
            observer_error_count++;
        }

        uint32_t timestamp_ms = sys_time::get_ms_tick();
        if(timestamp_ms - last_output_ms >= OBSERVER_OUTPUT_PERIOD_MS)
        {
            output_observer_status();
            last_output_ms = timestamp_ms;
        }

        vTaskDelayUntil(&last_wake_time,
            pdMS_TO_TICKS(OBSERVER_UPDATE_PERIOD_MS));
    }
}

/**
 * @brief 依次保持六个等间隔静态电压矢量
 */
static void run_static_vector_test()
{
    current_stage = debug_stage::STATIC_VECTOR_TEST;
    enable_output();

    for(uint8_t index = 0; index < STATIC_VECTOR_COUNT; index++)
    {
        static_vector_index = index;
        write_voltage_vector((float)index * STATIC_VECTOR_STEP_RAD);
        vTaskDelay(pdMS_TO_TICKS(STATIC_VECTOR_HOLD_TIME_MS));
        static_vector_angle_millirad[index] =
            read_observed_angle_millirad();
    }

    static_vector_index = STATIC_VECTOR_COUNT;
    disable_output();
}

/**
 * @brief 在 RTOS 任务中运行一次固定速度开环测试
 */
static void run_task_test()
{
    float electrical_angle_rad = 0.0f;

    current_stage = debug_stage::TASK_ALIGNMENT;
    enable_output();
    write_voltage_vector(electrical_angle_rad);
    vTaskDelay(pdMS_TO_TICKS(ALIGNMENT_TIME_MS));
    task_start_angle_millirad = read_observed_angle_millirad();

    current_stage = debug_stage::TASK_RUNNING;
    uint32_t last_timestamp_us = sys_time::get_us_tick();
    uint32_t run_start_ms = sys_time::get_ms_tick();
    TickType_t last_wake_time = xTaskGetTickCount();
    while(sys_time::get_ms_tick() - run_start_ms < SLOW_RUN_TIME_MS)
    {
        uint32_t timestamp_us = sys_time::get_us_tick();
        uint32_t elapsed_us = timestamp_us - last_timestamp_us;
        last_timestamp_us = timestamp_us;
        electrical_angle_rad = normalize_angle(
            electrical_angle_rad +
            TASK_MECHANICAL_VELOCITY_RAD_S * MOTOR_POLE_PAIRS *
            (float)elapsed_us * 1.0e-6f);
        write_voltage_vector(electrical_angle_rad);
        task_update_count++;
        vTaskDelayUntil(&last_wake_time,
            pdMS_TO_TICKS(TASK_UPDATE_PERIOD_MS));
    }

    disable_output();
    task_end_angle_millirad = read_observed_angle_millirad();
}

/**
 * @brief 启动由定时器比较中断推进的开环输出
 *
 * @param electrical_velocity_rad_s 目标电角速度
 */
static void start_interrupt_output(float electrical_velocity_rad_s)
{
    interrupt_event_count = 0;
    interrupt_update_count = 0;
    interrupt_electrical_angle_rad = 0.0f;
    interrupt_electrical_velocity_rad_s = electrical_velocity_rad_s;
    interrupt_last_timestamp_us = sys_time::get_us_tick();
    interrupt_test_active = true;

    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC4);
    HAL_NVIC_ClearPendingIRQ(TIM1_CC_IRQn);
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC4);
    enable_output();
}

/**
 * @brief 停止定时器比较中断开环输出
 */
static void stop_interrupt_output()
{
    interrupt_test_active = false;
    __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC4);
    HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);
    HAL_NVIC_ClearPendingIRQ(TIM1_CC_IRQn);
    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC4);
    disable_output();
}

/**
 * @brief 在定时器比较中断中推进一次开环角度和三相占空比
 */
static void update_from_interrupt()
{
    interrupt_event_count++;
    if(!interrupt_test_active ||
        interrupt_event_count % INTERRUPT_EVENT_DIVIDER != 0)
    {
        return;
    }

    uint32_t timestamp_us = sys_time::get_us_tick();
    uint32_t elapsed_us = timestamp_us - interrupt_last_timestamp_us;
    interrupt_last_timestamp_us = timestamp_us;
    interrupt_electrical_angle_rad = normalize_angle(
        interrupt_electrical_angle_rad +
        interrupt_electrical_velocity_rad_s *
        (float)elapsed_us * 1.0e-6f);
    write_voltage_vector(interrupt_electrical_angle_rad);
    interrupt_update_count++;
}

/**
 * @brief 依次执行静态矢量、任务和中断三种最简开环测试
 *
 * @param argument FreeRTOS 任务参数
 */
static void debug_task_entry(void *argument)
{
    vTaskDelay(pdMS_TO_TICKS(STARTUP_DELAY_MS));

    if(!init_pwm())
    {
        current_stage = debug_stage::ERROR;
        disable_output();
        vTaskDelete(nullptr);
        return;
    }

    run_static_vector_test();

    current_stage = debug_stage::PAUSE;
    vTaskDelay(pdMS_TO_TICKS(TEST_PAUSE_MS));

    run_task_test();

    current_stage = debug_stage::PAUSE;
    vTaskDelay(pdMS_TO_TICKS(TEST_PAUSE_MS));

    current_stage = debug_stage::INTERRUPT_ALIGNMENT;
    start_interrupt_output(0.0f);
    vTaskDelay(pdMS_TO_TICKS(ALIGNMENT_TIME_MS));
    interrupt_start_angle_millirad = read_observed_angle_millirad();

    current_stage = debug_stage::INTERRUPT_RUNNING;
    interrupt_electrical_velocity_rad_s =
        INTERRUPT_MECHANICAL_VELOCITY_RAD_S * MOTOR_POLE_PAIRS;
    vTaskDelay(pdMS_TO_TICKS(SLOW_RUN_TIME_MS));
    stop_interrupt_output();
    interrupt_end_angle_millirad = read_observed_angle_millirad();

    current_stage = debug_stage::COMPLETE;
    vTaskDelete(nullptr);
}

/**
 * @brief 创建最简开环对照测试任务
 */
void foc_open_loop_debug::init()
{
    BaseType_t observer_result = xTaskCreate(observer_task_entry,
        "foc_observer",
        OBSERVER_TASK_STACK_DEPTH,
        nullptr,
        OBSERVER_TASK_PRIORITY,
        nullptr);
    BaseType_t debug_result = xTaskCreate(debug_task_entry,
        "foc_open_loop",
        DEBUG_TASK_STACK_DEPTH,
        nullptr,
        DEBUG_TASK_PRIORITY,
        nullptr);

    if(observer_result != pdPASS || debug_result != pdPASS)
    {
        current_stage = debug_stage::ERROR;
        disable_output();
        Error_Handler();
    }
}

/**
 * @brief 处理三相 PWM 定时器的比较事件
 */
extern "C" void TIM1_CC_IRQHandler(void)
{
    if(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_CC4) != RESET &&
        __HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_CC4) != RESET)
    {
        __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC4);
        update_from_interrupt();
    }
}

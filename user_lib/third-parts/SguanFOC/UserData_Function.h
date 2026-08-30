#ifndef __USERDATA_FUNCTION_H
#define __USERDATA_FUNCTION_H
/* 电机控制User用户设置·功能接口 */
/* 用户自己的CODE BEGIN Includes */
#include "adc.h"
#include "main.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"

extern volatile float sguan_encoder_angle_rad;

/* 用户自己的CODE END Includes */

static inline void User_InitialInit(void){
    // TIM8 的 CH1/2/3 已在板级初始化中启动，这里只在中性占空比下使能功率级。
    uint32_t neutral_compare = (htim8.Init.Period + 1) / 2;
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, neutral_compare);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, neutral_compare);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, neutral_compare);
    TIM8->BDTR |= TIM_BDTR_MOE;
    HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_SET);
}

static inline void User_Delay(unsigned int ms){
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static inline signed int User_ReadADC_Raw(unsigned char Current_CH){
    // 电流采样通道0->AB相，1->AC相，2->BC相
    // Sguan.motor.Current_Num值自定义采样通道
    // 用户在UserData_Motor.h中定义“这个值”
    signed int ADC_num = 0;
    switch (Current_CH){
    case 0:
        ADC_num = (signed int)hadc2.Instance->JDR1;
        break;
    case 1:
        ADC_num = (signed int)hadc2.Instance->JDR2;
        break;
    default:
        break;
    }
    return ADC_num;
}

static inline float User_Encoder_ReadRad(void){
    return sguan_encoder_angle_rad;
}

static inline void User_PwmDuty_Set(unsigned short int Duty_u,
                                unsigned short int Duty_v,
                                unsigned short int Duty_w){
    // 驱动板相序为逻辑 U/V/W -> TIM8 CH3/CH2/CH1，沿用 foc_test 配置。
    TIM8->CCR3 = Duty_u;
    TIM8->CCR2 = Duty_v;
    TIM8->CCR1 = Duty_w;
}

static inline float User_VBUS_DataGet(void){
    return (float)hadc3.Instance->DR * 3.3f * 11.0f / 4095.0f;
}

static inline float User_Temperature_DataGet(void){
    // float Temp_num = 0.0f;
    /* Your code for motor Temperature Data return if you use it */
    
    // 如果不使用温度功能，返回-9999.0f（正常温度不会是这么大的负数）
    return -9999.0f;
}


#endif // USERDATA_FUNCTION_H

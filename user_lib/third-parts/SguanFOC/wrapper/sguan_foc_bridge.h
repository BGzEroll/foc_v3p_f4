#ifndef SGUAN_FOC_BRIDGE_H
#define SGUAN_FOC_BRIDGE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 由 UserData_Parameter.h 在第三方默认配置完成后调用。
void sguan_foc_wrapper_apply_config(void);

// 由 UserData_UserControl.h 在 ADC2 高速环中调用。
void sguan_foc_wrapper_apply_command(void);

// 由 UserData_Function.h 在第三方核心中读取最新机械角度。
float sguan_foc_wrapper_read_encoder_rad(void);

// 由 UserData_Function.h 在第三方核心中读取逻辑 A/B/C 相电流。
uint8_t sguan_foc_wrapper_read_phase_currents(
    float *ia,
    float *ib,
    float *ic);

// 由 SguanFOC 初始化流程确认 CurrentSense 已完成零偏校准。
uint8_t sguan_foc_wrapper_current_offset_prepared(void);

#ifdef __cplusplus
}
#endif

#endif

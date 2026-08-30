#ifndef SGUAN_FOC_BRIDGE_H
#define SGUAN_FOC_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

// 由 UserData_Parameter.h 在第三方默认配置完成后调用。
void sguan_foc_wrapper_apply_config(void);

// 由 UserData_UserControl.h 在 ADC2 高速环中调用。
void sguan_foc_wrapper_apply_command(void);

#ifdef __cplusplus
}
#endif

#endif

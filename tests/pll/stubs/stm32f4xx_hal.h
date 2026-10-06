#ifndef PLL_TEST_HAL_STUB_H
#define PLL_TEST_HAL_STUB_H
#include <stdint.h>
// Only the ADC injected data registers used by this production driver.
typedef struct {volatile uint32_t JDR1,JDR2;} ADC_TypeDef;
typedef struct {ADC_TypeDef *Instance;} ADC_HandleTypeDef;
#endif

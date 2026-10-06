#include "pll_rtt.h"
#ifdef PLL_ENABLE_RTT
#include "drivers/foc/sensorless/pll_experiment.h"
#include "third-parts/segger_rtt/SEGGER_RTT.h"
#include "stm32f4xx_hal.h"
void pll_rtt_poll() {
    static bool initialized=false;
    static uint32_t previous=0;
    if(!initialized) {
        SEGGER_RTT_Init();
        SEGGER_RTT_SetFlagsUpBuffer(0,SEGGER_RTT_MODE_NO_BLOCK_SKIP);
        SEGGER_RTT_WriteString(0,"PLL lab: encoder start requires request=3; sensorless trial request=1; stop request=2\n");
        initialized=true;
    }
    uint32_t now=HAL_GetTick();if(now-previous<100)return;previous=now;
    uint32_t mask=__get_PRIMASK();__disable_irq();
    int w=(int)(pll_experiment.estimator.speed_rad_s*1000);
    int error=(int)(pll_experiment.angle_error*1000);
    unsigned lock=pll_experiment.estimator.locked,active=pll_experiment.active,fault=pll_experiment.fault;
    unsigned seq=pll_experiment.samples,period=pll_experiment.last_period_us;
    __set_PRIMASK(mask);
    SEGGER_RTT_printf(0,"PLL ms=%u seq=%u dt_us=%u w_mrad_s=%d error_mrad=%d locked=%u active=%u fault=%u\n",
        (unsigned)now,seq,period,w,error,lock,active,fault);
}
#else
void pll_rtt_poll() {}
#endif

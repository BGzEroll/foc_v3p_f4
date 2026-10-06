#include "pll_rtt.h"
#ifdef PLL_ENABLE_RTT
#include "drivers/foc/sensorless/pll_experiment.h"
#include "third-parts/segger_rtt/SEGGER_RTT.h"
#include "stm32f4xx_hal.h"
#include <math.h>
void pll_rtt_poll() {
    static bool initialized=false;
    static uint32_t previous=0;
    if(!initialized) {
        SEGGER_RTT_Init();
        SEGGER_RTT_SetFlagsUpBuffer(0,SEGGER_RTT_MODE_NO_BLOCK_SKIP);
        SEGGER_RTT_WriteString(0,"PLL lab: direct start request=4, static vectors=5; legacy encoder start=3/trial=1; stop=2\n");
        initialized=true;
    }
    uint32_t now=HAL_GetTick();if(now-previous<100)return;previous=now;
    uint32_t mask=__get_PRIMASK();__disable_irq();
    int w=(int)(pll_experiment.estimator.speed_rad_s*1000);
    int error=isfinite(pll_experiment.angle_error)?(int)(pll_experiment.angle_error*1000):0;
    unsigned lock=pll_experiment.estimator.locked,active=pll_experiment.active,fault=pll_experiment.fault;
    unsigned seq=pll_experiment.samples,period=pll_experiment.last_period_us;
    unsigned refvalid=isfinite(pll_experiment.reference_speed) && isfinite(pll_experiment.reference_angle);
    int wref=refvalid?(int)(pll_experiment.reference_speed*1000):0;
    int iqcmd=(int)((pll_experiment.direct_mode?pll_experiment.startup.target_iq:pll_experiment.encoder_iq_command_a)*1000);
    unsigned stage=pll_experiment.startup.state,reason=pll_experiment.startup.reason;
    float energy=pll_experiment.estimator.phase_error_energy;
    __set_PRIMASK(mask);
    int phase_rms=(int)(sqrtf(fmaxf(energy,0))*1000);
    SEGGER_RTT_printf(0,"PLL ms=%u seq=%u dt_us=%u w_mrad_s=%d ref_mrad_s=%d ref_valid=%u error_mrad=%d phase_rms_mrad=%d iq_cmd_mA=%d locked=%u active=%u fault=%u stage=%u reason=%u\n",
        (unsigned)now,seq,period,w,wref,refvalid,error,phase_rms,iqcmd,lock,active,fault,stage,reason);
}
#else
void pll_rtt_poll() {}
#endif

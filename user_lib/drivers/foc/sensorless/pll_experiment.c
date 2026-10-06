#include "pll_experiment.h"
#include <math.h>
#include <string.h>
pll_experiment_state pll_experiment;
void pll_experiment_init(void) {
    memset(&pll_experiment,0,sizeof(pll_experiment));
#ifdef PLL_DIRECT_SENSORLESS
    pll_experiment.direct_mode=1;
#endif
    pll_startup_init(&pll_experiment.startup);
    pll_experiment.capture_stride=10;
    pll_experiment.encoder_target_speed_rad_s=40.0f; // Mechanical rad/s.
    pll_experiment.encoder_current_limit_a=0.10f;
    bemf_pll_config c=bemf_pll_default_config();
    c.resistance_ohm=3.53f; // Board static V/I estimate, nominal ADC gain.
    // Motor recordings showed periodic noise; keep the tracking bandwidth low.
    c.pll_wn_rad_s=60.0f;
    bemf_pll_init(&pll_experiment.estimator,&c,1);
}
int pll_experiment_feedback(float *angle, float *speed) {
    if(pll_experiment.direct_mode){
        *angle=pll_experiment.startup.control_angle;
        *speed=pll_experiment.startup.control_speed;
        return 1; // Never fall back to encoder in this build, including standby/fault.
    }
    if (!pll_experiment.active || pll_experiment.fault) return 0;
    bemf_pll *s=&pll_experiment.estimator;
    *angle=bemf_pll_wrap(s->angle_rad+s->speed_rad_s*s->config.dt_s);
    *speed=s->speed_rad_s;
    return 1;
}
void pll_experiment_step(float va,float vb,float ia,float ib,float ref,float wref,float bus,uint32_t us) {
    pll_experiment_state *d=&pll_experiment;
    bemf_pll *s=&d->estimator;
    if(d->current_mapping != d->previous_mapping && !d->active) {
        bemf_pll_reset(s,s->direction); d->switch_good=0;
        d->previous_mapping=d->current_mapping;
    }
    // Shadow-only wiring diagnosis: mode 1 swaps logical A/C in the observer,
    // leaving encoder FOC current feedback untouched. Not a wiring calibration.
    if(d->current_mapping==1) {
        float a=ia, b=(1.73205080757f*ib-a)*0.5f, cc=-a-b;
        ia=cc; ib=(cc+2.0f*b)*0.57735026919f;
    }
    d->last_period_us=us-d->previous_us; d->previous_us=us;
    if(d->samples>0) {
        if(d->last_period_us>d->max_period_us)d->max_period_us=d->last_period_us;
        if(d->last_period_us>75)d->missed_periods++;
    }
    /* Encoder determines the startup direction only while in shadow mode. */
    if (!d->direct_mode && !d->active && fabsf(wref)>30 && s->direction*wref<0) bemf_pll_reset(s,wref<0?-1:1);
    bemf_pll_step(s,va,vb,ia,ib);
    d->samples++; d->reference_angle=ref; d->reference_speed=wref;
    d->angle_error=bemf_pll_wrap(s->angle_rad-ref);
    if(d->direct_mode){
        if(d->samples>1 && d->last_period_us>75)pll_startup_stop(&d->startup,PLL_START_TIMING);
        pll_startup_step(&d->startup,s,bus);
        d->active=d->startup.state==PLL_START_BLEND || d->startup.state==PLL_START_CLOSED;
        d->active_samples=d->startup.closed_ticks;
        if(d->startup.state==PLL_START_FAULT)d->fault=1;
    }
    /* This is a laboratory transfer criterion using the encoder as oracle.
     * It is NOT a sensorless standstill startup or automatic restart policy. */
    int good=!d->direct_mode && d->current_mapping==0 && s->locked && isfinite(ref) && isfinite(wref) && fabsf(d->angle_error)<0.20f &&
        fabsf(wref)>30 && fabsf(s->speed_rad_s-wref)<0.2f*fabsf(wref) && bus>10 && bus<14;
    if (good) { if(d->switch_good<10000) d->switch_good++; } else d->switch_good=0;
    if(d->switch_good>d->max_switch_good)d->max_switch_good=d->switch_good;
    if (d->request==1 && !d->active && d->switch_good>=10000 && !d->fault) {
        d->active=1; d->active_samples=0; d->request=0;
    }
    if (d->active && !d->direct_mode) {
        d->active_samples++;
        if (!s->valid || !s->locked || d->active_samples>=200000 ||
            (d->last_period_us>75 && d->samples>1)) d->fault=1;
    }
    if (d->request==2) { d->fault=1; d->request=0; if(d->direct_mode)pll_startup_stop(&d->startup,PLL_START_STOP); }
    if (d->capture_request) { d->capture_count=0; d->capture_divider=0; d->capture_request=0; }
    unsigned stride=d->capture_stride>=10 && d->capture_stride<=1000?d->capture_stride:10;
    if (d->capture_count<512 && ++d->capture_divider>=stride) {
        d->capture_divider=0;
        float *r=d->capture[d->capture_count];
        r[0]=us*1e-6f; r[1]=va; r[2]=vb; r[3]=ia; r[4]=ib;
        r[5]=ref; r[6]=wref; r[7]=s->angle_rad; r[8]=s->speed_rad_s;
        r[9]=s->emf_alpha_v; r[10]=s->emf_beta_v; r[11]=s->phase_error_rad;
        r[12]=s->locked; r[13]=d->active; r[14]=bus; r[15]=d->angle_error;
        d->capture_count++;
    }
}

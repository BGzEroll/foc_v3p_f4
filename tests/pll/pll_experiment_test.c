#include "pll_experiment.h"
#include <math.h>
#include <stdio.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x); return 1; } } while(0)
static float theta, ia, ib;
static uint32_t us;
static void reset(void) {
    pll_experiment_init(); theta=2.9f; ia=ib=0; us=0;
}
static void tick(float w, int gap) {
    bemf_pll_config c=pll_experiment.estimator.config;
    float ea=-0.0035f*w*sinf(theta), eb=0.0035f*w*cosf(theta);
    float va=ea+0.4f*cosf(theta), vb=eb+0.4f*sinf(theta);
    us+=(uint32_t)gap;
    pll_experiment_step(va,vb,ia,ib,theta,w,12,us);
    /* Independent, exact RL zero-order hold plant. */
    float decay=expf(-c.resistance_ohm*c.dt_s/c.inductance_h);
    ia=decay*ia+(1-decay)*(va-ea)/c.resistance_ohm;
    ib=decay*ib+(1-decay)*(vb-eb)/c.resistance_ohm;
    theta=bemf_pll_wrap(theta+w*c.dt_s);
}
static int acquire(void) {
    pll_experiment.request=1;
    for (int n=0;n<40000 && !pll_experiment.active;n++) tick(100,50);
    return pll_experiment.active && !pll_experiment.fault;
}
int main(void) {
    float angle,speed;
    reset(); pll_experiment.request=1;
    for(int n=0;n<20000;n++) tick(0,50);
    CHECK(!pll_experiment.active && !pll_experiment.switch_good);
    reset(); CHECK(acquire()); CHECK(pll_experiment.switch_good==10000);
    CHECK(pll_experiment_feedback(&angle,&speed)); CHECK(fabsf(speed-100)<1);
    CHECK(fabsf(bemf_pll_wrap(angle-theta))<0.025f);
    for(int n=0;n<20000 && !pll_experiment.fault;n++) tick(0,50);
    CHECK(pll_experiment.fault); CHECK(!pll_experiment_feedback(&angle,&speed));
    reset(); CHECK(acquire()); tick(100,100);
    CHECK(pll_experiment.fault && pll_experiment.missed_periods==1);
    reset(); CHECK(acquire()); pll_experiment.request=2; tick(100,50);
    CHECK(pll_experiment.fault && pll_experiment.request==0);
    reset(); CHECK(acquire());
    while(pll_experiment.active_samples<200000 && !pll_experiment.fault) tick(100,50);
    CHECK(pll_experiment.fault && pll_experiment.active_samples==200000);
    reset(); pll_experiment.current_mapping=1; pll_experiment.request=1;
    for(int n=0;n<40000;n++) tick(100,50);
    CHECK(!pll_experiment.active && pll_experiment.switch_good==0);
    puts("PASS: standstill/map guard, qualified transfer, EMF loss, missed period, stop, 10s timeout");
    return 0;
}

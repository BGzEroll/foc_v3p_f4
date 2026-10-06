#include "pll_experiment.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
int main(void){
    pll_startup s;bemf_pll p={0};
    pll_startup_begin(&s);
    for(int n=0;n<14000;n++)pll_startup_step(&s,&p,12);
    assert(s.state==PLL_START_RAMP);
    for(int n=0;n<57000;n++)pll_startup_step(&s,&p,12);
    assert(s.state==PLL_START_WAIT && s.control_speed==280);
    for(int n=0;n<40000;n++)pll_startup_step(&s,&p,12);
    assert(s.state==PLL_START_FAULT && s.reason==PLL_START_NO_LOCK);
    pll_startup_begin(&s);pll_startup_step(&s,&p,NAN);
    assert(s.reason==PLL_START_BAD_BUS);
    pll_startup_begin(&s);s.state=PLL_START_WAIT;s.open_speed=280;
    p.valid=p.locked=1;p.direction=1;p.speed_rad_s=280;p.emf_magnitude_v=1;
    for(int n=0;n<8000;n++){
        p.angle_rad=s.open_angle;
        pll_startup_step(&s,&p,12);
    }
    assert(s.state==PLL_START_CLOSED && s.target_id==0 && s.target_iq>0);
    p.locked=0;pll_startup_step(&s,&p,12);assert(s.reason==PLL_START_LOST_LOCK);
    pll_startup_begin(&s);s.state=PLL_START_CLOSED;s.closed_ticks=199999;
    p.locked=1;pll_startup_step(&s,&p,12);assert(s.reason==PLL_START_TIMEOUT);
    bemf_pll recorded;pll_startup startup;
    for(int pass=0;pass<2;pass++){
        pll_experiment_init();pll_startup_begin(&pll_experiment.startup);
        for(unsigned n=1;n<15000;n++)pll_experiment_step(0,0,0,0,pass?NAN:1,pass?NAN:-100,12,n*50);
        if(!pass){recorded=pll_experiment.estimator;startup=pll_experiment.startup;}
        else {assert(!memcmp(&recorded,&pll_experiment.estimator,sizeof(recorded)));
              assert(!memcmp(&startup,&pll_experiment.startup,sizeof(startup)));}
        assert(pll_experiment.estimator.direction==1);
    }
    puts("PASS: direct startup gates, timeout, lost lock, encoder independence");
    pll_startup_begin(&s);s.static_scan=1;
    for(int n=0;n<84000;n++)pll_startup_step(&s,&p,12);
    assert(s.state==PLL_START_FAULT && s.reason==PLL_START_STATIC_DONE);
    pll_startup_stop(&s,PLL_START_STOP);assert(s.reason==PLL_START_STATIC_DONE);
}

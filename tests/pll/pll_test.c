#include "bemf_pll.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x); return 1; } } while(0)
int main(void) {
    bemf_pll s; bemf_pll_config c = bemf_pll_default_config();
    CHECK(!bemf_pll_init(&s, &c, 0));
    c.dt_s = NAN; CHECK(!bemf_pll_init(&s, &c, 1));
    c = bemf_pll_default_config();
    for (int direction=-1; direction<=1; direction+=2) {
        CHECK(bemf_pll_init(&s,&c,direction));
        float theta = 2.9f, ia=0, ib=0, max_error=0, max_speed_error=0;
        int lock_samples=0;
        for (int n=0;n<40000;n++) {
            float w=direction*(n<20000 ? 100.0f : 180.0f);
            float ea=-0.0035f*w*sinf(theta), eb=0.0035f*w*cosf(theta);
            float va=ea+0.4f*cosf(theta), vb=eb+0.4f*sinf(theta);
            CHECK(bemf_pll_step(&s,va,vb,ia,ib));
            if (n>5000 && (n<20000 || n>25000)) {
                float error=fabsf(bemf_pll_wrap(s.angle_rad-theta));
                float sw=fabsf(s.speed_rad_s-w);
                if (error>max_error) max_error=error;
                if (sw>max_speed_error) max_speed_error=sw;
                lock_samples+=s.locked;
            }
            /* Independent exact RL hold solution: not the observer's Euler plant. */
            float decay=expf(-c.resistance_ohm*c.dt_s/c.inductance_h);
            ia=decay*ia+(1-decay)*(va-ea)/c.resistance_ohm;
            ib=decay*ib+(1-decay)*(vb-eb)/c.resistance_ohm;
            theta=bemf_pll_wrap(theta+w*c.dt_s);
        }
        printf("direction=%d max_angle_error=%.6f rad max_speed_error=%.5f rad/s lock_samples=%d\n",direction,max_error,max_speed_error,lock_samples);
        CHECK(max_error<0.025f); CHECK(max_speed_error<1.0f); CHECK(lock_samples>28000);
        CHECK(!bemf_pll_step(&s,NAN,0,0,0)); CHECK(!s.locked && !s.valid && s.invalid_samples==1);
        for (int n=0;n<5000;n++) CHECK(bemf_pll_step(&s,0,0,0,0));
        CHECK(!s.locked && !s.valid && s.speed_rad_s==0);
    }
    CHECK(bemf_pll_init(&s,&c,1));
    /* DC model residual can exceed the EMF threshold at a stopped motor.
     * The speed floor must prevent advertising a rotational lock. */
    for(int n=0;n<10000;n++) CHECK(bemf_pll_step(&s,0.3f,0.2f,0,0));
    CHECK(!s.locked && fabsf(s.speed_rad_s)<1);
    puts("PASS: bidirectional tracking, speed step, wrapping, standstill, invalid input/config");
    return 0;
}

#include "pll_startup.h"
#include <math.h>
#include <string.h>
#define DT 0.00005f
static float clip(float x,float lo,float hi){return fminf(hi,fmaxf(lo,x));}
void pll_startup_init(pll_startup *s){
    memset(s,0,sizeof(*s));s->target_speed=280.0f;s->startup_id=0.25f;
}
void pll_startup_begin(pll_startup *s){pll_startup_init(s);s->state=PLL_START_ALIGN;}
int pll_startup_running(const pll_startup *s){return s->state>=PLL_START_ALIGN && s->state<=PLL_START_CLOSED;}
void pll_startup_stop(pll_startup *s,unsigned reason){
    if(s->state==PLL_START_FAULT)return; // Preserve first cause when the host subsequently stops.
    s->state=PLL_START_FAULT;s->reason=reason;s->target_id=s->target_iq=0;
}
void pll_startup_step(pll_startup *s,const bemf_pll *p,float bus){
    if(!pll_startup_running(s))return;
    s->ticks++;s->stage_ticks++;
    if(!isfinite(bus)||bus<10 ||bus>14){pll_startup_stop(s,PLL_START_BAD_BUS);return;}
    if(s->ticks>=360000){pll_startup_stop(s,PLL_START_TIMEOUT);return;}
    if(s->state==PLL_START_ALIGN){
        s->control_angle=s->static_scan?(s->ticks/14000)*1.0471975512f:0;
        s->control_speed=0;
        if(s->static_scan){
            if(s->ticks>=84000)pll_startup_stop(s,PLL_START_STATIC_DONE);
            return;
        }
        if(s->stage_ticks>=14000){s->state=PLL_START_RAMP;s->stage_ticks=0;}
        return;
    }
    if(s->state==PLL_START_RAMP){
        s->open_speed=fminf(s->target_speed,s->open_speed+100.0f*DT);
        if(s->open_speed>=s->target_speed){s->state=PLL_START_WAIT;s->stage_ticks=0;}
    }
    if(s->state==PLL_START_BLEND || s->state==PLL_START_CLOSED)s->open_speed=p->speed_rad_s;
    s->open_angle=bemf_pll_wrap(s->open_angle+s->open_speed*DT);
    s->angle_difference=bemf_pll_wrap(p->angle_rad+p->speed_rad_s*DT-s->open_angle);
    s->control_angle=s->open_angle;s->control_speed=s->open_speed;
    s->target_id=s->startup_id;s->target_iq=0;
    if(s->state==PLL_START_WAIT){
        int good=p->valid && p->locked && p->direction==1 && p->emf_magnitude_v>.15f &&
            fabsf(s->angle_difference)<.35f && fabsf(p->speed_rad_s-s->open_speed)<.25f*s->open_speed;
        s->qualified_ticks=good?s->qualified_ticks+1:0;
        if(s->qualified_ticks>=2000){s->state=PLL_START_BLEND;s->stage_ticks=0;}
        else if(s->stage_ticks>=40000){pll_startup_stop(s,PLL_START_NO_LOCK);return;}
    }
    if(s->state==PLL_START_BLEND || s->state==PLL_START_CLOSED){
        if(!p->valid || !p->locked || !isfinite(p->speed_rad_s) || p->speed_rad_s<30){
            pll_startup_stop(s,PLL_START_LOST_LOCK);return;
        }
        float a=s->state==PLL_START_CLOSED?1.0f:clip(s->stage_ticks/6000.0f,0,1);
        s->control_angle=bemf_pll_wrap(s->open_angle+a*s->angle_difference);
        s->control_speed=(1-a)*s->open_speed+a*p->speed_rad_s;
        s->target_id=(1-a)*s->startup_id;
        s->target_iq=a*clip(.020f+.002f*(s->target_speed-p->speed_rad_s)/7.0f,0,.10f);
        if(s->state==PLL_START_BLEND && s->stage_ticks>=6000){s->state=PLL_START_CLOSED;s->stage_ticks=0;}
        if(++s->closed_ticks>=200000){pll_startup_stop(s,PLL_START_TIMEOUT);return;}
    }
}

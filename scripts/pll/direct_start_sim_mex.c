#include "mex.h"
#include "pll_startup.h"
#include "Sguan_PID.h"
#include <math.h>
static float clip(float x,float lo,float hi){return fminf(hi,fmaxf(lo,x));}
void mexFunction(int nl,mxArray **out,int nr,const mxArray **in){
    (void)in;
    if(nl!=1||nr!=0)mexErrMsgIdAndTxt("pll:args","One output, no input required.");
    const float dt=.00005f,R=3.53f,L=.00086f,flux=.0035f,J=.000005f;
    pll_startup s;pll_startup_begin(&s);bemf_pll p;
    bemf_pll_config c=bemf_pll_default_config();c.pll_wn_rad_s=60;c.resistance_ohm=R;
    bemf_pll_init(&p,&c,1);
    PID_STRUCT d={0},q={0};
    d.Kp=q.Kp=.2995f;d.Ki=q.Ki=300;d.T=q.T=dt;
    d.OutMax=q.OutMax=3;d.OutMin=q.OutMin=-3;
    d.IntMax=q.IntMax=1;d.IntMin=q.IntMin=-1;
    PID_Init(&d);PID_Init(&q);
    float ia=0,ib=0,theta=1.3f,wm=0,peak=0;
    const int N=240000;out[0]=mxCreateDoubleMatrix(N/20,13,mxREAL);
    double *v=mxGetPr(out[0]);int row=0;
    for(int n=0;n<N;n++){
        float sine=sinf(s.control_angle),cosine=cosf(s.control_angle);
        float id=ia*cosine+ib*sine,iq=-ia*sine+ib*cosine,ud=0,uq=0;
        if(s.state==PLL_START_ALIGN){ud=3;PID_Init(&d);PID_Init(&q);}
        else if(pll_startup_running(&s)){
            d.run.Ref=s.target_id;d.run.Fbk=id;q.run.Ref=s.target_iq;q.run.Fbk=iq;
            PID_Loop(&d);PID_Loop(&q);
            ud=d.run.Output-s.control_speed*L*iq;
            uq=q.run.Output+s.control_speed*(L*id+flux);
        }
        float mag=hypotf(ud,uq);if(mag>3){ud*=3/mag;uq*=3/mag;}
        float va=ud*cosine-uq*sine,vb=ud*sine+uq*cosine;
        bemf_pll_step(&p,va,vb,ia,ib);pll_startup_step(&s,&p,12);
        float es=flux*7*wm,ea=-es*sinf(theta),eb=es*cosf(theta);
        ia+=dt*(va-R*ia-ea)/L;ib+=dt*(vb-R*ib-eb)/L;
        float phaseb=-.5f*ia+.8660254f*ib,phasec=-.5f*ia-.8660254f*ib;
        float instantaneous=fmaxf(fabsf(ia),fmaxf(fabsf(phaseb),fabsf(phasec)));
        peak=fmaxf(peak,instantaneous);
        if(instantaneous>1.8f)pll_startup_stop(&s,99);
        float torque=1.5f*7*flux*(-ia*sinf(theta)+ib*cosf(theta));
        float load=.000008f*wm;
        if(fabsf(wm)<.001f && fabsf(torque-load)<.0006f)wm=0;
        else wm+=dt*(torque-load-copysignf(.0006f,fabsf(wm)<.001f?torque:wm))/J;
        theta=bemf_pll_wrap(theta+7*wm*dt);
        if(n%20==0){
            float values[13]={n*dt,theta,7*wm,p.angle_rad,p.speed_rad_s,s.control_angle,
                s.open_speed,id,iq,(float)s.state,(float)p.locked,(float)s.reason,peak};
            for(int k=0;k<13;k++)v[row+k*(N/20)]=values[k];row++;
        }
    }
}

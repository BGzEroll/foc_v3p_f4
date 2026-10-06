#include "Sguan_PID.h"
#include <math.h>
#include <stdio.h>
#define CHECK(x) do {if(!(x)){fprintf(stderr,"FAIL line %d\n",__LINE__);return 1;}}while(0)
int main(void) {
    PID_STRUCT d={0},q={0};
    d.T=q.T=.001;d.Ki=q.Ki=1;d.Wc=q.Wc=100;
    d.IntMax=.01f;d.IntMin=-.01f;q.IntMax=1;q.IntMin=-1;
    d.OutMax=q.OutMax=3;d.OutMin=q.OutMin=-3;
    PID_Init(&d);PID_Init(&q);d.run.Ref=100;q.run.Ref=1;
    PID_Loop(&d);CHECK(d.run.integral_frozen);
    PID_Loop(&q);CHECK(fabsf(q.run.Io[0]-.0005f)<1e-7f);
    PID_Loop(&d);PID_Loop(&q);CHECK(fabsf(q.run.Io[0]-.0015f)<1e-7f);
    CHECK(d.run.integral_frozen && !q.run.integral_frozen);
    d.run.Ref=-1;PID_Loop(&d);PID_Loop(&d);CHECK(d.run.Io[0]<.01f);
    PID_Init(&d);CHECK(!d.run.integral_frozen && d.run.Io[0]==0);
    puts("PASS: independent D/Q anti-windup, reverse-error release and reset");
    return 0;
}

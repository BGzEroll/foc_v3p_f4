#include "Sguan_math.h"
#include <math.h>
#include <stdio.h>
#define CHECK(x) do { if(!(x)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x); return 1; } } while(0)
int main(void) {
    for(int n=0;n<360;n++) {
        float t=n*6.28318530718f/360, s=sinf(t), c=cosf(t), a,b,w;
        SVPWM(0.25f,0.08f,s,c,&a,&b,&w);
        float va=(2*a-b-w)/3, vb=(b-w)/sqrtf(3);
        CHECK(fabsf(va-(.25f*c-.08f*s))<1e-6f);
        CHECK(fabsf(vb-(.25f*s+.08f*c))<1e-6f);
        CHECK(a>=0 && a<=1 && b>=0 && b<=1 && w>=0 && w<=1);
    }
    float a,b,c; SVPWM(NAN,0,0,1,&a,&b,&c); CHECK(a==.5f && b==.5f && c==.5f);
    SVPWM(100,100,0,1,&a,&b,&c); CHECK(a>=0 && a<=1 && b>=0 && b<=1 && c>=0 && c<=1);
    puts("PASS: SI voltage reconstruction in every sector, nonfinite neutral, linear modulation bound");
    return 0;
}

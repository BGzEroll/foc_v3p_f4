#include "mex.h"
#include "bemf_pll.h"
#include <string.h>
static bemf_pll state;
void mexFunction(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[]) {
    if(nrhs==2 && mxIsChar(prhs[0])) {
        char cmd[16]; mxGetString(prhs[0],cmd,sizeof(cmd));
        if(strcmp(cmd,"reset") || !mxIsDouble(prhs[1]) || mxGetNumberOfElements(prhs[1])!=1)
            mexErrMsgIdAndTxt("pll:reset","Use pll_step_mex('reset',+/-1)");
        double d=mxGetScalar(prhs[1]); bemf_pll_config c=bemf_pll_default_config();
        if((d!=1 && d!=-1) || !bemf_pll_init(&state,&c,(int)d)) mexErrMsgIdAndTxt("pll:config","Invalid direction");
        return;
    }
    if(nrhs!=1 || nlhs!=1 || !mxIsDouble(prhs[0]) || mxIsComplex(prhs[0]) || mxGetNumberOfElements(prhs[0])!=4 || !state.configured)
        mexErrMsgIdAndTxt("pll:input","Reset first, then y=pll_step_mex([va vb ia ib])");
    const double *x=mxGetPr(prhs[0]); bemf_pll_step(&state,x[0],x[1],x[2],x[3]);
    plhs[0]=mxCreateDoubleMatrix(8,1,mxREAL); double *y=mxGetPr(plhs[0]);
    y[0]=state.angle_rad; y[1]=state.speed_rad_s; y[2]=state.emf_alpha_v; y[3]=state.emf_beta_v;
    y[4]=state.phase_error_rad; y[5]=state.valid; y[6]=state.locked; y[7]=state.invalid_samples;
}

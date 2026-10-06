#include "mex.h"
#include "bemf_pll.h"
/* out = pll_mex([va vb ia ib], direction, [R L observer_wn pll_wn min_emf dt]) */
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    if (nrhs < 2 || nrhs > 3 || nlhs != 1 || !mxIsDouble(prhs[0]) || mxIsComplex(prhs[0]) || mxGetN(prhs[0]) != 4 ||
        !mxIsDouble(prhs[1]) || mxGetNumberOfElements(prhs[1]) != 1)
        mexErrMsgIdAndTxt("pll:input","Use output = pll_mex(Nx4 real double input, +/-1, optional six-value config)");
    bemf_pll_config c=bemf_pll_default_config();
    if(nrhs==3) {
        if (!mxIsDouble(prhs[2]) || mxIsComplex(prhs[2]) || mxGetNumberOfElements(prhs[2])!=6)
            mexErrMsgIdAndTxt("pll:config","Config must have six real double values");
        const double *p=mxGetPr(prhs[2]);
        c.resistance_ohm=p[0]; c.inductance_h=p[1]; c.observer_wn_rad_s=p[2];
        c.pll_wn_rad_s=p[3]; c.min_emf_v=p[4]; c.dt_s=p[5];
    }
    double direction=mxGetScalar(prhs[1]);
    if(direction!=1 && direction!=-1) mexErrMsgIdAndTxt("pll:direction","Direction must be +1 or -1");
    bemf_pll s;
    if(!bemf_pll_init(&s,&c,(int)direction)) mexErrMsgIdAndTxt("pll:config","Invalid or unstable config");
    mwSize n=mxGetM(prhs[0]); const double *x=mxGetPr(prhs[0]);
    plhs[0]=mxCreateDoubleMatrix(n,8,mxREAL); double *y=mxGetPr(plhs[0]);
    for(mwSize k=0;k<n;k++) {
        bemf_pll_step(&s,x[k],x[k+n],x[k+2*n],x[k+3*n]);
        y[k]=s.angle_rad; y[k+n]=s.speed_rad_s; y[k+2*n]=s.emf_alpha_v; y[k+3*n]=s.emf_beta_v;
        y[k+4*n]=s.phase_error_rad; y[k+5*n]=s.valid; y[k+6*n]=s.locked; y[k+7*n]=s.invalid_samples;
    }
}

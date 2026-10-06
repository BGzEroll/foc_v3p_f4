function summary=run_observer_bandwidth_study
% Rebuild the shared C MEX, then compare observer noise sensitivity at the
% board experiment's speed. This remains an imposed electrical trajectory;
% it does not model the real inverter, friction, encoder or mechanical loop.
here=fileparts(mfilename('fullpath'));root=fileparts(fileparts(here));
staging=tempname;mkdir(staging);cleanup=onCleanup(@()rmdir(staging,'s'));
copyfile(fullfile(here,'pll_mex.c'),staging);
copyfile(fullfile(root,'user_lib','drivers','foc','sensorless','bemf_pll.*'),staging);
clear pll_mex
mex('-outdir',staging,['-I' staging],fullfile(staging,'pll_mex.c'),fullfile(staging,'bemf_pll.c'));
copyfile(fullfile(staging,['pll_mex.' mexext]),here);clear cleanup
addpath(here);rng(20261006);dt=50e-6;t=(0:dt:2-dt)';R=2.55;L=.00086;w=270;
theta=2.9+w*t;ia=.03*cos(theta);ib=.03*sin(theta);
va=R*ia-L*.03*w*sin(theta)-.0035*w*sin(theta);
vb=R*ib+L*.03*w*cos(theta)+.0035*w*cos(theta);
% Larger current noise than the initial idealized tutorial scenario.
va=va+.02*randn(size(t));vb=vb+.02*randn(size(t));
ia=ia+.015*randn(size(t));ib=ib+.015*randn(size(t));
summary=table;f=figure('Visible','off','Position',[50 50 1200 800]);tiledlayout(3,1);
for wn=[6000 3000 2000]
    y=pll_mex([va vb ia ib],1,[R L wn 60 .08 dt]);idx=t>1;
    er=atan2(sin(y(:,1)-theta),cos(y(:,1)-theta));
    summary=[summary;table(wn,sqrt(mean(er(idx).^2)),std(y(idx,5)),mean(y(idx,7)), ...
        'VariableNames',{'observer_wn_rad_s','angle_rms_rad','internal_phase_std_rad','lock_fraction'})]; %#ok<AGROW>
    nexttile;plot(t,er,t,y(:,5));ylabel('rad');title(sprintf('observer wn = %d rad/s',wn));
    legend('truth-relative error','internal phase error');grid on
    assert(mean(y(idx,6))>.99 && sqrt(mean(er(idx).^2))<.08,'Noise tracking failed')
end
xlabel('time s');out=fullfile(here,'results');
writetable(summary,fullfile(out,'observer_bandwidth_study.csv'));
exportgraphics(f,fullfile(out,'observer_bandwidth_study.png'));close(f);disp(summary)
fprintf('PASS: shared C observer bandwidth/noise study\n');
end

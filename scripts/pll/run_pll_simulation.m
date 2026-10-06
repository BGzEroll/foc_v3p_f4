function run_pll_simulation
% Run from any folder. Simulation uses the SAME C source as STM32 firmware.
here=fileparts(mfilename('fullpath')); root=fileparts(fileparts(here));
out=fullfile(here,'results'); if ~exist(out,'dir'), mkdir(out); end
addpath(here); rng(20261006);
% Always rebuild: a stale MEX would silently test an older firmware algorithm.
clear pll_mex
% Some Windows MinGW linkers cannot write paths containing Chinese text.
staging=tempname; mkdir(staging); cleanup=onCleanup(@()rmdir(staging,'s'));
copyfile(fullfile(here,'pll_mex.c'),staging);
copyfile(fullfile(root,'user_lib','drivers','foc','sensorless','bemf_pll.*'),staging);
mex('-outdir',staging,['-I' staging],fullfile(staging,'pll_mex.c'),fullfile(staging,'bemf_pll.c'));
copyfile(fullfile(staging,['pll_mex.' mexext]),here); clear cleanup
dt=50e-6; t=(0:dt:2-dt)'; R=2.55; L=.00086; flux=.0035;
names=["forward_step","reverse_step","current_noise","R_plus20pct","L_plus20pct","low_speed","emf_loss"];
summary=table;
for k=1:numel(names)
    direction=1; if k==2, direction=-1; end
    w=direction*(100+80*(t>=1));
    if k==6, w(:)=10; end
    if k==7, w(t>=1)=0; end
    theta=2.9+cumsum([0;w(1:end-1)])*dt;
    % Analytic SPMSM trajectory: rotating stator current 0.15 A.
    % Derivative is known from the imposed trajectory, no shared observer model.
    ia=.15*cos(theta); ib=.15*sin(theta);
    dia=-.15*w.*sin(theta); dib=.15*w.*cos(theta);
    ea=-flux*w.*sin(theta); eb=flux*w.*cos(theta);
    va=R*ia+L*dia+ea; vb=R*ib+L*dib+eb;
    if k==3
        ia=ia+.0016*randn(size(t)); ib=ib+.0016*randn(size(t));
        va=va+.01*randn(size(t)); vb=vb+.01*randn(size(t));
    end
    Rest=R; Lest=L;
    if k==4, Rest=R*1.2; end
    if k==5, Lest=L*1.2; end
    y=pll_mex([va vb ia ib],direction,[Rest Lest 6000 200 .08 dt]);
    angle_error=atan2(sin(y(:,1)-theta),cos(y(:,1)-theta));
    settled=(t>.3 & t<.95) | (t>1.3);
    tracking=settled & y(:,6)>0;
    if any(tracking)
        angle_rms=sqrt(mean(angle_error(tracking).^2)); speed_rms=sqrt(mean((y(tracking,2)-w(tracking)).^2));
    else, angle_rms=NaN; speed_rms=NaN; end
    lock_fraction=mean(y(settled,7));
    summary=[summary;table(names(k),angle_rms,speed_rms,lock_fraction,'VariableNames',{'case_name','angle_rms_rad','speed_rms_rad_s','lock_fraction'})]; %#ok<AGROW>
    data=table(t,theta,w,va,vb,ia,ib,y(:,1),y(:,2),y(:,3),y(:,4),angle_error,y(:,6),y(:,7), ...
        'VariableNames',{'time_s','true_angle_rad','true_speed_rad_s','v_alpha_v','v_beta_v','i_alpha_a','i_beta_a','pll_angle_rad','pll_speed_rad_s','emf_alpha_v','emf_beta_v','angle_error_rad','valid','locked'});
    writetable(data,fullfile(out,names(k)+".csv"));
    f=figure('Visible','off','Position',[50 50 1200 850]); tiledlayout(4,1);
    nexttile; plot(t,mod(theta,2*pi),t,mod(y(:,1),2*pi)); ylabel('electrical rad'); legend('truth','PLL'); title(strrep(names(k),'_',' ')); grid on
    nexttile; plot(t,w,t,y(:,2)); ylabel('electrical rad/s'); legend('truth','PLL'); grid on
    nexttile; plot(t,angle_error); ylabel('error rad'); grid on
    nexttile; plot(t,hypot(y(:,3),y(:,4)),t,y(:,7)); ylabel('EMF V / lock'); xlabel('time s'); legend('|estimated EMF|','locked'); grid on
    exportgraphics(f,fullfile(out,names(k)+".png")); close(f);
    if k<=2, assert(angle_rms<.025 && speed_rms<1 && lock_fraction>.99,'Nominal tracking failed'); end
    if k==6, assert(~any(y(end-2000:end,6)),'Low speed must be invalid'); end
    if k==7, assert(~any(y(end-2000:end,7)),'EMF loss must unlock'); end
end
disp(summary); writetable(summary,fullfile(out,'summary.csv'));
save(fullfile(out,'simulation_workspace.mat'),'summary','dt','R','L','flux');
% Noise/bandwidth tradeoff on the current_noise input file.
d=readtable(fullfile(out,'current_noise.csv')); bandwidth=[60 200 500]; sweep=table;
for wn=bandwidth
    y=pll_mex([d.v_alpha_v d.v_beta_v d.i_alpha_a d.i_beta_a],1,[R L 6000 wn .08 dt]);
    idx=d.time_s>1.5; er=atan2(sin(y(idx,1)-d.true_angle_rad(idx)),cos(y(idx,1)-d.true_angle_rad(idx)));
    sweep=[sweep;table(wn,sqrt(mean(er.^2)),std(y(idx,2)-d.true_speed_rad_s(idx)), ...
        'VariableNames',{'pll_wn_rad_s','angle_rms_rad','speed_noise_std_rad_s'})]; %#ok<AGROW>
end
writetable(sweep,fullfile(out,'bandwidth_sweep.csv')); disp(sweep);
fprintf('PASS: C implementation in MATLAB; results: %s\n',out);
end

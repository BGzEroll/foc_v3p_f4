function run_direct_start_simulation
% Ideal PMSM + identical startup/observer C. Mechanical parameters are assumed,
% not measured; no PWM deadtime, ADC error or current BPF is included here.
here=fileparts(mfilename('fullpath'));root=fileparts(fileparts(here));
src=fullfile(root,'user_lib','drivers','foc','sensorless');
pid=fullfile(root,'user_lib','third-parts','SguanFOC');
stage=fullfile(tempdir,'pll_direct_sim');if ~exist(stage,'dir'),mkdir(stage);end
names={'pll_startup.c','pll_startup.h','bemf_pll.c','bemf_pll.h'};
for k=1:numel(names),copyfile(fullfile(src,names{k}),stage);end
names={'Sguan_PID.c','Sguan_PID.h','Sguan_math.c','Sguan_math.h','UserData_Calculate.h'};
for k=1:numel(names),copyfile(fullfile(pid,names{k}),stage);end
copyfile(fullfile(here,'direct_start_sim_mex.c'),stage);
old=pwd;cleanup=onCleanup(@()cd(old));cd(stage);
clear direct_start_sim_mex
mex('-silent','direct_start_sim_mex.c','pll_startup.c','bemf_pll.c','Sguan_PID.c','Sguan_math.c');
a=direct_start_sim_mex;names={'time_s','true_angle_rad','true_speed_rad_s','pll_angle_rad','pll_speed_rad_s', ...
 'control_angle_rad','open_speed_rad_s','id_a','iq_a','stage','locked','stop_reason','peak_phase_a'};
d=array2table(a,'VariableNames',names);folder=fullfile(here,'results');
writetable(d,fullfile(folder,'direct_start_simulation.csv'));
f=figure('Visible','off','Position',[40 40 1200 950]);tiledlayout(4,1);
nexttile;plot(a(:,1),a(:,[3 5 7]));legend('true','PLL','open');ylabel('electrical rad/s');grid on
nexttile;plot(a(:,1),atan2(sin(a(:,4)-a(:,2)),cos(a(:,4)-a(:,2))));ylabel('PLL truth error rad');grid on
nexttile;plot(a(:,1),a(:,[8 9]));legend('control Id','control Iq');ylabel('A');grid on
nexttile;plot(a(:,1),a(:,[10 11]));legend('startup stage','locked');xlabel('time s');grid on
exportgraphics(f,fullfile(folder,'direct_start_simulation.png'));close(f)
closed=a(:,10)==5;assert(any(closed),'No sensorless closed-loop interval');
er=atan2(sin(a(closed,4)-a(closed,2)),cos(a(closed,4)-a(closed,2)));
assert(max(abs(er))<.2,'Closed-loop truth angle error too large');
assert(max(a(:,13))<=1.8,'Phase current exceeds bound');
summary=struct('closed_s',sum(closed)*.001,'truth_angle_rms_rad',sqrt(mean(er.^2)), ...
 'truth_angle_max_rad',max(abs(er)),'phase_peak_a',max(a(:,13)), ...
 'duration_s',12,'sample_interval_s',.001,'model','Ideal mechanical model; J/friction assumed; no PWM deadtime/ADC error/current BPF');
fid=fopen(fullfile(folder,'direct_start_summary.json'),'w');
fprintf(fid,'%s\n',jsonencode(summary,PrettyPrint=true));fclose(fid);
fprintf('PASS: ideal direct startup; closed %.3f s; error RMS %.5f rad; phase peak %.3f A\n', ...
 sum(closed)*.001,sqrt(mean(er.^2)),max(a(:,13)));
end

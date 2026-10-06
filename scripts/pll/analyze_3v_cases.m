function summary=analyze_3v_cases(folder)
% Recorded board data. These are encoder-relative metrics, not an independent
% calibrated rotor truth; AS5600 latency and magnetic geometry remain relevant.
files=dir(fullfile(folder,'3v_*.csv'));summary=table;
for k=1:numel(files)
    if contains(files(k).name,'summary'),continue,end
    path=fullfile(folder,files(k).name);d=readtable(path);
    if ~ismember('encoder_angle_rad',d.Properties.VariableNames),continue,end
    er=atan2(sin(d.pll_angle_rad-d.encoder_angle_rad),cos(d.pll_angle_rad-d.encoder_angle_rad));
    ia=d.i_alpha_a;ib=d.i_beta_a;th=d.encoder_angle_rad;
    id=ia.*cos(th)+ib.*sin(th);iq=-ia.*sin(th)+ib.*cos(th);
    rpm=d.encoder_speed_rad_s/7*60/(2*pi);
    partial=height(d)~=512;
    summary=[summary;table(string(files(k).name),height(d),partial,mean(rpm), ...
        sqrt(mean(er.^2)),mean(er),std(er),max(abs(er)),mean(d.locked), ...
        mean(d.active),mean(id),mean(iq), ...
        'VariableNames',{'capture','rows','incomplete','mean_rpm','angle_rms_rad', ...
        'angle_bias_rad','angle_std_rad','angle_max_rad','lock_fraction', ...
        'active_fraction','mean_id_a','mean_iq_a'})]; %#ok<AGROW>
    t=d.time_s-d.time_s(1);[~,name]=fileparts(path);
    f=figure('Visible','off','Position',[60 60 1200 1000]);tiledlayout(4,1);
    nexttile;plot(t,rpm,t,d.pll_speed_rad_s/7*60/(2*pi));ylabel('mechanical rpm');
    legend('AS5600','BEMF PLL');title(strrep(name,'_',' '));grid on
    nexttile;plot(t,er,t,d.phase_error_rad);yline(.2,'--');yline(-.2,'--');
    ylabel('electrical rad');legend('encoder-relative error','internal phase error');grid on
    nexttile;plot(t,id,t,iq);ylabel('encoder-frame current A');legend('Id','Iq');grid on
    nexttile;plot(t,d.locked,t,d.active);ylabel('state');xlabel('time s');
    legend('locked','sensorless active');ylim([-.1 1.1]);grid on
    exportgraphics(f,fullfile(folder,[name '.png']));close(f)
end
writetable(summary,fullfile(folder,'3v_summary.csv'));disp(summary)
end

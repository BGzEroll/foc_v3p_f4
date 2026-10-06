function summary=analyze_direct_cases(folder)
% AS5600 is a diagnostic reference only; NaN after sampling is disabled.
files=dir(fullfile(folder,'direct_*.csv'));summary=table;
for k=1:numel(files)
 path=fullfile(folder,files(k).name);d=readtable(path);
 if ~ismember('encoder_angle_rad',d.Properties.VariableNames),continue,end
 t=d.time_s-d.time_s(1);er=atan2(sin(d.pll_angle_rad-d.encoder_angle_rad),cos(d.pll_angle_rad-d.encoder_angle_rad));
 emf=hypot(d.emf_alpha_v,d.emf_beta_v);
 summary=[summary;table(string(files(k).name),height(d),mean(d.pll_speed_rad_s), ...
 mean(d.encoder_speed_rad_s,'omitnan'),sqrt(mean(er.^2,'omitnan')), ...
 sqrt(mean(d.phase_error_rad.^2)),mean(d.locked),mean(d.active),mean(emf), ...
 'VariableNames',{'capture','rows','pll_mean_rad_s','encoder_mean_rad_s','encoder_error_rms_rad', ...
 'internal_phase_rms_rad','lock_fraction','active_fraction','mean_emf_v'})]; %#ok<AGROW>
 f=figure('Visible','off','Position',[40 40 1200 1000]);tiledlayout(4,1);
 nexttile;plot(t,d.pll_speed_rad_s,t,d.encoder_speed_rad_s);ylabel('electrical rad/s');legend('BEMF PLL','diagnostic AS5600');grid on;title(strrep(files(k).name,'_',' '));
 nexttile;plot(t,d.phase_error_rad,t,er);ylabel('electrical rad');legend('internal phase error','encoder-relative error');grid on
 nexttile;plot(t,d.i_alpha_a,t,d.i_beta_a,t,emf);ylabel('A / V');legend('i alpha','i beta','EMF magnitude');grid on
 nexttile;plot(t,d.locked,t,d.active);ylim([-.1 1.1]);legend('locked','PLL feedback active');xlabel('time s');grid on
 [~,name]=fileparts(path);exportgraphics(f,fullfile(folder,[name '.png']));close(f)
end
writetable(summary,fullfile(folder,'direct_summary.csv'));disp(summary)
end

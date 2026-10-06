function summary=analyze_board_capture(csv_path)
% Electrical-angle validation against AS5600. Encoder is a reference, not truth
% with infinite accuracy (12-bit quantization, I2C latency, magnetic offset).
d=readtable(csv_path); t=d.time_s-d.time_s(1);
er=atan2(sin(d.pll_angle_rad-d.encoder_angle_rad),cos(d.pll_angle_rad-d.encoder_angle_rad));
speed_error=d.pll_speed_rad_s-d.encoder_speed_rad_s;
summary=table(sqrt(mean(er.^2)),mean(er),sqrt(mean(speed_error.^2)),mean(d.locked),mean(d.active), ...
    mean(d.encoder_speed_rad_s),mean(d.pll_speed_rad_s),mean(hypot(d.emf_alpha_v,d.emf_beta_v)), ...
    'VariableNames',{'angle_rms_rad','angle_bias_rad','speed_rms_rad_s','lock_fraction','active_fraction','encoder_mean_rad_s','pll_mean_rad_s','mean_emf_v'});
disp(summary); [folder,name]=fileparts(csv_path);
writetable(summary,fullfile(folder,[name '_summary.csv']));
f=figure('Visible','off','Position',[50 50 1200 1000]); tiledlayout(5,1);
nexttile; plot(t,unwrap(d.encoder_angle_rad),t,unwrap(d.pll_angle_rad)); ylabel('electrical rad'); legend('AS5600 reference','BEMF PLL'); title(strrep(name,'_',' ')); grid on
nexttile; plot(t,d.encoder_speed_rad_s,t,d.pll_speed_rad_s); ylabel('electrical rad/s'); grid on
nexttile; plot(t,er,t,d.phase_error_rad); ylabel('rad'); legend('error vs encoder','internal phase error'); grid on
nexttile; plot(t,d.emf_alpha_v,t,d.emf_beta_v); ylabel('EMF V'); grid on
nexttile; plot(t,d.locked,t,d.active); ylabel('state'); legend('locked','sensorless active'); ylim([-.1 1.1]); xlabel('time s'); grid on
exportgraphics(f,fullfile(folder,[name '.png'])); close(f)
end

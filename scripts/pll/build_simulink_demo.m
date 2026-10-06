function build_simulink_demo
% First run run_pll_simulation; then create, run and save a viewable .slx model.
here=fileparts(mfilename('fullpath'));root=fileparts(fileparts(here));addpath(here);
clear pll_step_mex
staging=tempname;mkdir(staging);cleanup=onCleanup(@()rmdir(staging,'s'));
copyfile(fullfile(here,'pll_step_mex.c'),staging);
copyfile(fullfile(root,'user_lib','drivers','foc','sensorless','bemf_pll.*'),staging);
mex('-outdir',staging,['-I' staging],fullfile(staging,'pll_step_mex.c'),fullfile(staging,'bemf_pll.c'));
copyfile(fullfile(staging,['pll_step_mex.' mexext]),here);clear cleanup
data=readtable(fullfile(here,'results','forward_step.csv'));
assignin('base','pll_input',[data.time_s data.v_alpha_v data.v_beta_v data.i_alpha_a data.i_beta_a]);
model='sensorless_pll_learning';if bdIsLoaded(model),close_system(model,0);end
new_system(model);set_param(model,'SolverType','Fixed-step','Solver','FixedStepDiscrete','FixedStep','50e-6','StopTime','1.99995');
set_param(model,'InitFcn','pll_prepare_demo_input');
add_block('simulink/Sources/From Workspace',[model '/SPMSM_voltage_current'],'VariableName','pll_input','Interpolate','off','OutputAfterFinalValue','Holding final value','Position',[40 80 220 140]);
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function',[model '/Observer_and_PLL_C'],'FunctionName','pll_sfun','Parameters','1','Position',[310 80 510 140]);
add_block('simulink/Signal Routing/Demux',[model '/Eight_outputs'],'Outputs','8','Position',[580 35 585 255]);
add_block('simulink/Sinks/Scope',[model '/Angle_and_speed'],'NumInputPorts','2','Position',[700 20 830 100]);
add_block('simulink/Sinks/Scope',[model '/Lock_state'],'Position',[700 180 830 235]);
add_block('simulink/Sinks/To Workspace',[model '/Save_all_outputs'],'VariableName','pll_simulink_output','SaveFormat','Array','Position',[570 290 820 335]);
add_line(model,'SPMSM_voltage_current/1','Observer_and_PLL_C/1');
add_line(model,'Observer_and_PLL_C/1','Eight_outputs/1');
add_line(model,'Observer_and_PLL_C/1','Save_all_outputs/1');
add_line(model,'Eight_outputs/1','Angle_and_speed/1');add_line(model,'Eight_outputs/2','Angle_and_speed/2');
add_line(model,'Eight_outputs/7','Lock_state/1');
note=Simulink.Annotation(model,sprintf('SPMSM electrical test trajectory -> firmware C observer -> PLL\nInput: [v_alpha, v_beta, i_alpha, i_beta], SI units, 20 kHz\nOutputs: angle, speed, e_alpha, e_beta, phase error, valid, locked, invalid count\nThis block adds one sample delay; it does not model inverter/mechanical load/startup.'));
note.Position=[35 370];
save_system(model,fullfile(here,[model '.slx']));
result=sim(model);y=result.pll_simulink_output;
assert(abs(y(end,2)-180)<1 && y(end,7)==1,'Simulink steady-state validation failed');
save(fullfile(here,'results','simulink_results.mat'),'y');
print(['-s' model],'-dpng',fullfile(here,'results','simulink_diagram.png'));
close_system(model,0);fprintf('PASS: Simulink C block demo generated and simulated.\n');
end

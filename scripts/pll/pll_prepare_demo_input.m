% Model initialization callback. Add this scripts/pll folder to MATLAB path.
pll_demo_folder=fileparts(mfilename('fullpath'));
pll_demo_data=readtable(fullfile(pll_demo_folder,'results','forward_step.csv'));
pll_input=[pll_demo_data.time_s pll_demo_data.v_alpha_v pll_demo_data.v_beta_v pll_demo_data.i_alpha_a pll_demo_data.i_beta_a];
clear pll_demo_data pll_demo_folder

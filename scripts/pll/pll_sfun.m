function pll_sfun(block)
% Level-2 MATLAB S-function wrapping the firmware C source through MEX.
% Dwork output makes scheduling explicit: output has ONE sample of delay.
block.NumDialogPrms=1;
block.NumInputPorts=1;block.NumOutputPorts=1;
block.SetPreCompPortInfoToDefaults;
block.InputPort(1).Dimensions=4; block.InputPort(1).DirectFeedthrough=false;
block.OutputPort(1).Dimensions=8;
block.SampleTimes=[50e-6 0];
block.SimStateCompliance='DefaultSimState';
block.RegBlockMethod('PostPropagationSetup',@setup);
block.RegBlockMethod('InitializeConditions',@init);
block.RegBlockMethod('Outputs',@outputs);
block.RegBlockMethod('Update',@update);
end
function setup(b)
b.NumDworks=1; b.Dwork(1).Name='previous_output'; b.Dwork(1).Dimensions=8;
b.Dwork(1).DatatypeID=0; b.Dwork(1).Complexity='Real'; b.Dwork(1).UsedAsDiscState=true;
end
function init(b)
pll_step_mex('reset',b.DialogPrm(1).Data); b.Dwork(1).Data=zeros(8,1);
end
function outputs(b)
b.OutputPort(1).Data=b.Dwork(1).Data;
end
function update(b)
b.Dwork(1).Data=pll_step_mex(b.InputPort(1).Data);
end

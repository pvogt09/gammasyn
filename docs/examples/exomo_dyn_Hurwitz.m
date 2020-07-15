
sys = omo_sys(1, 10, 1000);

controller = control.design.outputfeedback.DynamicOutputFeedback(2);

sys_augmented = controller.amend(sys);

[Ra_fixed, Rap_fixed, ~, ~, ~, ~, ~, ~, R_nonlin] = controller.gainpattern(sys);

Rfixed = {Ra_fixed, Rap_fixed};

%%
polearea = [
	control.design.gamma.area.Line(1, 0),...
	control.design.gamma.area.Imag(1, 1)
];

weight = 1;
R0 = ones(3, 3);

gammaopts = control.design.gamma.GammasynOptions();

optimizer = optimization.solver.Optimizer.IPOPT;

[R_opt, J_opt, info] = control.design.gamma.gammasyn(...
	sys_augmented,...
	polearea, weight,...
	Rfixed, R0, optimizer, gammaopts, [], R_nonlin...
);
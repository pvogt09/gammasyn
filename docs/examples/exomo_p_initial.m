
sys = omo_sys(1, 10, 1000);

polearea = [
	control.design.gamma.area.Line(1, 0),...
	control.design.gamma.area.Imag(1, 1)
];

weight = 1;
R0 = 1;
R0 = control.design.gamma.InitialValue(...
	control.design.gamma.RandomInitialValue(1),...
	cat(3, R0, 2*R0, 2000*R0)...
);
Rfixed = [];

gammaopts = control.design.gamma.GammasynOptions(...
	'type', GammaJType.NORMGAIN,...
	'objective.normgain.R', 1 ...
);

optimizer = optimization.solver.Optimizer.IPOPT;

[R_opt, J_opt, info] = control.design.gamma.gammasyn(...
	sys,...
	polearea, weight,...
	Rfixed, R0, optimizer, gammaopts...
);
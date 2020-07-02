
sys.A = -1;
sys.B = 1;
sys.C = 1;

polearea = @(re, im) mindistance_area(re, im, 3);


weight = 1;
R0 = 1;
Rfixed = [];

gammaopts = control.design.gamma.GammasynOptions();

optimizer = optimization.solver.Optimizer.IPOPT;

[R_opt, J_opt, info] = control.design.gamma.gammasyn(...
	sys,...
	polearea, weight,...
	Rfixed, R0, optimizer, gammaopts...
);


sys = omo_sys(1, 10, 1000);

controller = control.design.outputfeedback.PIDOutputFeedback();

sys_augmented = controller.amend(sys);

[~, Rap_fixed] = controller.gainpattern(sys);
R = sym('R', [2, 2]);
K_I = sym('K_I');
K_P = sym('K_P');
R(1, 1) = K_P;
R(1, 2) = K_I;
Ra_fixed = {R, [
	K_I == K_P;
	R(2, 1) == 1;
	R(2, 2) == 0
]};

Rfixed = {Ra_fixed, Rap_fixed};

%%
polearea = [
	control.design.gamma.area.Line(1, 0),...
	control.design.gamma.area.Imag(1, 1)
];

weight = 1;
%R0 = ones(2, 2);% {ones(2, 2), ones(2, 2)};
R0 = {ones(2, 2), [-1, 0; 0, 0]};

gammaopts = control.design.gamma.GammasynOptions();

optimizer = optimization.solver.Optimizer.IPOPT;

[R_opt, J_opt, info] = control.design.gamma.gammasyn(...
	sys_augmented,...
	polearea, weight,...
	Rfixed, R0, optimizer, gammaopts...
);

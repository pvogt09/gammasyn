%% configure system
% a = ureal('a', 15, 'Range', 0.000001*[10, 20])
% b = ureal('b', 5, 'Range', 0.00001*[-1, 10])
% d = ureal('A', 4, 'Percentage', 0.000001)
% k = realp('k', 2);
% k.Minimum = -1
% c = realp('k_fix', -5);
% c.Free = false;
% c.Minimum = -10;
% c.Maximum = 20;
% %k = parallel(k, pidstd(1, 2, 3, 4));
% %k = parallel(k, ltiblock.gain('K_gain', 1, 1));
% %k = parallel(k, ltiblock.ss('K', 2, 1, 1));
% k = parallel(k, ltiblock.ss('D', 3, 1, 1));
% %k = parallel(ltiblock.tf('K_tf', 1, 2), k);
% k = series(k, [
% 	realp('k1', 2), realp('k2', 2)
% ]*[
% 	realp('k3', 2);
% 	realp('k4', 2)
% ]);
% sys = [
% 	tf(1, [1, 1 + a + b + d]);
% 	tf(9, [1, 50 + c])
% ];
% system = feedback(sys, k, 1, 1)

m_K = 1000;
m_G = ureal('m_G', 50 + 1E-9, 'Range', [50, 4000]);
l = ureal('l', 10, 'Range', [4, 12]);
bridge = ss([
	0,	1,	0,							0;
	0,	0,	m_G/m_K*9.81,				0;
	0,	0,	0,							1;
	0,	0,	-(m_K + m_G)*9.81/m_K/l,	0
], [
	0;
	1/m_K;
	0;
	-1/m_K/l
], repmat([
	1,	0,	0,	0;
	0,	1,	0,	0;
	0,	0,	1,	0
], 2, 1), zeros(6, 1));

R_0 = [
	100,	10000,	-10000,	1;
	100,	100,	100,	1
];
R_controller = [
	realp('r_11', R_0(1, 1)),	realp('r_12', R_0(1, 2)),	realp('r_13', R_0(1, 3)),	realp('r_14', R_0(1, 4));
	realp('r_21', R_0(2, 1)),	realp('r_22', R_0(2, 2)),	realp('r_23', R_0(2, 3)),	realp('r_24', R_0(2, 4))
];
controller = ss(-R_controller(2, 4), [
	-R_controller(2, 1:3),	R_controller(2, 1:3)
], -R_controller(1, 4), [
	-R_controller(1, 1:3),	R_controller(1, 1:3)
]);
controller.InputGroup = struct('w', 4:6, 'y', 1:3);

system = lft(bridge, controller, 1, 3);

%% set initial values
a = 0.65;
b = 0.5;
R = 50;

%% 
weight = {[1, 1, 100], 1};
% polearea can either be a function handle with input arguments real and imaginary part of an eigenvalue, that outputs a row vector with as much elements as pole areas are defined
%polearea = @(re, im) flexargout(...
%	[sqrt(re^2 + im^2) - R, re + a/b*sqrt(im^2 + b^2)],...
%	[iftern(re == 0 && im == 0, 0, re/sqrt(re^2 + im^2)), 1],...
%	[iftern(re== 0 && im == 0, 0, im/sqrt(re^2 + im^2)), a/b*im/sqrt(im^2 + b^2)]...
%);
% polearea = @(re, im) flexargout(...
% 	[re^2 + im^2 - R^2, re + a/b*sqrt(im^2 + b^2)],...
% 	[2*re, 1],...
% 	[2*im, a/b*im/sqrt(im^2 + b^2)]...
% );
% or if predefined pole areas and therefore using compiled functions are used, a matrix of control.design.gamma.area.GammaArea objects
% if polearea is a cell, the first element will be used as constrained pole area and the second as objective pole area, if constrained optimization is used
polearea = {[
	control.design.gamma.area.Circlesquare(R),	control.design.gamma.area.Hyperbolasquare(a, b), control.design.gamma.area.Imag(1, a)
], [
	control.design.gamma.area.Circlesquare(R/2, -R/2, 0), control.design.gamma.area.Imag(1, a)
]};
polearea = polearea{1};
weight = weight{1};
solver = optimization.solver.Optimizer.FMINCON;% solver to use
options = optimization.options.OptionFactory.instance.options(solver,...
	'ProblemType',					optimization.options.ProblemType.CONSTRAINED,...% can be CONSTRAINED for constrained optimization and UNCONSTRAINED for unconstrained optimization
	'Retries',						1,...% number of retries
	'Algorithm',					solver.getDefaultAlgorithm(),...% algorithm of solver, for not builtin solvers name of the solver, e.g. 'snopt' for SNOPT
	'FunctionTolerance',			1E-8,...
	'StepTolerance',				1E-8,...
	'ConstraintTolerance',			1E-5,...
	'MaxFunctionEvaluations',		5E3,...
	'MaxIterations',				5E3,...
	'MaxSQPIter',					5E3,...
	'SpecifyObjectiveGradient',		true,...
	'SpecifyConstraintGradient',	true,...
	'CheckGradients',				false,...
	'FunValCheck',					false,...
	'FiniteDifferenceType',			'forward',...
	'Diagnostics',					false,...
	'Display',						'iter-detailed'...
);
[system_cl_opt, J_opt, info] = control.design.gamma.gammasyn_looptune(system, polearea, weight, options, struct(...
	'usecompiled',		true,...% indicator, if compiled functions should be used
	'type',				GammaJType.LINEAR,...% type of pole area weighting in objective function
	'allowvarorder',	false,...%allow variable state number for different multi models
	'system',			struct(...
		'samples',		struct(...% specify number of multiple models generated from tunable or uncertain blocks corresponding to fieldnames
			'm_G',	2,...
			'l',	(1)...
		)...
	)...
))
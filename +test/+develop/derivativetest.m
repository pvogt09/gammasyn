%K_0 = reshape(1:8, 2, 4);
%x = cat(3, K_0, 10 + K_0, 100 + K_0, 1000 + K_0);
%y = cat(4, x, 10000 + x);
%y = cat(5, y, y + 100000);
%z = permute(y, [3, 4, 5, 1, 2]);
%b = reshape(z, [], 2, 4);
%return;

derivativetype = 'R';
usemeasurements_xdot = true;
usereferences = true;







%% configure system
m_k = 1000;
m_g = 50;
l = 10;
bridge1 = ss([
	0,	1,	0,									0;
	0,	0,	m_g*9.81/m_k,				0;
	0,	0,	0,									1;
	0,	0,	-(m_g + m_k)*9.81/m_k/l,	0
], [
	0;
	1/m_k;
	0;
	-1/m_k/l
], [
	1,	0,	0,	0;
	0,	1,	0,	0;
	0,	0,	1,	0
], zeros(3, 1));
m_g = 4000;
bridge2 = ss([
	0,	1,	0,									0;
	0,	0,	m_g*9.81/m_k,				0;
	0,	0,	0,									1;
	0,	0,	-(m_g + m_k)*9.81/m_k/l,	0
], [
	0;
	1/m_k;
	0;
	-1/m_k/l
], [
	1,	0,	0,	0;
	0,	1,	0,	0;
	0,	0,	1,	0
], zeros(3, 1));
m_g = 2000;
bridge3 = ss([
	0,	1,	0,									0;
	0,	0,	m_g*9.81/m_k,				0;
	0,	0,	0,									1;
	0,	0,	-(m_g + m_k)*9.81/m_k/l,	0
], [
	0;
	1/m_k;
	0;
	-1/m_k/l
], [
	1,	0,	0,	0;
	0,	1,	0,	0;
	0,	0,	1,	0
], zeros(3, 1));
% sys1 = ss([0, 1;0,1], kron([1, 1], [0;1]), [1, 0], 0);
% sys2 = ss([-2, -1;4,-10], kron([1, 1], [0;1]), [1, 0], 0);
% sys3 = ss([-2, -1;4,-10], kron([1, 1], [0;1]), [1, 0], 0);
sys1 = ss([
	bridge1.a,						zeros(size(bridge1.a, 1), 1);
	zeros(1, size(bridge1.a, 2)),	0
], [
	bridge1.b,						zeros(size(bridge1.b, 1), 1);
	zeros(1, size(bridge1.b, 2)),	1
], [
	bridge1.c,						zeros(size(bridge1.c, 1), 1);
	zeros(1, size(bridge1.c, 2)),	1
], zeros(size(bridge1.d, 1) + 1, size(bridge1.d, 2) + 1));
sys2 = ss([
	bridge2.a,						zeros(size(bridge2.a, 1), 1);
	zeros(1, size(bridge2.a, 2)),	0
], [
	bridge2.b,						zeros(size(bridge2.b, 1), 1);
	zeros(1, size(bridge2.b, 2)),	1
], [
	bridge2.c,						zeros(size(bridge2.c, 1), 1);
	zeros(1, size(bridge2.c, 2)),	1
], zeros(size(bridge2.d, 1) + 1, size(bridge2.d, 2) + 1));
sys3 = ss([
	bridge3.a,						zeros(size(bridge3.a, 1), 1);
	zeros(1, size(bridge3.a, 2)),	0
], [
	bridge3.b,						zeros(size(bridge3.b, 1), 1);
	zeros(1, size(bridge3.b, 2)),	1
], [
	bridge3.c,						zeros(size(bridge3.c, 1), 1);
	zeros(1, size(bridge3.c, 2)),	1
], zeros(size(bridge3.d, 1) + 1, size(bridge3.d, 2) + 1));
sys4 = example.bridge(8, m_k, 50);
sys5 = example.bridge(12, m_k, 50);
sys6 = example.bridge(8, m_k, 4000);
sys7 = example.bridge(12, m_k, 4000);
systems = {
	sys1;
	sys2;
	sys3
	%ss(sys4.A, sys4.B, sys4.C, sys4.D);
	%ss(sys5.A, sys5.B, sys5.C, sys5.D);
	%ss(sys6.A, sys6.B, sys6.C, sys6.D);
	%ss(sys7.A, sys7.B, sys7.C, sys7.D)
};
mass = 50:10:4000;
len = 8:0.5:12;
testsystems = struct('A', {}, 'B', {}, 'C', {}, 'C_dot', {}, 'D', {}, 'C_ref', {}, 'D_ref', {});
for ii = 1:size(mass, 2)
	for jj = 1:size(len, 2)
		temp = example.bridge(len(jj), m_k, mass(ii));
		temp.C_dot = temp.C(1:3, :);
		temp.C_ref = temp.C(1:3, :);
		temp.D_ref = temp.D(1:3, :);
		testsystems(ii, jj) = temp;
	end
end
testsystems = reshape(testsystems, [], 1);

%% set initial values
R = 50;
a = 0.15;%[0.01, 0.65];
b = 0.5;%[0.1, 0.5];
weight = 5;%[0.01, 5];
iter = 2;
N = 5;
R_0 = [
	100,	10000,	-10000,	1;
	100,	100,	100,	1
];

%% transform parameters
system = struct(...
	'A',		{},...
	'B',		{},...
	'C',		{},...
	'C_dot',	{},...
	'D',		{},...
	'C_ref',	{},...
	'D_ref',	{}...
);
for jj = 1:size(systems, 1)
	%systems{jj} = c2d(systems{jj}, 0.005);
	[systemsA, systemsB, systemsC, systemsD] = ssdata(systems{jj});
	system(jj) = struct(...
		'A',		systemsA,...
		'B',		systemsB,...
		'C',		systemsC,...
		'C_dot',	systemsC(1:3, :),...
		'D',		systemsD,...
		'C_ref',	systemsC(1:3, :),...
		'D_ref',	systemsD(1:3, :)...
	);
end
if ~usemeasurements_xdot
	system = rmfield(system, 'C_dot');
end
if ~usereferences
	system = rmfield(system, 'C_ref');
	system = rmfield(system, 'D_ref');
end
a = 0.65;
b = 0.5;
R = 50;
if ~isscalar(a) || ~isscalar(b) || ~isscalar(weight)
	if ~isscalar(a)
		a_range = linspace(a(1), a(2), N);
	else
		a_range = a;
	end
	if ~isscalar(b)
		b_range = linspace(b(1), b(2), N);
	else
		b_range = b;
	end
	if ~isscalar(weight)
		weight_range = linspace(weight(1), weight(2), N);
	else
		weight_range = weight;
	end
else
	a_range = a;
	b_range = b;
	weight_range = weight;
end





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
% J = @(x) sum(exp([1, 1].*polearea(x(1), x(2))));
% J = @(x) iftern(x(2) < 0, x(1) - exp(0.5*atan2(x(2), -x(1)))*cos(atan2(x(2), -x(1))), x(1) - exp(0.5*atan2(x(2), x(1)))*cos(atan2(x(2), x(1))));
% J = @(x) abs(x(1)) - sign(x(1))*exp(0.5*atan2(-abs(x(2)), x(1)))*cos(atan2(-abs(x(2)), x(1)));
% J = @(x) x(1)^2 + x(2)^2 - exp(2*0.5*atan2(-abs(x(2)), x(1)));
%return;
%%
R_0 = [
	100,	10000,	-10000,	1;
	100,	100,	100,	1
];
if usemeasurements_xdot
	K_0 = 100*[
		1,	2,	4;
		-1,	1,	-3
	];
else
	K_0 = zeros(2, 0);
end
if usereferences
	F_0 = 100*[
		1,	5,	-70;
		-1,	1,	10
	];
else
	F_0 = zeros(2, 0);
end
%K_0 = (sys1.c*lqr(sys1, eye(size(sys1.a)), eye(size(sys1.b, 2)))')';

weight = [
	5
];
polearea = repmat([
	control.design.gamma.area.Circlesquare(R),	control.design.gamma.area.Hyperbolasquare(a, b), control.design.gamma.area.Imag(1, a)
], size(systems, 1), 1);
%%
R_fixed = [];
solver = optimization.solver.Optimizer.FMINCON;
options = optimization.options.OptionFactory.instance.options(solver,...
	'ProblemType',					optimization.options.ProblemType.CONSTRAINED,...
	'Retries',						1,...
	'Algorithm',					solver.defaultalgorithm,...
	'FunctionTolerance',			1E-10,...
	'StepTolerance',				1E-10,...
	'ConstraintTolerance',			1E-7,...
	'MaxFunctionEvaluations',		25E3,...
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
%[R_opt, J_opt, info] = control.design.gamma.gammasyn(system, polearea, weight, R_fixed, R_0, options, struct('usecompiled', true, 'type', %[GammaJType.EIGENVALUECONDITION;GammaJType.LINEAR], 'allowvarorder', false))
%return;
%% objective function check
if true
calculate_hessian = false;
indices_R = [
	1,	1,	1000;
	1,	2,	10000;
	1,	3,	-10000;
	1,	4,	1000;
	2,	1,	1000;
	2,	2,	1000;
	2,	3,	1000;
	2,	4,	1000
];
indices_K = [
	1,	1,	1000;
	1,	2,	10000;
	1,	3,	-10000;
	2,	1,	1000;
	2,	2,	1000;
	2,	3,	1000
];
indices_F = [
	1,	1,	1000;
	1,	2,	10000;
	1,	3,	-10000;
	2,	1,	1000;
	2,	2,	1000;
	2,	3,	1000
];
switch derivativetype
	case 'R'
		indices = indices_R;
		gainval = R_0;
	case 'K'
		indices = indices_K;
		gainval = K_0;
		if ~usemeasurements_xdot
			error('control:design:gamma:test', 'Requested to show derivative gain but no derivative measurement available.');
		end
	case 'F'
		indices = indices_F;
		gainval = F_0;
		if ~usereferences
			error('control:design:gamma:test', 'Requested to show prefilter gain but no prefilter available.');
		end
	otherwise
		indices = indices_R;
		gainval = R_0;
end
objectivefigure = figure;
gradientfigure = figure;
if calculate_hessian
	hessianfigure = figure;
end
opt = control.design.gamma.GammasynOptions.PROTOTYPE;
opt = rmfield(opt, 'decouplingcontrol');
opt.type = [GammaJType.LYAPUNOV];
opt.usecompiled = false;
opt.weight = 1;
opt.eigenvaluederivative = GammaEigenvalueDerivativeType.VANDERAA;
opt.objective.normgain.R = magic(5)/100000;
opt.objective.normgain.K = magic(5)/100000;
opt.objective.normgain.F = magic(5)/100000;
opt.objective.normgain.R_shift = 100*ones(5);
if isfield(system, 'C_dot')
	opt.objective.normgain.K = opt.objective.normgain.K(1:2, 1:size(system(1).C_dot, 1));
else
	opt.objective.normgain.K = opt.objective.normgain.K(1:2, []);
end
if isfield(system, 'C_ref')
	opt.objective.normgain.F = opt.objective.normgain.K(1:2, 1:size(system(1).C_ref, 1));
else
	opt.objective.normgain.F = opt.objective.normgain.K(1:2, []);
end
opt.objective.normgain.R = opt.objective.normgain.R(1:2, 1:size(system(1).C, 1));
opt.objective.normgain.R_shift = opt.objective.normgain.R_shift(1:2, 1:size(system(1).C, 1));
opt.objective.lyapunov.Q = eye(5);
weight = 1;
%[system(:).T] = deal(0.005);
%[system(:).E] = deal(diag([1, 4, 2, 1, 1]));
for jj = 1:size(indices, 1)
	index = indices(jj, 1:2);
	range = linspace(-abs(indices(jj, 3)), abs(indices(jj, 3)), 100);
	switch derivativetype
		case 'R'
			R = zeros(size(sys1.b, 2), size(sys1.c, 1), size(range, 2));
			for ii = 1:size(range, 2)
				R(:, :, ii) = R_0;
				R(index(1), index(2), ii) = range(ii);
			end
			x_0 = {R, K_0, F_0};
		case 'K'
			K = zeros(size(sys1.b, 2), size(system(1).C_dot, 1), size(range, 2));
			for ii = 1:size(range, 2)
				K(:, :, ii) = K_0;
				K(index(1), index(2), ii) = range(ii);
			end
			x_0 = {R_0, K, F_0};
		case 'F'
			F = zeros(size(sys1.b, 2), size(system(1).C_ref, 1), size(range, 2));
			for ii = 1:size(range, 2)
				F(:, :, ii) = F_0;
				F(index(1), index(2), ii) = range(ii);
			end
			x_0 = {R_0, K_0, F};
		otherwise
			R = zeros(size(sys1.b, 2), size(sys1.c, 1), size(range, 2));
			for ii = 1:size(range, 2)
				R(:, :, ii) = R_0;
				R(index(1), index(2), ii) = range(ii);
			end
			x_0 = {R, K_0, F_0};
	end
	if calculate_hessian
		[J, gradJ, hessJ] = control.design.gamma.Jeval(x_0, system(1), polearea(:, 1), weight, opt, derivativetype);
	else
		[J, gradJ] = control.design.gamma.Jeval(x_0, system(1), polearea(:, 1), weight, opt, derivativetype);
	end
	figure(objectivefigure);
	subplot(max(indices(:, 1)), max(indices(:, 2)), jj);
	plot(range, J);
	xlabel(sprintf('$%s(%d, %d)$', lower(derivativetype), index(1), index(2)));
	ylabel('$J$');
	grid on;
	figure(gradientfigure);
	subplot(max(indices(:, 1)), max(indices(:, 2)), jj);
	plot(range, squeeze(gradJ(index(1), index(2), :)), '-', range(1:end - 1), diff(J)./diff(range), ':');
	grid on;
	xlabel(sprintf('$%s(%d, %d)$', lower(derivativetype), index(1), index(2)));
	ylabel('$\nabla J$');
	legend('analytic', 'numeric');
	if calculate_hessian
		figure(hessianfigure);
		hessianidx = sub2ind(size(gainval), index(1), index(2));
		subplot(max(indices(:, 1)), max(indices(:, 2)), jj);
		plot(range, squeeze(hessJ(hessianidx, hessianidx, :)), '-', range(1:end - 2), diff(diff(J)./diff(range))./diff(range(1:end - 1)), ':', range(1:end - 1), diff(squeeze(gradJ(index(1), index(2), :))')./diff(range), '.-');
		grid on;
		xlabel(sprintf('$%s(%d, %d)$', lower(derivativetype), index(1), index(2)));
		ylabel('$\nabla^2 J$');
		legend('analytic', 'numeric', 'numeric gradient');
	end
end
end
%return;
%% constraint function check
if true
calculate_hessian = true;
indices_R = [
	1,	1,	1000;
	1,	2,	10000;
	1,	3,	-10000;
	1,	4,	1000;
	2,	1,	1000;
	2,	2,	1000;
	2,	3,	1000;
	2,	4,	1000
];
indices_K = [
	1,	1,	1000;
	1,	2,	10000;
	1,	3,	-10000;
	2,	1,	1000;
	2,	2,	1000;
	2,	3,	1000
];
indices_F = [
	1,	1,	1000;
	1,	2,	10000;
	1,	3,	-10000;
	2,	1,	1000;
	2,	2,	1000;
	2,	3,	1000
];
switch derivativetype
	case 'R'
		indices = indices_R;
		gainval = R_0;
	case 'K'
		indices = indices_K;
		gainval = K_0;
		if ~usemeasurements_xdot
			error('control:design:gamma:test', 'Requested to show derivative gain but no derivative measurement available.');
		end
	case 'F'
		indices = indices_F;
		gainval = F_0;
		if ~usereferences
			error('control:design:gamma:test', 'Requested to show prefilter gain but no prefilter available.');
		end
	otherwise
		indices = indices_R;
		gainval = R_0;
end
cindex = 4;
objectivefigure = figure;
gradientfigure = figure;
if calculate_hessian
	hessianfigure = figure;
end
opt = control.design.gamma.GammasynOptions.PROTOTYPE;
opt = rmfield(opt, 'decouplingcontrol');
opt.type = GammaJType.LINEAR;
opt.usecompiled = true;
opt.weight = 1;
opt.eigenvaluederivative = GammaEigenvalueDerivativeType.VANDERAA;
weight = [1, 2];
for jj = 1:size(indices, 1)
	index = indices(jj, 1:2);
	range = linspace(-abs(indices(jj, 3)), abs(indices(jj, 3)), 100);
	switch derivativetype
		case 'R'
			R = zeros(size(sys1.b, 2), size(sys1.c, 1), size(range, 2));
			for ii = 1:size(range, 2)
				R(:, :, ii) = R_0;
				R(index(1), index(2), ii) = range(ii);
			end
			x_0 = {R, K_0, F_0};
		case 'K'
			K = zeros(size(sys1.b, 2), size(system(1).C_dot, 1), size(range, 2));
			for ii = 1:size(range, 2)
				K(:, :, ii) = K_0;
				K(index(1), index(2), ii) = range(ii);
			end
			x_0 = {R_0, K, F_0};
		case 'F'
			F = zeros(size(sys1.b, 2), size(system(1).C_ref, 1), size(range, 2));
			for ii = 1:size(range, 2)
				F(:, :, ii) = F_0;
				F(index(1), index(2), ii) = range(ii);
			end
			x_0 = {R_0, K_0, F};
		otherwise
			R = zeros(size(sys1.b, 2), size(sys1.c, 1), size(range, 2));
			for ii = 1:size(range, 2)
				R(:, :, ii) = R_0;
				R(index(1), index(2), ii) = range(ii);
			end
			x_0 = {R, K_0, F_0};
	end
	if calculate_hessian
		[c, ceq, gradc, gradceq, hessc, hessceq] = control.design.gamma.ceval(x_0, system(1), polearea(:, 1:2), weight, opt, derivativetype);
	else
		[c, ceq, gradc, gradceq] = control.design.gamma.ceval(x_0, system(1), polearea(:, 1:2), weight, opt, derivativetype);
	end
	figure(objectivefigure);
	subplot(max(indices(:, 1)), max(indices(:, 2)), jj);
	plot(range, c(cindex, :));
	xlabel(sprintf('$%s(%d, %d)$', lower(derivativetype), index(1), index(2)));
	ylabel(['$c_', num2str(cindex), '$']);
	grid on;
	figure(gradientfigure);
	subplot(max(indices(:, 1)), max(indices(:, 2)), jj);
	plot(range, squeeze(gradc(cindex, index(1), index(2), :)), '-', range(1:end - 1), diff(c(cindex, :))./diff(range), ':');
	grid on;
	xlabel(sprintf('$%s(%d, %d)$', lower(derivativetype), index(1), index(2)));
	ylabel(['$\nabla c_', num2str(cindex), '$']);
	legend('analytic', 'numeric');
	if calculate_hessian
		figure(hessianfigure);
		hessianidx = sub2ind(size(gainval), index(1), index(2));
		subplot(max(indices(:, 1)), max(indices(:, 2)), jj);
		plot(range, squeeze(hessc(hessianidx, hessianidx, cindex, :)), '-', range(1:end - 2), diff(diff(c(cindex, :))./diff(range))./diff(range(1:end - 1)), ':', range(1:end - 1), diff(squeeze(gradc(cindex, index(1), index(2), :))')./diff(range), '.-');
		grid on;
		xlabel(sprintf('$%s(%d, %d)$', lower(derivativetype), index(1), index(2)));
		ylabel(['$\nabla^2 c_', num2str(cindex), '$']);
		legend('analytic', 'numeric', 'numeric gradient');
	end
end
end
%% equality constraint function check
[system_coupling, loaded_system_information] = example.load_system_coupling('threetank_exact');
R_0 = loaded_system_information.RKF_0{1};
if true
calculate_hessian = false;
indices = [
	1,	1,	1000;
	1,	2,	10000;
	1,	3,	-10000;
	2,	1,	1000;
	2,	2,	1000;
	2,	3,	1000
];
cindex = 4;
objectivefigure = figure;
gradientfigure = figure;
if calculate_hessian
	hessianfigure = figure;
end
opt = control.design.gamma.objectiveoptions_prototype();
opt.type = GammaJType.LINEAR;
opt.usecompiled = false;
opt.weight = 1;
opt.eigenvaluederivative = GammaEigenvalueDerivativeType.VANDERAA;
opt.couplingcontrol.couplingstrategy = GammaCouplingStrategy.NUMERIC_NONLINEAR_EQUALITY;
opt.couplingcontrol.couplingconditions = loaded_system_information.number_couplingconditions;
weight = [1, 2];
for jj = 1:size(indices, 1)
	r_index = indices(jj, 1:2);
	r_range = linspace(-abs(indices(jj, 3)), abs(indices(jj, 3)), 100);
	R = zeros(size(system_coupling(1).B, 1) - loaded_system_information.number_couplingconditions, size(system_coupling(1).A, 2), size(r_range, 2));
	for ii = 1:size(r_range, 2)
		R(:, :, ii) = R_0(1:size(R, 1), 1:size(R, 2));
		R(r_index(1), r_index(2), ii) = r_range(ii);
	end
	if calculate_hessian
		[c, ceq, gradc, gradceq, hessc, hessceq] = control.design.gamma.ceval(R, system_coupling(1), polearea(:, 1:2), weight, opt, 'R');
	else
		[c, ceq, gradc, gradceq] = control.design.gamma.ceval(R, system_coupling(1), polearea(:, 1:2), weight, opt, 'R');
	end
	figure(objectivefigure);
	subplot(max(indices(:, 1)), max(indices(:, 2)), jj);
	plot(r_range, ceq(cindex, :));
	xlabel(sprintf('$r(%d, %d)$', r_index(1), r_index(2)));
	ylabel(['$c^=_', num2str(cindex), '$']);
	grid on;
	figure(gradientfigure);
	subplot(max(indices(:, 1)), max(indices(:, 2)), jj);
	plot(r_range, squeeze(gradceq(cindex, r_index(1), r_index(2), :)), '-', r_range(1:end - 1), diff(ceq(cindex, :))./diff(r_range), ':');
	grid on;
	xlabel(sprintf('$r(%d, %d)$', r_index(1), r_index(2)));
	ylabel(['$\nabla c^=_', num2str(cindex), '$']);
	legend('analytic', 'numeric');
	if calculate_hessian
		figure(hessianfigure);
		hessianidx = sub2ind([size(R_0, 1), size(R_0, 2)], r_index(1), r_index(2));
		subplot(max(indices(:, 1)), max(indices(:, 2)), jj);
		plot(r_range, squeeze(hessceq(hessianidx, hessianidx, cindex, :)), '-', r_range(1:end - 2), diff(diff(ceq(cindex, :))./diff(r_range))./diff(r_range(1:end - 1)), ':', r_range(1:end - 1), diff(squeeze(gradceq(cindex, r_index(1), r_index(2), :))')./diff(r_range), '.-');
		grid on;
		xlabel(sprintf('$r(%d, %d)$', r_index(1), r_index(2)));
		ylabel(['$\nabla^2 c^=_', num2str(cindex), '$']);
		legend('analytic', 'numeric', 'numeric gradient');
	end
end
end
%% configure system
m_k = 1000;
m_g = 50;
l = 10;
bridge1 = ss([
	0,	1,	0,							0;
	0,	0,	m_g*9.81/m_k,				0;
	0,	0,	0,							1;
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
	0,	1,	0,							0;
	0,	0,	m_g*9.81/m_k,				0;
	0,	0,	0,							1;
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
	0,	1,	0,							0;
	0,	0,	m_g*9.81/m_k,				0;
	0,	0,	0,							1;
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
testsystems = struct('A', {}, 'B', {}, 'C', {}, 'D', {});
for ii = 1:size(mass, 2)
	for jj = 1:size(len, 2)
		testsystems(ii, jj) = example.bridge(len(jj), m_k, mass(ii));
	end
end
testsystems = reshape(testsystems, [], 1);
%% transform parameters
system = struct(...
	'A',	{},...
	'B',	{},...
	'C',	{},...
	'D',	{}...
);
for jj = 1:size(systems, 1)
	[systemsA, systemsB, systemsC, systemsD] = ssdata(systems{jj});
	system(jj, 1) = struct(...
		'A',	systemsA,...
		'B',	systemsB,...
		'C',	systemsC,...
		'D',	systemsD...
	);
end

%% set initial values
a = 0.65;
b = 0.5;
R = 50;

R_0 = [
	100,	10000,	-10000,	1;
	100,	100,	100,	1
];
R_fixed = false(size(R_0));
%% 
weight = {5, repmat([
	1, 10
], size(systems, 1), 1)};
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
polearea = {repmat([
	control.design.gamma.area.Circlesquare(R),	control.design.gamma.area.Hyperbolasquare(a, b), control.design.gamma.area.Imag(1, a)
], size(systems, 1), 1), repmat([
	control.design.gamma.area.Circlesquare(R/2, -R/2, 0), control.design.gamma.area.Imag(1, a)
], size(systems, 1), 1)};
%polearea = polearea{1};
weight = weight{1};
solver = optimization.solver.Optimizer.IPOPT;% solver to use
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
	'SpecifyObjectiveHessian',		true,...
	'SpecifyConstraintGradient',	true,...
	'SpecifyConstraintHessian',		true,...
	'CheckGradients',				false,...
	'FunValCheck',					false,...
	'FiniteDifferenceType',			'forward',...
	'Diagnostics',					false,...
	'UseParallel',					false,...
	'Display',						'iter-detailed'...
);
[R_opt, J_opt, info] = control.design.gamma.gammasyn(system, polearea, weight, R_fixed, R_0, options, struct(...
	'usecompiled',		false,...% indicator, if compiled functions should be used
	'eigenvaluederivative',	GammaEigenvalueDerivativeType.VANDERAA,...
	'type',				GammaJType.LINEAR,...% type of pole area weighting in objective function
	'allowvarorder',	false...%allow variable state number for different multi models
))
return;

%% plot poles of systems
plotopenloop = false;
figure;
hold('all');
if exist('a', 'var') && exist('b', 'var')
	phi = atan(b/a);
	s = (-phi:0.01:phi).';
	plot(-R*cos(s), R*sin(s), 'k');
	wp = (-sin(phi)*R:0.1:sin(phi)*R).';
	plot(-a/b * sqrt(wp.^2 + b^2), wp, 'k');
	plot([0, 0], [-R, R], 'r');
else
	wp = linspace(0, 2*pi, 500);
	plot(R*cos(wp), R*sin(wp), 'k');
end

eigstest = NaN(size(testsystems(1).A, 1), length(testsystems));
for ii = 1:length(testsystems)
	A = testsystems(ii).A;
	B = testsystems(ii).B;
	C = testsystems(ii).C;
	if plotopenloop
		eigstest(:, ii) = eig(A);
	else
		eigstest(:, ii) = eig(A - B*R_opt*C);
	end
end
plot(real(eigstest), imag(eigstest), '*');
if ~exist('systems', 'var') || exist('system', 'var')
	systems = system;
	delsystems = ~exist('systems', 'var');
else
	delsystems = false;
end
polesinarea = control.design.gamma.hasallpolesinarea(systems, R_opt, polearea);
colors = jet(length(systems));
if iscell(systems)
	A = ssdata(systems{1});
	numbereig = size(A);
elseif isstruct(systems)
	numbereig = size(systems(1).A, 1);
else
	numbereig = 0;
end
eigssys = NaN(numbereig, length(systems));
for ii = 1:length(systems)
	if iscell(systems)
		[A, B, C, ~] = ssdata(systems{ii});
	elseif isstruct(systems)
		A = systems(ii).A;
		B = systems(ii).B;
		C = systems(ii).C;
	else
		error();
	end
	if plotopenloop
		eigssys(:, ii) = eig(A);
	else
		eigssys(:, ii) = eig(A - B*R_opt*C);
	end
	plot(real(eigssys(:, ii)), imag(eigssys(:, ii)), 'd', 'MarkerSize', 10, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', colors(ii, :));
	plot(real(eigssys(:, ii)), imag(eigssys(:, ii)), '.', 'MarkerSize', 7, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', [0, 0, 0]);
end
grid('on');
hold('off');
if delsystems
	clear systems;
end
clear delsystems

%%
t = (0:0.01:15).';
w = 0.05*ones(length(t), 3);

figure;
subplot(2, 1, 1);
for ii = 1:length(testsystems)
	A = testsystems(ii).A;
	B = testsystems(ii).B;
	C = testsystems(ii).C;
	
	Acl = A - B*R_opt*C;
	Bcl = B*R_opt;

	Gcl = ss(Acl, Bcl, C, [], T);
	y = lsim(Gcl, w, t);
	
	subplot(2, 1, 1);
	plot(t, y(:,1));
	hold('all');
	subplot(2, 1, 2);
	plot(t, y(:,2));
	hold('all');
end
ylabel('$v$');
grid('on');
subplot(2, 1, 1);
grid('on')
ylabel('$\lambda$');
hold('off');
subplot(2, 1, 1);
for ii = 1:length(system)
	A = system(ii).A;
	B = system(ii).B;
	C = system(ii).C;
	
	Acl = A - B*R_opt*C;
	Bcl = B*R_opt;

	Gcl = ss(Acl, Bcl, C, [], T);
	y = lsim(Gcl, w, t);
	
	subplot(2, 1, 1);
	plot(t, y(:, 1));
	hold('all');
	subplot(2, 1, 2);
	plot(t, y(:, 2));
	hold('all');
end
ylabel('$v$');
grid('on');
subplot(2, 1, 1);
grid('on')
ylabel('$\lambda$');
hold('off');
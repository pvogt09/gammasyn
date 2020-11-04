clear; close all;

%% Settings
[systems, system_properties] = example.load_system_decoupling('threetank', 'exact output13');
% EXACT								structurally constrained controller only EXAKT solution
% APPROXIMATE						structurally constrained controller also approximate solution
% APPROXIMATE_INEQUALITY			structurally constrained controller also approximate solution. Formulate decoupling constraints as bounds.
% NUMERIC_NONLINEAR_EQUALITY		fully numeric design with non-linear equality constraints
% NUMERIC_NONLINEAR_INEQUALITY		fully numeric design with non-linear inequality constraints

% control_design_type = GammaDecouplingStrategy.EXACT;
% control_design_type = GammaDecouplingStrategy.APPROXIMATE;
% control_design_type = GammaDecouplingStrategy.APPROXIMATE_INEQUALITY
control_design_type = GammaDecouplingStrategy.NUMERIC_NONLINEAR_EQUALITY;
% control_design_type = GammaDecouplingStrategy.NUMERIC_NONLINEAR_INEQUALITY;

%% pole area parameters
a = 0.8;
b = 0.5;
r = 5;

%% gammasyn options
solver = optimization.solver.Optimizer.SNOPT; % solver to use
options = optimization.options.OptionFactory.instance.options(solver,...
	'Retries',						1,...											% number of retries
	'Algorithm',					solver.getDefaultAlgorithm(),...				% algorithm of solver, for not builtin solvers name of the solver, e.g. 'snopt' for SNOPT
	'FunctionTolerance',			1E-5,...
	'StepTolerance',				1E-5,...
	'ConstraintTolerance',			1.4e-5,...
	'MaxFunctionEvaluations',		5E3,...
	'MaxIterations',				5E3,...
	'MaxSQPIter',					5E3,...
	'SpecifyObjectiveGradient',		true,...
	'SpecifyConstraintGradient',	false,...
	'CheckGradients',				false,...
	'FunValCheck',					false,...
	'FiniteDifferenceType',			'forward',...
	'Diagnostics',					false,...
	'Display',						'iter-detailed',...
	'UseParallel',					false...
	);
objectiveoptions = struct(...
	'usecompiled',				true,...											% indicator, if compiled functions should be used
	'type',						GammaJType.LINEAR,...								% type of pole area weighting in objective function
	'allowvarorder',			false,...											% allow variable state number for different multi models
	'eigenvaluederivative',		GammaEigenvalueDerivativeType.VANDERAA,...
	'errorhandler',				GammaErrorHandler.ERROR,...
	'strategy',					GammaSolutionStrategy.SINGLESHOT...
	);

%% Pole area
weight = {5, repmat([1, 10], system_properties.number_models, 1)};
polearea = {repmat([
	control.design.gamma.area.Circle(r), control.design.gamma.area.Hyperbola(a, b)...
%   	control.design.gamma.area.Imag(1, a)
], system_properties.number_models, 1), repmat([
	control.design.gamma.area.Circlesquare(r/2, -r/2, 0), control.design.gamma.area.Imag(1, a)
], system_properties.number_models, 1)};
polearea = polearea{1};
weight = weight{1};


%% gammasyn_decouplingcontrol
objectiveoptions.decouplingcontrol.decouplingstrategy = control_design_type;
objectiveoptions.decouplingcontrol.tolerance_prefilter = 70;
objectiveoptions.decouplingcontrol.tolerance_decoupling = 0*1e-6;
objectiveoptions.decouplingcontrol.tf_structure = system_properties.tf_structure;
objectiveoptions.decouplingcontrol.solvesymbolic = true;
objectiveoptions.decouplingcontrol.round_equations_to_digits = 5;
objectiveoptions.decouplingcontrol.sortingstrategy_decoupling = GammaDecouplingconditionSortingStrategy.MINIMUMNORM;
% objectiveoptions.decouplingcontrol.sortingstrategy_decoupling = GammaDecouplingconditionSortingStrategy.EIGENVALUETRACKING;
objectiveoptions.decouplingcontrol.weight_decoupling = [];
objectiveoptions.decouplingcontrol.allowoutputdecoupling = true;
[Kopt, Jopt, information] = control.design.gamma.gammasyn_decouplingcontrol(systems, polearea, weight, [], system_properties.RKF_0, options, objectiveoptions);
if isempty(Kopt)
	return;
end

R = Kopt{1} %#ok<*NOPTS>
F = Kopt{end}
information

%% Analysis
if ~any(any(isnan(R)))
	[gain_ratio, ~, poles_all, F] = test.develop.analyze_results(systems, Kopt, polearea, system_properties.tf_structure);
	Kopt{end} = F;
	minimal_deviation = 100/min(abs(gain_ratio))
end
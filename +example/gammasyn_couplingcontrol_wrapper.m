clear; close all;

%% Settings
[systems, system_properties] = example.load_system_coupling('threetank');
% EXACT								structurally constrained controller only EXAKT solution
% APPROXIMATE						structurally constrained controller also approximate solution
% NUMERIC_NONLINEAR_EQUALITY		fully numeric design with non-linear equality constraints
% NUMERIC_NONLINEAR_INEQUALITY		fully numeric design with non-linear inequality constraints

% control_design_type = GammaCouplingStrategy.EXACT;
control_design_type = GammaCouplingStrategy.APPROXIMATE;
% control_design_type = GammaCouplingStrategy.NUMERIC_NONLINEAR_EQUALITY;
% control_design_type = GammaCouplingStrategy.NUMERIC_NONLINEAR_INEQUALITY;

tolerance_NUMERIC_NONLINEAR_INEQUALITY = 0.001;

%% pole area parameters
a = 0.8;
b = 0.5;
r = 5;

%% gammasyn options
solver = optimization.solver.Optimizer.FMINCON; %FMINCON;% solver to use
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
	'SpecifyConstraintGradient',	true,...
	'CheckGradients',				false,...
	'FunValCheck',					false,...
	'FiniteDifferenceType',			'forward',...
	'Diagnostics',					false,...
	'Display',						'iter-detailed'...
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
	control.design.gamma.area.Circle(r), control.design.gamma.area.Hyperbola(a, b),...
%   	control.design.gamma.area.Imag(1, a)
], system_properties.number_models, 1), repmat([
	control.design.gamma.area.Circlesquare(r/2, -r/2, 0), control.design.gamma.area.Imag(1, a)
], system_properties.number_models, 1)};
polearea = polearea{1};
weight = weight{1};


%% gammasyn_couplingcontrol
objectiveoptions.couplingcontrol.couplingstrategy = control_design_type;
objectiveoptions.couplingcontrol.tolerance_coupling = tolerance_NUMERIC_NONLINEAR_INEQUALITY;
objectiveoptions.couplingcontrol.couplingconditions = uint32(system_properties.number_couplingconditions);
objectiveoptions.couplingcontrol.solvesymbolic = true;
objectiveoptions.couplingcontrol.round_equations_to_digits = 5;
[Kopt, Jopt, information] = control.design.gamma.gammasyn_couplingcontrol(systems, polearea, weight, [], system_properties.RKF_0, options, objectiveoptions);
if isempty(Kopt)
	return;
end

R = Kopt{1}
F = Kopt{end}
information

%% Analysis
if ~any(any(isnan(R)))
	[gain_ratio, ~, poles_all, F] = test.develop.analyze_results(systems, Kopt, polearea, system_properties.number_couplingconditions);
	Kopt{end} = F;
	minimal_deviation = 100/min(abs(gain_ratio))
end
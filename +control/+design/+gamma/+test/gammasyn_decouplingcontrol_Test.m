function [pass] = gammasyn_decouplingcontrol_Test(~)
	%GAMMASYN_DECOUPLINGONTROLTEST test cases for checking gammasyn_decouplingcontrol for correct argument handling
	%	Input:
	%		silent:	idicator, if information should be output
	%	Output:
	%		pass:	indicator, if test was passed
	pass = identifier.TestResult.PASSED;

	%% default variables
	number_controls = 2;
	number_measurements = 3;
	number_measurements_xdot = 0;
	number_states = 3;
	number_references = 2;
	number_models = 2;

	R_fixed_default = {
		{zeros(number_controls, number_measurements, 0), zeros(0, 1)},...
		{zeros(number_controls, number_measurements_xdot, 0), zeros(0, 1)},...
		{zeros(number_controls, number_references, 0), zeros(0, 1)},...
		{zeros(number_controls, (number_measurements + number_measurements_xdot + number_references), 0), zeros(0, 1)}
	};
	R_bounds_default = R_fixed_default;
	systems_default = struct(...
		'E',		{eye(number_states),																	eye(number_states)},...
		'A',		{reshape(1:number_states^2, number_states, number_states),								reshape((1:number_states^2) + sqrt(2), number_states, number_states)},...
		'B',		{reshape(1:2:2*number_states*number_controls, number_states, number_controls),			reshape(1:3:3*number_states*number_controls, number_states, number_controls)},...
		'C',		{eye(number_measurements, number_states),												2*eye(number_measurements, number_states)},...
		'C_ref',	{reshape((1:number_references*number_states)/pi, number_references, number_states),		reshape((1:number_references*number_states), number_references, number_states)},...
		'C_dot',	{ones(number_measurements_xdot, number_states),											ones(number_measurements_xdot, number_states)},...
		'D',		{zeros(number_measurements, number_controls),											zeros(number_measurements, number_controls)},...
		'D_ref',	{zeros(number_references, number_controls),												zeros(number_references, number_controls)}...
	);
	systems_default = systems_default';
	R_0_default = {randi(10, [number_controls, number_measurements]), randi(10, [number_controls, number_measurements_xdot]), randi(10, [number_controls, number_references])};
	tf_structure = zeros(number_references, number_references);
	tf_structure(1:(number_references + 1):number_references^2) = NaN;
	objectiveoptions_default = struct(...
		'usecompiled',				configuration.control.design.gamma.hascompiled(),...	% indicator, if compiled functions should be used
		'type',						[],...													% type of pole area weighting in objective function
		'allowvarorder',			false,...												% allow variable state number for different multi models
		'eigenvaluederivative',		GammaEigenvalueDerivativeType.VANDERAA,...
		'errorhandler',				GammaErrorHandler.ERROR,...
		'strategy',					GammaSolutionStrategy.SINGLESHOT...
	);
	control_design_types = {
		GammaDecouplingStrategy.EXACT;
		GammaDecouplingStrategy.APPROXIMATE;
		GammaDecouplingStrategy.APPROXIMATE_INEQUALITY
		GammaDecouplingStrategy.NUMERIC_NONLINEAR_EQUALITY;
		GammaDecouplingStrategy.NUMERIC_NONLINEAR_INEQUALITY;
		GammaDecouplingStrategy.MERIT_FUNCTION
	};
	objectiveoptions_default.decouplingcontrol = struct(...
		'decouplingstrategy',			control_design_types{1},...
		'tolerance_prefilter',			1e-3,...
		'tolerance_decoupling',			1e-3,...
		'tf_structure',					tf_structure,...
		'solvesymbolic',				true,...
		'round_equations_to_digits',	5,...
		'sortingstrategy_decoupling',	GammaDecouplingconditionSortingStrategy.MINIMUMNORM,...	% EIGENVALUETRACKING
		'weight_decoupling',			[],...
		'allowoutputdecoupling',		true...
	);
	solver = optimization.solver.Optimizer.FMINCON; % solver to use
	solveroptions_default = optimization.options.OptionFactory.instance.options(solver,...
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
		'SpecifyObjectiveHessian',		true,...
		'SpecifyConstraintHessian',		true,...
		'CheckGradients',				false,...
		'FunValCheck',					false,...
		'FiniteDifferenceType',			'forward',...
		'Diagnostics',					false,...
		'Display',						'iter-detailed',...
		'UseParallel',					false...
	);
	weights_default = 1;
	areafun_default = repmat([
		control.design.gamma.area.Circle(100), control.design.gamma.area.Hyperbola(1, 1)
	], number_models, 1);
	R_nonlin_default = [];
	number_passed_parameters_default = Inf;
	test_object = control.design.gamma.test.gammasyn_decouplingcontrol_Test_Class(systems_default, areafun_default, weights_default, R_fixed_default, R_0_default, solveroptions_default, objectiveoptions_default, R_bounds_default, R_nonlin_default, number_passed_parameters_default);

	%% testcases
	testparallelfunctions = false;
	if testparallelfunctions && isempty(gcp('nocreate'))
		parpool;
	end
	positive_testcases = [];
% 	positive_testcases = create_testcase(positive_testcases, {'sys', {{1, 'E', 1, 2}; {2, 'E', 1, 3}}});
% 	positive_testcases = create_testcase(positive_testcases, {'sys', {{1, 'E', 1, 0}; {2, 'E', 1, 0}}});
% 	positive_testcases = create_testcase(positive_testcases, {'sys', {{1, 'E', 1, 0}; {2, 'E', 1, 0}}}, {'bnd', {{1, 1, [1 0 0; 0 0 0]}; {1, 2, 10}}});
% 	positive_testcases = create_testcase(positive_testcases, {'obj', {{'decouplingcontrol', 'decouplingstrategy'}, GammaDecouplingStrategy.NUMERIC_NONLINEAR_EQUALITY}});
% 	positive_testcases = create_testcase(positive_testcases, {'obj', {{'decouplingcontrol', 'decouplingstrategy'}, GammaDecouplingStrategy.NUMERIC_NONLINEAR_EQUALITY; {'decouplingcontrol', 'sortingstrategy_decoupling'}, GammaDecouplingconditionSortingStrategy.EIGENVALUETRACKING}});
% 	positive_testcases = create_testcase(positive_testcases, {'obj', {{'decouplingcontrol', 'decouplingstrategy'}, GammaDecouplingStrategy.NUMERIC_NONLINEAR_EQUALITY; {'decouplingcontrol', 'sortingstrategy_decoupling'}, GammaDecouplingconditionSortingStrategy.EIGENVALUETRACKING}}, {'sol', {'UseParallel', true}});
% 	positive_testcases = create_testcase(positive_testcases, {'obj', {{'decouplingcontrol', 'decouplingstrategy'}, GammaDecouplingStrategy.MERIT_FUNCTION}});
% 	positive_testcases = create_testcase(positive_testcases, {'num', {3}});
% 	positive_testcases = create_testcase(positive_testcases, {'num', {4}});
% 	positive_testcases = create_testcase(positive_testcases, {'num', {5}});
% 	positive_testcases = create_testcase(positive_testcases, {'num', {6}});
% 	positive_testcases = create_testcase(positive_testcases, {'num', {7}});
% 	positive_testcases = create_testcase(positive_testcases, {'num', {8}});
	positive_testcases = create_testcase(positive_testcases, {'num', {4}}, {'fix', {{[], [], []}}}, {'obj', {{'decouplingcontrol', 'tf_structure'}, NaN(number_references, number_references)}});
	positive_testcases = create_testcase(positive_testcases, {'fix', {{1, 1, reshape(eye(number_controls*number_measurements), number_controls, number_measurements, number_controls*number_measurements)}; {1, 2, 10*ones(number_controls*number_measurements, 1)}}});
	positive_testcases = create_testcase(positive_testcases, {'fix', {{1, 1, [1 0 0; 0 0 0]}; {1, 2, 1}}}, {'obj', {{'decouplingcontrol', 'decouplingstrategy'}, GammaDecouplingStrategy.APPROXIMATE; {'decouplingcontrol', 'tf_structure'}, [NaN, NaN; 0, NaN]}});
	positive_testcases = create_testcase(positive_testcases, {'fix', {{1, 1, [1 0 0; 0 0 0]}; {1, 2, 1}}}, {'obj', {{'decouplingcontrol', 'decouplingstrategy'}, GammaDecouplingStrategy.APPROXIMATE_INEQUALITY; {'decouplingcontrol', 'tf_structure'}, [NaN, NaN; 0, NaN]}});
	positive_testcases = create_testcase(positive_testcases, {'bnd', {{1, 1, [1 0 0; 0 0 0]}; {1, 2, 1e6}}}, {'obj', {{'decouplingcontrol', 'decouplingstrategy'}, GammaDecouplingStrategy.APPROXIMATE_INEQUALITY; {'decouplingcontrol', 'tf_structure'}, [NaN, NaN; 0, NaN]}});
	positive_testcases = create_testcase(positive_testcases, {'nlin', {@nonlin}});

	negative_testcases = [];
	negative_testcases = create_testcase(negative_testcases, {'obj', {[], control.design.gamma.GammasynOptions}}); % because tf_structure == [] by default
	negative_testcases = create_testcase(negative_testcases, {'obj', {[], 5}}); % because objectiveoptions must be struct
	negative_testcases = create_testcase(negative_testcases, {'obj', {{'decouplingcontrol', 'decouplingstrategy'}, [GammaDecouplingStrategy.NONE; GammaDecouplingStrategy.APPROXIMATE]}});
	negative_testcases = create_testcase(negative_testcases, {'obj', {{'decouplingcontrol', 'decouplingstrategy'}, 'NO VALID DECOUPLINGSTRATEGY'}});
	negative_testcases = create_testcase(negative_testcases, {'sys', {{1, 'E', [], eye(number_states + 1)}; {1, 'A', [], eye(number_states + 1)}; {1, 'B', [], eye(number_states + 1, number_controls)}; {1, 'C', [], eye(number_measurements, number_states + 1)}; {1, 'C_ref', [], eye(number_references, number_states + 1)}; {1, 'C_dot', [], eye(number_measurements_xdot, number_states + 1)}}}); % for line 109. All systems must have same number_states for decoupling design
	negative_testcases = create_testcase(negative_testcases, {'sys', {{1, 'C', [], eye(number_measurements - 1, number_states)}; {1, 'D', [], eye(number_measurements - 1, number_controls)}}}, {'obj', {{'decouplingcontrol', 'allowoutputdecoupling'}, false}});
	negative_testcases = create_testcase(negative_testcases, {'sys', {{1, 'C', [], eye(number_measurements, number_states)}; {1, 'C', 1, 0}}}, {'obj', {{'decouplingcontrol', 'allowoutputdecoupling'}, false}});
	negative_testcases = create_testcase(negative_testcases, {'sys', {{1, 'D', 1, 1}}});
	negative_testcases = create_testcase(negative_testcases, {'sys', {{1, 'E', 1, 0}; {1, 'C_dot', [], 2*eye(1, number_states)}; {2, 'C_dot', [], eye(1, number_states)}}}); % for line 127
	negative_testcases = create_testcase(negative_testcases, {'sys', {{1, 'E', [1, number_states + 2], 0}; {2, 'E', 1, 0}}}); % for line 282
	negative_testcases = create_testcase(negative_testcases, {'init', {{1, [], zeros(number_controls, number_measurements, 2)}}});
	negative_testcases = create_testcase(negative_testcases, {'init', {{1, [], zeros(number_controls, number_measurements + 1)}}});
	negative_testcases = create_testcase(negative_testcases, {'bnd', {{2, 1, zeros(2, 1, 0)}; {4, 1, zeros(2, 6, 0)}}}, {'sys', {{1, 'C_dot', [], eye(1, number_states)}; {2, 'C_dot', [], eye(1, number_states)}}}, {'init', {{2, [], zeros(number_controls, number_measurements_xdot + 1)}}});
	negative_testcases = create_testcase(negative_testcases, {'fix', {{2, 1, reshape([1 0 0 1], 2, 1, 2)}; {2, 2, [0; 0]}}}, {'bnd', {{2, 1, zeros(2, 1, 0)}; {4, 1, zeros(2, 6, 0)}}}, {'sys', {{1, 'C_dot', [], eye(1, number_states)}; {2, 'C_dot', [], eye(1, number_states)}}}, {'init', {{2, [], zeros(number_controls, number_measurements_xdot + 2)}}});
	negative_testcases = create_testcase(negative_testcases, {'fix', {{2, 1, reshape([1 0 0 1], 2, 1, 2)}; {2, 2, [0; 0]}}}, {'bnd', {{2, 1, zeros(2, 1, 0)}; {4, 1, zeros(2, 6, 0)}}}, {'sys', {{1, 'C_dot', [], eye(1, number_states)}; {2, 'C_dot', [], eye(1, number_states)}}}, {'init', {{2, [], eye(number_controls, number_measurements_xdot + 1)}}});
	negative_testcases = create_testcase(negative_testcases, {'init', {{3, [], []}}});
	negative_testcases = create_testcase(negative_testcases, {'init', {{3, [], zeros(10, 5)}}});
	negative_testcases = create_testcase(negative_testcases, {'sol', {'ProblemType', optimization.options.ProblemType.UNCONSTRAINED}}, {'obj', {{'decouplingcontrol', 'decouplingstrategy'}, GammaDecouplingStrategy.NUMERIC_NONLINEAR_EQUALITY}});
% 	delete(gcp('nocreate')); %shut down parpool

	%% test
	ok_positive_testcases = false(size(positive_testcases, 1), 1);
	for ii = 1:size(positive_testcases, 1)
		test_object.reset();
		[systems, areafun, weights, R_fixed, R_0, solveroptions, objectiveoptions, R_bounds, R_nonlin, number_passed_parameters] = test_object.implement_testcase(positive_testcases(ii)); %#ok<ASGLU>

		number_controls_tmp = size(systems(1).B, 2);
		number_measurements_tmp = size(systems(1).C, 1);
		number_measurements_xdot_tmp = size(systems(1).C_dot, 1);
		number_references_tmp = size(systems(1).C_ref, 1);

		codetotest = create_codetotest(number_passed_parameters);
		test.TestSuite.assertNoException(codetotest, 'any', 'gammasyn_decouplingcontrol must not throw an exception.');
		ok_positive_testcases(ii) = check_output(Ropt, number_controls_tmp, number_measurements_tmp, number_measurements_xdot_tmp, number_references_tmp);
	end
	for ii = 1:size(negative_testcases, 1)
		test_object.reset();
		[systems, areafun, weights, R_fixed, R_0, solveroptions, objectiveoptions, R_bounds, R_nonlin, number_passed_parameters] = test_object.implement_testcase(negative_testcases(ii)); %#ok<ASGLU>
		codetotest = create_codetotest(number_passed_parameters);
		test.TestSuite.assertException(codetotest, 'any', 'gammasyn_decouplingcontrol must throw an exception.');
	end
	failed_positive_testcases = find(~ok_positive_testcases); %#ok<EFIND>
	if ~isempty(failed_positive_testcases)
		pass = identifier.TestResult.FAILED;
	end
end

function [codetotest] = create_codetotest(number_passed_parameters)
	if number_passed_parameters <= 3
		codetotest = '[Ropt, Jopt, information] = control.design.gamma.gammasyn_decouplingcontrol(systems, areafun, weights);';
	elseif number_passed_parameters <= 4
		codetotest = '[Ropt, Jopt, information] = control.design.gamma.gammasyn_decouplingcontrol(systems, areafun, weights, R_fixed);';
	elseif number_passed_parameters <= 5
		codetotest = '[Ropt, Jopt, information] = control.design.gamma.gammasyn_decouplingcontrol(systems, areafun, weights, R_fixed, R_0);';
	elseif number_passed_parameters <= 6
		codetotest = '[Ropt, Jopt, information] = control.design.gamma.gammasyn_decouplingcontrol(systems, areafun, weights, R_fixed, R_0, solveroptions);';
	elseif number_passed_parameters <= 7
		codetotest = '[Ropt, Jopt, information] = control.design.gamma.gammasyn_decouplingcontrol(systems, areafun, weights, R_fixed, R_0, solveroptions, objectiveoptions);';
	elseif number_passed_parameters <= 8
		codetotest = '[Ropt, Jopt, information] = control.design.gamma.gammasyn_decouplingcontrol(systems, areafun, weights, R_fixed, R_0, solveroptions, objectiveoptions, R_bounds);';
	else
		codetotest = '[Ropt, Jopt, information] = control.design.gamma.gammasyn_decouplingcontrol(systems, areafun, weights, R_fixed, R_0, solveroptions, objectiveoptions, R_bounds, R_nonlin);';
	end
end

function [struct_out] = create_testcase(struct_in, varargin)
	%CREATE_TESTCASE creates and appends structure specifying the characteristic testcase options different from the default options
	%	Input:
	%		struct_in:	structure specifying other testcases. Put [], if this is the first testcase created.
	%		varargin:	cell array(s) of options of the form {identifier-string, cell-array for specifying testcase options}. For precise information, see amend_... functions in gammasyn_decouplingcontrol_Test_Class
	%	Output:
	%		struct_out:	appended or newly created testcase struct.
	if ~isstruct(struct_in)
		struct_out = struct(...
			'systems',					{},...
			'R_fixed',					{},...
			'R_0',						{},...
			'solveroptions',			{},...
			'objectiveoptions',			{},...
			'R_bounds',					{},...
			'R_nonlin',					{},...
			'number_passed_parameters',	{},...
			'areafun',					{}...
		);
		size_old_struct = 0;
	else
		struct_out = struct_in;
		size_old_struct = size(struct_in, 1);
	end
	for ii = 1:size(varargin, 2)
		identifier = varargin{1, ii}{1};
		option = varargin{1, ii}{2};
		if strcmp(identifier, 'sys')
			struct_out(size_old_struct + 1, 1).systems = option;
		elseif strcmp(identifier, 'fix')
			struct_out(size_old_struct + 1, 1).R_fixed = option;
		elseif strcmp(identifier, 'init')
			struct_out(size_old_struct + 1, 1).R_0 = option;
		elseif strcmp(identifier, 'sol')
			struct_out(size_old_struct + 1, 1).solveroptions = option;
		elseif strcmp(identifier, 'obj')
			struct_out(size_old_struct + 1, 1).objectiveoptions = option;
		elseif strcmp(identifier, 'bnd')
			struct_out(size_old_struct + 1, 1).R_bounds = option;
		elseif strcmp(identifier, 'nlin')
			struct_out(size_old_struct + 1, 1).R_nonlin = option;
		elseif strcmp(identifier, 'num')
			struct_out(size_old_struct + 1, 1).number_passed_parameters = option;
		elseif strcmp(identifier, 'area')
			struct_out(size_old_struct + 1, 1).areafun = option;
		else
			error('control:gamma:decoupling:test', 'Wrong identifier');
		end
	end
end

function [ok] = check_output(R, number_controls, number_measurements, number_measurements_xdot, number_references)
	ok = true;
	dim_2 = [
		number_measurements, number_measurements_xdot, number_references
	];
	if iscell(R)
		sz = size(R, 1);
		if sz ~= 3
			ok = false;
		else
			for ii = 1:sz
				if ~all(size(R{ii}) == [number_controls, dim_2(ii)])
					ok = false;
				end
			end
		end
	else
		if ~all(size(R) == [number_controls, number_measurements])
			ok = false;
		end
	end
end

function [c_R, ceq_R, c_K, ceq_K, c_F, ceq_F, gradc_R, gradceq_R, gradc_K, gradceq_K, gradc_F, gradceq_F] = nonlin(R, K, F) %#ok<INUSD>
	c_R = [];
	ceq_R = [];
	c_K = [];
	ceq_K = [];
	c_F = [];
	ceq_F = [];
	gradc_R = [];
	gradceq_R = [];
	gradc_K = [];
	gradceq_K = [];
	gradc_F = [];
	gradceq_F = [];
end
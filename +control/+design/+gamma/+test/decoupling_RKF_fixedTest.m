function [pass, failed_positive_testcases] = decoupling_RKF_fixedTest(~)
	%DECOUPLING_RKF_FIXEDTEST test cases for checking decoupling constraints function
	%	Input:
	%		silent:						idicator, if information should be output
	%	Output:
	%		pass:						indicator, if test was passed
	%		failed_positive_testcases:	testcases which returned wrong output arguments

	%% setup
	pass = identifier.TestResult.PASSED;
	path = realpath(fullfile(mfilename('fullpath'), '..', '..', 'private'));
	decoupling_RKF_fixed = get_private_function(path, 'decoupling_RKF_fixed'); %#ok<NASGU>

	%% default variables
	number_controls = 2;
	number_measurements = 2;
	number_measurements_xdot = 0;
	number_states = 3;
	number_references = 2;

	R_fixed_ext_default = {
		{zeros(number_controls, number_measurements, 0), zeros(0, 1)},...
		{zeros(number_controls, number_measurements_xdot, 0), zeros(0, 1)},...
		{zeros(number_controls, number_references, 0), zeros(0, 1)},...
		{zeros(number_controls, (number_measurements + number_measurements_xdot + number_references), 0), zeros(0, 1)}
	};
	systems_default = struct(...
		'E',		{eye(number_states),									eye(number_states)},...
		'A',		{randi(10, [number_states, number_states]),				eye(number_states)},...
		'B',		{randi(10, [number_states, number_controls]),			randi(10, [number_states, number_controls])},...
		'C',		{randi(10, [number_measurements, number_states]),		randi(10, [number_measurements, number_states])},...
		'C_ref',	{randi(10, [number_references, number_states]),			randi(10, [number_references, number_states])},...
		'C_dot',	{randi(10, [number_measurements_xdot, number_states]),	randi(10, [number_measurements_xdot, number_states])},...
		'D',		{zeros(number_measurements, number_controls),			zeros(number_measurements, number_controls)},...
		'D_ref',	{zeros(number_references, number_controls),				zeros(number_references, number_controls)}...
	);
	tf_structure = zeros(number_measurements, number_references);
	tf_structure(1:number_references + 1:number_measurements*number_references) = nan;
	objectiveoptions_default.decouplingcontrol = struct(...
		'decouplingstrategy',			GammaDecouplingStrategy.APPROXIMATE,...
		'tolerance_prefilter',			0e-0,...
		'tolerance_decoupling',			1e-0,...
		'tf_structure',					tf_structure,...
		'solvesymbolic',				true,...
		'round_equations_to_digits',	5,...
		'weight_decoupling',			[],...
		'allowoutputdecoupling',		true...
	);
	solveroptions_default.Display = 'off';
	descriptor_default = false;

	test_object = control.design.gamma.test.decoupling_RKF_fixedTest_Class(systems_default, R_fixed_ext_default, objectiveoptions_default, solveroptions_default, descriptor_default);

	%% testcases
	positive_testcases = [];
	positive_testcases = create_testcase(positive_testcases,	{'sys', {{1, 'E', 1, 1}; {2, 'E', 1, 3}}},...
																{'dsc', true});
	positive_testcases = create_testcase(positive_testcases,	{'obj', {{'decouplingcontrol', 'decouplingstrategy'}, GammaDecouplingStrategy.EXACT}});
	positive_testcases = create_testcase(positive_testcases,	{'obj', {{'decouplingcontrol', 'decouplingstrategy'}, GammaDecouplingStrategy.APPROXIMATE_INEQUALITY}});
	positive_testcases = create_testcase(positive_testcases,	{'obj', {{'decouplingcontrol', 'solvesymbolic'}, false}});
	positive_testcases = create_testcase(positive_testcases,	{'dsc', -1});
	positive_testcases = create_testcase(positive_testcases,	{'obj', {{'decouplingcontrol', 'tf_structure'}, nan(number_references, number_references)}});
	positive_testcases = create_testcase(positive_testcases,	{'sys', {{1, 'D_ref', 1, 1}; {2, 'D_ref', 4, -1}}});
	positive_testcases = create_testcase(positive_testcases,	{'sys', {{1, 'D_ref', 1, 1}; {2, 'D_ref', 4, -1}}},...
																{'dsc', true});
	positive_testcases = create_testcase(positive_testcases,	{'sys', {{1, 'C', [], [0 0 0]}; {2, 'C', [], [0 0 0]}}},...
																{'obj', {{'decouplingcontrol', 'tf_structure'}, zeros(number_references, number_references)}}, ...
																{'fix', {{1, 1, zeros(2, 1, 0)}; {4, 1, zeros(2, 3, 0)}}});
	positive_testcases = create_testcase(positive_testcases,	{'sol', {'Display', 'iter-detailed'}});
	positive_testcases = create_testcase(positive_testcases,	{'fix', {{4, 1, cat(3, [1 0 0 0; 0 0 0 0], [1 0 0 0; 0 0 0 0])}; {4, 2, [1; 2]}}},...
																{'sol', {'Display', 'iter-detailed'}});
	positive_testcases = create_testcase(positive_testcases,	{'fix', {{4, 1, cat(3, [1 0 0 0; 0 0 0 0], [1 0 0 0; 0 0 0 0])}; {4, 2, [1; 2]}}},...
																{'sol', {'Display', 'iter-detailed'}},...
																{'obj', {{'decouplingcontrol', 'decouplingstrategy'}, GammaDecouplingStrategy.EXACT}});
	positive_testcases = create_testcase(positive_testcases,	{'fix', {{1, 1, cat(3, [1 0; 0 0], [1 0; 0 0])}; {1, 2, [1; 2]}}},...
																{'sol', {'Display', 'iter-detailed'}});
	positive_testcases = create_testcase(positive_testcases,	{'fix', {{1, 1, cat(3, [1 0; 0 0], [1 0; 0 0])}; {1, 2, [1; 2]}}},...
																{'sol', {'Display', 'iter-detailed'}},...
																{'obj', {{'decouplingcontrol', 'decouplingstrategy'}, GammaDecouplingStrategy.EXACT}});
	positive_testcases = create_testcase(positive_testcases,	{'fix', {{3, 1, cat(3, [1 0; 0 0], [1 0; 0 0])}; {3, 2, [1; 2]}}},...
																{'sol', {'Display', 'iter-detailed'}});
	positive_testcases = create_testcase(positive_testcases,	{'fix', {{3, 1, cat(3, [1 0; 0 0], [1 0; 0 0])}; {3, 2, [1; 2]}}},...
																{'sol', {'Display', 'iter-detailed'}},...
																{'obj', {{'decouplingcontrol', 'decouplingstrategy'}, GammaDecouplingStrategy.EXACT}});
	positive_testcases = create_testcase(positive_testcases,	{'fix', {{3, 1, cat(3, [1 0; 0 0], [0 0; 1 0])}; {3, 2, [0; 0]}}},...
																{'sol', {'Display', 'iter-detailed'}});
	positive_testcases = create_testcase(positive_testcases,	{'fix', {{3, 1, cat(3, [1 0; 0 0], [0 0; 1 0])}; {3, 2, [0; 0]}}},...
																{'sol', {'Display', 'iter-detailed'}},...
																{'obj', {{'decouplingcontrol', 'decouplingstrategy'}, GammaDecouplingStrategy.EXACT}});
	positive_testcases = create_testcase(positive_testcases,	{'obj', {{'decouplingcontrol', 'solvesymbolic'}, false; {'decouplingcontrol', 'tf_structure'}, nan(number_references, number_references)}},...
																{'sol', {'Display', 'iter-detailed'}});
	for n = 1:3
		for p = 1:n
			for q = 1:n
				for r = 1:p
					for q_dot = 0:n
						positive_testcases = create_testcase(positive_testcases,	{'sys', {{1, 'E', [], eye(n, n)}; {2, 'E', [], eye(n, n)};
																							{1, 'A', [], zeros(n, n)}; {2, 'A', [], zeros(n, n)};
																							{1, 'B', [], eye(n, p)}; {2, 'B', [], eye(n, p)};
																							{1, 'C', [], eye(q, n)}; {2, 'C', [], eye(q, n)};
																							{1, 'C_dot', [], eye(q_dot, n)}; {2, 'C_dot', [], eye(q_dot, n)};
																							{1, 'C_ref', [], eye(r, n)}; {2, 'C_ref', [], eye(r, n)};
																							{1, 'D', [], zeros(q, p)}; {2, 'D', [], zeros(q, p)};
																							{1, 'D_ref', [], eye(r, p)}; {2, 'D_ref', [], eye(r, p)}}},...
																					{'obj', {{'decouplingcontrol', 'tf_structure'}, diag(nan(1, r))}},...
																					{'fix', {{1, 1, zeros(p, q, 0)}; {1, 2, zeros(0, 1)};
																							{2, 1, reshape(eye(p*q_dot, p*q_dot), p, q_dot, p*q_dot)}; {2, 2, zeros(p*q_dot, 1)};
																							{3, 1, zeros(p, r, 0)}; {3, 2, zeros(0, 1)};
																							{4, 1, zeros(p, q + q_dot + r, 0)}; {4, 2, zeros(0, 1)}}});
					end
				end
			end
		end
	end

	negative_testcases = [];
	negative_testcases = create_testcase(negative_testcases,	{'sys', {{1, 'E', 1, 0}; {2, 'E', 1, 3}}},...
																{'dsc', true});
	negative_testcases = create_testcase(negative_testcases,	{'obj', {{'decouplingcontrol', 'decouplingstrategy'}, GammaDecouplingStrategy.NUMERIC_NONLINEAR_EQUALITY}});
	negative_testcases = create_testcase(negative_testcases,	{'obj', {{'decouplingcontrol', 'decouplingstrategy'}, GammaDecouplingStrategy.NUMERIC_NONLINEAR_INEQUALITY}});
	negative_testcases = create_testcase(negative_testcases,	{'obj', {{'decouplingcontrol', 'decouplingstrategy'}, GammaDecouplingStrategy.NONE}});
	negative_testcases = create_testcase(negative_testcases,	{'obj', {{'decouplingcontrol', 'decouplingstrategy'}, 'Not a GammaDecouplingStrategy'}});
	negative_testcases = create_testcase(negative_testcases,	{'obj', {{'decouplingcontrol', 'allowoutputdecoupling'}, false}});
	negative_testcases = create_testcase(negative_testcases,	{'sys', {{1, 'C_ref', [], eye(number_states, number_states)}; {1, 'D_ref', [], zeros(number_states, number_states)}; {2, 'C_ref', [], eye(number_states, number_states)}; {2, 'D_ref', [], zeros(number_states, number_states)}}},...
																{'obj', {{'decouplingcontrol', 'tf_structure'}, zeros(3, 3)}});
	negative_testcases = create_testcase(negative_testcases,	{'sys', {{1, 'C_ref', [], zeros(number_states - 2, number_states)}; {1, 'D_ref', [], zeros(number_states - 2, number_states)}; {2, 'C_ref', [], zeros(number_states - 2, number_states)}; {2, 'D_ref', [], zeros(number_states - 2, number_states)}}},...
																{'obj', {{'decouplingcontrol', 'tf_structure'}, zeros(1, 1)}},...
																{'fix', {{3, 1, zeros(number_controls, number_states - 2, 0)}; {4, 1, zeros(number_controls, number_measurements + + number_measurements_xdot + number_states - 2, 0)}}});
	negative_testcases = create_testcase(negative_testcases,	{'sys', {{1, 'C_dot', [], eye(number_states - 2, number_states)}; {2, 'C_dot', [], zeros(number_states - 2, number_states)}}},...
																{'fix', {{2, 1, zeros(number_controls, number_states - 2, 0)}; {4, 1, zeros(number_controls, number_measurements + number_states - 2 + number_references, 0)}}});

	%% test
	ok_positive_testcases = false(size(positive_testcases, 1), 1);
	for ii = 1:size(positive_testcases, 1)
		fprintf('\n========================== \nPositive Testcase %d\n==========================\n', ii);
		test_object.reset();
		[systems, R_fixed_ext, objectiveoptions, solveroptions, descriptor] = test_object.implement_testcase(positive_testcases(ii)); %#ok<ASGLU>
		number_controls_tmp = size(systems(1).B, 2);
		number_measurements_tmp = size(systems(1).C, 1);
		number_measurements_xdot_tmp = size(systems(1).C_dot, 1);
		number_references_tmp = size(systems(1).C_ref, 1);
		if descriptor == -1
			codetotest = '[RKF_fixed, RKF_bounds, valid, output_message] = decoupling_RKF_fixed(systems, R_fixed_ext, objectiveoptions, solveroptions);';
		else
			codetotest = '[RKF_fixed, RKF_bounds, valid, output_message] = decoupling_RKF_fixed(systems, R_fixed_ext, objectiveoptions, solveroptions, descriptor);';
		end
		test.TestSuite.assertNoException(codetotest, 'any', 'decoupling_RKF_fixed must not throw an exception.');
		ok_positive_testcases(ii) = check_outputs(RKF_fixed, RKF_bounds, valid, output_message, number_controls_tmp, number_measurements_tmp, number_measurements_xdot_tmp, number_references_tmp);
	end
	for ii = 1:size(negative_testcases, 1)
		fprintf('\n========================== \nNegative Testcase %d\n==========================\n', ii);
		test_object.reset();
		[systems, R_fixed_ext, objectiveoptions, solveroptions, descriptor] = test_object.implement_testcase(negative_testcases(ii)); %#ok<ASGLU>
		codetotest = '[RKF_fixed, RKF_bounds, valid, output_message] = decoupling_RKF_fixed(systems, R_fixed_ext, objectiveoptions, solveroptions, descriptor);';
		test.TestSuite.assertException(codetotest, 'any', 'decoupling_RKF_fixed must throw an exception.');
	end
	failed_positive_testcases = find(~ok_positive_testcases);
	if ~isempty(failed_positive_testcases)
		pass = identifier.TestResult.FAILED;
	end
end

function [struct_out] = create_testcase(struct_in, varargin)
	%CREATE_TESTCASE creates and appends structure specifying the characteristic testcase options different from the default options
	%	Input:
	%		struct_in:	structure specifying other testcases. Put [], if this is the first testcase created.
	%		varargin:	cell array(s) of options of the form {identifier-string, cell-array for specifying testcase options}. For precise information, see amend_... functions in decoupling_RKF_fixed_Test_Class
	%	Output:
	%		struct_out:	appended or newly created testcase struct.
	if ~isstruct(struct_in)
		struct_out = struct(...
			'systems',			{},...
			'R_fixed',			{},...
			'objectiveoptions',	{},...
			'solveroptions',	{},...
			'descriptor',		{}...
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
		elseif strcmp(identifier, 'obj')
			struct_out(size_old_struct + 1, 1).objectiveoptions = option;
		elseif strcmp(identifier, 'sol')
			struct_out(size_old_struct + 1, 1).solveroptions = option;
		elseif strcmp(identifier, 'dsc')
			struct_out(size_old_struct + 1, 1).descriptor = option;
		else
			error('control:gamma:RKF_fixed:test', 'Wrong identifier');
		end
	end
end

function [ok] = check_outputs(RKF_fixed, RKF_bounds, valid, output_message, number_controls, number_measurements, number_measurements_xdot, number_references)
	%CHECK_OUTPUTS checks dimensions and correctness of output arguments of decoupling_RKF_fixed
	%	Input:
	%		RKF_fixed:					cell array of RKF constraints
	% 		RKF_bounds:					cell array of RKF bounds
	% 		valid:						indicator if constraints were calculated successfully
	% 		output_message:				character string in case constraints were not calculated successfully
	% 		number_controls:			number of controls
	% 		number_measurements:		number of measurements
	% 		number_measurements_xdot:	number of differential measurements
	% 		number_references:			number of references
	%	Output:
	%		ok:							indicator if all ouput arguments have correct dimensions and content
	ok_fixed = check_fixed(RKF_fixed,  number_controls, number_measurements, number_measurements_xdot, number_references);
	ok_bounds = check_fixed(RKF_bounds, number_controls, number_measurements, number_measurements_xdot, number_references);
	ok_message = ischar(output_message);
	ok_valid = islogical(valid);
	if ~valid && isempty(output_message) || valid && ~isempty(output_message)
		ok_message = false;
	end
	ok = ok_fixed && ok_bounds && ok_valid && ok_message;
end

function [ok] = check_fixed(RKF_fixed, number_controls, number_measurements, number_measurements_xdot, number_references)
	%CHECK_FIXED checks dimensions and correctness of RKF_fixed and RKF_bounds
	%	Input:
	%		RKF_fixed:					cell array of RKF constraints (or bounds)
	% 		number_controls:			number of controls
	% 		number_measurements:		number of measurements
	% 		number_measurements_xdot:	number of differential measurements
	% 		number_references:			number of references
	%	Output:
	%		ok:							indicator if RKF_fixed (or bounds) has correct dimensions and content
	ok = false;
	if ~iscell(RKF_fixed)
		return;
	end
	if any(size(RKF_fixed) ~= [4, 1])
		return;
	end
	sizes_RKF_dim2 = [
		number_measurements;
		number_measurements_xdot;
		number_references;
		number_measurements + number_measurements_xdot + number_references
	];
	for jj = 1:4
		fixed = RKF_fixed{jj};
		if ~iscell(fixed)
			return;
		end
		if any(size(fixed) ~= [1, 2])
			return;
		end
		A = fixed{1};
		b = fixed{2};
		if ~isnumeric(A) || ~isnumeric(b)
			return;
		end
		size_A = [
			size(A, 1),	size(A, 2),	size(A, 3)
		];
		size_b = [
			size(b, 1),	size(b, 2),	size(b, 3)
		];
		if any(size_A(1:2) ~= [number_controls, sizes_RKF_dim2(jj)])
			return;
		end
		if any(size_b ~= [size_A(3), 1, 1])
			return;
		end
		if any(isinf(A(:))) || any(isnan(A(:))) || any(~isa(A(:), 'double')) || any(isinf(b(:))) || any(isnan(b(:))) || any(~isa(b(:), 'double'))
			return;
		end
	end
	ok = true;
end
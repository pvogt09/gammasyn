function [pass, failed_positive_testcases] = decoupling_RKF_fixed_Test(~)
	%DECOUPLING_RKF_FIXED_TEST test cases for checking decoupling constraints function
	%	Input:
	%		silent:						idicator, if information should be output
	%	Output:
	%		pass:						indicator, if test was passed
	%		failed_positive_testcases:	testcases which returned wrong output arguments

	%% setup
	pass = identifier.TestResult.PASSED;
	decoupling_RKF_fixed = get_private_function(realpath(fullfile(mfilename('fullpath'), '..', '..', 'private')), 'decoupling_RKF_fixed'); %#ok<NASGU>

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
		'E',		{eye(number_states), eye(number_states)},...
		'A',		{eye(number_states), eye(number_states)},...
		'B',		{randi(10, [number_states, number_controls]), randi(10, [number_states, number_controls])},...
		'C',		{randi(10, [number_measurements, number_states]), randi(10, [number_measurements, number_states])},...
		'C_ref',	{randi(10, [number_references, number_states]), randi(10, [number_references, number_states])},...
		'C_dot',	{randi(10, [number_measurements_xdot, number_states]), randi(10, [number_measurements_xdot, number_states])},...
		'D',		{zeros(number_measurements, number_controls), zeros(number_measurements, number_controls)},...
		'D_ref',	{zeros(number_references, number_controls), zeros(number_references, number_controls)}...
	);
	tf_structure = zeros(number_measurements, number_references);
	tf_structure(1:number_references + 1:number_measurements*number_references) = nan;
	objectiveoptions_default = struct(...
		'decouplingcontrol', struct(...
			'solvesymbolic',				true,...
			'round_equations_to_digits',	4,...
			'decouplingstrategy',			GammaDecouplingStrategy.EXACT,...
			'allowoutputdecoupling',		true,...
			'tf_structure',					tf_structure,...
			'tolerance_decoupling',			0.4,...
			'tolerance_prefilter',			0.6...
		)...
	);
	solveroptions_default.Display = 'off'; %'iter-detailed';
	descriptor_default = false;
	
	test_object = control.design.gamma.test.decoupling_RKF_fixed_Test_Class(systems_default, R_fixed_ext_default, objectiveoptions_default, solveroptions_default, descriptor_default);

	%% testcases
	positive_testcases = struct(...
		'systems',				{ % syntax: which system? which matrix? which element(s)? which value?
									{{1, 'E', 1, 1}; {2, 'E', 1, 3}}
									{}
								},...
		'R_fixed',				{ % syntax: which constraints? which matrix? which value?
									{}
									{}
								},...
		'objectiveoptions',		{ % syntax: which option(s)? which value?
									{}
									{}
								},...
		'solveroptions',		{ % syntax: which option? which value?
									{}
									{}
								},...
		'descriptor',			{ % syntax: which value?
									{}
									{}
								}...
	);
	negative_testcases = struct(...
		'systems',				{ % syntax: which system? which matrix? which element(s)? which value?
									{{1, 'E', 1, 0}; {2, 'E', 1, 3}}
								},...
		'R_fixed',				{ % syntax: which constraints? which matrix? which value?
									{}
								},...
		'objectiveoptions',		{ % syntax: which option(s)? which value?
									{}
								},...
		'solveroptions',		{ % syntax: which option? which value?
									{}
								},...
		'descriptor',			{ % syntax: which value?
									{true}
								}...
	);

	%% test
	ok_positive_testcases = false(size(positive_testcases, 1), 1);
	for ii = 1:size(positive_testcases, 1)
		test_object.reset();
		[systems, R_fixed_ext, objectiveoptions, solveroptions, descriptor] = test_object.implement_testcase(positive_testcases(ii)); %#ok<ASGLU>
		codetotest = '[RKF_fixed, RKF_bounds, valid, output_message] = decoupling_RKF_fixed(systems, R_fixed_ext, objectiveoptions, solveroptions, descriptor);';
		test.TestSuite.assertNoException(codetotest, 'any', 'decoupling_RKF_fixed must not throw an exception.');
		ok_positive_testcases(ii) = check_outputs(RKF_fixed, RKF_bounds, valid, output_message, number_controls, number_measurements, number_measurements_xdot, number_references);
	end
	for ii = 1:size(negative_testcases, 1)
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

function ok = check_outputs(RKF_fixed, RKF_bounds, valid, output_message, number_controls, number_measurements, number_measurements_xdot, number_references)
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

function ok = check_fixed(RKF_fixed, number_controls, number_measurements, number_measurements_xdot, number_references)
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
		size_A = size(A, 1, 2, 3);
		size_b = size(b, 1, 2, 3);
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
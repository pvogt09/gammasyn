function [pass] = decoupling_RKF_fixed_Test(~)
	%DECOUPLING_RKF_FIXEDTEST test cases for checking decoupling constraints function
	%	Input:
	%		silent:	idicator, if information should be output
	%	Output:
	%		pass:	indicator, if test was passed
	pass = identifier.TestResult.PASSED;
	decoupling_RKF_fixed = get_private_function(realpath(fullfile(mfilename('fullpath'), '..', '..', 'private')), 'decoupling_RKF_fixed');
	
	number_controls = 2;
	number_measurements = 2;
	number_states = 3;
	number_references = 2;
	
	R_fixed_ext_standard = {
		{zeros(number_controls, number_measurements, 0), zeros(0, 1)},...
		{zeros(number_controls, number_measurements, 0), zeros(0, 1)},...
		{zeros(number_controls, number_references, 0), zeros(0, 1)},...
		{zeros(number_controls, (number_measurements*2 + number_references), 0), zeros(0, 1)}
	};
	
	tf_structure = NaN(number_references);
	
	systems_standard = struct(...
		'E',		{eye(number_states), eye(number_states)},...
		'A',		{eye(number_states), eye(number_states)},...
		'B',		{randi(10, [number_states, number_controls]), randi(10, [number_states, number_controls])},...
		'C',		{randi(10, [number_measurements, number_states]), randi(10, [number_measurements, number_states])},...
		'C_ref',	{randi(10, [number_references, number_states]), randi(10, [number_references, number_states])},...
		'C_dot',	{randi(10, [number_measurements, number_states]), randi(10, [number_measurements, number_states])},...
		'D',		{zeros(number_measurements, number_controls), zeros(number_measurements, number_controls)},...
		'D_ref',	{randi(10, [number_references, number_controls]), randi(10, [number_references, number_controls])}...
	);
	objectiveoptions_standard = struct(...
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
	solveroptions_standard.Display = 'off'; %'iter-detailed';
	descriptor_standard = false;
	
	positive_testcases = {
		{{'tolerance_prefilter', -1}, {'tolerance_decoupling', -1}}
		{{'andere options', -1}, {'noch andere options', -1}}
		{{'nur eine option', -1}}
	};
	negative_testcases = {
		{{'systems(1).E(1)', 0}}
	};
	% braucht noch eine bessere Art und Weise die Testcases zu codieren, sodass man die Optionen einfach umsetzen kann.

	for ii = 1:numel(positive_testcases, 1)
		systems = systems_standard;
		R_fixed_ext = R_fixed_ext_standard;
		objectiveoptions = objectiveoptions_standard;
		solveroptions = solveroptions_standard;
		descriptor = descriptor_standard;

		testcase = positive_testcases{ii};
		for jj = 1:numel(testcase)
			option = testcase{jj}{1};
			value  = testcase{jj}{2};
			assignin_wrapper(option, value);
		end
		codetotest = '[RKF_fixed, RKF_bounds, valid, message] = decoupling_RKF_fixed(systems, R_fixed_ext, objectiveoptions, solveroptions, descriptor);';
		test.TestSuite.assertNoException(codetotest, 'control:decoupling:test', 'decoupling_RKF_fixed must not throw an exception.');
		% TODO: check output arguments
	end

	for ii = 1:numel(positive_testcases, 1)
		systems = systems_standard;
		R_fixed_ext = R_fixed_ext_standard;
		objectiveoptions = objectiveoptions_standard;
		solveroptions = solveroptions_standard;
		descriptor = descriptor_standard;

		testcase = negative_testcases{ii};
		for jj = 1:numel(testcase)
			option = testcase{jj}{1};
			value  = testcase{jj}{2};
			assignin_wrapper(option, value);
		end
		codetotest = '[RKF_fixed, RKF_bounds, valid, message] = decoupling_RKF_fixed(systems, R_fixed_ext, objectiveoptions, solveroptions, descriptor);';
		test.TestSuite.assertException(codetotest, 'control:decoupling:test', 'decoupling_RKF_fixed must throw an exception.');
	end
end

function assignin_wrapper(var, val)
	assignin('caller', var, val);
end
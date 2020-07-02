function [pass] = Checkandtransform_gain_fixed_isforced2zeroTest(~)
	%CHECKANDTRANSFORMARG_GAIN_FIXED_ISFORCED2ZEROTEST test cases for checking isforced2zero function
	%	Input:
	%		silent:	idicator, if information should be output
	%	Output:
	%		pass:	indicator, if test was passed
	pass = identifier.TestResult.PASSED;
	fun = get_private_function(realpath(fullfile(mfilename('fullpath'), '..', '..', 'private')), 'checkandtransform_gain_fixed_forced2zero');
	number_controls = 3;
	number_measurements = 5;
	constraint_system_zero = zeros(number_controls, number_measurements, number_controls*number_measurements);
	constraint_border_zero = zeros(number_controls*number_measurements, 1);
	constraint_number = 1;
	for ii = 1:number_controls
		for jj = 1:number_measurements
			constraint_system_zero(ii, jj, constraint_number) = 1;
			constraint_number = constraint_number + 1;
		end
	end
	noconstraints = {zeros(number_controls, number_measurements, 0), zeros(0, 1)};
	zeroendend = false(number_controls, number_measurements);
	zeroendend(end, end) = true;
	constraint_system_partial = zeros(number_controls, number_measurements, number_controls*number_measurements - number_controls);
	constraint_border_partial = zeros(number_controls*number_measurements - number_controls, 1);
	constraint_number = 1;
	for ii = 1:number_controls
		for jj = 1:number_measurements - 1
			constraint_system_partial(ii, jj, constraint_number) = 1;
			constraint_number = constraint_number + 1;
		end
	end
	
	constraint_system_partial_zero = cat(3, constraint_system_partial, [
		1,	0,	0,	0,	-1;
		0,	0,	0,	0,	0;
		0,	0,	0,	0,	0
	], [
		0,	0,	0,	0,	0;
		1,	0,	0,	0,	-1;
		0,	0,	0,	0,	0
	], [
		0,	0,	0,	0,	0;
		0,	0,	0,	0,	0;
		1,	0,	0,	0,	-1
	]);
	constraint_border_partial_zero = [
		constraint_border_partial;
		0;
		0;
		0
	];
	constraint_border_partial_ones = [
		constraint_border_partial;
		1;
		1;
		1
	];
	constraint_border_partial_nonzero = [
		constraint_border_partial;
		0;
		0;
		0
	];
	constraint_border_partial_nonzero(number_measurements - 1 + 1, 1) = 10;
	zeropartial = false(number_controls, number_measurements);
	zeropartial(:, end) = true;
	combinations = {
		% no constraints
		false,	noconstraints,															true(number_controls, number_measurements);
		false,	noconstraints,															false(number_controls, number_measurements);
		false,	noconstraints,															zeroendend;
		false,	noconstraints,															~zeroendend;
		% all zero constraints
		true,	{constraint_system_zero, constraint_border_zero},						true(number_controls, number_measurements);
		false,	{constraint_system_zero, constraint_border_zero},						false(number_controls, number_measurements);
		true,	{constraint_system_zero, constraint_border_zero},						zeroendend;
		true,	{constraint_system_zero, constraint_border_zero},						~zeroendend;
		% partial zero constraints forcing requested coefficients to zero
		true,	{constraint_system_partial_zero, constraint_border_partial_zero},		zeropartial;
		true,	{constraint_system_partial_zero, constraint_border_partial_zero},		~zeropartial;
		false,	{constraint_system_partial_zero, constraint_border_partial_ones},		zeropartial;
		true,	{constraint_system_partial_zero, constraint_border_partial_ones},		~zeropartial;
		% partial zero constraints forcing at least one requested coefficients to nonzero
		false,	{constraint_system_partial_zero, constraint_border_partial_nonzero},	zeropartial;
		false,	{constraint_system_partial_zero, constraint_border_partial_nonzero},	~zeropartial;
	};
	for ii = 1:size(combinations, 1) %#ok<FORPF> parfor is used in substitution functions
		c = combinations(ii, :);
		test.TestSuite.assertNoException('[isforced2zero] = fun(c{2}{1}, c{2}{2}, number_controls, number_measurements, ''proportional'', c{3});', 'control:outputfeedback:test', 'optimize must not throw an exception for six input arguments.');
		test.TestSuite.assertEqual(isforced2zero, c{1}, 'control:gamma:arguments:test', 'Systems for call as cell array and without cell array must be equal.');
	end
end
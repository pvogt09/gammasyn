function [pass] = GammasynOptionsTest(~)
	%GAMMASYNOPTIONSTEST test cases for checking GammasynOptions
	%	Input:
	%		silent:	idicator, if information should be output
	%	Output:
	%		pass:	indicator, if test was passed
	pass = identifier.TestResult.PASSED;

	number_objects = 6;
	testcases = {
		'usecompiled',					{true, false},														{true(2, 1), 'd', 1, struct()},																								{},									false;
		'numthreads',					{1, 0, -1},															{1.1, ones(2, 1), true, struct(), 'a'},																						{},									false;
		'type',							{GammaJType.getDefaultValue(), [GammaJType.CUBIC;GammaJType.EXP]},	{true, 1, 'd'},																												{},									false;
		'weight',						{1, [-1;4], 1.1},													{true, 's', struct(), {}},																									{},									false;
		'allowvarorder',				{true, false},														{true(2, 1), 's', struct(), {}},																							{},									false;
		'eigenvaluederivative',			{GammaEigenvalueDerivativeType.getDefaultValue()},					{[GammaEigenvalueDerivativeType.getDefaultValue();GammaEigenvalueDerivativeType.getDefaultValue()], true, 1, struct()},		{},									false;
		'eigenvaluefilter',				{GammaEigenvalueFilterType.getDefaultValue(), [GammaEigenvalueFilterType.getDefaultValue();GammaEigenvalueFilterType.getDefaultValue()], 1},						{true, struct()},			{},									false;
		'eigenvalueignoreinf',			{true, false},														{true(2, 1), 'd', 1, struct()},																								{},									false;
		'couplingconditions',			{1, 0, 2},															{-1, NaN, ones(2, 1), true(2, 1), struct(), {}},																			{'couplingcontrol'},				false;
		'couplingstrategy',				{GammaCouplingStrategy.getDefaultValue()},							{[GammaCouplingStrategy.getDefaultValue();GammaCouplingStrategy.getDefaultValue()], -1, ones(2, 1), true, struct(), {}},	{'couplingcontrol'},				false;
		'tolerance_coupling',			{1, 0, 2, 1.1, NaN, [], true},										{-1, ones(2, 1), true(2, 1), struct()},																						{'couplingcontrol'},				false;
		'tolerance_prefilter',			{1, 0, 2, 1.1, NaN, [], true},										{-1, ones(2, 1), true(2, 1), struct()},																						{'couplingcontrol'},				false;
		'solvesymbolic',				{true, false},														{-1, ones(2, 1), true(2, 1), struct(), {}},																					{'couplingcontrol'},				false;
		'round_equations_to_digits',	{1, 0, 2, NaN, -1, []},												{1.1, ones(2, 1), true, struct()},																							{'couplingcontrol'},				false;
		'rho',							{1, 0, -1, NaN, 1.1},												{ones(2, 1), true, struct(), {}},																							{'objective', 'kreisselmeier'},		false;
		'max',							{1, 0, -1, NaN, 1.1},												{ones(2, 1), true, struct(), {}},																							{'objective', 'kreisselmeier'},		false;
		'Q',							{1, 0, -1, NaN, ones(2, 2), 1.1},									{Inf, true, struct(), 1 + 1j},																								{'objective', 'lyapunov'},			false;
		'R',							{1, 0, -1, 1.1},													{NaN, Inf, true, struct()},																									{'objective', 'normgain'},			false;
		'R_shift',						{1, 0, -1, 1.1},													{NaN, Inf, true, struct()},																									{'objective', 'normgain'},			false;
		'K',							{1, 0, -1, 1.1},													{NaN, Inf, true, struct()},																									{'objective', 'normgain'},			false;
		'K_shift',						{1, 0, -1, 1.1},													{NaN, Inf, true, struct()},																									{'objective', 'normgain'},			false;
		'F',							{1, 0, -1, 1.1},													{NaN, Inf, true, struct()},																									{'objective', 'normgain'},			false;
		'F_shift',						{1, 0, -1, 1.1},													{NaN, Inf, true, struct()},																									{'objective', 'normgain'},			false;
		'allownegativeweight',			{true, false},														{true(2, 1), 's', struct(), {}},																							{},									false;
		'strategy',						{GammaSolutionStrategy.getDefaultValue()},							{[GammaSolutionStrategy.getDefaultValue();GammaSolutionStrategy.getDefaultValue()], true, 1, struct()},						{},									false;
		'errorhandler',					{GammaErrorHandler.getDefaultValue()},								{[GammaErrorHandler.getDefaultValue();GammaErrorHandler.getDefaultValue()], true, 1, struct()},								{},									false;
		'errorhandler_function',		{[], @(x) disp(x), {}},												{true, 1, struct()},																										{},									false;
		'usereferences',				{true, false},														{true(2, 1), 's', struct(), {}},																							{'system'},							false;
		'usemeasurements_xdot',			{true, false},														{true(2, 1), 's', struct(), {}},																							{'system'},							false;
		'samples',						{struct('a', 1)},													{true, 1},																													{'system'},							true;
		'Blocks',						{struct('a', 1)},													{true, 1},																													{'system'},							true;
	};
	for ii = 1:size(testcases, 1)
		test.TestSuite.assertNoException('o = control.design.gamma.GammasynOptions();', 'control:gammasyn:options:test', 'constructor must not throw an exception.');
		o = control.design.gamma.GammasynOptions();
		test.TestSuite.assertNoException('struct(o);', 'control:gammasyn:options:test', 'converting to struct must not throw an exception.');
		for jj = 1:numel(testcases{ii, 2})
			if isempty(testcases{ii, 4})
				structarg = struct();
				structarg = isset(structarg, testcases{ii, 1}, testcases{ii, 2}{jj});
				test.TestSuite.assertNoException('o = control.design.gamma.GammasynOptions(structarg);', 'control:gammasyn:options:test', 'constructor must not throw an exception.');
				test.TestSuite.assertEqualNaN(testcases{ii, 2}{jj}, o.(testcases{ii, 1}), 'control:gammasyn:area:test', 'Value must be set correctly.');
			end
			o = control.design.gamma.GammasynOptions();
			test.TestSuite.assertNoException('o.(testcases{ii, 1}) = testcases{ii, 2}{jj};', 'control:gammasyn:options:test', sprintf('setter must not throw an exception for ''%s''.', testcases{ii, 1}));
			test.TestSuite.assertEqualNaN(testcases{ii, 2}{jj}, o.(testcases{ii, 1}), 'control:gammasyn:area:test', 'Value must be set correctly.');
			test.TestSuite.assertNoException(['o.', testcases{ii, 1}, ';'], 'control:gammasyn:options:test', 'getter must not throw an exception.');
			if ~isempty(o.(testcases{ii, 1}))
				test.TestSuite.assertNoException(['o.', testcases{ii, 1}, '(1);'], 'control:gammasyn:options:test', 'subsref must not throw an exception.');
				if ~isempty(testcases{ii, 4})
					test.TestSuite.assertNoException(['o.', strjoin(testcases{ii, 4}, '.'), '.', testcases{ii, 1}, '(1);'], 'control:gammasyn:options:test', 'subsref must not throw an exception.');
				end
			end
			if ~isempty(testcases{ii, 2}{jj}) && ~isfunctionhandle(testcases{ii, 2}{jj})
				test.TestSuite.assertNoException(['o.', testcases{ii, 1}, '(1) = testcases{ii, 2}{jj}(1);'], 'control:gammasyn:options:test', 'subsassgn must not throw an exception.');
				if ~isempty(testcases{ii, 4}) && ~isstruct(testcases{ii, 2}{jj})
					test.TestSuite.assertNoException(['o.', strjoin(testcases{ii, 4}, '.'), '.', testcases{ii, 1}, '(1) = testcases{ii, 2}{jj}(1);'], 'control:gammasyn:options:test', 'subsassgn must not throw an exception.');
				end
			end
		end
		for jj = 1:numel(testcases{ii, 3})
			o = control.design.gamma.GammasynOptions();
			test.TestSuite.assertException('o.(testcases{ii, 1}) = testcases{ii, 3}{jj};', 'any', 'control:gammasyn:options:test', sprintf('setter must throw an exception for ''%s''.', testcases{ii, 1}));
		end

		o = control.design.gamma.GammasynOptions();
		for oo = 1:number_objects - 1
			o = [o, control.design.gamma.GammasynOptions()];
		end
		test.TestSuite.assertNoException('struct(o);', 'control:gammasyn:options:test', 'converting to struct must not throw an exception.');
		test.TestSuite.assertSameSize(o, struct(o), 'control:gammasyn:options:test', 'Structure representation must have same size as object.');
		for jj = 1:numel(testcases{ii, 2})
			o = control.design.gamma.GammasynOptions();
			for oo = 1:number_objects - 1
				o = [o, control.design.gamma.GammasynOptions()];
			end
			val = repmat({testcases{ii, 2}{jj}}, size(o));
			test.TestSuite.assertNoException('[o.(testcases{ii, 1})] = val{:};', 'control:gammasyn:options:test', sprintf('setter must not throw an exception for ''%s''.', testcases{ii, 1}));
			for kk = 1:number_objects
				test.TestSuite.assertEqualNaN(testcases{ii, 2}{jj}, o(kk).(testcases{ii, 1}), 'control:gammasyn:area:test', 'Value must be set correctly.');
			end
		end
	end
end
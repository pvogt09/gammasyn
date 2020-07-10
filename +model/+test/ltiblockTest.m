function [pass] = ltiblockTest(~)
	%LTIBLOCKTEST test cases for ltiblock conversion
	%	Input:
	%		silent:	idicator, if information should be output
	%	Output:
	%		pass:	indicator, if test was passed
	pass = identifier.TestResult.PASSED;

	testcases = {
		ltiblock.gain('gain', 2, 3),			{},							{};
		ltiblock.pid('P', 'P'),					{'IFormula', 'DFormula'},	repmat({{'ForwardEuler', 'BackwardEuler', 'Trapezoidal'}}, 1, 2);
		ltiblock.pid('P', 'P', 0.001),			{'IFormula', 'DFormula'},	repmat({{'ForwardEuler', 'BackwardEuler', 'Trapezoidal'}}, 1, 2);
		ltiblock.pid('PI', 'PI'),				{'IFormula', 'DFormula'},	repmat({{'ForwardEuler', 'BackwardEuler', 'Trapezoidal'}}, 1, 2);
		ltiblock.pid('PI', 'PI', 0.001),		{'IFormula', 'DFormula'},	repmat({{'ForwardEuler', 'BackwardEuler', 'Trapezoidal'}}, 1, 2);
		ltiblock.pid('PD', 'PD'),				{'IFormula', 'DFormula'},	repmat({{'ForwardEuler', 'BackwardEuler', 'Trapezoidal'}}, 1, 2);
		ltiblock.pid('PD', 'PD', 0.001),		{'IFormula', 'DFormula'},	repmat({{'ForwardEuler', 'BackwardEuler', 'Trapezoidal'}}, 1, 2);
		ltiblock.pid('PID', 'PID'),				{'IFormula', 'DFormula'},	repmat({{'ForwardEuler', 'BackwardEuler', 'Trapezoidal'}}, 1, 2);
		ltiblock.pid('PID', 'PID', 0.001),		{'IFormula', 'DFormula'},	repmat({{'ForwardEuler', 'BackwardEuler', 'Trapezoidal'}}, 1, 2);
% 		ltiblock.pid2('P2', 'P'),				{'IFormula', 'DFormula'},	repmat({{'ForwardEuler', 'BackwardEuler', 'Trapezoidal'}}, 1, 2);
% 		ltiblock.pid2('P2', 'P', 0.001),		{'IFormula', 'DFormula'},	repmat({{'ForwardEuler', 'BackwardEuler', 'Trapezoidal'}}, 1, 2);
% 		ltiblock.pid2('PI2', 'PI'),				{'IFormula', 'DFormula'},	repmat({{'ForwardEuler', 'BackwardEuler', 'Trapezoidal'}}, 1, 2);
% 		ltiblock.pid2('PI2', 'PI', 0.001),		{'IFormula', 'DFormula'},	repmat({{'ForwardEuler', 'BackwardEuler', 'Trapezoidal'}}, 1, 2);
% 		ltiblock.pid2('PD2', 'PD'),				{'IFormula', 'DFormula'},	repmat({{'ForwardEuler', 'BackwardEuler', 'Trapezoidal'}}, 1, 2);
% 		ltiblock.pid2('PD2', 'PD', 0.001),		{'IFormula', 'DFormula'},	repmat({{'ForwardEuler', 'BackwardEuler', 'Trapezoidal'}}, 1, 2);
% 		ltiblock.pid2('PID2', 'PID'),			{'IFormula', 'DFormula'},	repmat({{'ForwardEuler', 'BackwardEuler', 'Trapezoidal'}}, 1, 2);
% 		ltiblock.pid2('PID2', 'PID', 0.001),	{'IFormula', 'DFormula'},	repmat({{'ForwardEuler', 'BackwardEuler', 'Trapezoidal'}}, 1, 2);
		ltiblock.ss('ss', 4, 2, 3),				{},							{};
		ltiblock.ss('ss', 4, 2, 3, 0.001),		{},							{};
		ltiblock.tf('tf', 2, 4),				{},							{};
		ltiblock.tf('tf', 2, 4, 0.001),			{},							{}
	};
	if matlab.Version.CURRENT >= matlab.Version.R2011A
		testcases = [
			testcases;
			{
				tunableGain('gain', 2, 3),			{},							{};
				tunablePID('P', 'P'),				{'IFormula', 'DFormula'},	repmat({{'ForwardEuler', 'BackwardEuler', 'Trapezoidal'}}, 1, 2);
				tunablePID('P', 'P', 0.001),		{'IFormula', 'DFormula'},	repmat({{'ForwardEuler', 'BackwardEuler', 'Trapezoidal'}}, 1, 2);
				tunablePID('PI', 'PI'),				{'IFormula', 'DFormula'},	repmat({{'ForwardEuler', 'BackwardEuler', 'Trapezoidal'}}, 1, 2);
				tunablePID('PI', 'PI', 0.001),		{'IFormula', 'DFormula'},	repmat({{'ForwardEuler', 'BackwardEuler', 'Trapezoidal'}}, 1, 2);
				tunablePID('PD', 'PD'),				{'IFormula', 'DFormula'},	repmat({{'ForwardEuler', 'BackwardEuler', 'Trapezoidal'}}, 1, 2);
				tunablePID('PD', 'PD', 0.001),		{'IFormula', 'DFormula'},	repmat({{'ForwardEuler', 'BackwardEuler', 'Trapezoidal'}}, 1, 2);
				tunablePID('PID', 'PID'),			{'IFormula', 'DFormula'},	repmat({{'ForwardEuler', 'BackwardEuler', 'Trapezoidal'}}, 1, 2);
				tunablePID('PID', 'PID', 0.001),	{'IFormula', 'DFormula'},	repmat({{'ForwardEuler', 'BackwardEuler', 'Trapezoidal'}}, 1, 2);
% 				tunablePID2('P2', 'P'),				{'IFormula', 'DFormula'},	repmat({{'ForwardEuler', 'BackwardEuler', 'Trapezoidal'}}, 1, 2);
% 				tunablePID2('P2', 'P', 0.001),		{'IFormula', 'DFormula'},	repmat({{'ForwardEuler', 'BackwardEuler', 'Trapezoidal'}}, 1, 2);
% 				tunablePID2('PI2', 'PI'),			{'IFormula', 'DFormula'},	repmat({{'ForwardEuler', 'BackwardEuler', 'Trapezoidal'}}, 1, 2);
% 				tunablePID2('PI2', 'PI', 0.001),	{'IFormula', 'DFormula'},	repmat({{'ForwardEuler', 'BackwardEuler', 'Trapezoidal'}}, 1, 2);
% 				tunablePID2('PD2', 'PD'),			{'IFormula', 'DFormula'},	repmat({{'ForwardEuler', 'BackwardEuler', 'Trapezoidal'}}, 1, 2);
% 				tunablePID2('PD2', 'PD', 0.001),	{'IFormula', 'DFormula'},	repmat({{'ForwardEuler', 'BackwardEuler', 'Trapezoidal'}}, 1, 2);
% 				tunablePID2('PID2', 'PID'),			{'IFormula', 'DFormula'},	repmat({{'ForwardEuler', 'BackwardEuler', 'Trapezoidal'}}, 1, 2);
% 				tunablePID2('PID2', 'PID', 0.001),	{'IFormula', 'DFormula'},	repmat({{'ForwardEuler', 'BackwardEuler', 'Trapezoidal'}}, 1, 2);
				tunableSS('ss', 4, 2, 3),			{},							{};
				tunableSS('ss', 4, 2, 3, 0.001),	{},							{};
				tunableTF('tf', 2, 4),				{},							{};
				tunableTF('tf', 2, 4, 0.001),		{},							{}
			}
		];
	end
	for ii = 1:size(testcases, 1)
		if ~isempty(testcases{ii, 2})
			for jj = 1:numel(testcases{ii, 2})
				for kk = 1:numel(testcases{ii, 3}{jj})
					testcases{ii, 1}.(testcases{ii, 2}{jj}) = testcases{ii, 3}{jj}{kk};
					test.TestSuite.assertNoException('o = model.ltiblock2ss(testcases{ii, 1});', 'model:lti:test', 'conversion to ltiblock.ss must not throw an exception.');
				end
			end
		else
			test.TestSuite.assertNoException('o = model.ltiblock2ss(testcases{ii, 1});', 'model:lti:test', 'conversion to ltiblock.ss must not throw an exception.');
		end
	end
end
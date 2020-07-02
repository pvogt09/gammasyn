function [pass] = OptionTest(~)
	%OPTIONTEST test cases for checking options for correct argument handling
	%	Input:
	%		silent:	idicator, if information should be output
	%	Output:
	%		pass:	indicator, if test was passed
	pass = identifier.TestResult.PASSED;
	solvers = enumeration('optimization.solver.Optimizer');
	options = {
		optimset('fminsearch');
		struct('Display', 'iter-detailed')
	};
	if configuration.optimization.hasoptimization()
		if matlab.Version.CURRENT >= matlab.Version.R2013A
			options = [
				options;
				{
					optimoptions(@fmincon);
					optimoptions(@fminunc);
					%optimoptions(@lsqnonneg);
					optimoptions(@linprog);
					optimoptions(@quadprog);
				}
			];
		else
			options = [
				options;
				{
					optimset('fmincon');
					optimset('fminunc');
					%optimset('lsqnonneg');
					optimset('linprog');
					optimset('quadprog');
				}
			];
		end
	end
	if configuration.optimization.hasglobaloptimization()
		options = [
			options;
			{
				saoptimset();
				gaoptimset();
				psoptimset()
			}
		];
	end
	TolFun = 1E-3;
	TolX = 1E-3;
	TolCon = 1E-3;
	MaxFunEvals = 50;
	MaxIter = 20;
	MaxSQPIter = 500;
	GradObj = false;
	GradConstr = false;
	GradCheck = false;
	FunValCheck = false;
	FinDiffType = 'forward';
	Diagnostics = false;
	Display = 'final-detailed';
	MaxTime = 1;
	for ii = 1:size(solvers, 1) %#ok<FORPF> parfor is used in optimization functions and parfor here would prevent parfor in the optimization functions from beeing parallelized
		algorithms = solvers(ii, 1).getAlgorithmChoices();
		if isempty(algorithms)
			algorithms = {'default'};
		end
		problemtype = solvers(ii, 1).getSupportedProblemTypes();
		for jj = 1:size(options, 1)
			for kk = 1:size(problemtype, 1)
				option = optimization.options.OptionFactory.instance.options(solvers(ii, 1),...
					'ProblemType',					problemtype(kk, 1),...
					'Algorithm',					algorithms{1, 1},...
					'FunctionTolerance',			TolFun,...
					'StepTolerance',				TolX,...
					'ConstraintTolerance',			TolCon,...
					'MaxFunctionEvaluations',		MaxFunEvals,...
					'MaxIterations',				MaxIter,...
					'MaxSQPIter',					MaxSQPIter,...
					'SpecifyObjectiveGradient',		GradObj,...
					'SpecifyConstraintGradient',	GradConstr,...
					'CheckGradients',				GradCheck,...
					'FunValCheck',					FunValCheck,...
					'FiniteDifferenceType',			FinDiffType,...
					'Diagnostics',					Diagnostics,...
					'Display',						Display,...
					'MaxTime',						MaxTime...
				);
				option.NumberVariables = 15;
				option.NumberConstraintsInequality = 100;
				option.NumberConstraintsEquality = 10;
				option.NumberConstraintsBounds = 0;
				if ~solvers(ii, 1).needsGlobalOptimizationToolbox() || any(solvers(ii, 1) == [
					optimization.solver.Optimizer.FMINCONGLOBAL;
					optimization.solver.Optimizer.FMINUNCGLOBAL;
					optimization.solver.Optimizer.FMINIMAX
				]) || (solvers(ii, 1).needsGlobalOptimizationToolbox() && configuration.optimization.hasglobaloptimization())
					test.TestSuite.assertNoException('option.getpreferred();', 'control:optimization:option:test', 'getpreferred must not throw an exception.');
					test.TestSuite.assertEqual(option.FunctionTolerance, TolFun, 'control:optimization:option:test');
					test.TestSuite.assertEqual(option.StepTolerance, TolX, 'control:optimization:option:test');
					test.TestSuite.assertEqual(option.ConstraintTolerance, TolCon, 'control:optimization:option:test');
					test.TestSuite.assertEqual(option.MaxFunctionEvaluations, MaxFunEvals, 'control:optimization:option:test');
					test.TestSuite.assertEqual(option.MaxIterations, MaxIter, 'control:optimization:option:test');
					test.TestSuite.assertEqual(option.MaxSQPIter, MaxSQPIter, 'control:optimization:option:test');
					test.TestSuite.assertEqual(option.SpecifyObjectiveGradient, GradObj, 'control:optimization:option:test');
					test.TestSuite.assertEqual(option.SpecifyConstraintGradient, GradConstr, 'control:optimization:option:test');
					test.TestSuite.assertEqual(option.CheckGradients, GradCheck, 'control:optimization:option:test');
					test.TestSuite.assertEqual(option.FunValCheck, FunValCheck, 'control:optimization:option:test');
					test.TestSuite.assertEqual(option.FiniteDifferenceType, FinDiffType, 'control:optimization:option:test');
					test.TestSuite.assertEqual(option.Diagnostics, Diagnostics, 'control:optimization:option:test');
					test.TestSuite.assertEqual(option.Display, Display, 'control:optimization:option:test');
					test.TestSuite.assertEqual(option.MaxTime, MaxTime, 'control:optimization:option:test');
				elseif solvers(ii, 1) == optimization.solver.Optimizer.PARTICLESWARM
					if matlab.Version.CURRENT >= matlab.Version.R2013A
						test.TestSuite.assertException('option.getpreferred();', 'optimlib:options:createSolverOptions:InvalidSolver', 'control:optimization:option:test', 'getpreferred must not throw an exception.');
					else
						test.TestSuite.assertException('option.getpreferred();', 'optimization:options:version', 'control:optimization:option:test', 'getpreferred must not throw an exception.');
					end
				else
					test.TestSuite.assertException('option.getpreferred();', 'optimization:options:version', 'control:optimization:option:test', 'getpreferred must not throw an exception.');
				end
				test.TestSuite.assertNoException('optioncopy = optimization.options.OptionFactory.instance.options(solvers(ii, 1), options{jj, 1});', 'control:optimization:option:test', 'creating an option object from builtin optimization option. types must not throw an error');
				optioncopy.NumberVariables = 15;
				optioncopy.NumberConstraintsInequality = 100;
				optioncopy.NumberConstraintsEquality = 10;
				optioncopy.NumberConstraintsBounds = 0;
				if ~solvers(ii, 1).needsGlobalOptimizationToolbox() || any(solvers(ii, 1) == [
					optimization.solver.Optimizer.FMINCONGLOBAL;
					optimization.solver.Optimizer.FMINUNCGLOBAL;
					optimization.solver.Optimizer.FMINIMAX
				]) || (solvers(ii, 1).needsGlobalOptimizationToolbox() && configuration.optimization.hasglobaloptimization())
					test.TestSuite.assertNoException('opt = option.getpreferred();', 'control:optimization:option:test', 'getpreferred must not throw an exception.');
					test.TestSuite.assertNoException('optionpref = optimization.options.OptionFactory.instance.options(solvers(ii, 1), opt);', 'control:optimization:option:test', 'Converting preferred options back to an object must not throw an exception.');
					optionpref.NumberVariables = 15;
					optionpref.NumberConstraintsInequality = 100;
					optionpref.NumberConstraintsEquality = 10;
					optionpref.NumberConstraintsBounds = 0;
					if ~isempty(optionpref.FunctionTolerance)
						test.TestSuite.assertEqual(optionpref.FunctionTolerance, TolFun, 'control:optimization:option:test');
					end
					if ~isempty(optionpref.StepTolerance)
						test.TestSuite.assertEqual(optionpref.StepTolerance, TolX, 'control:optimization:option:test');
					end
					if ~isempty(optionpref.ConstraintTolerance)
						test.TestSuite.assertEqual(optionpref.ConstraintTolerance, TolCon, 'control:optimization:option:test');
					end
					if ~isempty(optionpref.MaxFunctionEvaluations)
						test.TestSuite.assertEqual(optionpref.MaxFunctionEvaluations, MaxFunEvals, 'control:optimization:option:test');
					end
					if ~isempty(optionpref.MaxIterations)
						test.TestSuite.assertEqual(optionpref.MaxIterations, MaxIter, 'control:optimization:option:test');
					end
					if ~isempty(optionpref.MaxSQPIter)
						test.TestSuite.assertEqual(optionpref.MaxSQPIter, MaxSQPIter, 'control:optimization:option:test');
					end
					if ~isempty(optionpref.SpecifyObjectiveGradient)
						test.TestSuite.assertEqual(optionpref.SpecifyObjectiveGradient, GradObj, 'control:optimization:option:test');
					end
					if ~isempty(optionpref.SpecifyConstraintGradient)
						test.TestSuite.assertEqual(optionpref.SpecifyConstraintGradient, GradConstr, 'control:optimization:option:test');
					end
					if ~isempty(optionpref.CheckGradients)
						test.TestSuite.assertEqual(optionpref.CheckGradients, GradCheck, 'control:optimization:option:test');
					end
					if ~isempty(optionpref.FunValCheck)
						test.TestSuite.assertEqual(optionpref.FunValCheck, FunValCheck, 'control:optimization:option:test');
					end
					if ~isempty(optionpref.FiniteDifferenceType)
						test.TestSuite.assertEqual(optionpref.FiniteDifferenceType, FinDiffType, 'control:optimization:option:test');
					end
					if ~isempty(optionpref.Diagnostics)
						test.TestSuite.assertEqual(optionpref.Diagnostics, Diagnostics, 'control:optimization:option:test');
					end
					%if ~isempty(optionpref.Display)
						%test.TestSuite.assertEqual(optionpref.Display, Display, 'control:optimization:option:test');
					%end
					if ~isempty(optionpref.MaxTime)% MaxTime is ignored by most of the algorithms that use optimoptions
						test.TestSuite.assertEqual(optionpref.MaxTime, MaxTime, 'control:optimization:option:test');
					end
				end
			end
		end
	end
end
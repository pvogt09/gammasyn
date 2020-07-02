function [solveroptions] = checkandtransform_solveroptions(solveroptions)
	%CHECKANDTRANSFORM_SOLVEROPTIONS parse options for optimization supplied by user and set reasonable initial values
	%	Input:
	%		solveroptions:	Options, optimoptions or structure with options for optimization
	%	Output:
	%		solveroptions:	Options object with options for optimization
	defaultsolver = optimization.solver.Optimizer.getDefaultValue();
	if nargin <= 0 || isempty(solveroptions)
		solveroptions = defaultsolver;
	end
	if isa(solveroptions, 'optimization.solver.Optimizer')
		defaultsolver = solveroptions;
	end
	if ~isa(solveroptions, 'optimization.options.Options')
		if isstruct(solveroptions)
			solveroptions = optimization.options.OptionFactory.instance.options(defaultsolver, solveroptions);
		elseif isa(solveroptions, 'optim.options.SolverOptions')
			solveroptions = optimization.options.OptionFactory.instance.options(defaultsolver, solveroptions);
		elseif isa(solveroptions, 'optimization.solver.Optimizer')
			solveroptions = optimization.options.OptionFactory.instance.options(defaultsolver,...
				'Algorithm',					defaultsolver.getDefaultAlgorithm(),...
				'Display',						'iter-detailed',...
				'FunctionTolerance',			1E-10,...
				'StepTolerance',				1E-10,...
				'ConstraintTolerance',			1E-7,...
				'MaxFunctionEvaluations',		25E3,...
				'MaxIterations',				5E3,...
				'SpecifyObjectiveGradient',		true,...
				'SpecifyConstraintGradient',	true,...
				'CheckGradients',				false,...
				'FunValCheck',					false,...
				'FiniteDifferenceType',			'forward',...
				'Diagnostics',					false,...
				'PlotFcn',						{
					%@optimplotx;
					%@optimplotfunccount;
					%@optimplotfval;
					%@optimplotstepsize;
					%@optimplotfirstorderopt
			});
			if ~isempty(solveroptions.ProblemType) && solveroptions.ProblemType == optimization.options.ProblemType.UNCONSTRAINED
				solveroptions.FunctionTolerance = 1E-35;
				solveroptions.StepTolerance = 1E-13;
				solveroptions.MaxFunctionEvaluations = 5E3;
				solveroptions.MaxIterations = 5E3;
			end
		else
			error('control:design:gamma', 'Options must be of type optimization.options.Options or optimoptions or optimset.');
		end
	end
end
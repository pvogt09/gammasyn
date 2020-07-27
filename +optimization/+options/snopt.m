classdef snopt < optimization.options.Options
	%SNOPT solver options for snopt

	methods(Access=protected)
		function [names] = optimoptionsnames(~)
			%OPTIMOPTIONSNAMES mapping from properties to optimoptions names
			%	Input:
			%		this:	instance
			%	Output:
			%		names:	mapping of names
			names = {
				%'Algorithm',					'Algorithm';
				%'CheckGradients',				'DerivativeCheck';
				'ConstraintTolerance',			'TolCon';
				%'Diagnostics',					'Diagnostics';
				%'DiffMaxChange',				'DiffMaxChange';
				%'DiffMinChange',				'DiffMinChange';
				'Display',						'Display';
				%'FiniteDifferenceStepSize',	'FinDiffRelSize';
				%'FiniteDifferenceType',		'FinDiffType';
				'FunctionTolerance',			'TolFun';
				%'FunValCheck',					'FunValCheck';
				%'HessianApproximation',		'Hessian';
				%'HessianFcn',					'HessFcn';
				%'HessianMultiplyFcn',			'HessMult';
				%'HessianPattern',				'HessPattern';
				%'HessianUpdate',				'HessUpdate';
				%'HonorBounds',					'AlwaysHonorConstraints';
				%'InitBarrierParam',			'InitBarrierParam';
				%'InitDamping',					'InitDamping';
				%'InitTrustRegionRadius',		'InitTrustRegionRadius';
				%'JacobianMultiplyFcn',			'JacobianMultiplyFcn';
				%'JacobianPattern',				'JacobPattern';
				%'MaxFunctionEvaluations',		'MaxFunEvals';
				'MaxIterations',				'MaxIter';
				%'MaxPCGIter',					'MaxPCGIter';
				%'MaxProjCGIter',				'MaxProjCGIter';
				'MaxSQPIter',					'MaxSQPIter';
				'MaxTime',						'MaxTime';
				'ObjectiveLimit',				'ObjectiveLimit';
				'OptimalityTolerance',			'TolFun';
				%'OutputFcn',					'OutputFcn';
				%'PlotFcn',						'PlotFcns';
				%'PrecondBandWidth',			'PrecondBandWidth';
				%'RelLineSearchBound',			'RelLineSrchBnd';
				%'RelLineSearchBoundDuration',	'RelLineSrchBndDuration';
				%'ScaleProblem',				'ScaleProblem';
				%'SubproblemAlgorithm',			'SubproblemAlgorithm';
				'SpecifyConstraintGradient',	'GradConstr';
				'SpecifyObjectiveGradient',		'GradObj';
				%'StepTolerance',				'TolX';
				'TolConSQP',					'TolConSQP';
				%'TolPCG',						'TolPCG';
				%'TolProjCG',					'TolProjCG';
				%'TolProjCGAbs',				'TolProjCGAbs';
				%'TypicalX',					'TypicalX';
				%'UseParallel',					'UseParallel'
			};
		end

		function [names] = optimsetnames(~)
			%OPTIMSETNAMES mapping from properties to optimset names
			%	Input:
			%		this:	instance
			%	Output:
			%		names:	mapping of names
			names = {
				%'Algorithm',					'Algorithm';
				%'CheckGradients',				'DerivativeCheck';
				'ConstraintTolerance',			'TolCon';
				%'Diagnostics',					'Diagnostics';
				%'DiffMaxChange',				'DiffMaxChange';
				%'DiffMinChange',				'DiffMinChange';
				'Display',						'Display';
				%'FiniteDifferenceStepSize',	'FinDiffRelSize';
				%'FiniteDifferenceType',		'FinDiffType';
				'FunctionTolerance',			'TolFun';
				%'FunValCheck',					'FunValCheck';
				%'HessianApproximation',		'Hessian';
				%'HessianFcn',					'HessFcn';
				%'HessianMultiplyFcn',			'HessMult';
				%'HessianPattern',				'HessPattern';
				%'HessianUpdate',				'HessUpdate';
				%'HonorBounds',					'AlwaysHonorConstraints';
				%'InitBarrierParam',			'InitBarrierParam';
				%'InitDamping',					'InitDamping';
				%'InitTrustRegionRadius',		'InitTrustRegionRadius';
				%'JacobianMultiplyFcn',			'JacobianMultiplyFcn';
				%'JacobianPattern',				'JacobPattern';
				%'MaxFunctionEvaluations',		'MaxFunEvals';
				'MaxIterations',				'MaxIter';
				%'MaxPCGIter',					'MaxPCGIter';
				%'MaxProjCGIter',				'MaxProjCGIter';
				'MaxSQPIter',					'MaxSQPIter';
				'MaxTime',						'MaxTime';
				'ObjectiveLimit',				'ObjectiveLimit';
				'OptimalityTolerance',			'TolFun';
				%'OutputFcn',					'OutputFcn';
				%'PlotFcn',						'PlotFcns';
				%'PrecondBandWidth',			'PrecondBandWidth';
				%'RelLineSearchBound',			'RelLineSrchBnd';
				%'RelLineSearchBoundDuration',	'RelLineSrchBndDuration';
				%'ScaleProblem',				'ScaleProblem';
				%'SubproblemAlgorithm',			'SubproblemAlgorithm';
				'SpecifyConstraintGradient',	'GradConstr';
				'SpecifyObjectiveGradient',		'GradObj';
				%'StepTolerance',				'TolX';
				'TolConSQP',					'TolConSQP';
				%'TolPCG',						'TolPCG';
				%'TolProjCG',					'TolProjCG';
				%'TolProjCGAbs',				'TolProjCGAbs';
				%'TypicalX',					'TypicalX';
				%'UseParallel',					'UseParallel'
			};
		end

		function [names] = structnames(~)
			%SSTRUCTNAMES mapping from properties to struct names
			%	Input:
			%		this:	instance
			%	Output:
			%		names:	mapping of names
			names = {
				%'Algorithm',					'Algorithm';
				%'CheckGradients',				'CheckGradients';
				'ConstraintTolerance',			'feasibility_tolerance';
				%'Diagnostics',					'Diagnostics';
				%'DiffMaxChange',				'DiffMaxChange';
				%'DiffMinChange',				'DiffMinChange';
				'Display',						'Display';
				%'FiniteDifferenceStepSize',	'FiniteDifferenceStepSize';
				%'FiniteDifferenceType',		'FiniteDifferenceType';
				%'FunctionTolerance',			'FunctionTolerance';
				%'FunValCheck',					'FunValCheck';
				%'HessianApproximation',		'HessianApproximation';
				%'HessianFcn',					'HessianFcn';
				%'HessianMultiplyFcn',			'HessianMultiplyFcn';
				%'HessianPattern',				'HessPattern';
				%'HessianUpdate',				'HessUpdate';
				%'HonorBounds',					'HonorBounds';
				%'InitBarrierParam',			'InitBarrierParam';
				%'InitDamping',					'InitDamping';
				%'InitTrustRegionRadius',		'InitTrustRegionRadius';
				%'JacobianMultiplyFcn',			'JacobianMultiplyFcn';
				%'JacobianPattern',				'JacobPattern';
				%'MaxFunctionEvaluations',		'MaxFunctionEvaluations';
				'MaxIterations',				'major_iterations_limit';
				%'MaxPCGIter',					'MaxPCGIter';
				%'MaxProjCGIter',				'MaxProjCGIter';
				'MaxSQPIter',					'iterations_limit';
				'MaxTime',						'time_limit';
				'ObjectiveLimit',				'infinite_bound';
				'OptimalityTolerance',			'major_optimality_tolerance';
				%'OutputFcn',					'OutputFcn';
				%'PlotFcn',						'PlotFcn';
				%'PrecondBandWidth',			'PrecondBandWidth';
				%'RelLineSearchBound',			'RelLineSrchBnd';
				%'RelLineSearchBoundDuration',	'RelLineSrchBndDuration';
				%'ScaleProblem',				'ScaleProblem';
				%'SubproblemAlgorithm',			'SubproblemAlgorithm';
				'SpecifyConstraintGradient',	'derivative_option';
				'SpecifyObjectiveGradient',		'derivative_option';
				%'StepTolerance',				'StepTolerance';
				%'TolConSQP',					'TolConSQP';
				%'TolPCG',						'TolPCG';
				%'TolProjCG',					'TolProjCG';
				%'TolProjCGAbs',				'TolProjCGAbs';
				%'TypicalX',					'TypicalX';
				%'UseParallel',					'UseParallel'
			};
		end
	end

	methods
		function [this] = snopt(varargin)
			%SNOPT create new optimization option set
			%	Input:
			%		varargin:	options to set
			%	Output:
			%		this:		instance
			this@optimization.options.Options(optimization.solver.Optimizer.SNOPT, optimization.options.OptionType.STRUCT, optimization.options.OptionType.STRUCT, false, [
				optimization.options.ProblemType.UNCONSTRAINED;
				optimization.options.ProblemType.CONSTRAINED
			], optimization.options.ProblemType.CONSTRAINED);
			if nargin >= 1
				this.useoptions(varargin{:});
			end
		end

		function [algorithms] = possiblealgorithms(~)
			%POSSIBLEALGORITHMS list with possible algorithms for optimizer
			%	Input:
			%		this:		instance
			%	Output:
			%		algorithms:	possible algorithms
			algorithms = {
				'snopt'
			};
		end

		function [display] = displaymapping(~)
			%DISPLAYMAPPING mapping from optimoptions display names to solver display names
			%	Input:
			%		this:		instance
			%	Output:
			%		display:	mapping of display names
			display = {
				'off',				'off';
				'iter',				'on';
				'iter-detailed',	'on';
				'notify',			'on';
				'notify-detailed',	'on';
				'final',			'on';
				'final-detailed',	'on'
			};
		end

		function [information] = formatOutput(this, errorcode, time, xmin, fmin, nvars, overalliterations, overallfunevals, retries, output, alloutputs)
			%FORMATOUTPUT unify output of optimization
			%	Input:
			%		this:				instance
			%		errorcode:			exit code of optimization
			%		time:				time for optimization
			%		xmin:				solution minimizing the objective function
			%		fmin:				minimum objective function value
			%		nvars:				number of optimization variables
			%		overalliterations:	total number of iterations over all optimization retries
			%		overallfunevals:	total number of function evaluations over all optimization retries
			%		retries:			number of retries
			%		output:				optimization output
			%		alloutputs:			cell array with outputs for different optimization runs to add to the overall optimization output
			%	Output:
			%		information:	unified optimization output
			information = this.OUTPUTPROTOTYPE;
			outputstruct = nargin >= 10 && isstruct(output);
			if outputstruct && isfield(output, 'iter')
				information.iterations		= output.iter;
			end
			if outputstruct && isfield(output, 'funevals')
				information.funcCount		= output.funevals;
			end
			if outputstruct && isfield(output, 'constrviol')
				information.constrviolation = output.constrviol;
			end
			information.algorithm			= 'SNOPT';
			if outputstruct && isfield(output, 'primalinfeas')
				information.firstorderopt	= output.primalinfeas;
			end
			if outputstruct && isfield(output, 'outputstr')
				information.message			= output.outputstr;
			end
			if nargin >= 10
				information.additional			= output;
			end
			information.information.t					= time;
			information.information.Nvar				= nvars;
			if outputstruct && isfield(output, 'iter')
				information.information.iterations		= output.iter;
			end
			if outputstruct && isfield(output, 'funevals')
				information.information.funcCount		= output.funevals;
			end
			information.information.overalliterations	= overalliterations;
			information.information.overallfunCount		= overallfunevals;
			information.information.retries				= retries;
			information.information.feasibility			= errorcode;
			information.information.xmin				= xmin;
			information.information.fmin				= fmin;
			if outputstruct && isfield(output, 'constrviol')
				information.information.constrviol		= output.constrviol;
			end
			if outputstruct && isfield(output, 'infeas')
				information.information.optimality		= output.infeas;
			end
			if outputstruct && isfield(output, 'outputstr')
				information.information.output			= output.outputstr;
			end
			if nargin >= 11
				if iscell(alloutputs) && numel(alloutputs) > 1
					information.runs = alloutputs;
				else
					information.runs = {};
				end
			end
		end
	end

end
classdef minfunc < optimization.options.Options
	%MINFUNC solver options for minfunc

	methods(Access=protected)
		function [names] = optimoptionsnames(~)
			%OPTIMOPTIONSNAMES mapping from properties to optimoptions names
			%	Input:
			%		this:	instance
			%	Output:
			%		names:	mapping of names
			names = {
				'Algorithm',					'Algorithm';
				'CheckGradients',				'DerivativeCheck';
				%'ConstraintTolerance',			'TolCon';
				'Diagnostics',					'Diagnostics';
				'DiffMaxChange',				'DiffMaxChange';
				'DiffMinChange',				'DiffMinChange';
				'Display',						'Display';
				'FiniteDifferenceStepSize',		'FinDiffRelStep';
				'FiniteDifferenceType',			'FinDiffType';
				'FunctionTolerance',			'TolFun';
				'FunValCheck',					'FunValCheck';
				'HessianApproximation',			'Hessian';
				%'HessianFcn',					'HessFcn';
				'HessianMultiplyFcn',			'HessMult';
				'HessianPattern',				'HessPattern';
				'HessianUpdate',				'HessUpdate';
				%'HonorBounds',					'AlwaysHonorConstraints';
				%'InitBarrierParam',			'InitBarrierParam';
				%'InitDamping',					'InitDamping';
				%'InitTrustRegionRadius',		'InitTrustRegionRadius';
				%'JacobianMultiplyFcn',			'JacobianMultiplyFcn';
				%'JacobianPattern',				'JacobPattern';
				'MaxFunctionEvaluations',		'MaxFunEvals';
				'MaxIterations',				'MaxIter';
				'MaxPCGIter',					'MaxPCGIter';
				%'MaxProjCGIter',				'MaxProjCGIter';
				%'MaxSQPIter',					'MaxSQPIter';
				%'MaxTime',						'MaxTime';
				'ObjectiveLimit',				'ObjectiveLimit';
				'OptimalityTolerance',			'TolFun';
				'OutputFcn',					'OutputFcn';
				'PlotFcn',						'PlotFcns';
				'PrecondBandWidth',				'PrecondBandWidth';
				%'RelLineSearchBound',			'RelLineSrchBnd';
				%'RelLineSearchBoundDuration',	'RelLineSrchBndDuration';
				%'ScaleProblem',				'ScaleProblem';
				%'SubproblemAlgorithm',			'SubproblemAlgorithm';
				%'SpecifyConstraintGradient',	'GradConstr';
				'SpecifyObjectiveGradient',		'GradObj';
				'StepTolerance',				'TolX';
				%'TolConSQP',					'TolConSQP';
				'TolPCG',						'TolPCG';
				%'TolProjCG',					'TolProjCG';
				%'TolProjCGAbs',				'TolProjCGAbs';
				'TypicalX',						'TypicalX';
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
				'Algorithm',					'Algorithm';
				'CheckGradients',				'DerivativeCheck';
				%'ConstraintTolerance',			'TolCon';
				'Diagnostics',					'Diagnostics';
				'DiffMaxChange',				'DiffMaxChange';
				'DiffMinChange',				'DiffMinChange';
				'Display',						'Display';
				'FiniteDifferenceStepSize',		'FinDiffRelSize';
				'FiniteDifferenceType',			'FinDiffType';
				'FunctionTolerance',			'TolFun';
				'FunValCheck',					'FunValCheck';
				'HessianApproximation',			'Hessian';
				%'HessianFcn',					'HessFcn';
				'HessianMultiplyFcn',			'HessMult';
				'HessianPattern',				'HessPattern';
				'HessianUpdate',				'HessUpdate';
				%'HonorBounds',					'AlwaysHonorConstraints';
				%'InitBarrierParam',			'InitBarrierParam';
				%'InitDamping',					'InitDamping';
				%'InitTrustRegionRadius',		'InitTrustRegionRadius';
				%'JacobianMultiplyFcn',			'JacobianMultiplyFcn';
				%'JacobianPattern',				'JacobPattern';
				'MaxFunctionEvaluations',		'MaxFunEvals';
				'MaxIterations',				'MaxIter';
				'MaxPCGIter',					'MaxPCGIter';
				%'MaxProjCGIter',				'MaxProjCGIter';
				%'MaxSQPIter',					'MaxSQPIter';
				%'MaxTime',						'MaxTime';
				'ObjectiveLimit',				'ObjectiveLimit';
				'OptimalityTolerance',			'TolFun';
				'OutputFcn',					'OutputFcn';
				'PlotFcn',						'PlotFcns';
				'PrecondBandWidth',				'PrecondBandWidth';
				%'RelLineSearchBound',			'RelLineSrchBnd';
				%'RelLineSearchBoundDuration',	'RelLineSrchBndDuration';
				%'ScaleProblem',				'ScaleProblem';
				%'SubproblemAlgorithm',			'SubproblemAlgorithm';
				%'SpecifyConstraintGradient',	'GradConstr';
				'SpecifyObjectiveGradient',		'GradObj';
				'StepTolerance',				'TolX';
				%'TolConSQP',					'TolConSQP';
				'TolPCG',						'TolPCG';
				%'TolProjCG',					'TolProjCG';
				%'TolProjCGAbs',				'TolProjCGAbs';
				'TypicalX',						'TypicalX';
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
				'Algorithm',					'Algorithm';
				'CheckGradients',				'CheckGradients';
				%'ConstraintTolerance',			'ConstraintTolerance';
				'Diagnostics',					'Diagnostics';
				'DiffMaxChange',				'DiffMaxChange';
				'DiffMinChange',				'DiffMinChange';
				'Display',						'Display';
				'FiniteDifferenceStepSize',		'FiniteDifferenceStepSize';
				'FiniteDifferenceType',			'FiniteDifferenceType';
				'FunctionTolerance',			'FunctionTolerance';
				'FunValCheck',					'FunValCheck';
				'HessianApproximation',			'HessianApproximation';
				%'HessianFcn',					'HessianFcn';
				'HessianMultiplyFcn',			'HessianMultiplyFcn';
				'HessianPattern',				'HessPattern';
				'HessianUpdate',				'HessUpdate';
				%'HonorBounds',					'AlwaysHonorConstraints';
				%'InitBarrierParam',			'InitBarrierParam';
				%'InitDamping',					'InitDamping';
				%'InitTrustRegionRadius',		'InitTrustRegionRadius';
				%'JacobianMultiplyFcn',			'JacobianMultiplyFcn';
				%'JacobianPattern',				'JacobPattern';
				'MaxFunctionEvaluations',		'MaxFunctionEvaluations';
				'MaxIterations',				'MaxIterations';
				%'MaxPCGIter',					'MaxPCGIter';
				'MaxProjCGIter',				'MaxProjCGIter';
				%'MaxSQPIter',					'MaxSQPIter';
				%'MaxTime',						'MaxTime';
				'ObjectiveLimit',				'ObjectiveLimit';
				'OptimalityTolerance',			'OptimalityTolerance';
				'OutputFcn',					'OutputFcn';
				'PlotFcn',						'PlotFcn';
				'PrecondBandWidth',				'PrecondBandWidth';
				%'RelLineSearchBound',			'RelLineSrchBnd';
				%'RelLineSearchBoundDuration',	'RelLineSrchBndDuration';
				%'ScaleProblem',				'ScaleProblem';
				%'SubproblemAlgorithm',			'SubproblemAlgorithm';
				%'SpecifyConstraintGradient',	'SpecifyConstraintGradient';
				'SpecifyObjectiveGradient',		'SpecifyObjectiveGradient';
				'StepTolerance',				'StepTolerance';
				%'TolConSQP',					'TolConSQP';
				'TolPCG',						'TolPCG';
				%'TolProjCG',					'TolProjCG';
				%'TolProjCGAbs',				'TolProjCGAbs';
				'TypicalX',						'TypicalX';
				%'UseParallel',					'UseParallel'
			};
		end
	end

	methods
		function [this] = minfunc(varargin)
			%MINFUNC create new optimization option set
			%	Input:
			%		varargin:	options to set
			%	Output:
			%		this:		instance
			preferredtype = optimization.options.OptionType.STRUCT;
			supportedtype = optimization.options.OptionType.STRUCT;
			this@optimization.options.Options(optimization.solver.Optimizer.MINFUNC, supportedtype, preferredtype, false, optimization.options.ProblemType.UNCONSTRAINED, optimization.options.ProblemType.UNCONSTRAINED);
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
				'sd';
				'csd';
				'bb';
				'cg';
				'scg';
				'pcg';
				'lbfgs';
				'newton0';
				'pnewton0';
				'qnewton';
				'mnewton';
				'newton'
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
				'iter',				'iter';
				'iter-detailed',	'iter-detailed';
				'notify',			'notify';
				'notify-detailed',	'notify-detailed';
				'final',			'final';
				'final-detailed',	'final-detailed'
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
			if outputstruct && isfield(output, 'iterations')
				information.iterations		= output.iterations;
			end
			if outputstruct && isfield(output, 'funcCount')
				information.funcCount		= output.funcCount;
			end
			if outputstruct && isfield(output, 'lssteplength')
				information.lssteplength	= output.lssteplength;
			end
			if outputstruct && isfield(output, 'constrviolation')
				information.constrviolation = output.constrviolation;
			end
			if outputstruct && isfield(output, 'stepsize')
				information.stepsize		= output.stepsize;
			end
			if outputstruct && isfield(output, 'algorithm')
				information.algorithm		= output.algorithm;
			else
				information.algorithm		= 'FMINUNC';
			end
			if outputstruct && isfield(output, 'cgiterations')
				information.cgiterations	= output.cgiterations;
			end
			if outputstruct && isfield(output, 'firstorderopt')
				information.firstorderopt	= output.firstorderopt;
			end
			if outputstruct && isfield(output, 'message')
				information.message			= output.message;
			end
			if nargin >= 10
				information.additional		= output;
			end
			information.information.t					= time;
			information.information.Nvar				= nvars;
			if outputstruct && isfield(output, 'iterations')
				information.information.iterations		= output.iterations;
			end
			if outputstruct && isfield(output, 'funcCount')
				information.information.funcCount		= output.funcCount;
			end
			information.information.overalliterations	= overalliterations;
			information.information.overallfunCount		= overallfunevals;
			information.information.retries				= retries;
			information.information.feasibility			= errorcode;
			information.information.xmin				= xmin;
			information.information.fmin				= fmin;
			if outputstruct && isfield(output, 'constrviolation')
				information.information.constrviol		= output.constrviolation;
			end
			if outputstruct && isfield(output, 'firstorderopt')
				information.information.optimality		= output.firstorderopt;
			end
			if outputstruct && isfield(output, 'message')
				information.information.output			= output.message;
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
classdef ipopt < optimization.options.Options
	%IPOPT solver options for ipopt

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
				'Algorithm',					'Algorithm';
				'CheckGradients',				'DerivativeCheck';
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
				'Algorithm',					'linear_solver';
				'CheckGradients',				'check_derivatives_for_naninf';
				'ConstraintTolerance',			'constr_viol_tol';
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
				'MaxIterations',				'max_iter';
				%'MaxPCGIter',					'MaxPCGIter';
				%'MaxProjCGIter',				'MaxProjCGIter';
				'MaxSQPIter',					'acceptable_iter';
				'MaxTime',						'max_cpu_time';
				%'ObjectiveLimit',				'infinite_bound';
				'OptimalityTolerance',			'tol';
				%'OutputFcn',					'OutputFcn';
				%'PlotFcn',						'PlotFcn';
				%'PrecondBandWidth',			'PrecondBandWidth';
				%'RelLineSearchBound',			'RelLineSrchBnd';
				%'RelLineSearchBoundDuration',	'RelLineSrchBndDuration';
				%'ScaleProblem',				'ScaleProblem';
				%'SubproblemAlgorithm',			'linear_solver';
				%'SpecifyConstraintGradient',	'derivative_option';
				%'SpecifyObjectiveGradient',		'derivative_option';
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
		function [this] = ipopt(varargin)
			%IPOPT create new optimization option set
			%	Input:
			%		varargin:	options to set
			%	Output:
			%		this:		instance
			this@optimization.options.Options(optimization.solver.Optimizer.IPOPT, optimization.options.OptionType.STRUCT, optimization.options.OptionType.STRUCT, false, [
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
				'ma27';
				'ma57';
				'ma77';
				'ma86';
				'ma97';
				'pardiso';
				'wsmp';
				'mumps';
				'custom'
			};
		end

		function [display] = displaymapping(~)
			%DISPLAYMAPPING mapping from optimoptions display names to solver display names
			%	Input:
			%		this:		instance
			%	Output:
			%		display:	mapping of display names
			display = {
				'off',				'2';
				'iter',				'5';
				'iter-detailed',	'5';
				'notify',			'3';
				'notify-detailed',	'3';
				'final',			'4';
				'final-detailed',	'4'
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
			if outputstruct && isfield(output, 'eval') && isfield(output.eval, 'objective')
				information.funcCount		= output.eval.objective;
			end
			if outputstruct && isfield(output, 'funevals')
				information.funcCount		= output.funevals;
			end
			if outputstruct && isfield(output, 'constrviol')
				information.constrviolation = output.constrviol;
			end
			information.algorithm			= 'IPOPT';
			if nargin >= 10
				information.additional			= output;
			end
			if outputstruct && isfield(output, 'outputstr')
				information.message			= output.outputstr;
			end
			information.information.t					= time;
			information.information.Nvar				= nvars;
			if outputstruct && isfield(output, 'iter')
				information.information.iterations		= output.iter;
			end
			if outputstruct && isfield(output, 'eval') && isfield(output.eval, 'objective')
				information.information.funcCount		= output.eval.objective;
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
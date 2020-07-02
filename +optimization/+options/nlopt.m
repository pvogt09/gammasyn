classdef(Abstract) nlopt < optimization.options.Options
	%NLOPT solver options for nlopt
	
	methods(Access=protected)
		function [names] = optimoptionsnames(~)
			%OPTIMOPTIONSNAMES mapping from properties to optimoptions names
			%	Input:
			%		this:	instance
			%	Output:
			%		names:	mapping of names
			names = {
				'Algorithm',					'Algorithm';
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
				'MaxFunctionEvaluations',		'MaxFunEvals';
				%'MaxIterations',				'MaxIter';
				%'MaxPCGIter',					'MaxPCGIter';
				%'MaxProjCGIter',				'MaxProjCGIter';
				%'MaxSQPIter',					'MaxSQPIter';
				'MaxTime',						'MaxTime';
				%'ObjectiveLimit',				'ObjectiveLimit';
				'OptimalityTolerance',			'TolFun';
				%'OutputFcn',					'OutputFcn';
				%'PlotFcn',						'PlotFcns';
				%'PrecondBandWidth',			'PrecondBandWidth';
				%'RelLineSearchBound',			'RelLineSrchBnd';
				%'RelLineSearchBoundDuration',	'RelLineSrchBndDuration';
				%'ScaleProblem',				'ScaleProblem';
				'SubproblemAlgorithm',			'SubproblemAlgorithm';
				'SpecifyConstraintGradient',	'GradConstr';
				'SpecifyObjectiveGradient',		'GradObj';
				'StepTolerance',				'TolX';
				%'TolConSQP',					'TolConSQP';
				%'TolPCG',						'TolPCG';
				%'TolProjCG',					'TolProjCG';
				%'TolProjCGAbs',				'TolProjCGAbs';
				%'TypicalX',					'TypicalX';
				'UseParallel',					'UseParallel'
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
				'MaxFunctionEvaluations',		'MaxFunEvals';
				%'MaxIterations',				'MaxIter';
				%'MaxPCGIter',					'MaxPCGIter';
				%'MaxProjCGIter',				'MaxProjCGIter';
				%'MaxSQPIter',					'MaxSQPIter';
				'MaxTime',						'MaxTime';
				%'ObjectiveLimit',				'ObjectiveLimit';
				'OptimalityTolerance',			'TolFun';
				%'OutputFcn',					'OutputFcn';
				%'PlotFcn',						'PlotFcns';
				%'PrecondBandWidth',			'PrecondBandWidth';
				%'RelLineSearchBound',			'RelLineSrchBnd';
				%'RelLineSearchBoundDuration',	'RelLineSrchBndDuration';
				%'ScaleProblem',				'ScaleProblem';
				'SubproblemAlgorithm',			'SubproblemAlgorithm';
				'SpecifyConstraintGradient',	'GradConstr';
				'SpecifyObjectiveGradient',		'GradObj';
				'StepTolerance',				'TolX';
				%'TolConSQP',					'TolConSQP';
				%'TolPCG',						'TolPCG';
				%'TolProjCG',					'TolProjCG';
				%'TolProjCGAbs',				'TolProjCGAbs';
				%'TypicalX',					'TypicalX';
				'UseParallel',					'UseParallel'
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
				%'CheckGradients',				'check_derivatives_for_naninf';
				'ConstraintTolerance',			'fc_tol';
				%'Diagnostics',					'Diagnostics';
				%'DiffMaxChange',				'DiffMaxChange';
				%'DiffMinChange',				'DiffMinChange';
				'Display',						'verbose';
				%'FiniteDifferenceStepSize',	'FiniteDifferenceStepSize';
				%'FiniteDifferenceType',		'FiniteDifferenceType';
				'FunctionTolerance',			'ftol_abs';
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
				'MaxFunctionEvaluations',		'maxeval';
				%'MaxIterations',				'max_iter';
				%'MaxPCGIter',					'MaxPCGIter';
				%'MaxProjCGIter',				'MaxProjCGIter';
				%'MaxSQPIter',					'acceptable_iter';
				'MaxTime',						'maxtime';
				%'ObjectiveLimit',				'infinite_bound';
				'OptimalityTolerance',			'ftol_rel';
				%'OutputFcn',					'OutputFcn';
				%'PlotFcn',						'PlotFcn';
				%'PrecondBandWidth',			'PrecondBandWidth';
				%'RelLineSearchBound',			'RelLineSrchBnd';
				%'RelLineSearchBoundDuration',	'RelLineSrchBndDuration';
				%'ScaleProblem',				'ScaleProblem';
				'SubproblemAlgorithm',			'local_optimizer';
				%'SpecifyConstraintGradient',	'derivative_option';
				%'SpecifyObjectiveGradient',	'derivative_option';
				'StepTolerance',				'xtol_rel';
				%'TolConSQP',					'TolConSQP';
				%'TolPCG',						'TolPCG';
				%'TolProjCG',					'TolProjCG';
				%'TolProjCGAbs',				'TolProjCGAbs';
				%'TypicalX',					'TypicalX';
				%'UseParallel',					'UseParallel'
			};
		end
	end
	
	%methods(Abstract=true)
	%	%POSSIBLEALGORITHMS list with possible algorithms for optimizer
	%	%	Input:
	%	%		this:		instance
	%	%	Output:
	%	%		algorithms:	possible algorithms
	%	[algorithms] = possiblealgorithms(~);
	%end
	
	methods
		function [this] = nlopt(solver, supportedproblems, preferredproblem, varargin)
			%NLOPT create new optimization option set
			%	Input:
			%		solver:					solver the options are set for
			%		supportedproblems:		array with ProblemTypes supported by the solver
			%		preferredproblem:		preferred type of problems
			%		varargin:				options to set
			%	Output:
			%		this:					instance
			this@optimization.options.Options(solver, optimization.options.OptionType.STRUCT, optimization.options.OptionType.STRUCT, false, supportedproblems, preferredproblem);
			if nargin >= 4
				this.useoptions(varargin{:});
			end
		end
		
		function [display] = displaymapping(~)
			%DISPLAYMAPPING mapping from optimoptions display names to solver display names
			%	Input:
			%		this:		instance
			%	Output:
			%		display:	mapping of display names
			display = {
				'off',				'0';
				'iter',				'5';
				'iter-detailed',	'6';
				'notify',			'3';
				'notify-detailed',	'4';
				'final',			'1';
				'final-detailed',	'2'
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
			if outputstruct && isfield(output, 'objective')
				information.funcCount		= output.objective;
			end
			if outputstruct && isfield(output, 'funevals')
				information.funcCount		= output.funevals;
			end
			if outputstruct && isfield(output, 'constrviol')
				information.constrviolation = output.constrviol;
			end
			information.algorithm			= 'NLOPT';
			if nargin >= 10
				information.additional		= output;
			end
			if outputstruct && isfield(output, 'message')
				information.message			= output.message;
			end
			information.information.t					= time;
			information.information.Nvar				= nvars;
			information.information.overalliterations	= overalliterations;
			information.information.overallfunCount		= overallfunevals;
			information.information.retries				= retries;
			information.information.feasibility			= errorcode;
			information.information.xmin				= xmin;
			information.information.fmin				= fmin;
			if outputstruct && isfield(output, 'constrviol')
				information.information.constrviol		= output.constrviol;
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
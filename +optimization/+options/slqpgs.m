classdef(Abstract) slqpgs < optimization.options.Options
	%SLQPGS solver options for slqpgs

	methods(Static=true)
		function [this] = fromDCM(value, ~)
			%FROMDCM convert structure from DCM import to instance
			%	Input:
			%		value:	value imported from DCM file
			%		name:	optional name of parameter
			%	Output:
			%		this:	instance
			this = optimization.options.sqpgs();
			if nargin == 0 || isempty(value)
				return;
			end
			if isstruct(value)
				this.useoptions(value);
			end
		end
	end

	methods(Access=protected)
		function [names] = optimoptionsnames(~)
			%OPTIMOPTIONSNAMES mapping from properties to optimoptions names
			%	Input:
			%		this:	instance
			%	Output:
			%		names:	mapping of names
			names = {
				%'Algorithm',					'Algorithm';
				'CheckGradients',				'DerivativeCheck';
				'ConstraintTolerance',			'TolCon';
				%'Diagnostics',					'Diagnostics';
				%'DiffMaxChange',				'DiffMaxChange';
				%'DiffMinChange',				'DiffMinChange';
				'Display',						'Display';
				%'FiniteDifferenceStepSize',		'FinDiffRelStep';
				%'FiniteDifferenceType',			'FinDiffType';
				%'FunctionTolerance',			'TolFun';
				'FunValCheck',					'FunValCheck';
				%'HessianApproximation',			'Hessian';
				%'HessianFcn',					'HessFcn';
				%'HessianMultiplyFcn',			'HessMult';
				%'HessianPattern',				'HessPattern';
				%'HessianUpdate',				'HessUpdate';
				%'HonorBounds',					'AlwaysHonorConstraints';
				%'InitBarrierParam',				'InitBarrierParam';
				%'InitDamping',					'InitDamping';
				%'InitTrustRegionRadius',		'InitTrustRegionRadius';
				%'JacobianMultiplyFcn',			'JacobianMultiplyFcn';
				%'JacobianPattern',				'JacobPattern';
				'MaxFunctionEvaluations',		'MaxFunEvals';
				'MaxIterations',				'MaxIter';
				%'MaxPCGIter',					'MaxPCGIter';
				%'MaxProjCGIter',				'MaxProjCGIter';
				%'MaxSQPIter',					'MaxSQPIter';
				%'MaxTime',						'MaxTime';
				%'ObjectiveLimit',				'ObjectiveLimit';
				'OptimalityTolerance',			'TolFun';
				%'OutputFcn',					'OutputFcn';
				%'PlotFcn',						'PlotFcns';
				%'PrecondBandWidth',				'PrecondBandWidth';
				%'RelLineSearchBound',			'RelLineSrchBnd';
				%'RelLineSearchBoundDuration',	'RelLineSrchBndDuration';
				%'ScaleProblem',				'ScaleProblem';
				'SubproblemAlgorithm',			'SubproblemAlgorithm';
				'SpecifyConstraintGradient',	'GradConstr';
				'SpecifyObjectiveGradient',		'GradObj';
				%'StepTolerance',				'TolX';
				%'TolConSQP',					'TolConSQP';
				%'TolPCG',						'TolPCG';
				%'TolProjCG',					'TolProjCG';
				%'TolProjCGAbs',					'TolProjCGAbs';
				%'TypicalX',						'TypicalX';
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
				%'Algorithm',					'Algorithm';
				'CheckGradients',				'DerivativeCheck';
				'ConstraintTolerance',			'TolCon';
				%'Diagnostics',					'Diagnostics';
				%'DiffMaxChange',				'DiffMaxChange';
				%'DiffMinChange',				'DiffMinChange';
				'Display',						'Display';
				%'FiniteDifferenceStepSize',		'FinDiffRelSize';
				%'FiniteDifferenceType',			'FinDiffType';
				%'FunctionTolerance',			'TolFun';
				'FunValCheck',					'FunValCheck';
				%'HessianApproximation',			'Hessian';
				%'HessianFcn',					'HessFcn';
				%'HessianMultiplyFcn',			'HessMult';
				%'HessianPattern',				'HessPattern';
				%'HessianUpdate',				'HessUpdate';
				%'HonorBounds',					'AlwaysHonorConstraints';
				%'InitBarrierParam',				'InitBarrierParam';
				%'InitDamping',					'InitDamping';
				%'InitTrustRegionRadius',		'InitTrustRegionRadius';
				%'JacobianMultiplyFcn',			'JacobianMultiplyFcn';
				%'JacobianPattern',				'JacobPattern';
				'MaxFunctionEvaluations',		'MaxFunEvals';
				'MaxIterations',				'MaxIter';
				%'MaxPCGIter',					'MaxPCGIter';
				%'MaxProjCGIter',				'MaxProjCGIter';
				%'MaxSQPIter',					'MaxSQPIter';
				%'MaxTime',						'MaxTime';
				%'ObjectiveLimit',				'ObjectiveLimit';
				'OptimalityTolerance',			'TolFun';
				%'OutputFcn',					'OutputFcn';
				%'PlotFcn',						'PlotFcns';
				%'PrecondBandWidth',				'PrecondBandWidth';
				%'RelLineSearchBound',			'RelLineSrchBnd';
				%'RelLineSearchBoundDuration',	'RelLineSrchBndDuration';
				%'ScaleProblem',				'ScaleProblem';
				'SubproblemAlgorithm',			'SubproblemAlgorithm';
				'SpecifyConstraintGradient',	'GradConstr';
				'SpecifyObjectiveGradient',		'GradObj';
				%'StepTolerance',				'TolX';
				%'TolConSQP',					'TolConSQP';
				%'TolPCG',						'TolPCG';
				%'TolProjCG',					'TolProjCG';
				%'TolProjCGAbs',					'TolProjCGAbs';
				%'TypicalX',						'TypicalX';
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
				'CheckGradients',				'DerivativeCheck';
				'ConstraintTolerance',			'ineq_tol';
				%'Diagnostics',					'Diagnostics';
				%'DiffMaxChange',				'DiffMaxChange';
				%'DiffMinChange',				'DiffMinChange';
				'Display',						'Display';
				%'FiniteDifferenceStepSize',		'FiniteDifferenceStepSize';
				%'FiniteDifferenceType',			'FiniteDifferenceType';
				%'FunctionTolerance',			'FunctionTolerance';
				'FunValCheck',					'FunValCheck';
				%'HessianApproximation',			'HessianApproximation';
				%'HessianFcn',					'HessianFcn';
				%'HessianMultiplyFcn',			'HessianMultiplyFcn';
				%'HessianPattern',				'HessPattern';
				%'HessianUpdate',				'HessUpdate';
				%'HonorBounds',					'AlwaysHonorConstraints';
				%'InitBarrierParam',				'InitBarrierParam';
				%'InitDamping',					'InitDamping';
				%'InitTrustRegionRadius',		'InitTrustRegionRadius';
				%'JacobianMultiplyFcn',			'JacobianMultiplyFcn';
				%'JacobianPattern',				'JacobPattern';
				'MaxFunctionEvaluations',		'funevals_max';
				'MaxIterations',				'iter_max';
				%'MaxPCGIter',					'MaxPCGIter';
				%'MaxProjCGIter',				'MaxProjCGIter';
				%'MaxSQPIter',					'MaxSQPIter';TODO: implement
				'MaxTime',						'MaxTime';
				%'ObjectiveLimit',				'ObjectiveLimit';
				'OptimalityTolerance',			'stat_tol';
				%'OutputFcn',					'OutputFcn';
				%'PlotFcn',						'PlotFcn';
				%'PrecondBandWidth',				'PrecondBandWidth';
				%'RelLineSearchBound',			'RelLineSrchBnd';
				%'RelLineSearchBoundDuration',	'RelLineSrchBndDuration';
				%'ScaleProblem',				'ScaleProblem';
				'SubproblemAlgorithm',			'SubproblemAlgorithm';
				'SpecifyConstraintGradient',	'SpecifyConstraintGradient';
				'SpecifyObjectiveGradient',		'SpecifyObjectiveGradient';
				%'StepTolerance',				'ProgTol';
				%'TolConSQP',					'TolConSQP';
				%'TolPCG',						'TolPCG';
				%'TolProjCG',					'TolProjCG';
				%'TolProjCGAbs',					'TolProjCGAbs';
				%'TypicalX',						'TypicalX';
				'UseParallel',					'UseParallel'
			};
		end
	end

	methods
		function [this] = slqpgs(solver, varargin)
			%SLQPGS create new optimization option set
			%	Input:
			%		varargin:	options to set
			%	Output:
			%		this:		instance
			preferredtype = optimization.options.OptionType.STRUCT;
			supportedtype = [
				optimization.options.OptionType.STRUCT;
			];
			this@optimization.options.Options(solver, supportedtype, preferredtype, true, [
				optimization.options.ProblemType.CONSTRAINED;
				optimization.options.ProblemType.UNCONSTRAINED
			], optimization.options.ProblemType.CONSTRAINED);
			if nargin >= 2
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
				'iter',				'1';
				'iter-detailed',	'1';
				'notify',			'1';
				'notify-detailed',	'1';
				'final',			'1';
				'final-detailed',	'1'
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
			%		information:		unified optimization output
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
				information.algorithm		= 'SLQPGS';
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
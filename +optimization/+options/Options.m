classdef(Abstract) Options < handle
	%OPTIONS class for representation of options for optimization
	
	% https://de.mathworks.com/help/optim/ug/current-and-legacy-option-name-tables.html
	properties(SetAccess=protected)
		%SOLVER solver to use for optimization
		Solver = [];
	end
	
	properties(AbortSet=true)
		%ALGORITHM Chooses the algorithm used by the solver
		Algorithm;
		%CHECKGRADIENTS Compare user-supplied derivatives to finite-differencing derivatives
		CheckGradients = false;%_DerivativeCheck;
		%CONSTRAINTTOLERANCE Tolerance on the constraint violation
		ConstraintTolerance = [];%1E-6;%_TolCon;
		%DIAGNOSTICS Display diagnostic information
		Diagnostics = false;
		%DIFFMAXCHANGE Maximum change in variables for finite-difference gradients
		DiffMaxChange = [];%Inf;
		%DIFFMINCHANGE Minimum change in variables for finite-difference gradients
		DiffMinChange = [];%0;
		%DISPLAY Level of display
		Display = 'final';
		%FINITEDIFFERENCESTEPSIZE Scalar or vector step size factor
		FiniteDifferenceStepSize = [];%'sqrt(eps)';%_FinDiffRelSize;
		%FINITEDIFFERENCETYPE Finite difference type
		FiniteDifferenceType = [];%'forward';%_FinDiffType;
		%FUNCTIONTOLERANCE Termination tolerance on the function value
		FunctionTolerance = [];%1E-6;%_TolFun;
		%FUNVALCHECK Check whether objective function and constraints values are valid
		FunValCheck = false;
		%HESSIANAPPROXIMATION Specify whether a user-supplied Hessian will be supplied
		HessianApproximation = [];%'bfgs';%_Hessian;
		%HESSIANFCN Function handle to a user-supplied Hessian
		HessianFcn = [];%_HessFcn;
		%HESSIANMULTIPLYFCN Function handle for Hessian multiply function
		HessianMultiplyFcn = [];%_HessMult;
		%HESSIANPATTERN Sparsity pattern of the Hessian for finite differencing
		HessianPattern = [];%'sparse(ones(numberOfVariables))';
		%HESSIANUPDATE Quasi-Newton updating scheme
		HessianUpdate = [];
		%HONORBOUNDS Determine whether bounds are satisfied at every iteration
		HonorBounds;%_AlwaysHonorConstraints;
		%INITBARRIERPARAM Initial barrier value
		InitBarrierParam = [];%0.1;
		%INITDAMPING Initial Levenberg-Marquardt parameter
		InitDamping = [];%0.01;
		%INITTRUSTREGIONRADIUS Initial radius of the trust region
		InitTrustRegionRadius = [];%'sqrt(numberOfVariables)';
		%JACOBIANMULTIPLYFCN User-defined Jacobian multiply function
		JacobianMultiplyFcn = [];
		%JACOBIANPPATTERN Sparsity pattern of the Jacobian for finite differencing
		JacobianPattern = [];
		%MAXFUNCTIONEVALUATIONS Maximum number of function evaluations allowed
		MaxFunctionEvaluations = [];%3000;%_MaxFunEvals;
		%MAXITERATIONS Maximum number of iterations allowed
		MaxIterations = [];%1000;%_MaxIter;
		%MAXPCGITER Maximum number of PCG (preconditioned conjugate gradient) iterations
		MaxPCGIter = [];%'max(1,floor(numberOfVariables/2))';
		%MAXPROJCGITER A tolerance for the number of projected conjugate gradient iterations
		MaxProjCGIter = [];%'2*(numberOfVariables-numberOfEqualities)';
		%MAXSQPITER Maximum number of SQP iterations allowed
		MaxSQPIter = [];%1000;
		%MAXTIME Maximum amount of time in seconds allowed for the algorithm
		MaxTime = [];%Inf;
		%OBJECTIVELIMIT Lower limit on the objective function
		ObjectiveLimit = [];%-1E20;
		%OPTIMALITYTOLERANCE Termination tolerance on the first-order optimality
		OptimalityTolerance = [];%1E-6;%_TolFun;
		%OUTPUTFCN User-defined functions that are called at each iteration
		OutputFcn = {};
		%PLOTFCN Plots various measures of progress while the algorithm executes
		PlotFcn = {};%_PlotFcns;
		%PRECONDBANDWIDTH Upper bandwidth of preconditioner for PCG
		PrecondBandWidth = [];%0;
		%RELLINESEARCHBOUND Relative bound on the line search step length
		RelLineSearchBound = [];%[];
		%RELLINESEARCHBOUNDDURATION Number of iterations for which the bound specified in RelLineSrchBnd should be active
		RelLineSearchBoundDuration = [];%1;
		%SCALEPROBLEM Determine whether all constraints and the objective function are normalized
		ScaleProblem = [];%'none';
		%SPECIFYCONSTRAINTGRADIENT Gradient for nonlinear constraint functions defined by the user
		SpecifyConstraintGradient = false;%_GradConstr;
		%SPECIFYCONSTRAINTHESSIAN Hessian for nonlinear constraint functions defined by the user
		SpecifyConstraintHessian = false;
		%SPECIFYOBJECTIVEGRADIENT Gradient for the objective function defined by the user
		SpecifyObjectiveGradient = false;%_GradObj;
		%SPECIFYOBJECTIVEHESSIAN Hessian for the objective function defined by the user
		SpecifyObjectiveHessian = false;
		%STEPTOLERANCE Termination tolerance on x
		StepTolerance = [];%1E-10;%_TolX;
		%SUBPROBLEMALGORITHM Determines how the iteration step is calculated
		SubproblemAlgorithm;
		%TOLCONSQP Termination tolerance on inner iteration SQP constraint violation
		TolConSQP = [];%1E-6;
		%TOLPCG Termination tolerance on the PCG iteration
		TolPCG = [];%0.1;
		%TOLPCG Termination tolerance on the PCG iteration
		TolProjCG = [];%1E-10;
		%TOLPROJCGABS Absolute tolerance for projected conjugate gradient algorithm
		TolProjCGAbs = [];%1E-10;
		%TYPICALX Typical x values
		TypicalX = [];%'ones(numberOfVariables,1)';
		%USEPARALLEL Estimate gradients in parallel
		UseParallel = configuration.optimization.hasparallelsupport(false);
		
		% number of retries for the optimization (warm start in a loop with last solution)
		Retries = 1;
		% type of problem to solve
		ProblemType = [];
		% number of problem variables
		NumberVariables = [];
		% number of inequality constraints
		NumberConstraintsInequality = [];
		% number of equality constraints
		NumberConstraintsEquality = [];
		% number of bound constraints
		NumberConstraintsBounds = [];
	end
	
	properties(Constant=true)
		% prototype for structure with information about optimization run
		INFORMATIONPROTOTYPE = struct(...
			't',					0,...
			'Nvar',					0,...
			'iterations',			NaN,...
			'funcCount',			NaN,...
			'overalliterations',	NaN,...
			'overallfunCount',		NaN,...
			'retries',				1,...
			'feasibility',			-1,...
			'xmin',					[],...
			'fmin',					NaN,...
			'constrviol',			NaN,...
			'optimality',			NaN,...
			'output',				''...
		);
		% prototype for structure with information returned by optimization run in fmincon style
		OUTPUTPROTOTYPE = struct(...
			'iterations',		NaN,...
			'funcCount',		NaN,...
			'lssteplength',		NaN,...
			'constrviolation',	NaN,...
			'stepsize',			NaN,...
			'algorithm',		NaN,...
			'cgiterations',		NaN,...
			'firstorderopt',	NaN,...
			'message',			'',...
			'information',		optimization.options.Options.INFORMATIONPROTOTYPE,...
			'additional',		struct([])...
		);
		% prototype for structure with information returned by optimization run in fmincon style when multiple optimization runs were made
		OUTPUTPROTOTYPEMULTIPLE = struct(...
			'iterations',		NaN,...
			'funcCount',		NaN,...
			'lssteplength',		NaN,...
			'constrviolation',	NaN,...
			'stepsize',			NaN,...
			'algorithm',		NaN,...
			'cgiterations',		NaN,...
			'firstorderopt',	NaN,...
			'message',			'',...
			'information',		optimization.options.Options.INFORMATIONPROTOTYPE,...
			'additional',		struct([]),...
			'runs',				{{}}...
		);
	end
	
	properties(Constant=true, Access=protected)
		% properties, where variable size options are allowed
		VARIABLESIZEALLOWED = {
			'PrecondBandWidth';
			'MinAbsMax';
			'GoalsExactAchieve';
            'RelLineSrchBndDuration';
			'RelLineSearchBoundDuration';
			'DisplayInterval';
            'RootLPMaxIter';
			'MaxFunctionEvaluations';
			'MaxFunEvals';
			'MaxProjCGIter';
            'MaxSQPIter';
			'MaxPCGIter';
			'MaxNodes';
			'MaxIterations';
			'MaxIter';
			'JacobPattern';
			'JacobianPattern';
			'HessPattern';
			'HessianPattern';
			'TypicalX';
			'InitBarrierParam';
			'InitTrustRegionRadius';
			'StallTimeLimit';
			'FiniteDifferenceStepSize'
		};
	end
	
	properties(Access=protected)
		% indicator, if optimization algorithm supports optimset
		supportsoptimset = false;
		% indicator, if optimization algorithm supports optimoptions
		supportsoptimoptions = false;
		% indicator, if optimization algorithm supports structure options
		supportsstruct = true;
		% indicator, if optimization algorithm supports gaoptimset
		supportsoptimsetga = false;
		% indicator, if optimization algorithm supports psoptimset
		supportsoptimsetps = false;
		% indicator, if optimization algorithm supports saoptimset
		supportsoptimsetsa = false;
		% preferred option set
		preferredoption = optimization.options.OptionType.STRUCT;
		% indicator, if optimization algorithm is a builtin function or an external library
		builtinfun = false;
		% supported optimization problem types
		supportedproblems;
	end
	
	properties(Access=protected, Transient=true)% avoid saving "constant" properties containing property name mappings to files
		% names for corresponding options in optimoptions
		optimoptionsname;
		% names for corresponding options in optimset
		optimsetname;
		% names for corresponding options in gaoptimset
		optimsetganame;
		% names for corresponding options in psoptimset
		optimsetpsname;
		% names for corresponding options in saoptimset
		optimsetsaname;
		% names for corresponding options in structure
		structname;
		% user setable options for optimization
 		options;
	end
	
	methods(Access=protected)
		function [value, validvalue, errmsg, errid] = checkProperty(this, name, value, possValues, replacechar)
			%CHECKPROPERTY set a value for a solver option
			%	Input:
			%		this:			instance
			%		name:			name of option to set
			%		value:			value to set
			%		possValues:		possible values to set
			%		replacechar:	indicator, if char values should be replaced by an interpreted version
			%	Output:
			%		value:			value to set
			%		validvalue:		indicator, if value is valid
			%		errmsg:			error message, if value is not valid
			%		errid:			error identifier, if value is not valid
			if isempty(value)
				validvalue = true;
				errmsg = '';
				errid = '';
				return;
			end
			numvars = this.NumberVariables;
			if nargin < 4
				possValues = [];
			end
			if nargin <= 4
				replacechar = ~isempty(numvars);
			end
			if ischar(value)
				value = deblank(value);
			end
			replaceallowed = any(strcmpi(name, this.VARIABLESIZEALLOWED));
			if replaceallowed && replacechar && (isempty(numvars) || isnan(numvars))
				error('optimization:options:replace', 'Problem dimensions have to be specified, when variable size options are used.');
			end
			[value, validvalue, errmsg, errid] = checkfield(this, name, value, possValues, replacechar && replaceallowed);
			if ~validvalue
				ME = MException(errid, '%s', errmsg);
				throwAsCaller(ME);
			end
		end
		
		function [value] = convertProperty(this, name, value, optionstype, replacechar)
			%CONVERTPROPERTY convert a property to the specified option type
			%	Input:
			%		this:			instance
			%		name:			name of the option to convert
			%		value:			value of the option to convert
			%		optionstype:	type of option set, the value is returned for
			%		replacechar:	indicator, if char options should be replaced by corresponding numerical values
			%	Output:
			%		value:			value to set
			if nargin <= 4
				replacechar = ~isempty(this.NumberVariables);
			end
			if ischar(value)
				value = lower(deblank(value));
			end
			replaceallowed = any(strcmpi(name, this.VARIABLESIZEALLOWED));
			if replaceallowed && replacechar && (isempty(this.NumberVariables) || isnan(this.NumberVariables))
				error('optimization:options:replace', 'Problem dimensions have to be specified, when variable size options are used.');
			end
			value = convertfield(this, name, value, optionstype, replacechar && replaceallowed);
		end
	end
	
	methods
		function [optimoptionsname] = get.optimoptionsname(this)
			%OPTIMOPTIONSNAME getter method for transient property optimoptionsname
			%	Input:
			%		this:				instance
			%	Output:
			%		optimoptionsname:	cell array with corresponding names in optimoptions function
			optimoptionsname = this.optimoptionsname;
			if isempty(optimoptionsname)
				this.optimoptionsname = this.optimoptionsnames();
				optimoptionsname = this.optimoptionsname;
			end
		end
		
		function [optimsetname] = get.optimsetname(this)
			%OPTIMSETNAME getter method for transient property optimsetname
			%	Input:
			%		this:				instance
			%	Output:
			%		optimsetname:		cell array with corresponding names in optimset function
			optimsetname = this.optimoptionsname;
			if isempty(optimsetname)
				this.optimsetname = this.optimsetnames();
				optimsetname = this.optimsetname;
			end
		end
		
		function [optimsetganame] = get.optimsetganame(this)
			%OPTIMSETGANAME getter method for transient property optimsetganame
			%	Input:
			%		this:				instance
			%	Output:
			%		optimsetganame:		cell array with corresponding names in gaoptimset function
			optimsetganame = this.optimsetganame;
			if isempty(optimsetganame)
				this.optimsetganame = this.optimsetganames();
				optimsetganame = this.optimsetganame;
			end
		end
		
		function [optimsetpsname] = get.optimsetpsname(this)
			%OPTIMSETPSNAME getter method for transient property optimsetpsname
			%	Input:
			%		this:				instance
			%	Output:
			%		optimsetname:		cell array with corresponding names in psoptimset function
			optimsetpsname = this.optimsetpsname;
			if isempty(optimsetpsname)
				this.optimsetpsname = this.optimsetpsnames();
				optimsetpsname = this.optimsetpsname;
			end
		end
		
		function [optimsetsaname] = get.optimsetsaname(this)
			%OPTIMSETSANAME getter method for transient property optimsetsaname
			%	Input:
			%		this:				instance
			%	Output:
			%		optimsetsaname:		cell array with corresponding names in saoptimset function
			optimsetsaname = this.optimsetsaname;
			if isempty(optimsetsaname)
				this.optimsetsaname = this.optimsetsanames();
				optimsetsaname = this.optimsetsaname;
			end
		end
		
		function [structname] = get.structname(this)
			%STRUCTNAME getter method for transient property structname
			%	Input:
			%		this:				instance
			%	Output:
			%		structname:			cell array with corresponding names in structure
			structname = this.structname;
			if isempty(structname)
				this.structname = this.structnames();
				structname = this.structname;
			end
		end
		
		function [options] = get.options(this)
			%OPTIONS getter method for setable options
			%	Input:
			%		this:				instance
			%	Output:
			%		options:			cell array with names of setable options
			options = this.options;
			if isempty(options)
				this.options = fieldnames(this);
				this.options = this.options(~strcmpi(this.options, 'Solver') & ~strcmpi(this.options, 'NumberVariables') & ~strcmpi(this.options, 'NumberConstraintsInequality') & ~strcmpi(this.options, 'NumberConstraintsEquality') & ~strcmpi(this.options, 'NumberConstraintsBounds') & ~strcmpi(this.options, 'VARIABLESIZEALLOWED') & ~strcmpi(this.options, 'INFORMATIONPROTOTYPE') & ~strcmpi(this.options, 'OUTPUTPROTOTYPE') & ~strcmpi(this.options, 'OUTPUTPROTOTYPEMULTIPLE'), :);
				options = this.options;
			end
		end
	end
	
	methods
		function [this] = Options(solver, optionsupport, preferred, builtin, supportedproblems, preferredproblem)
			%OPTIONS create new option set
			%	Input:
			%		solver:					solver the options are set for
			%		optimoptionssupport:	indicator, if optimoptions are supported
			%		optimsetsupport:		indicator, if optimset is supported
			%		structsupport:			indicator, if struct are supported
			%		preferred:				preferred type of options
			%		builtin:				indicator, if the solver is a builtin solver
			%		supportedproblems:		array with ProblemTypes supported by the solver
			%		preferredproblem:		preferred type of problems
			%	Output:
			%		this:					instance
			this.supportsoptimoptions = any(optionsupport == optimization.options.OptionType.OPTIMOPTIONS);
			this.supportsoptimset = any(optionsupport == optimization.options.OptionType.OPTIMSET);
			this.supportsstruct = any(optionsupport == optimization.options.OptionType.STRUCT);
			this.supportsoptimsetga = any(optionsupport == optimization.options.OptionType.GAOPTIMSET);
			this.supportsoptimsetps = any(optionsupport == optimization.options.OptionType.PSOPTIMSET);
			this.supportsoptimsetsa = any(optionsupport == optimization.options.OptionType.SAOPTIMSET);
			this.preferredoption = preferred;
			this.builtinfun = builtin;
			this.supportedproblems = supportedproblems;
			this.ProblemType = preferredproblem;
			this.optimoptionsname = this.optimoptionsnames();
			this.optimsetname = this.optimsetnames();
			this.optimsetganame = this.optimsetganames();
			this.optimsetpsname = this.optimsetpsnames();
			this.optimsetsaname = this.optimsetsanames();
			this.structname = this.structnames();
			this.Solver = solver;
			temp = fieldnames(this);
			this.options = temp(~strcmpi(temp, 'Solver') & ~strcmpi(temp, 'NumberVariables') & ~strcmpi(temp, 'NumberConstraintsInequality') & ~strcmpi(temp, 'NumberConstraintsEquality') & ~strcmpi(temp, 'NumberConstraintsBounds') & ~strcmpi(temp, 'VARIABLESIZEALLOWED') & ~strcmpi(temp, 'INFORMATIONPROTOTYPE') & ~strcmpi(temp, 'OUTPUTPROTOTYPE') & ~strcmpi(temp, 'OUTPUTPROTOTYPEMULTIPLE'), :);
		end
		
		function [options] = getpreferred(this)
			%GETPREFERRED return current options in preferred option format
			%	Input:
			%		this:		instance
			%	Output:
			%		options:	current options in preferred format
			switch this.preferredoption
				case optimization.options.OptionType.OPTIMOPTIONS
					if this.supportsoptimoptions
						if matlab.Version.CURRENT >= matlab.Version.R2013A
							options = optimoptions(strrep(this.Solver.algorithmname, 'global', ''));
						else
							if any(exist('optimoptions') == [2, 3, 5, 6]) %#ok<EXIST> exist('optimoptions', 'function')
								options = optimoptions(strrep(this.Solver.algorithmname, 'global', ''));
							else
								error('optimization:options:version', 'Optimoptions is available for Matlab R2013A and newer.');
							end
						end
					else
						error('optimization:options:type', 'Solver does not support optimoptions.');
					end
				case optimization.options.OptionType.OPTIMSET
					if this.supportsoptimset
						if any(exist('optimset') == [2, 3, 5, 6]) %#ok<EXIST> exist('optimset', 'function')
							options = optimset(strrep(this.Solver.algorithmname, 'global', ''));
						else
							error('optimization:options:version', 'Optimset is not available.');
						end
					else
						error('optimization:options:type', 'Solver does not support optimset.');
					end
				case optimization.options.OptionType.STRUCT
					options = struct();
				case optimization.options.OptionType.GAOPTIMSET
					if this.supportsoptimsetga
						if configuration.optimization.hasglobaloptimization()
							options = gaoptimset(this.Solver.algorithmname);
						else
							if any(exist('gaoptimset') == [2, 3, 5, 6]) %#ok<EXIST> exist('gaoptimset', 'function')
								options = gaoptimset(this.Solver.algorithmname);
							else
								error('optimization:options:version', 'Gaoptimset is only available with a Global Optimization Toolbox license.');
							end
						end
					else
						error('optimization:options:type', 'Solver does not support gaoptimset.');
					end
				case optimization.options.OptionType.PSOPTIMSET
					if this.supportsoptimsetps
						if configuration.optimization.hasglobaloptimization()
							options = psoptimset(this.Solver.algorithmname);
						else
							if any(exist('psoptimset') == [2, 3, 5, 6]) %#ok<EXIST> exist('psoptimset', 'function')
								options = psoptimset(this.Solver.algorithmname);
							else
								error('optimization:options:version', 'Psoptimset is only available with a Global Optimization Toolbox license.');
							end
						end
					else
						error('optimization:options:type', 'Solver does not support psoptimset.');
					end
				case optimization.options.OptionType.SAOPTIMSET
					if this.supportsoptimsetsa
						if configuration.optimization.hasglobaloptimization()
							options = saoptimset(this.Solver.algorithmname);
						else
							if any(exist('saoptimset') == [2, 3, 5, 6]) %#ok<EXIST> exist('saoptimset', 'function')
								options = saoptimset(this.Solver.algorithmname);
							else
								error('optimization:options:version', 'Saoptimset is only available with a Global Optimization Toolbox license.');
							end
						end
					else
						error('optimization:options:type', 'Solver does not support saoptimset.');
					end
				otherwise
					error('optimization:options:type', 'Solver does not support unknown options type.');
			end
			options = this.setoptions(options);
		end
		
		function [options] = getoptions(this, type, options)
			%GETOPTIONS return options as specified type for current option set
			%	Input:
			%		this:		instance
			%		type:		type of options to return
			%		options:	options to set current options in
			%	Output:
			%		options:	options of specified type with current options set
			%	TODO:	add a flag for ignoring options in optimoptions, that are not set by the user
			%	TODO:	set SpecifyObjectiveHessian and SpecifyConstraintHessian in dependence on the supplied values
			if nargin == 1
				options = this.getpreferred();
				return;
			end
			if nargin <= 2
				if optimization.options.isoptimset(type)
					options = type;
					type = optimization.options.OptionType.OPTIMSET;
				elseif optimization.options.isgaoptimset(type)
					options = type;
					type = optimization.options.OptionType.GAOPTIMSET;
				elseif optimization.options.ispsoptimset(type)
					options = type;
					type = optimization.options.OptionType.PSOPTIMSET;
				elseif optimization.options.issaoptimset(type)
					options = type;
					type = optimization.options.OptionType.SAOPTIMSET;
				elseif isstruct(type)
					options = type;
					type = optimization.options.OptionType.STRUCT;
				elseif isa(type, 'optim.options.SolverOptions')
					options = type;
					type = optimization.options.OptionType.OPTIMOPTIONS;
				else
					error('optimization:options:type', 'Undefined function of variable getoptions for input arguments of type ''%s''.', class(type));
				end
			end
			if ~isa(type, 'optimization.options.OptionType')
				error('optimization:options:type', 'Undefined function of variable getoptions for input arguments of type ''%s''.', class(type));
			end
			if ~isstruct(options) && ~optimization.options.isoptimset(options) && ~isa(options, 'optim.options.SolverOptions')
				error('optimization:options:type', 'Undefined function of variable getoptions for input arguments of type ''%s''.', class(options));
			end
			if optimization.options.isoptimset(options)
				switch type
					case optimization.options.OptionType.OPTIMOPTIONS
						if this.supportsoptimoptions
							optimsetnames = this.optimsetname;
							optimoptionsnames = this.optimoptionsname;
							if matlab.Version.CURRENT >= matlab.Version.R2013A
								optionsasoptimoptions = optimoptions(strrep(this.Solver.algorithmname, 'global', ''));
							else
								if any(exist('optimoptions') == [2, 3, 5, 6]) %#ok<EXIST> exist('optimoptions', 'function')
									optionsasoptimoptions = optimoptions(strrep(this.Solver.algorithmname, 'global', ''));
								else
									error('optimization:options:version', 'Optimoptions is available for Matlab R2013A and newer.');
								end
							end
							isparticleswarm = isa(optionsasoptimoptions, 'optim.options.Particleswarm');
							for ii = 1:size(optimsetnames, 1)
								if isfield(options, optimsetnames{ii, 2})
									fieldname = optimoptionsnames(strcmpi(optimsetnames{ii, 2}, optimoptionsnames(:, 1)), :);
									if isempty(fieldname)
										error('optimization:options:type', 'Undefined property ''%s'' for class optimoptions.', optimsetnames{ii, 2});
									end
									if isprop(optionsasoptimoptions, fieldname)
										if isparticleswarm && strcmpi(fieldname, 'Display')
											% WORKAROUND: optimoptions for particleswarm algorithm does not support iter-detailed and has to be converted
											optionsasoptimoptions.(fieldname) = this.convertProperty(optimsetnames{ii, 1}, options.(optimsetnames{ii, 2}), optimization.options.OptionType.STRUCT);
										else
											optionsasoptimoptions.(fieldname) = this.convertProperty(optimsetnames{ii, 1}, options.(optimsetnames{ii, 2}), type);
										end
									end
								end
							end
							options = optionsasoptimoptions;
						else
							error('optimization:options:type', 'Solver does not support optimoptions.');
						end
					case optimization.options.OptionType.OPTIMSET
						if this.supportsoptimset
							if any(exist('optimset') == [2, 3, 5, 6]) %#ok<EXIST> exist('optimset', 'function')
								options = optimset(optimset(this.Solver.algorithmname), options);
							else
								error('optimization:options:type', 'Optimset is not available.');
							end
						else
							error('optimization:options:type', 'Solver does not support optimset.');
						end
					case optimization.options.OptionType.GAOPTIMSET
						if this.supportsoptimsetga
							if configuration.optimization.hasglobaloptimization()
								options = gaoptimset(gaoptimset(this.Solver.algorithmname), options);
							else
								if any(exist('gaoptimset') == [2, 3, 5, 6]) %#ok<EXIST> exist('gaoptimset', 'function')
									options = gaoptimset(gaoptimset(this.Solver.algorithmname), options);
								else
									error('optimization:options:version', 'Gaoptimset is only available with a Global Optimization Toolbox license.');
								end
							end
						else
							error('optimization:options:type', 'Solver does not support gaoptimset.');
						end
					case optimization.options.OptionType.PSOPTIMSET
						if this.supportsoptimsetps
							if configuration.optimization.hasglobaloptimization()
								options = psoptimset(psoptimset(this.Solver.algorithmname), options);
							else
								if any(exist('psoptimset') == [2, 3, 5, 6]) %#ok<EXIST> exist('psoptimset', 'function')
									options = psoptimset(psoptimset(this.Solver.algorithmname), options);
								else
									error('optimization:options:version', 'Psoptimset is only available with a Global Optimization Toolbox license.');
								end
							end
						else
							error('optimization:options:type', 'Solver does not support psoptimset.');
						end
					case optimization.options.OptionType.SAOPTIMSET
						if this.supportsoptimsetsa
							if configuration.optimization.hasglobaloptimization()
								options = saoptimset(saoptimset(this.Solver.algorithmname), options);
							else
								if any(exist('saoptimset') == [2, 3, 5, 6]) %#ok<EXIST> exist('saoptimset', 'function')
									options = saoptimset(saoptimset(this.Solver.algorithmname), options);
								else
									error('optimization:options:version', 'Saoptimset is only available with a Global Optimization Toolbox license.');
								end
							end
						else
							error('optimization:options:type', 'Solver does not support saoptimset.');
						end
					case optimization.options.OptionType.STRUCT
					otherwise
						error('optimization:options:type', 'Return type must be of class ''optimization.options.OptionType''.');
				end
			elseif isa(options, 'optim.options.SolverOptions')
				switch type
					case optimization.options.OptionType.OPTIMOPTIONS
						if this.supportsoptimoptions
							if matlab.Version.CURRENT >= matlab.Version.R2013A
								options = optimoptions(strrep(this.Solver.algorithmname, 'global', ''), options);
							else
								if exist('optimoptions', 'function')
									options = optimoptions(strrep(this.Solver.algorithmname, 'global', ''), options);
								else
									error('optimization:options:version', 'Optimoptions is available for Matlab R2013A and newer.');
								end
							end
						else
							error('optimization:options:type', 'Solver does not support optimoptions.');
						end
					case optimization.options.OptionType.OPTIMSET
						if this.supportsoptimset
							optimsetnames = this.optimsetname;
							optimoptionsnames = this.optimoptionsname;
							if any(exist('optimset') == [2, 3, 5, 6]) %#ok<EXIST> exist('optimset', 'function')
								optionsasoptimset = optimset(this.Solver.algorithmname);
							else
								error('optimization:options:type', 'Optimset is not available.');
							end
							for ii = 1:size(optimoptionsnames, 1)
								if isprop(options, optimoptionsnames{ii, 2})
									fieldname = optimsetnames(strcmpi(optimoptionsnames{ii, 1}, optimsetnames(:, 1)), :);
									if isempty(fieldname)
										error('optimization:options:type', 'Undefined property ''%s'' for class optimoptions.', optimoptionsnames{ii, 2});
									end
									if isfield(optionsasoptimset, fieldname{1, 2})
										optionsasoptimset.(fieldname{1, 2}) = this.convertProperty(optimsetnames{ii, 1}, options.(optimoptionsnames{ii, 2}), type);
									end
								end
							end
							options = optionsasoptimset;
						else
							error('optimization:options:type', 'Solver does not support optimset.');
						end
					case optimization.options.OptionType.GAOPTIMSET
						if this.supportsoptimsetga
							optimsetnames = this.optimsetganame;
							optimoptionsnames = this.optimoptionsname;
							if configuration.optimization.hasglobaloptimization()
								optionsasoptimset = gaoptimset(this.Solver.algorithmname);
							else
								if any(exist('gaoptimset') == [2, 3, 5, 6]) %#ok<EXIST> exist('gaoptimset', 'function')
									optionsasoptimset = gaoptimset(this.Solver.algorithmname);
								else
									error('optimization:options:version', 'Gaoptimset is only available with a Global Optimization Toolbox license.');
								end
							end
							for ii = 1:size(optimoptionsnames, 1)
								if isprop(options, optimoptionsnames{ii, 2})
									fieldname = optimsetnames(strcmpi(optimoptionsnames{ii, 1}, optimsetnames(:, 1)), :);
									if isempty(fieldname)
										error('optimization:options:type', 'Undefined property ''%s'' for class optimoptions.', optimoptionsnames{ii, 2});
									end
									if isfield(optionsasoptimset, fieldname{1, 2})
										optionsasoptimset.(fieldname{1, 2}) = this.convertProperty(optimsetnames{ii, 1}, options.(optimoptionsnames{ii, 2}), type);
									end
								end
							end
							options = optionsasoptimset;
						else
							error('optimization:options:type', 'Solver does not support gaoptimset.');
						end
					case optimization.options.OptionType.PSOPTIMSET
						if this.supportsoptimsetps
							optimsetnames = this.optimsetpsname;
							optimoptionsnames = this.optimoptionsname;
							if configuration.optimization.hasglobaloptimization()
								optionsasoptimset = psoptimset(this.Solver.algorithmname);
							else
								if any(exist('psoptimset') == [2, 3, 5, 6]) %#ok<EXIST> exist('psoptimset', 'function')
									optionsasoptimset = psoptimset(this.Solver.algorithmname);
								else
									error('optimization:options:version', 'Psoptimset is only available with a Global Optimization Toolbox license.');
								end
							end
							for ii = 1:size(optimoptionsnames, 1)
								if isprop(options, optimoptionsnames{ii, 2})
									fieldname = optimsetnames(strcmpi(optimoptionsnames{ii, 1}, optimsetnames(:, 1)), :);
									if isempty(fieldname)
										error('optimization:options:type', 'Undefined property ''%s'' for class optimoptions.', optimoptionsnames{ii, 2});
									end
									if isfield(optionsasoptimset, fieldname{1, 2})
										optionsasoptimset.(fieldname{1, 2}) = this.convertProperty(optimsetnames{ii, 1}, options.(optimoptionsnames{ii, 2}), type);
									end
								end
							end
							options = optionsasoptimset;
						else
							error('optimization:options:type', 'Solver does not support psoptimset.');
						end
					case optimization.options.OptionType.SAOPTIMSET
						if this.supportsoptimsetsa
							optimsetnames = this.optimsetsaname;
							optimoptionsnames = this.optimoptionsname;
							if configuration.optimization.hasglobaloptimization()
								optionsasoptimset = saoptimset(this.Solver.algorithmname);
							else
								if any(exist('saoptimset') == [2, 3, 5, 6]) %#ok<EXIST> exist('saoptimset', 'function')
									optionsasoptimset = saoptimset(this.Solver.algorithmname);
								else
									error('optimization:options:version', 'Saoptimset is only available with a Global Optimization Toolbox license.');
								end
							end
							for ii = 1:size(optimoptionsnames, 1)
								if isprop(options, optimoptionsnames{ii, 2})
									fieldname = optimsetnames(strcmpi(optimoptionsnames{ii, 1}, optimsetnames(:, 1)), :);
									if isempty(fieldname)
										error('optimization:options:type', 'Undefined property ''%s'' for class optimoptions.', optimoptionsnames{ii, 2});
									end
									if isfield(optionsasoptimset, fieldname{1, 2})
										optionsasoptimset.(fieldname{1, 2}) = this.convertProperty(optimsetnames{ii, 1}, options.(optimoptionsnames{ii, 2}), type);
									end
								end
							end
							options = optionsasoptimset;
						else
							error('optimization:options:type', 'Solver does not support saoptimset.');
						end
					case optimization.options.OptionType.STRUCT
					otherwise
						error('optimization:options:type', 'Return type must be of class ''optimization.options.OptionType''.');
				end
			else
				switch type
					case optimization.options.OptionType.OPTIMOPTIONS
						if this.supportsoptimoptions
							structnames = this.structname;
							optimoptionsnames = this.optimoptionsname;
							if matlab.Version.CURRENT >= matlab.Version.R2013A
								optionsasoptimoptions = optimoptions(strrep(this.Solver.algorithmname, 'global', ''));
							else
								if exist('optimoptions', 'function')
									optionsasoptimoptions = optimoptions(strrep(this.Solver.algorithmname, 'global', ''));
								else
									error('optimization:options:version', 'Optimoptions is available for Matlab R2013A and newer.');
								end
							end
							isparticleswarm = isa(optionsasoptimoptions, 'optim.options.Particleswarm');
							for ii = 1:size(structnames, 1)
								if isfield(options, structnames{ii, 2})
									fieldname = optimoptionsnames(strcmpi(structnames{ii, 2}, optimoptionsnames(:, 1)), :);
									if isempty(fieldname)
										error('optimization:options:type', 'Undefined property ''%s'' for class optimoptions.', optimoptionsnames{ii, 2});
									end
									if isprop(optionsasoptimoptions, fieldname)
										if isparticleswarm && strcmpi(fieldname, 'Display')
											% WORKAROUND: optimoptions for particleswarm algorithm does not support iter-detailed and has to be converted
											optionsasoptimoptions.(fieldname) = this.convertProperty(optimsetnames{ii, 1}, options.(optimsetnames{ii, 2}), optimization.options.OptionType.STRUCT);
										else
											optionsasoptimoptions.(fieldname) = this.convertProperty(structnames{ii, 1}, options.(structnames{ii, 2}), type);
										end
									end
								end
							end
							options = optionsasoptimoptions;
						else
							error('optimization:options:type', 'Solver does not support optimoptions.');
						end
					case optimization.options.OptionType.OPTIMSET
						if this.supportsoptimset
							if any(exist('optimset') == [2, 3, 5, 6]) %#ok<EXIST> exist('optimset', 'function')
								options = optimset(optimset(this.Solver.algorithmname), options);
							else
								error('optimization:options:type', 'Optimset is not available.');
							end
						else
							error('optimization:options:type', 'Solver does not support optimset.');
						end
					case optimization.options.OptionType.GAOPTIMSET
						if this.supportsoptimsetga
							if configuration.optimization.hasglobaloptimization()
								options = gaoptimset(gaoptimset(this.Solver.algorithmname), options);
							else
								if any(exist('gaoptimset') == [2, 3, 5, 6]) %#ok<EXIST> exist('gaoptimset', 'function')
									options = gaoptimset(gaoptimset(this.Solver.algorithmname), options);
								else
									error('optimization:options:version', 'Gaoptimset is only available with a Global Optimization Toolbox license.');
								end
							end
						else
							error('optimization:options:type', 'Solver does not support gaoptimset.');
						end
					case optimization.options.OptionType.PSOPTIMSET
						if this.supportsoptimsetps
							if configuration.optimization.hasglobaloptimization()
								options = psoptimset(psoptimset(this.Solver.algorithmname), options);
							else
								if any(exist('psoptimset') == [2, 3, 5, 6]) %#ok<EXIST> exist('psoptimset', 'function')
									options = psoptimset(psoptimset(this.Solver.algorithmname), options);
								else
									error('optimization:options:version', 'Psoptimset is only available with a Global Optimization Toolbox license.');
								end
							end
						else
							error('optimization:options:type', 'Solver does not support psoptimset.');
						end
					case optimization.options.OptionType.SAOPTIMSET
						if this.supportsoptimsetsa
							if configuration.optimization.hasglobaloptimization()
								options = saoptimset(saoptimset(this.Solver.algorithmname), options);
							else
								if any(exist('saoptimset') == [2, 3, 5, 6]) %#ok<EXIST> exist('saoptimset', 'function')
									options = saoptimset(saoptimset(this.Solver.algorithmname), options);
								else
									error('optimization:options:version', 'Saoptimset is only available with a Global Optimization Toolbox license.');
								end
							end
						else
							error('optimization:options:type', 'Solver does not support saoptimset.');
						end
					case optimization.options.OptionType.STRUCT
					otherwise
						error('optimization:options:type', 'Return type must be of class ''optimization.options.OptionType''.');
				end
			end
			options = this.setoptions(options);
		end
		
		function [] = useoptions(this, options, varargin)
			%USEOPTIONS set supplied options to current option set
			%	Input:
			%		this:		instance
			%		options:	structure, optimoptions or optimset to set
			%		varargin:	name value pairs (options is the first name) to set
			%	TODO: set SpecifyObjectiveHessian and SpecifyConstraintHessian in dependence on the supplied values
			persistent parser;
			if isempty(parser)
				parser = inputParser();
				parser.CaseSensitive = false;
				if matlab.Version.CURRENT >= matlab.Version.R2013B
					parser.PartialMatching = false;
				end
				parser.StructExpand = false;
				opt = this.options;
				for ii = 1:size(opt)
					addOptional(parser, opt{ii, 1}, this.(opt{ii, 1}));
				end
			end
			if ~ischar(options)
				optionnames = this.options;
				isoptimoptions = false;
				issameclass = false;
				if isa(options, 'optim.options.SolverOptions')
					solveroptionnames = this.optimoptionsname;
					isoptimoptions = true;
				elseif isstruct(options)
					if optimization.options.isoptimset(options)
						solveroptionnames = this.optimsetname;
					elseif optimization.options.isgaoptimset(options)
						solveroptionnames = this.optimsetganame;
					elseif optimization.options.ispsoptimset(options)
						solveroptionnames = this.optimsetpsname;
					elseif optimization.options.issaoptimset(options)
						solveroptionnames = this.optimsetsaname;
					else
						solveroptionnames = this.structname;
					end
				elseif isa(options, 'optimization.options.Options')
					solveroptionnames = repmat(this.options, 1, 2);
					issameclass = true;
				else
					error('optimization:options:type', 'Undefined function of variable useoptions for input arguments of type ''%s''.', class(options));
				end
				[lia, locb] = ismember(optionnames, solveroptionnames(:, 1));
				setablenames = solveroptionnames(locb(lia), :);
				for ii = 1:size(setablenames, 1)
					if isoptimoptions && isprop(options, setablenames{ii, 2}) && (iscell(options.(setablenames{ii, 2})) || isfunctionhandle(options.(setablenames{ii, 2})) || (~any(isnan(options.(setablenames{ii, 2}))) && ~isempty(options.(setablenames{ii, 2}))))
						if strcmpi(setablenames{ii, 1}, 'Algorithm')
							classname = this.Solver.getOptimoptionsClassname();
							if ~isempty(classname)
								if (~isempty(this.Solver) && any(strcmpi(options.(setablenames{ii, 2}), this.Solver.getAlgorithmChoices()))) || isa(options, classname)
									this.(setablenames{ii, 1}) = options.(setablenames{ii, 2});
								else
									warning('optimization:options:algorithm', 'Algorithm ''%s'' can not be used for solver ''%s''.', char(options.(setablenames{ii, 2})), char(this.Solver));
								end
							end
						elseif strcmpi(setablenames{ii, 1}, 'SubproblemAlgorithm')
							classname = this.Solver.getOptimoptionsClassname();
							if ~isempty(classname)
								if isa(options, classname)
									this.(setablenames{ii, 1}) = options.(setablenames{ii, 2});
								else
									warning('optimization:options:algorithm', 'SubproblemAlgorithm ''%s'' can not be used for solver ''%s''.', char(options.(setablenames{ii, 2})), char(this.Solver));
								end
							end
						elseif strcmpi(setablenames{ii, 1}, 'HessianApproximation')
							classname = this.Solver.getOptimoptionsClassname();
							if ~isempty(classname)
								if isa(options, classname)
									hessval = options.(setablenames{ii, 2});
									if ~strcmpi(hessval, 'not applicable')
										this.(setablenames{ii, 1}) = hessval;
									end
								else
									warning('optimization:options:algorithm', 'HessianApproximation ''%s'' can not be used for solver ''%s''.', char(options.(setablenames{ii, 2})), char(this.Solver));
								end
							end
						else
							%this.(settablenames{ii, 1}) = this.convertProperty(settablenames{ii, 1}, options.(settablenames{ii, 2}), type);
							this.(setablenames{ii, 1}) = options.(setablenames{ii, 2});
						end
					elseif issameclass && isprop(options, setablenames{ii, 2}) && (iscell(options.(setablenames{ii, 2})) || isfunctionhandle(options.(setablenames{ii, 2})) || (~any(isnan(options.(setablenames{ii, 2}))) && ~isempty(options.(setablenames{ii, 2}))))
						if strcmpi(setablenames{ii, 2}, 'Algorithm') && this.Solver ~= options.Solver
							this.(setablenames{ii, 1}) = this.Solver.defaultalgorithm;
						else
							this.(setablenames{ii, 1}) = options.(setablenames{ii, 2});
						end
					elseif isfield(options, setablenames{ii, 2}) && (iscell(options.(setablenames{ii, 2})) || isfunctionhandle(options.(setablenames{ii, 2})) || ~any(isnan(options.(setablenames{ii, 2}))) && ~isempty(options.(setablenames{ii, 2})))
						%this.(settablenames{ii, 1}) = this.convertProperty(settablenames{ii, 1}, options.(settablenames{ii, 2}), type);
						if strcmpi(setablenames{ii, 1}, 'Algorithm')
							classname = this.Solver.getOptimoptionsClassname();
							if ~isempty(classname)
								if (~isempty(this.Solver) && any(strcmpi(options.(setablenames{ii, 2}), this.Solver.getAlgorithmChoices()))) || isa(options, classname)
									this.(setablenames{ii, 1}) = options.(setablenames{ii, 2});
								else
									warning('optimization:options:algorithm', 'Algorithm ''%s'' can not be used for solver ''%s''.', char(options.(setablenames{ii, 2})), char(this.Solver));
								end
							end
						elseif strcmpi(setablenames{ii, 1}, 'SubproblemAlgorithm')
							classname = this.Solver.getOptimoptionsClassname();
							if ~isempty(classname)
								if isa(options, classname)
									this.(setablenames{ii, 1}) = options.(setablenames{ii, 2});
								else
									warning('optimization:options:algorithm', 'SubproblemAlgorithm ''%s'' can not be used for solver ''%s''.', char(options.(setablenames{ii, 2})), char(this.Solver));
								end
							end
						elseif strcmpi(setablenames{ii, 1}, 'HessianApproximation')
							classname = this.Solver.getOptimoptionsClassname();
							if ~isempty(classname)
								if isa(options, classname)
									hessval = options.(setablenames{ii, 2});
									if ~strcmpi(hessval, 'not applicable')
										this.(setablenames{ii, 1}) = hessval;
									end
								else
									warning('optimization:options:algorithm', 'HessianApproximation ''%s'' can not be used for solver ''%s''.', char(options.(setablenames{ii, 2})), char(this.Solver));
								end
							end
						else
							this.(setablenames{ii, 1}) = options.(setablenames{ii, 2});
						end
					end
				end
			end
			if ischar(options) || nargin >= 3
				args = varargin;
				if ischar(options)
					args = [{options}, args];
				end
				parse(parser, args{:});

				results = fieldnames(parser.Results);
				results = setdiff(results, parser.UsingDefaults);
				for ii = 1:length(results)
					this.(results{ii}) = parser.Results.(results{ii});
				end
			end
		end
		
		function [supports] = supportsProblem(this, problem)
			%SUPPORTSPROBLEM return, if the selected solver supports the supplied problem type
			%	Input:
			%		this:		instance
			%		problem:	problem type to check
			%	Output:
			%		supports:	true, if the selected solver supports the specified problem type, else false
			if nargin <= 1
				problem = this.ProblemType;
			end
			if isa(problem, 'optimization.options.ProblemType')
				supports = any(this.supportedproblems == problem);
			elseif ischar(problem)
				supports = any(this.supportedproblems == optimization.options.ProblemType.fromchar(problem));
			else
				error('optimization:options:type', 'Undefined function of variable supportsProblem for input arguments of type ''%s''.', class(solver));
			end
		end
		
		function [supports] = supportsHessian(this)
			%SUPPORTSHESSIAN return, if the selected solver supports hessian information
			%	Input:
			%		this:		instance
			%	Output:
			%		supports:	true, if the selected solver supports hessian information, else false
			supports = this.Solver.getHessianSupport();
		end
		
		function [] = set.Solver(this, solver)
			%SOLVER set solver to use
			%	Input:
			%		this:		instance
			%		solver:		solver to use
			if isa(solver, 'optimization.solver.Optimizer')
				this.Solver = solver;
% 				if ~this.supportsProblem()
% 					if size(this.supportedproblems, 1) == 1
% 						this.ProblemType = this.supportedproblems;
% 					else
% 						warning('optimization:options:solver', 'Solver %s does not support problem type %s.', char(solver), upper(this.ProblemType));
% 					end
% 				end
			else
				error('optimization:options:type', 'Undefined function of variable Solver for input arguments of type ''%s''.', class(solver));
			end
		end
		
		function [] = set.ProblemType(this, type)
			%RETRIES set number of retries
			%	Input:
			%		this:		instance
			%		retries:	number of retries
			this.ProblemType = this.checkProperty('ProblemType', type);
		end
		
		function [] = set.Retries(this, retries)
			%RETRIES set number of retries
			%	Input:
			%		this:		instance
			%		retries:	number of retries
			this.Retries = this.checkProperty('Retries', retries);
		end
		
		function [] = set.Algorithm(this, algorithm)
			%ALGORITHM set Algorithm
			%	Input:
			%		this:		instance
			%		algorithm:	algorithm to use
			possible = this.possiblealgorithms();
			if size(possible, 1) == 1
				algorithm = possible{1};
			end
			this.Algorithm = this.checkProperty('Algorithm', algorithm, possible);
		end
		
		function [] = set.CheckGradients(this, checkgrad)
			%CHECKGRADIENTS set CheckGradients
			%	Input:
			%		this:		instance
			%		checkgrad:	indicator, if gradient should be checked for proper values
			this.CheckGradients = this.checkProperty('CheckGradients', checkgrad);
		end
		
		function [] = set.ConstraintTolerance(this, tolcon)
			%CONSTRAINTTOLERANCE set ConstraintTolerance
			%	Input:
			%		this:		instance
			%		tolcon:		tolerance value for constraint violation
			this.ConstraintTolerance = this.checkProperty('ConstraintTolerance', tolcon);
		end
		
		function [] = set.Diagnostics(this, diagnostics)
			%DIAGNOSTICS set Diagnostics
			%	Input:
			%		this:			instance
			%		diagnostics:	set diagnostic indicator
			this.Diagnostics = this.checkProperty('Diagnostics', diagnostics);
		end
		
		function [] = set.DiffMaxChange(this, diffmaxchange)
			%DIFFMAXCHANGE set DiffMaxChange
			%	Input:
			%		this:			instance
			%		diffmaxchange:	maximum change in finite difference gradient
			this.DiffMaxChange = this.checkProperty('DiffMaxChange', diffmaxchange);
		end
		
		function [] = set.DiffMinChange(this, diffminchange)
			%DIFFMINCHANGE set DiffMinChange
			%	Input:
			%		this:			instance
			%		diffminchange:	minimum change in finite difference gradient
			this.DiffMinChange = this.checkProperty('DiffMinChange', diffminchange);
		end
		
		function [] = set.Display(this, display)
			%DISPLAY set Display
			%	Input:
			%		this:		instance
			%		display:	display settings
			this.Display = this.checkProperty('Display', display, this.displaymapping());
		end
		
		function [] = set.FiniteDifferenceStepSize(this, findiffrelstep)
			%FINITDIFFERENCESTEPSIZE set FiniteDifferenceStepSize
			%	Input:
			%		this:			instance
			%		findiffrelstep:	relative step size for finite difference
			this.FiniteDifferenceStepSize = this.checkProperty('FiniteDifferenceStepSize', findiffrelstep);
		end
		
		function [] = set.FiniteDifferenceType(this, findifftype)
			%FINITEDIFFERENCETYPE set FiniteDifferenceType
			%	Input:
			%		this:			instance
			%		findiffreltype:	finite difference type
			this.FiniteDifferenceType = this.checkProperty('FiniteDifferenceType', findifftype);
		end
		
		function [] = set.FunctionTolerance(this, tolfun)
			%FUNCTIONTOLERANCE set FunctionTolerance
			%	Input:
			%		this:		instance
			%		tolfun:		function tolerance
			this.FunctionTolerance = this.checkProperty('FunctionTolerance', tolfun);
		end
		
		function [] = set.FunValCheck(this, funvalcheck)
			%FUNVALCHECK set FunValCheck
			%	Input:
			%		this:			instance
			%		funvalcheck:	set indicator for function value check
			this.FunValCheck = this.checkProperty('FunValCheck', funvalcheck);
		end
		
		function [] = set.HessianApproximation(this, hessapprox)
			%HESSIANAPPROXIMATION set HessianApproximation
			%	Input:
			%		this:		instance
			%		hessapprox:	hessian approximation type
			this.HessianApproximation = this.checkProperty('HessianApproximation', hessapprox);
		end
		
		function [] = set.HessianFcn(this, hessfun)
			%HESSIANFUNCTION set HessianFcn
			%	Input:
			%		this:		instance
			%		hessfun:	hessian function
			this.HessianFcn = this.checkProperty('HessianFcn', hessfun);
		end
		
		function [] = set.HessianMultiplyFcn(this, hessmult)
			%HESSIANMULTIPLYFCN set HessianMultiplyFcn
			%	Input:
			%		this:		instance
			%		hessmult:	hessian multiplication function
			this.HessianMultiplyFcn = this.checkProperty('HessianMultiplyFcn', hessmult);
		end
		
		function [] = set.HessianPattern(this, hesspattern)
			%HESSIANPATTERN set HessianPattern
			%	Input:
			%		this:			instance
			%		hesspattern:	hessian sparsity pattern
			this.HessianPattern = this.checkProperty('HessianPattern', hesspattern);
		end
		
		function [] = set.HessianUpdate(this, hessupdate)
			%HESSIANUPDATE set HessianUpdate
			%	Input:
			%		this:		instance
			%		hessupdate:	hessian update function
			this.HessianUpdate = this.checkProperty('HessianUpdate', hessupdate);
		end
		
		function [] = set.HonorBounds(this, honorbounds)
			%HONORBOUNDS set HonorBounds
			%	Input:
			%		this:			instance
			%		honorbounds:	indicator, if bound constraints are always honored
			this.HonorBounds = this.checkProperty('HonorBounds', honorbounds);
		end
		
		function [] = set.InitBarrierParam(this, initbarrierparam)
			%INITBARRIERPARAM set InitBarrierParam
			%	Input:
			%		this:				instance
			%		initbarrierparam:	initial barriar parameter
			this.InitBarrierParam = this.checkProperty('InitBarrierParam', initbarrierparam);
		end
		
		function [] = set.InitDamping(this, initdamping)
			%INITDAMPING set InitDamping
			%	Input:
			%		this:			instance
			%		initdamping:	initial damping
			this.InitDamping = this.checkProperty('InitDamping', initdamping);
		end
		
		function [] = set.InitTrustRegionRadius(this, inittrustradius)
			%INITTRUSTREGIONRADIUS set InitTrustRegionRadius
			%	Input:
			%		this:					instance
			%		inittrustregionradius:	initial trust region radius
			this.InitTrustRegionRadius = this.checkProperty('InitTrustRegionRadius', inittrustradius);
		end
		
		function [] = set.JacobianMultiplyFcn(this, jacmult)
			%JACOBIANMULTIPLYFCN set JacobianMultiplyFcn
			%	Input:
			%		this:		instance
			%		jacmult:	jacobian multiply function
			this.JacobianMultiplyFcn = this.checkProperty('JacobianMultiplyFcn', jacmult);
		end
		
		function [] = set.JacobianPattern(this, jacpattern)
			%JACOBIANPATTERN set JacobianPattern
			%	Input:
			%		this:		instance
			%		jacpattern:	jacobian sparsity pattern
			this.JacobianPattern = this.checkProperty('JacobianPattern', jacpattern);
		end
		
		function [] = set.MaxFunctionEvaluations(this, maxfunevals)
			%MAXFUNCTIONEVALUATIONS set MaxFunctionEvaluations
			%	Input:
			%		this:			instance
			%		maxfunevals:	maximum number of function evaulations
			this.MaxFunctionEvaluations = this.checkProperty('MaxFunctionEvaluations', maxfunevals);
		end
		
		function [] = set.MaxIterations(this, maxiter)
			%MAXITERATIONS set MaxIterations
			%	Input:
			%		this:		instance
			%		maxiter:	maximum number of iterations
			this.MaxIterations = this.checkProperty('MaxIterations', maxiter);
		end
		
		function [] = set.MaxPCGIter(this, maxpcgiter)
			%MAXPCGITER set MaxPCGIter
			%	Input:
			%		this:		instance
			%		maxpcgiter:	maximum numer of iterations for conjugate gradient
			this.MaxPCGIter = this.checkProperty('MaxPCGIter', maxpcgiter);
		end
		
		function [] = set.MaxProjCGIter(this, maxpcgiter)
			%MAXPROJPCGITER set MaxProjCGIter
			%	Input:
			%		this:		instance
			%		maxpcgiter:	maximum number of iterations for projected conjugate gradient
			this.MaxProjCGIter = this.checkProperty('MaxProjCGIter', maxpcgiter);
		end
		
		function [] = set.MaxSQPIter(this, maxsqpiter)
			%MAXSQPITER set MaxSQPIter
			%	Input:
			%		this:		instance
			%		maxsqpiter:	maximum number of iterations for sqp subproblems
			this.MaxSQPIter = this.checkProperty('MaxSQPIter', maxsqpiter);
		end
		
		function [] = set.MaxTime(this, maxtime)
			%MAXTIME set MaxTime
			%	Input:
			%		this:		instance
			%		maxtime:	maximum runtime of optimization
			this.MaxTime = this.checkProperty('MaxTime', maxtime);
		end
		
		function [] = set.ObjectiveLimit(this, objlimit)
			%OBJECTIVELIMIT set ObjectiveLimit
			%	Input:
			%		this:		instance
			%		objlimit:	maximum value of objective function
			this.ObjectiveLimit = this.checkProperty('ObjectiveLimit', objlimit);
		end
		
		function [] = set.OptimalityTolerance(this, tolfun)
			%OPTIMALITYTOLERANCE set OptimalityTolerance
			%	Input:
			%		this:	instance
			%		tolfun:	maximum value for optimality tolerance
			this.OptimalityTolerance = this.checkProperty('OptimalityTolerance', tolfun);
		end
		
		function [] = set.OutputFcn(this, outputfcn)
			%OUTPUTFCN set OutputFcn
			%	Input:
			%		this:		instance
			%		outputfcn:	output function
			this.OutputFcn = this.checkProperty('OutputFcn', outputfcn);
		end
		
		function [] = set.PlotFcn(this, plotfcn)
			%PLOTFCN set PlotFcn
			%	Input:
			%		this:		instance
			%		plotfcn:	plot functions
			this.PlotFcn = this.checkProperty('PlotFcn', plotfcn);
		end
		
		function [] = set.PrecondBandWidth(this, bandwidth)
			%PRECONDBANDWIDTH set PrecondBandWidth
			%	Input:
			%		this:		instance
			%		bandwidth:	???
			this.PrecondBandWidth = this.checkProperty('PrecondBandWidth', bandwidth);
		end
		
		function [] = set.RelLineSearchBound(this, bound)
			%RELLINESEARCHBOUND set RelLineSearchBound
			%	Input:
			%		this:		instance
			%		bound:		bound for relative line search
			this.RelLineSearchBound = this.checkProperty('RelLineSearchBound', bound);
		end
		
		function [] = set.RelLineSearchBoundDuration(this, bound)
			%RELLINESEARCHBOUNDDURATION set RelLineSearchBoundDuration
			%	Input:
			%		this:		instance
			%		bound:		bound for relative line search duration
			this.RelLineSearchBoundDuration = this.checkProperty('RelLineSearchBoundDuration', bound);
		end
		
		function [] = set.ScaleProblem(this, scale)
			%SCALEPROBLEM set ScaleProblem
			%	Input:
			%		this:	instance
			%		scale:	indicator, if problem should be scaled
			this.ScaleProblem = this.checkProperty('ScaleProblem', scale);
		end
		
		function [] = set.SpecifyConstraintGradient(this, gradconstr)
			%SPECIFYCONSTRAINTGRADIENT set SpecifyConstraintGradient
			%	Input:
			%		this:		instance
			%		gradconstr:	indicator, if constraint function gradient is specified
			this.SpecifyConstraintGradient = this.checkProperty('SpecifyConstraintGradient', gradconstr);
		end
		
		function [] = set.SpecifyConstraintHessian(this, hessconstr)
			%SPECIFYCONSTRAINTHESSIAN set SpecifyConstraintHessian
			%	Input:
			%		this:		instance
			%		hessconstr:	indicator, if constraint function hessian is specified
			this.SpecifyConstraintHessian = this.checkProperty('SpecifyConstraintHessian', hessconstr);
		end
		
		function [] = set.SpecifyObjectiveGradient(this, gradobj)
			%SPECIFYOBJECTIVEGRADIENT set SpecifyObjectiveGradient
			%	Input:
			%		this:		instance
			%		gradobj:	indicator, if objective function gradient is specified
			this.SpecifyObjectiveGradient = this.checkProperty('SpecifyObjectiveGradient', gradobj);
		end
		
		function [] = set.SpecifyObjectiveHessian(this, hessobj)
			%SPECIFYOBJECTIVEHESSIAN set SpecifyObjectiveHessian
			%	Input:
			%		this:		instance
			%		hessobj:	indicator, if objective function hessian is specified
			this.SpecifyObjectiveHessian = this.checkProperty('SpecifyObjectiveHessian', hessobj);
		end
		
		function [] = set.StepTolerance(this, tolx)
			%STEPTOLERANCE set StepTolerance
			%	Input:
			%		this:	instance
			%		tolx:	maximum step tolerance
			this.StepTolerance = this.checkProperty('StepTolerance', tolx);
		end
		
		function [] = set.SubproblemAlgorithm(this, algorithm)
			%SUBPROBLEMALGORITHM set SubproblemAlgorithm
			%	Input:
			%		this:		instance
			%		algorithm:	algorithm for subproblems
			possible = this.possiblesubalgorithms();
			if size(possible, 1) == 1
				algorithm = possible{1};
			end
			this.SubproblemAlgorithm = this.checkProperty('SubproblemAlgorithm', algorithm, possible);
		end
		
		function [] = set.TolConSQP(this, tolcon)
			%TOLCONSQP set TolConSQP
			%	Input:
			%		this:	instance
			%		tolcon:	constraint tolerance for sqp subproblems
			this.TolConSQP = this.checkProperty('TolConSQP', tolcon);
		end
		
		function [] = set.TolPCG(this, tolcg)
			%TOLPCG set TolPCG
			%	Input:
			%		this:	instance
			%		tolpcg:	tolerance for conjugate gradient
			this.TolPCG = this.checkProperty('TolPCG', tolcg);
		end
		
		function [] = set.TolProjCG(this, tolcg)
			%TOLPROJPCG set TolProjCG
			%	Input:
			%		this:	instance
			%		tolpcg:	tolerance for projected conjugate gradient
			this.TolProjCG = this.checkProperty('TolProjCG', tolcg);
		end
		
		function [] = set.TolProjCGAbs(this, tolcgabs)
			%TOLPROJCGABS set TolProjCGAbs
			%	Input:
			%		this:	instance
			%		tolcgabs:	tolerance for absolute conjugate gradient
			this.TolProjCGAbs = this.checkProperty('TolProjCGAbs', tolcgabs);
		end
		
		function [] = set.TypicalX(this, typicalx)
			%TYPICALX set TypicalX
			%	Input:
			%		this:		instance
			%		typicalx:	typical x values
			this.TypicalX = this.checkProperty('TypicalX', typicalx);
		end
		
		function [] = set.UseParallel(this, useparallel)
			%USEPARALLEL set UseParallel
			%	Input:
			%		this:			instance
			%		useparallel:	indicator, if parallel calulation of objective and constraints should be used
			this.UseParallel = this.checkProperty('UseParallel', useparallel);
		end
		
		function [] = set.NumberVariables(this, numvar)
			%NUMBERVARIABLES set number of variables
			%	Input:
			%		this:			instance
			%		numvar:			number of problem variables
			if isempty(numvar)
				this.NumberVariables = [];
				return;
			end
			if ~isnumeric(numvar) || ~isscalar(numvar) || numvar <= 0 || floor(numvar) ~= ceil(numvar)
				error('optimization:options:variable', 'Number of variables must be a positive integer value');
			end
			this.NumberVariables = numvar;
		end
		
		function [] = set.NumberConstraintsInequality(this, numineq)
			%NUMBERCONSTRAINTSINEQUALITY set number of inequality constraints
			%	Input:
			%		this:			instance
			%		numineq:		number of inequality cnstraints
			if isempty(numineq)
				this.NumberConstraintsInequality = [];
				return;
			end
			if ~isnumeric(numineq) || ~isscalar(numineq) || numineq < 0 || floor(numineq) ~= ceil(numineq)
				error('optimization:options:variable', 'Number of inequality constraints must be a positive integer value');
			end
			this.NumberConstraintsInequality = numineq;
		end
		
		function [] = set.NumberConstraintsEquality(this, numeq)
			%NUMBERCONSTRAINTSEQUALITY set number of equality constraints
			%	Input:
			%		this:			instance
			%		numeq:			number of equality cnstraints
			if isempty(numeq)
				this.NumberConstraintsEquality = [];
				return;
			end
			if ~isnumeric(numeq) || ~isscalar(numeq) || numeq < 0 || floor(numeq) ~= ceil(numeq)
				error('optimization:options:variable', 'Number of equality constraints must be a positive integer value');
			end
			this.NumberConstraintsEquality = numeq;
		end
		
		function [] = set.NumberConstraintsBounds(this, numbounds)
			%NUMBERCONSTRAINTSBOUNSD set number of bound constraints
			%	Input:
			%		this:			instance
			%		numbounds:		number of bound cnstraints
			if isempty(numbounds)
				this.NumberConstraintsBounds = [];
				return;
			end
			if ~isnumeric(numbounds) || ~isscalar(numbounds) || numbounds < 0 || floor(numbounds) ~= ceil(numbounds)
				error('optimization:options:variable', 'Number of bound constraints must be a positive integer value');
			end
			this.NumberConstraintsBounds = numbounds;
		end
		
		function [subalgorithms] = possiblesubalgorithms(~)
			%POSSIBLESUBALGORITHMS list with possible algorithms for subproblems for optimizer
			%	Input:
			%		this:			instance
			%	Output:
			%		subalgorithms:	possible algorithms for subproblems
			subalgorithms = {
				'ldl-factorization';
				'cg'
			};
			if matlab.Version.CURRENT >= matlab.Version.R2016B
				subalgorithms = [
					subalgorithms;
					{'factorization'}
				];
			end
		end
		
		function [s] = struct(this)
			%STRUCT convert current option set to structure with public setable fields
			%	Input:
			%		this:	instance
			%	Output:
			%		s:		instance converted to a structure
			publicfields = {
				'Solver';
				'Algorithm';
				'CheckGradients';
				'ConstraintTolerance';
				'Diagnostics';
				'DiffMaxChange';
				'DiffMinChange';
				'Display';
				'FiniteDifferenceStepSize';
				'FiniteDifferenceType';
				'FunctionTolerance';
				'FunValCheck';
				'HessianApproximation';
				'HessianFcn';
				'HessianMultiplyFcn';
				'HessianPattern';
				'HessianUpdate';
				'HonorBounds';
				'InitBarrierParam';
				'InitDamping';
				'InitTrustRegionRadius';
				'JacobianMultiplyFcn';
				'JacobianPattern';
				'MaxFunctionEvaluations';
				'MaxIterations';
				'MaxPCGIter';
				'MaxProjCGIter';
				'MaxSQPIter';
				'MaxTime';
				'ObjectiveLimit';
				'OptimalityTolerance';
				'OutputFcn';
				'PlotFcn';
				'PrecondBandWidth';
				'RelLineSearchBound';
				'RelLineSearchBoundDuration';
				'ScaleProblem';
				'SpecifyConstraintGradient';
				'SpecifyConstraintHessian';
				'SpecifyObjectiveGradient';
				'SpecifyObjectiveHessian';
				'StepTolerance';
				'SubproblemAlgorithm';
				'TolConSQP';
				'TolPCG';
				'TolProjCG';
				'TolProjCGAbs';
				'TypicalX';
				'UseParallel';
				'Retries';
				'ProblemType'
			};
			values = {
				this.Solver;
				this.Algorithm;
				this.CheckGradients;
				this.ConstraintTolerance;
				this.Diagnostics;
				this.DiffMaxChange;
				this.DiffMinChange;
				this.Display;
				this.FiniteDifferenceStepSize;
				this.FiniteDifferenceType;
				this.FunctionTolerance;
				this.FunValCheck;
				this.HessianApproximation;
				this.HessianFcn;
				this.HessianMultiplyFcn;
				this.HessianPattern;
				this.HessianUpdate;
				this.HonorBounds;
				this.InitBarrierParam;
				this.InitDamping;
				this.InitTrustRegionRadius;
				this.JacobianMultiplyFcn;
				this.JacobianPattern;
				this.MaxFunctionEvaluations;
				this.MaxIterations;
				this.MaxPCGIter;
				this.MaxProjCGIter;
				this.MaxSQPIter;
				this.MaxTime;
				this.ObjectiveLimit;
				this.OptimalityTolerance;
				this.OutputFcn;
				this.PlotFcn;
				this.PrecondBandWidth;
				this.RelLineSearchBound;
				this.RelLineSearchBoundDuration;
				this.ScaleProblem;
				this.SpecifyConstraintGradient;
				this.SpecifyConstraintHessian;
				this.SpecifyObjectiveGradient;
				this.SpecifyObjectiveHessian;
				this.StepTolerance;
				this.SubproblemAlgorithm;
				this.TolConSQP;
				this.TolPCG;
				this.TolProjCG;
				this.TolProjCGAbs;
				this.TypicalX;
				this.UseParallel;
				this.Retries;
				this.ProblemType
			};
			% s = struct('Solver', this.Solver...) does not work here, because some properties can be empty cell arrays which leads to an empty struct with the defined fields
			% s = struct(this) does not work either because it generates a warning about preventing the object from hiding its implementation
			s = cell2struct(values, publicfields, 1);
		end
	end
	
	methods(Access=protected)
		function [options] = setoptions(this, options)
			%SETOPTIONS set options to input argument
			%	Input:
			%		this:		instance
			%		options:	options to set
			%	Output:
			%		options:	set options
			optionnames = this.options;
			isoptimoptions = false;
			isoptimset = false;
			isparticleswarm = false;
			if isa(options, 'optim.options.SolverOptions')
				solveroptionnames = this.optimoptionsname;
				isoptimoptions = true;
				type = optimization.options.OptionType.OPTIMOPTIONS;
				if isa(options, 'optim.options.Particleswarm')
					isparticleswarm = true;
				end
			elseif isstruct(options)
				if optimization.options.isoptimset(options)
					solveroptionnames = this.optimsetname;
					isoptimset = true;
					type = optimization.options.OptionType.OPTIMSET;
				elseif optimization.options.isgaoptimset(options)
					solveroptionnames = this.optimsetganame;
					isoptimset = true;
					type = optimization.options.OptionType.GAOPTIMSET;
				elseif optimization.options.ispsoptimset(options)
					solveroptionnames = this.optimsetpsname;
					isoptimset = true;
					type = optimization.options.OptionType.PSOPTIMSET;
				elseif optimization.options.issaoptimset(options)
					solveroptionnames = this.optimsetsaname;
					isoptimset = true;
					type = optimization.options.OptionType.SAOPTIMSET;
				else
					solveroptionnames = this.structname;
					type = optimization.options.OptionType.STRUCT;
				end
			else
				error('optimization:options:type', 'Undefined function of variable setoptions for input arguments of type ''%s''.', class(options));
			end
			[lia, locb] = ismember(optionnames, solveroptionnames(:, 1));
			setablenames = solveroptionnames(locb(lia), :);
			isstructtype = type == optimization.options.OptionType.STRUCT;
			% FMINCON and FMINUNC do not support variable MaxFunctionEvaluations, MaxIterations, etc in all versions, except for the default values
			isoptimtoolboxfunction = isa(this, 'optimization.options.fmincon') || isa(this, 'optimization.options.fminconglobal') || isa(this, 'optimization.options.fminunc') || isa(this, 'optimization.options.fminuncglobal') || isa(this, 'optimization.options.particleswarm') || isa(this, 'optimization.options.fminimax');
			for ii = 1:size(setablenames, 1)
				currentoption = this.(setablenames{ii, 1});
				if iscell(currentoption) || ~any(isnan(currentoption))
					if isoptimoptions
						% TODO: this is 4 times faster than isprop, see optimget
						try
							temp = options.(setablenames{ii, 2});
							hasprop = true;
						catch e
							if ~any(strcmpi(e.identifier, {'MATLAB:noSuchMethodOrField', 'MATLAB:nonExistentField'}))
								rethrow(e);
							end
							hasprop = false;
						end
					else
						hasprop = false;
					end
					if isoptimoptions && hasprop%isprop(options, setablenames{ii, 2})
						if ~isempty(currentoption)
							if isparticleswarm && strcmpi(setablenames{ii, 1}, 'Display')
								% WORKAROUND: optimoptions for particleswarm algorithm does not support iter-detailed and has to be converted
								options.(setablenames{ii, 2}) = this.convertProperty(setablenames{ii, 1}, currentoption, optimization.options.OptionType.STRUCT, isstructtype);
							else
								if isoptimtoolboxfunction
									isvariablesizeallowed = any(strcmpi(setablenames{ii, 1}, this.VARIABLESIZEALLOWED));
									options.(setablenames{ii, 2}) = this.convertProperty(setablenames{ii, 1}, currentoption, type, isstructtype || isvariablesizeallowed || strcmpi(setablenames{ii, 1}, 'MaxFunctionEvaluations'));
								else
									options.(setablenames{ii, 2}) = this.convertProperty(setablenames{ii, 1}, currentoption, type, isstructtype);
								end
							end
						end
					elseif ~isoptimoptions && isfield(options, setablenames{ii, 2})
						options.(setablenames{ii, 2}) = this.convertProperty(setablenames{ii, 1}, currentoption, type, isstructtype);
					elseif ~isoptimoptions && ~isfield(options, setablenames{ii, 2})
						options.(setablenames{ii, 2}) = this.convertProperty(setablenames{ii, 1}, currentoption, type, isstructtype);
					else
						error('optimization:options:definition', 'Property ''%s'' must exist, probably a programming error in one of the subclasses of ''optimization.options.Options''.', setablenames{ii, 2});
					end
				elseif ~isoptimoptions && ~isoptimset && ~isfield(options, setablenames{ii, 2})
					options.(setablenames{ii, 2}) = [];
				end
			end
		end
	end
	
	methods(Access=protected)
		function [names] = optimsetganames(~)
			%OPTIMSETGANAMES mapping from properties to gaoptimset names
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
				'MaxIterations',				'Generations';
				%'MaxPCGIter',					'MaxPCGIter';
				%'MaxProjCGIter',				'MaxProjCGIter';
				%'MaxSQPIter',					'MaxSQPIter';
				'MaxTime',						'TimeLimit';
				'ObjectiveLimit',				'FitnessLimit';
				'OptimalityTolerance',			'TolFun';
				'OutputFcn',					'OutputFcns';
				'PlotFcn',						'PlotFcns';
				%'PrecondBandWidth',			'PrecondBandWidth';
				%'RelLineSearchBound',			'RelLineSrchBnd';
				%'RelLineSearchBoundDuration',	'RelLineSrchBndDuration';
				%'ScaleProblem',				'ScaleProblem';
				%'SubproblemAlgorithm',			'SubproblemAlgorithm';
				%'SpecifyConstraintGradient',	'GradConstr';
				%'SpecifyObjectiveGradient',	'GradObj';
				%'StepTolerance',				'TolX';
				%'TolConSQP',					'TolConSQP';
				%'TolPCG',						'TolPCG';
				%'TolProjCG',					'TolProjCG';
				%'TolProjCGAbs',				'TolProjCGAbs';
				%'TypicalX',					'TypicalX';
				'UseParallel',					'UseParallel'
			};
		end
		
		function [names] = optimsetpsnames(~)
			%OPTIMSETPSNAMES mapping from properties to psoptimset names
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
				'MaxFunctionEvaluations',		'MaxFunEvals';
				'MaxIterations',				'MaxIter';
				%'MaxPCGIter',					'MaxPCGIter';
				%'MaxProjCGIter',				'MaxProjCGIter';
				%'MaxSQPIter',					'MaxSQPIter';
				'MaxTime',						'TimeLimit';
				%'ObjectiveLimit',				'FitnessLimit';
				'OptimalityTolerance',			'TolFun';
				'OutputFcn',					'OutputFcns';
				'PlotFcn',						'PlotFcns';
				%'PrecondBandWidth',			'PrecondBandWidth';
				%'RelLineSearchBound',			'RelLineSrchBnd';
				%'RelLineSearchBoundDuration',	'RelLineSrchBndDuration';
				%'ScaleProblem',				'ScaleProblem';
				%'SubproblemAlgorithm',			'SubproblemAlgorithm';
				%'SpecifyConstraintGradient',	'GradConstr';
				%'SpecifyObjectiveGradient',	'GradObj';
				'StepTolerance',				'TolX';
				%'TolConSQP',					'TolConSQP';
				%'TolPCG',						'TolPCG';
				%'TolProjCG',					'TolProjCG';
				%'TolProjCGAbs',				'TolProjCGAbs';
				%'TypicalX',					'TypicalX';
				'UseParallel',					'UseParallel'
			};
		end
		
		function [names] = optimsetsanames(~)
			%OPTIMSETSANAMES mapping from properties to saoptimset names
			%	Input:
			%		this:	instance
			%	Output:
			%		names:	mapping of names
			names = {
				%'Algorithm',					'Algorithm';
				%'CheckGradients',				'DerivativeCheck';
				%'ConstraintTolerance',			'TolCon';
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
				%'MaxIterations',				'MaxIter';
				%'MaxPCGIter',					'MaxPCGIter';
				%'MaxProjCGIter',				'MaxProjCGIter';
				%'MaxSQPIter',					'MaxSQPIter';
				'MaxTime',						'TimeLimit';
				'ObjectiveLimit',				'ObjectiveLimit';
				'OptimalityTolerance',			'TolFun';
				'OutputFcn',					'OutputFcns';
				'PlotFcn',						'PlotFcns';
				%'PrecondBandWidth',			'PrecondBandWidth';
				%'RelLineSearchBound',			'RelLineSrchBnd';
				%'RelLineSearchBoundDuration',	'RelLineSrchBndDuration';
				%'ScaleProblem',				'ScaleProblem';
				%'SubproblemAlgorithm',			'SubproblemAlgorithm';
				%'SpecifyConstraintGradient',	'GradConstr';
				%'SpecifyObjectiveGradient',	'GradObj';
				%'StepTolerance',				'TolX';
				%'TolConSQP',					'TolConSQP';
				%'TolPCG',						'TolPCG';
				%'TolProjCG',					'TolProjCG';
				%'TolProjCGAbs',				'TolProjCGAbs';
				%'TypicalX',					'TypicalX';
				%'UseParallel',					'UseParallel'
			};
		end
	end
	
	methods(Abstract=true)
		%POSSIBLEALGORITHMS list with possible algorithms for optimizer
		%	Input:
		%		this:		instance
		%	Output:
		%		algorithms:	possible algorithms
		[algorithms] = possiblealgorithms(this);
		
		%DISPLAYMAPPING mapping from optimoptions display names to solver display names
		%	Input:
		%		this:		instance
		%	Output:
		%		display:	mapping of display names
		[display] = displaymapping(this);
		
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
		%	Output:
		%		information:	unified optimization output
		[information] = formatOutput(this, errorcode, time, xmin, fmin, nvars, overalliterations, overallfunevals, retries, output);
	end
	
	methods(Abstract=true, Access=protected)
		%OPTIMOPTIONSNAMES mapping from properties to optimoptions names
		%	Input:
		%		this:	instance
		%	Output:
		%		names:	mapping of names
		[names] = optimoptionsnames(this);
		
		%OPTIMSETNAMES mapping from properties to optimset names
		%	Input:
		%		this:	instance
		%	Output:
		%		names:	mapping of names
		[names] = optimsetnames(this);
		
		%SSTRUCTNAMES mapping from properties to struct names
		%	Input:
		%		this:	instance
		%	Output:
		%		names:	mapping of names
		[names] = structnames(this);
	end
	
end
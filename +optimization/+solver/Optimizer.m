classdef(Enumeration) Optimizer < handle
	%OPTIMIZER enumeration with information about different optimization algorithms

	enumeration
		% fmincon standard settings
		FMINCON('fmincon', 'fmincon', [
			true,	true,	true,	true;
			false,	true,	false,	false
		], 'Algorithm', {'sqp', 'active-set', 'interior-point', 'trust-region-reflective'}, 'sqp', [
			optimization.options.ProblemType.CONSTRAINED;
			optimization.options.ProblemType.UNCONSTRAINED
		], optimization.options.ProblemType.CONSTRAINED, [
			true;
			false
		], 'optim.options.Fmincon', true, 'MaxIter', 'MaxFunEvals', 'TolFun', 'TolCon', 'TolX', [0, 0, 1, 2, 0, 0, 0, 0], 'Display', struct(...
				'UseParallel',	struct(...
					'value',	configuration.optimization.hasparallelsupport(true),...
					'isfactor',	false...
				),...
				'ScaleProblem',	struct(...
					'value',	'none',...
					'isfactor',	false...
				)...
			)...
		);
		% global fmincon standard settings
		FMINCONGLOBAL('fminconglobal', 'fminconglobal', [
			true,	true,	true,	true;
			false,	true,	false,	false
		], 'Algorithm', {'sqp', 'active-set', 'interior-point', 'trust-region-reflective'}, 'sqp', [
			optimization.options.ProblemType.CONSTRAINED;
			optimization.options.ProblemType.UNCONSTRAINED
		], optimization.options.ProblemType.CONSTRAINED, true(2, 1), 'optim.options.Fmincon', true, 'MaxIter', 'MaxFunEvals', 'TolFun', 'TolCon', 'TolX', [0, 0, 1, 2, 0, 0, 0, 0], 'Display', struct(...
				'UseParallel',	struct(...
					'value',	configuration.optimization.hasparallelsupport(true),...
					'isfactor',	false...
				),...
				'ScaleProblem',	struct(...
					'value',	'none',...
					'isfactor',	false...
				)...
			)...
		);
		% snopt standard settings
		SNOPT('snopt', 'snopt', [
			true,	false,	false,	true;
			true,	false,	false,	false
		], [], {}, '', [
			optimization.options.ProblemType.CONSTRAINED;
			optimization.options.ProblemType.UNCONSTRAINED
		], optimization.options.ProblemType.CONSTRAINED, false(2, 1), '', false, 'major_iterations_limit', [], 'major_optimality_tolerance', 'feasibility_tolerance', [], [0, 0, 2, 2, 0, 0, 0, 0], [], struct());
		% ipopt standard settings
		IPOPT('ipopt', 'ipopt', [
			true,	false,	false,	true;
			true,	false,	false,	false
		], [], {
			'ma27', 'ma57', 'ma77', 'ma86', 'ma97', 'pardiso', 'wsmp', 'mumps', 'custom'
		}, 'ma97', [
			optimization.options.ProblemType.CONSTRAINED;
			optimization.options.ProblemType.UNCONSTRAINED
		], optimization.options.ProblemType.CONSTRAINED, false(2, 1), '', true, 'MaxIter', [], 'tol', 'constr_viol_tol', [], [-1, -1, 3, 3, 1, 2, 1, 2], [], struct(...
				'acceptable_tol',				struct(...
					'value',	10,...
					'isfactor',	true...
				),...
				'acceptable_iter',				struct(...
					'value',	20,...
					'isfactor',	false...
				),...
				'acceptable_constr_viol_tol',	struct(...
					'value',	10,...
					'isfactor',	true...
				)...
			)...
		);
		% fminunc standard settings
		FMINUNC('fminunc', 'fminunc', [
			true,	true,	true,	true;
			false,	true,	false,	false
		], 'Algorithm', {'quasi-newton', 'trust-region'}, 'quasi-newton', optimization.options.ProblemType.UNCONSTRAINED, optimization.options.ProblemType.UNCONSTRAINED, [
			true;
			false
		], 'optim.options.Fminunc', true, 'MaxIter', 'MaxFunEvals', 'TolFun', '', 'TolX', [0, 0, 1, 2, 0, 0, 0, 0], 'Display', struct());
		% global fminunc standard settings
		FMINUNCGLOBAL('fminuncglobal', 'fminuncglobal', [
			true,	true,	true,	true;
			false,	true,	false,	false
		], 'Algorithm', {'quasi-newton', 'trust-region'}, 'quasi-newton', optimization.options.ProblemType.UNCONSTRAINED, optimization.options.ProblemType.UNCONSTRAINED, true(2, 1), 'optim.options.Fminunc', true, 'MaxIter', 'MaxFunEvals', 'TolFun', '', 'TolX', [0, 0, 1, 2, 0, 0, 0, 0], 'Display', struct());
		% fminunc standard settings
		FMINSEARCH('fminsearch', 'fminsearch', [
			true,	false,	true,	true;
			false,	false,	true,	false
		], [], {}, '', optimization.options.ProblemType.UNCONSTRAINED, optimization.options.ProblemType.UNCONSTRAINED, false(2, 1), '', false, 'MaxIter', 'MaxFunEvals', 'TolFun', '', 'TolX', [0, 0, 1, 2, 0, 0, 0, 0], 'Display', struct());
		% ga standard settings
		GA('ga', 'ga', [
			true,	false,	true,	true;
			false,	false,	true,	false
		], [], {'ga', 'gamulti'}, '', [
			optimization.options.ProblemType.CONSTRAINED;
			optimization.options.ProblemType.UNCONSTRAINED;
			optimization.options.ProblemType.CONSTRAINEDMULTI;
			optimization.options.ProblemType.UNCONSTRAINEDMULTI
		], optimization.options.ProblemType.CONSTRAINED, true(2, 1), '', false, 'MaxIter', 'MaxFunEvals', 'TolFun', '', 'TolX', [0, 0, 1, 2, 0, 0, 0, 0], 'Display', struct());
		% particleswarm standard settings
		PARTICLESWARM('particleswarm', 'particleswarm', [
			true,	false,	true,	true;
			false,	false,	true,	false
		], [], {}, '', optimization.options.ProblemType.UNCONSTRAINED, optimization.options.ProblemType.UNCONSTRAINED, true(2, 1), 'optim.options.Particleswarm', false, 'MaxIter', 'MaxFunEvals', 'TolFun', '', 'TolX', [0, 0, 1, 2, 0, 0, 0, 0], 'Display', struct());
		% patternsearch standard settings
		PATTERNSEARCH('patternsearch', 'patternsearch', [
			true,	false,	true,	true;
			false,	false,	true,	false
		], [], {}, '', [
			optimization.options.ProblemType.CONSTRAINED;
			optimization.options.ProblemType.UNCONSTRAINED
		], optimization.options.ProblemType.CONSTRAINED, true(2, 1), '', false, 'MaxIter', 'MaxFunEvals', 'TolFun', '', 'TolX', [0, 0, 1, 2, 0, 0, 0, 0], 'Display', struct());
		% simulanneal standard settings
		SIMULANNEAL('simulanneal', 'simulanneal', [
			true,	false,	true,	true;
			false,	false,	true,	false
		], [], {}, '', optimization.options.ProblemType.UNCONSTRAINED, optimization.options.ProblemType.UNCONSTRAINED, true(2, 1), '', false, 'MaxIter', 'MaxFunEvals', 'TolFun', '', 'TolX', [0, 0, 1, 2, 0, 0, 0, 0], 'Display', struct());
		NLOPTUNC('nloptunc', 'nloptunc', [
			true,	false,	false,	true;
			true,	false,	false,	false
		], [], {
			'stogo', 'stogo_rand', 'crs2', 'direct', 'direct_l', 'direct_l_noscal', 'direct_l_rand', 'direct_l_rand_noscal', 'direct_noscal', 'esch', 'direct_orig', 'direct_l_orig', 'ccsaq', 'lbfgs', 'lbfgs_nocedal', 'tnewton', 'tnewton_precond', 'tnewton_precond_restart', 'tnewton_restart', 'var1', 'var2', 'bobyoa', 'neldermead', 'newuoa', 'newuoa_bound', 'praxis', 'subplex'
		}, 'praxis', optimization.options.ProblemType.UNCONSTRAINED, optimization.options.ProblemType.UNCONSTRAINED, false(2, 1), '', false, 'MaxIter', 'maxevals', 'ftol_rel', '', 'xtol_rel', [0, 0, 1, 2, 0, 0, 0, 0], 'Display', struct());
		NLOPTCON('nloptcon', 'nloptcon', [
			true,	false,	false,	true;
			true,	false,	false,	false
		], [], {
			'isres', 'auglag', 'auglag_eq', 'mma', 'slsqp', 'cobyla'
		}, 'auglag', [
			optimization.options.ProblemType.CONSTRAINED;
			optimization.options.ProblemType.UNCONSTRAINED
		], optimization.options.ProblemType.CONSTRAINED, false(2, 1), '', false, 'MaxIter', 'maxevals', 'ftol_rel', '', 'xtol_rel', [0, 0, 1, 2, 0, 0, 0, 0], 'Display', struct());
		NLOPTUNCGLOBAL('nloptuncglobal', 'nloptuncglobal', [
			true,	false,	false,	true;
			true,	false,	false,	false
		], [], {
			'mlsl', 'mlsl_lds'
		}, 'mlsl', optimization.options.ProblemType.UNCONSTRAINED, optimization.options.ProblemType.UNCONSTRAINED, false(2, 1), '', false, 'MaxIter', 'maxevals', 'ftol_rel', '', 'xtol_rel', [0, 0, 1, 2, 0, 0, 0, 0], 'Display', struct());
		NLOPTCONGLOBAL('nloptconglobal', 'nloptconglobal', [
			true,	false,	false,	true;
			true,	false,	false,	false
		], [], {
			'auglag', 'auglag_eq'
		}, 'auglag', [
			optimization.options.ProblemType.CONSTRAINED;
			optimization.options.ProblemType.UNCONSTRAINED
		], optimization.options.ProblemType.CONSTRAINED, false(2, 1), '', false, 'MaxIter', 'maxevals', 'ftol_rel', '', 'xtol_rel', [0, 0, 1, 2, 0, 0, 0, 0], 'Display', struct());
		% fminimax standard settings
		FMINIMAX('fminimax', 'fminimax', [
			true,	true,	true,	true;
			false,	true,	false,	false
		], 'Algorithm', {'fminimax'}, [], [
			optimization.options.ProblemType.CONSTRAINED;
			optimization.options.ProblemType.UNCONSTRAINED;
			optimization.options.ProblemType.UNCONSTRAINEDMULTI;
			optimization.options.ProblemType.CONSTRAINEDMULTI
		], optimization.options.ProblemType.CONSTRAINEDMULTI, [
			true;
			false
		], 'optim.options.Fminimax', false, 'MaxIter', 'MaxFunEvals', 'TolFun', 'TolCon', 'TolX', [0, 0, 1, 2, 0, 0, 0, 0], 'Display', struct(...
				'UseParallel',	struct(...
					'value',	configuration.optimization.hasparallelsupport(true),...
					'isfactor',	false...
				)...
			)...
		);
		% PPPBox standard settings
		PPPBOX('pppbox', 'pppbox', [
			true,	false,	false,	true;
			true,	false,	false,	false
		], 'Algorithm', {'single', 'multi'}, 'multi', [
			optimization.options.ProblemType.UNCONSTRAINED;
			optimization.options.ProblemType.UNCONSTRAINEDMULTI
		], optimization.options.ProblemType.UNCONSTRAINEDMULTI, false(2, 1), '', false, 'MAXITER', [], 'OPTTOL', [], [], [0, 0, 2, 2, 0, 0, 0, 0], [], struct());
		% SLP-GS standard settings
		SLPGS('slpgs', 'slpgs', [
			true,	false,	false,	true;
			true,	false,	false,	false
		], 'Algorithm', {'slpgs'}, 'slpgs', [
			optimization.options.ProblemType.CONSTRAINED;
			optimization.options.ProblemType.UNCONSTRAINED
		], optimization.options.ProblemType.CONSTRAINED, false(2, 1), '', false, 'iter_max', [], 'stat_tol', 'ineq_tol', [], [0, 0, 2, 2, 0, 0, 0, 0], [], struct());
		% SQP-GS standard settings
		SQPGS('sqpgs', 'sqpgs', [
			true,	false,	false,	true;
			true,	false,	false,	false
		], 'Algorithm', {'sqpgs'}, 'sqpgs', [
			optimization.options.ProblemType.CONSTRAINED;
			optimization.options.ProblemType.UNCONSTRAINED
		], optimization.options.ProblemType.CONSTRAINED, false(2, 1), '', false, 'iter_max', [], 'stat_tol', 'ineq_tol', [], [0, 0, 2, 2, 0, 0, 0, 0], [], struct());
		% SC-BFGS standard settings
		SCBFGS('scbfgs', 'scbfgs', [
			true,	false,	false,	true;
			true,	false,	false,	false
		], 'Algorithm', {}, '', [
			optimization.options.ProblemType.UNCONSTRAINED
		], optimization.options.ProblemType.UNCONSTRAINED, false(2, 1), '', false, 'iter_max', [], 'stat_tol', 'ineq_tol', [], [0, 0, 2, 2, 0, 0, 0, 0], [], struct());
		% ksopt standard settings
		KSOPT('ksopt', 'ksopt', [
			true,	true,	true,	true;
			false,	true,	false,	false
		], 'Algorithm', {'ksopt'}, 'ksopt', [
			optimization.options.ProblemType.CONSTRAINED;
			optimization.options.ProblemType.UNCONSTRAINED;
			optimization.options.ProblemType.UNCONSTRAINEDMULTI;
			optimization.options.ProblemType.CONSTRAINEDMULTI
		], optimization.options.ProblemType.CONSTRAINED, false(2, 1), '', false, 'MaxIter', 'MaxFunEvals', 'TolFun', 'TolCon', 'TolX', [0, 0, 1, 2, 0, 0, 0, 0], 'Display', struct());
		% solvopt standard settings
		SOLVOPT('solvopt', 'solvopt', [
			true,	false,	false,	true;
			true,	false,	false,	false
		], 'Algorithm', {'solvopt'}, 'solvopt', [
			optimization.options.ProblemType.CONSTRAINED;
			optimization.options.ProblemType.UNCONSTRAINED
		], optimization.options.ProblemType.CONSTRAINED, false(2, 1), '', false, 'MaxIter', 'MaxFunEvals', 'TolFun', 'TolCon', 'TolX', [0, 0, 1, 2, 0, 0, 0, 0], 'Display', struct());
		% minfunc standard settings
		MINFUNC('minfunc', 'minfunc', [
			true,	false,	false,	true;
			true,	false,	false,	false
		], 'Algorithm', {
			'sd', 'csd', 'bb', 'cg', 'scg', 'pcg', 'lbfgs', 'newton0', 'pnewton0', 'qnewton', 'mnewton', 'newton'
		}, 'lbfgs', optimization.options.ProblemType.UNCONSTRAINED, optimization.options.ProblemType.UNCONSTRAINED, false(2, 1), '', false, 'MaxIter', 'MaxFunEvals', 'TolFun', '', 'TolX', [0, 0, 1, 2, 0, 0, 0, 0], 'Display', struct());
	end

	properties(Access=private)
		% fieldname in parameterstructure of YALMIP
		fieldname,
		% type the options are supplied to the underlying algorithm [struct, optimoptions, optimset, yalmipstruct]
		optiontype,
		% fieldname of algorithm name
		algorithm,
		% possible algorithm choices
		algorithmchoices,
		% array of ProblemTypes the solver supports
		problemtypes,
		% preferred ProblemType the solver solves
		preferredproblemtype,
		% indicator for Optimization Toolbox and Global Optimization Toolbox dependency
		needstoolbox = false(2, 1),
		% name of corresponding optimoptions class
		optimoptionsclassname,
		% fieldname of maximum number of iterations
		maxitername,
		% fieldname of maximum number of function evaluations
		maxfunevalsname,
		% fieldname of function tolerance
		tolfunname,
		% fieldname of contraint tolerance
		tolconname,
		% fieldname of tolerance of relative change of function
		tolxname,
		% mapping of 'display' values to 'verbose' values of YALMIP
		verbose,
		% fieldname of display settings
		displayname,
		% additional constant options to add to the optimization settings
		options,
		% indicator if the solver support hessian information
		supportshessian
	end

	properties(SetAccess=private)
		% name of optimization algorithm
		algorithmname,
		% default algorithm
		defaultalgorithm
	end

	methods(Static=true)
		function [default] = getDefaultValue()
			%GETDEFAULTVALUE return default value for Optimizer
			%	Output:
			%		default:	default optimizer
			persistent def;
			if isempty(def)
				if ~isempty(which('snopt'))
					def = optimization.solver.Optimizer.SNOPT;
				else
					v = ver;
					[installedToolboxes{1:length(v)}] = deal(v.Name);
					hasoptimization = ismember('Optimization Toolbox', installedToolboxes);
					hasoptimization = hasoptimization && logical(license('test', 'Optimization_Toolbox'));
					if hasoptimization
						def = optimization.solver.Optimizer.FMINCON;
					else
						def = optimization.solver.Optimizer.FMINSEARCH;
					end
				end
			end
			default = def;
		end

		function [id] = fromname(name)
			%FROMNAME create OptimizerID from name
			%	Input:
			%		name:	name of a Optimizer
			%	Output:
			%		id:		OptimizerID, if one of the specified name exists
			if ~ischar(name)
				if isa(name, 'optimization.solver.Optimizer')
					id = name;
					return;
				else
					error('optimization:solver:Optimizer:name', 'Optimizer name is invalid.');
				end
			end
			enum = enumeration('optimization.solver.Optimizer');
			id = [];
			for i = 1:length(enum) %#ok<FORPF> no parfor for early exit loops
				if strcmpi(enum(i).algorithmname, name)
					id = enum(i);
					return;
				end
			end
			if isempty(id)
				error('optimization:solver:Optimizer:name', 'Optimizer does not exist.');
			end
		end
	end

	methods(Access=private)
		function [this] = Optimizer(algorithmname, fieldname, optiontype, algorithm, algorithmchoices, defaultalgorithm, problemtypes, preferredproblemtype, needstoolbox, optimoptionsclassname, supportshessian, maxitername, maxfunevalsname, tolfunname, tolconname, tolxname, verbose, displayname, options)
			%OPTIMIZER return new identifier for an optimization algorithm
			%	Input:
			%		algorithmane:			name of optimization algorithm
			%		fieldname:				fieldname in parameterstructure of YALMIP
			%		optiontype:				type the options are supplied to the underlying algorithm [struct, optimoptions, optimset, yalmipstruct]
			%		algorithmn:				fieldname of algorithm name
			%		algorithmchoices:		possible algorithm choices
			%		defaultalgorithm:		default algorithm
			%		problemtypes:			array of ProblemTypes the solver supports
			%		preferredproblemtype:	preferred ProblemType the solver solves
			%		needstoolbox:			indicator for Optimization Toolbox and Global Optimization Toolbox dependency
			%		optimoptionsclassname:	name of corresponding optimoptions class
			%		supportshessian:		indicator if hessian information is supported
			%		maxitername:			fieldname of maximum number of iterations
			%		maxfunevalsname:		fieldname of maximum number of function evaluations
			%		tolfunname:				fieldname of function tolerance
			%		tolconname:				fieldname of contraint tolerance
			%		tolxname:				fieldname of tolerance of relative change of function
			%		verbose:				mapping of 'display' values to 'verbose' values of YALMIP
			%		displayname:			fieldname of display settings
			%		options:				additional constant options to add to the optimization settings
			%	Output:
			%		this:					instance
			this.algorithmname = algorithmname;
			this.fieldname = fieldname;
			this.optiontype = optiontype;
			this.algorithm = algorithm;
			this.algorithmchoices = algorithmchoices;
			if isempty(defaultalgorithm)
				this.defaultalgorithm = algorithmname;
			else
				if ~isempty(algorithmchoices) && ~ismember(defaultalgorithm, algorithmchoices)
					error('optimization:solver:Optimizer', 'Default algorithm is not in list of allowed algorithms.');
				end
				this.defaultalgorithm = defaultalgorithm;
			end
			this.problemtypes = problemtypes;
			if ~any(problemtypes == preferredproblemtype)
				error('optimization:solver:Optimizer', 'Preferred problem type is not in list of supported problem types.');
			end
			this.preferredproblemtype = preferredproblemtype;
			this.needstoolbox = needstoolbox;
			this.optimoptionsclassname = optimoptionsclassname;
			this.supportshessian = supportshessian;
			this.maxitername = maxitername;
			this.maxfunevalsname = maxfunevalsname;
			this.tolfunname = tolfunname;
			this.tolconname = tolconname;
			this.tolxname = tolxname;
			this.verbose = verbose;
			this.displayname = displayname;
			this.options = options;
		end
	end

	methods
		function [algorithm] = getDefaultAlgorithm(this)
			%GETDEFAULTALGORITHM return default algorithm for current optimizer
			%	Input:
			%		this:		instance
			%	Output:
			%		algorithm:	default algorithm
			algorithm = this.defaultalgorithm;
		end

		function [algorithms] = getAlgorithmChoices(this)
			%GETALGORITHMCHOICES return cell array of supported algorithm choices for current optimizer
			%	Input:
			%		this:		instance
			%	Output:
			%		algorithms:	cell array with algorithms
			algorithms = this.algorithmchoices;
		end

		function [supported] = getSupportedProblemTypes(this)
			%GETSUPPORTEDPROBLEMTYPES return list of ProblemTypes the current optimizer supports
			%	Input:
			%		this:		instance
			%	Output:
			%		supported:	supported ProblemTypes
			supported = this.problemtypes;
		end

		function [preferred] = getPreferredProblemType(this)
			%GETPREFERREDPROBLEMTYPE return the preferred ProblemType of the current optimizer
			%	Input:
			%		this:		instance
			%	Output:
			%		preferred:	preferred ProblemType
			preferred = this.preferredproblemtype;
		end

		function [needsoptimization] = needsOptimizationToolbox(this)
			%NEEDSOPTIMIZATIONTOOLBOX return if the solver needs the Optimization Toolbox
			%	Input:
			%		this:				instance
			%	Output:
			%		needsoptimization:	true, if the solver needs the Optimization Toolbox, else false
			needsoptimization = cat(2, this(:).needstoolbox);
			needsoptimization = reshape(needsoptimization(1, :), size(this));
		end

		function [needsglobaloptimization] = needsGlobalOptimizationToolbox(this)
			%NEEDSGLOBALOPTIMIZATIONTOOLBOX return if the solver needs the Global Optimization Toolbox
			%	Input:
			%		this:						instance
			%	Output:
			%		needsglobaloptimization:	true, if the solver needs the Global Optimization Toolbox, else false
			needsglobaloptimization = cat(2, this(:).needstoolbox);
			needsglobaloptimization = reshape(needsglobaloptimization(2, :), size(this));
		end

		function [classname] = getOptimoptionsClassname(this)
			%GETOPTIMOPTIONSCLASSNAME return the corresponding optimoptions classname of the current optimizer
			%	Input:
			%		this:		instance
			%	Output:
			%		classname:	corresponding optimoptions classname
			classname = this.optimoptionsclassname;
		end

		function [supportshessian] = getHessianSupport(this)
			%GETHESSIANSUPPORT return if the solver supports hessian information
			%	Input:
			%		this:				instance
			%	Output:
			%		supportshessian:	solver supports hessian
			supportshessian = this.supportshessian;
		end

		function [equal] = eq(this, that)
			%EQ return if the object is equal to another object
			%	Input:
			%		this:	instance
			%		that:	instance to compare to
			%	Output:
			%		equal:	the two object are equal, if they have the same algorithm- and fieldname or if one of them is a char equal to the algorithmname of the other
			thisopt = isa(this, 'optimization.solver.Optimizer');
			thatopt = isa(that, 'optimization.solver.Optimizer');
			if thisopt && thatopt
				if isscalar(this) && isscalar(that)
					equal = strcmpi(this.algorithmname, that.algorithmname) && strcmpi(this.fieldname, that.fieldname);
				elseif isscalar(this)
					equal = strcmpi(this.algorithmname, {that.algorithmname}) & strcmpi(this.fieldname, {that.fieldname});
					equal = reshape(equal, size(that));
				elseif isscalar(that)
					equal = strcmpi({this.algorithmname}, that.algorithmname) & strcmpi({this.fieldname}, that.fieldname);
					equal = reshape(equal, size(this));
				else
					equal = builtin('eq', {this.algorithmname}, {that.algorithmname});
					equal = reshape(equal, size(this));
				end
			elseif thisopt && ~thatopt && ischar(that)
				equal = strcmpi(this.algorithmname, that);
			elseif ~thisopt && thatopt && ischar(this)
				equal = strcmpi(that.algorithmname, this);
			else
				equal = builtin('eq', this, that);
			end
		end

		function [c] = char(this)
			%CHAR convert Optimizer object to char
			%	Input:
			%		this:	instance
			%	Output:
			%		c:		character representation of the instance
			if isscalar(this)
				c = upper(this.algorithmname);
			else
				t = reshape(this, [], 1);
				ctemp = cell(size(t, 1), 1);
				len = zeros(size(t, 1), 1);
				parfor ii = 1:size(t, 1)
					ctemp{ii, 1} = char(t(ii, 1));
					len(ii, 1) = length(ctemp{ii, 1});
				end
				maxlen = max(len);
				c = char(zeros(size(ctemp, 1), maxlen));
				parfor ii = 1:size(c, 1)
					c(ii, :) = [ctemp{ii, 1}, 32*ones(1, maxlen - length(ctemp{ii, 1}))];
				end
			end
		end

		function [settings] = getoptions(this, as)
			%GETOPTIONS return options
			%	Input:
			%		this:		instance
			%	Output:
			%		settings:	constant options
			warning('optimization:solver:Optimizer', 'This method is deprecated.');
			if nargin <= 1
				as = this.optiontype(2, :);
			end
			if ~ischar(as) && ~islogical(as)
				switch lower(class(as))
					case 'struct'
						as = [
							true, false, false, false
						];
					case 'optimset'
						as = [
							false, false, true, false
						];
					case 'optimoptions'
						as = [
							false, true, false, false
						];
					case 'yalmip'
						as = [
							false, false, false, true
						];
					otherwise
						as = [
							true, false, false, false
						];
				end
			end
			if ischar(as)
				switch lower(as)
					case 'struct'
						as = [
							true, false, false, false
						];
					case 'optimset'
						as = [
							false, false, true, false
						];
					case 'optimoptions'
						as = [
							false, true, false, false
						];
					case 'yalmip'
						as = [
							false, false, false, true
						];
					otherwise
						as = [
							true, false, false, false
						];
				end
			elseif ~islogical(as)
				if isa(as, 'optim.options.SolverOptions')
					as = [
						false, true, false, false
					];
				else
					error('optimization:solver:Optimizer:get', 'Undefined function or variable for input arguments of type ''%s''.', class(as));
				end
			end
			if size(as, 1) ~= 1 || size(as, 2) ~= 4
				error('optimization:solver:Optimizer:get', 'Invalid desired return value.');
			end
			if sum(as) > 1
				error('optimization:solver:Optimizer:get', 'Only one desired return value is possible');
			end
			if ~any(as(this.optiontype(1, :)))
				temp = {'struct', 'optimoptions', 'optimset', 'yalmipstruct'};
				error('optimization:solver:Optimizer:get', 'The desired return value ''%s'' is not supported by the selected algorithm.', temp{as});
			end
			if as(1)
				settings = struct();
			elseif as(2)
				settings = optimoptions(this.algorithmname);
			elseif as(3)
				settings = optimset(this.algorithmname);
			elseif as(4)
				settings = struct();
			else
				error('optimization:solver:Optimizer:get', 'Undefined function or variable for input arguments of type ''%s''.', class(as));
			end
			settings = this.addoptions(settings);
		end

		function [settings] = getconstantoptions(this)
			%GETCONSTANTOPTIONS return constant options
			%	Input:
			%		this:		instance
			%	Output:
			%		settings:	constant options
			warning('optimization:solver:Optimizer', 'This method is deprecated.');
			settings = struct();
			opt = this.options;
			fields = fieldnames(opt);
			for i = 1:length(fields) %#ok<FORPF> no parfor for changes in structures
				if opt.(fields{i}).isfactor
					settings.(fields{i}) = opt.(fields{i}).value;
				else
					settings.(fields{i}) = opt.(fields{i}).value;
				end
			end
		end

% 		function [information] = formatOutput(this, errorcode, output, time, nvars)
% 			%FORMATOUTPUT unify output of optimization
% 			%	Input:
% 			%		this:			instance
% 			%		errorcode:		exit code of optimization
% 			%		output:			optimization output
% 			%		time:			time for optimization
% 			%		nvars:			number of optimization variables
% 			%	Output:
% 			%		information:	unified optimization output
% 			information = struct(...
% 				't',			time,...
% 				'Nvar',			nvars,...
% 				'iter',			NaN,...
% 				'eval',			NaN,...
% 				'feasibility',	errorcode,...
% 				'fmin',			NaN,...
% 				'constr',		NaN,...
% 				'opt',			NaN,...
% 				'output',		''...
% 			);
% 			if ~isempty(this.outputname) && isfield(output, this.outputname)
% 				temp = output.(this.outputname);
% 				temp.solvertime = output.solvertime;
% 				temp.fulloutput = output;
% 				information = this.outputhandle(information, temp);
% 			end
% 		end

		function [settings] = setDisplay(this, settings, display)
			%SETDISPLAY set display settings
			%	Input:
			%		this:		instance
			%		settings:	optimization settings as structure for optimset, YALMIP or optimoptions object
			%		display:	value of 'display' like used with fmincon
			%	Output:
			%		settings:	optimization settings
			warning('optimization:solver:Optimizer', 'This method is deprecated.');
			if ~ischar(display)
				error('optimization:solver:Optimizer:display', 'Display level is invalid.');
			end
			displayoptions = {'off', 'none', 'iter', 'iter-detailed', 'notify', 'notify-detailed', 'final', 'final-detailed'};
			idx = cellfun(@(x) strcmpi(x, display), displayoptions, 'UniformOutput', true);
			if ~any(idx)
				error('optimization:solver:Optimizer:display', 'Display level is invalid.');
			end
			if isstruct(settings)
				if isfield(settings, 'verbose')% YALMIP
					settings.verbose = this.verbose(idx);
					if ~isempty(this.displayname)
						settings.(this.fieldname).(this.displayname) = displayoptions{idx};
					end
					if idx(1) || idx(2)
						settings.warning		= 1;
						settings.beeponproblem	= false;
						settings.showprogress	= false;
					else
						settings.warning		= 1;
						settings.beeponproblem	= true;
						settings.showprogress	= true;
					end
				else% optimset
					if ~isempty(this.displayname)
						settings.(this.displayname) = displayoptions{idx};
					end
				end
			elseif isa(settings, 'optim.options.SolverOptions')% optimoptions
				if ~isempty(this.displayname)
					settings.(this.displayname) = displayoptions{idx};
				end
			else
				error('optimization:solver:Optimizer:setting', 'The supplied optimization settings are invalid.');
			end
		end

		function [settings] = setAlgorithm(this, settings, algorithm)
			%SETALGORITHM set optimization algorith in optimization settings
			%	Input:
			%		this:		instance
			%		settings:	optimization settings as structure for optimset, YALMIP or optimoptions object
			%		algorithm:	optimization algorithm to use
			%	Output:
			%		settings:	optimization settings
			warning('optimization:solver:Optimizer', 'This method is deprecated.');
			if ~isempty(this.algorithm) && ~any(isnan(algorithm)) && ~isempty(algorithm)
				if ~isempty(this.algorithmchoices)
					idx = cellfun(@(x) strcmpi(x, algorithm), this.algorithmchoices, 'UniformOutput', true);
					if ~any(idx)
						error('optimization:solver:Optimizer:algorithm', 'Algorithm %s is not valid for optimizer %s.', algorithm, this.algorithmname);
					end
					if isstruct(settings)
						if isfield(settings, this.fieldname)% YALMIP
							settings.(this.fieldname).(this.algorithm) = this.algorithmchoices{idx};
						else% optimset
							settings.(this.algorithm) = this.algorithmchoices{idx};
						end
					elseif isa(settings, 'optim.options.SolverOptions')% optimoptions
						settings.(this.algorithm) = this.algorithmchoices{idx};
					else
						error('optimization:solver:Optimizer:setting', 'The supplied optimization settings are invalid.');
					end
				end
			end
		end

		function [settings] = setMaxIter(this, settings, maxiter)
			%SETMAXITER set maximum number of iterations in optimization settings
			%	Input:
			%		this:		instance
			%		settings:	optimization settings as structure for optimset, YALMIP or optimoptions object
			%		maxiter:	maximum number of iterations
			%	Output:
			%		settings:	optimization settings
			warning('optimization:solver:Optimizer', 'This method is deprecated.');
			if ~isempty(this.maxitername) && ~isnan(maxiter)
				if isstruct(settings)
					if isfield(settings, this.fieldname)% YALMIP
						settings.(this.fieldname).(this.maxitername) = maxiter;
					else% optimset
						settings.(this.maxitername) = maxiter;
					end
				elseif isa(settings, 'optim.options.SolverOptions')% optimoptions
					if isprop(settings, this.maxitername)
						settings.(this.maxitername) = maxiter;
					else
						warning('optimization:solver:Optimizer:set:tolcon', 'Options for algorithm %s do not support setting %s.', lower(strrep(class(settings), 'optim.options.', '')), this.maxitername);
					end
				else
					error('optimization:solver:Optimizer:setting', 'The supplied optimization settings are invalid.');
				end
			end
		end

		function [settings] = setMaxFunEvals(this, settings, maxfunevals)
			%SETMAXFUNEVALS set maximum number of function evaluations in optimization settings
			%	Input:
			%		this:			instance
			%		settings:		optimization settings as structure for optimset, YALMIP or optimoptions object
			%		maxfunevals:	maximum number of function evaluations
			%	Output:
			%		settings:	optimization settings
			warning('optimization:solver:Optimizer', 'This method is deprecated.');
			if ~isempty(this.maxfunevalsname) && ~isnan(maxfunevals)
				if isstruct(settings)
					if isfield(settings, this.fieldname)% YALMIP
						settings.(this.fieldname).(this.maxfunevalsname) = maxfunevals;
					else% optimset
						settings.(this.maxfunevalsname) = maxfunevals;
					end
				elseif isa(settings, 'optim.options.SolverOptions')% optimoptions
					if isprop(settings, this.maxfunevalsname)
						settings.(this.maxfunevalsname) = maxfunevals;
					else
						warning('optimization:solver:Optimizer:set:tolcon', 'Options for algorithm %s do not support setting %s.', lower(strrep(class(settings), 'optim.options.', '')), this.maxfunevalsname);
					end
				else
					error('optimization:solver:Optimizer:setting', 'The supplied optimization settings are invalid.');
				end
			end
		end

		function [settings] = setTolFun(this, settings, tolfun)
			%SETTOLFUN set fuction tolerance in optimization settings
			%	Input:
			%		this:		instance
			%		settings:	optimization settings as structure for optimset, YALMIP or optimoptions object
			%		tolfun:		function tolerance
			%	Output:
			%		settings:	optimization settings
			warning('optimization:solver:Optimizer', 'This method is deprecated.');
			if ~isempty(this.tolfunname)
				if isstruct(settings)
					if isfield(settings, this.fieldname)% YALMIP
						settings.(this.fieldname).(this.tolfunname) = tolfun;
					else% optimset
						settings.(this.tolfunname) = tolfun;
					end
				elseif isa(settings, 'optim.options.SolverOptions')% optimoptions
					if isprop(settings, this.tolfunname)
						settings.(this.tolfunname) = tolfun;
					else
						warning('optimization:solver:Optimizer:set:tolcon', 'Options for algorithm %s do not support setting %s.', lower(strrep(class(settings), 'optim.options.', '')), this.tolfunname);
					end
				else
					error('optimization:solver:Optimizer:setting', 'The supplied optimization settings are invalid.');
				end
			end
		end

		function [settings] = setTolCon(this, settings, tolcon)
			%SETTOLCON set constraint tolerance in optimization settings
			%	Input:
			%		this:		instance
			%		settings:	optimization settings as structure for optimset, YALMIP or optimoptions object
			%		tolcon:		constraint tolerance
			%	Output:
			%		settings:	optimization settings
			warning('optimization:solver:Optimizer', 'This method is deprecated.');
			if ~isempty(this.tolconname)
				if isstruct(settings)
					if isfield(settings, this.fieldname)% YALMIP
						settings.(this.fieldname).(this.tolconname) = tolcon;
					else% optimset
						settings.(this.tolconname) = tolcon;
					end
				elseif isa(settings, 'optim.options.SolverOptions')% optimoptions
					if isprop(settings, this.tolconname)
						settings.(this.tolconname) = tolcon;
					else
						warning('optimization:solver:Optimizer:set:tolcon', 'Options for algorithm %s do not support setting %s.', lower(strrep(class(settings), 'optim.options.', '')), this.tolconname);
					end
				else
					error('optimization:solver:Optimizer:setting', 'The supplied optimization settings are invalid.');
				end
			end
		end

		function [settings] = setTolX(this, settings, tolx)
			%SETTOLX set tolerance for relative change in function in optimization settinge
			%	Input:
			%		this:		instance
			%		settings:	optimization settings as structure for optimset, YALMIP or optimoptions object
			%		tolx:		tolerance for relative change in function
			%	Output:
			%		settings:	optimization settings
			warning('optimization:solver:Optimizer', 'This method is deprecated.');
			if ~isempty(this.tolxname) && ~isnan(tolx)
				if isstruct(settings)
					if isfield(settings, this.fieldname)% YALMIP
						settings.(this.fieldname).(this.tolxname) = tolx;
					else% optimset
						settings.(this.tolxname) = tolx;
					end
				elseif isa(settings, 'optim.options.SolverOptions')% optimoptions
					if isprop(settings, this.tolxname)
						settings.(this.tolxname) = tolx;
					else
						warning('optimization:solver:Optimizer:set:tolcon', 'Options for algorithm %s do not support setting %s.', lower(strrep(class(settings), 'optim.options.', '')), this.tolxname);
					end
				else
					error('optimization:solver:Optimizer:setting', 'The supplied optimization settings are invalid.');
				end
			end
		end

		function [settings] = addoptions(this, settings)
			%ADDOPTIONS add constant options to optimization settinge
			%	Input:
			%		this:		instance
			%		settings:	optimization settings as structure for optimset, YALMIP or optimoptions object
			%	Output:
			%		settings:	optimization settinge with constant options added
			warning('optimization:solver:Optimizer', 'This method is deprecated.');
			if isempty(this.fieldname)
				return;
			end
% 			if ~isfield(settings, this.fieldname)
% 				error('optimization:solver:Optimizer:setting', 'The supplied optimization settings are invalid.');
% 			end
			opt = this.options;
			fields = fieldnames(opt);
			if isstruct(settings)
				if isfield(settings, this.fieldname)% YALMIP
					tempstruct = settings.(this.fieldname);
					for i = 1:length(fields) %#ok<FORPF> no parfor for changes in structures
						if isfield(tempstruct, fields{i})
							if opt.(fields{i}).isfactor
								tempstruct.(fields{i}) = tempstruct.(fields{i})*opt.(fields{i}).value;
							else
								tempstruct.(fields{i}) = opt.(fields{i}).value;
							end
						else
							if opt.(fields{i}).isfactor
								error('optimization:solver:Optimizer:setting', 'The supplied optimization settings are invalid.');
							else
								tempstruct.(fields{i}) = opt.(fields{i}).value;
							end
						end
					end
					settings.(this.fieldname) = tempstruct;
				else% optimset
					for i = 1:length(fields) %#ok<FORPF> no parfor for changes in structures
						if isfield(settings, fields{i})
							if opt.(fields{i}).isfactor
								settings.(fields{i}) = settings.(fields{i})*opt.(fields{i}).value;
							else
								settings.(fields{i}) = opt.(fields{i}).value;
							end
						else
							if opt.(fields{i}).isfactor
								error('optimization:solver:Optimizer:setting', 'The supplied optimization settings are invalid.');
							else
								settings.(fields{i}) = opt.(fields{i}).value;
							end
						end
					end
				end
			elseif isa(settings, 'optim.options.SolverOptions')% optimoptions
				for i = 1:length(fields) %#ok<FORPF> no parfor for changes in objects
					if isprop(settings, fields{i})
						if opt.(fields{i}).isfactor
							settings.(fields{i}) = settings.(fields{i})*opt.(fields{i}).value;
						else
							settings.(fields{i}) = opt.(fields{i}).value;
						end
					else
						if opt.(fields{i}).isfactor
							error('optimization:solver:Optimizer:setting', 'The supplied optimization settings are invalid.');
						else
							warning('optimization:solver:Optimizer:set:tolcon', 'Options for algorithm %s do not support setting %s.', lower(strrep(class(settings), 'optim.options.', '')), fields{i});
						end
					end
				end
			else
				error('optimization:solver:Optimizer:setting', 'The supplied optimization settings are invalid.');
			end
		end

		function [settings] = setoptions(this, settings, options)
			%SETOPTIONS add arbitrary options to optimization settings, that can not be set by other setters of this class
			%	Input:
			%		this:		instance
			%		settings:	optimization settings as structure for optimset, YALMIP or optimoptions object
			%		options:	arbitrary optimization options to add
			%	Output:
			%		settings:	optimization setting with additional options added
			warning('optimization:solver:Optimizer', 'This method is deprecated.');
			if isempty(this.fieldname)
				return;
			end
			if isempty(options)
				return;
			end
% 			if ~isfield(settings, this.fieldname)
% 				error('optimization:solver:Optimizer:setting', 'The supplied optimization settings are invalid.');
% 			end
% 			if ~isstruct(options)
% 				error('optimization:solver:Optimizer:setting', 'Die Optionsstruktur ist ungültig.');
% 			end
			fields = fieldnames(options);
			if isempty(fields)
				return;
			end
			thisfields = {
				this.algorithm;
				this.maxitername;
				this.maxfunevalsname;
				this.tolfunname;
				this.tolconname;
				this.tolxname;
				this.displayname
			};
			thisfields = [thisfields; fieldnames(this.options)];
			idx = cellfun(@isempty, thisfields, 'UniformOutput', true);
			fields = setdiff(fields, thisfields(~idx));
			if isempty(fields)
				return;
			end
			if isstruct(settings)
				if isfield(settings, this.fieldname)% YALMIP
					tempstruct = settings.(this.fieldname);
					for i = 1:length(fields) %#ok<FORPF> no parfor for changes in structures
						if isfield(tempstruct, fields{i})
							if ~isempty(options.(fields{i}))
								if ~isnumeric(options.(fields{i})) || (isnumeric(options.(fields{i})) && ~isnan(options.(fields{i})))
									tempstruct.(fields{i}) = options.(fields{i});
								end
							end
						end
					end
					settings.(this.fieldname) = tempstruct;
				else% optimset
					for i = 1:length(fields) %#ok<FORPF> no parfor for changes in structures
						if isfield(settings, fields{i})
							if ~isempty(options.(fields{i}))
								if ~isnumeric(options.(fields{i})) || (isnumeric(options.(fields{i})) && ~isnan(options.(fields{i})))
									settings.(fields{i}) = options.(fields{i});
								end
							end
						end
					end
				end
			elseif isa(settings, 'optim.options.SolverOptions')% optimoptions
				for i = 1:length(fields) %#ok<FORPF> no parfor for changes in objects
					if isprop(settings, fields{i})
						if ~isempty(options.(fields{i}))
							if ~isnumeric(options.(fields{i})) || (isnumeric(options.(fields{i})) && ~isnan(options.(fields{i})))
								settings.(fields{i}) = options.(fields{i});
							end
						end
					end
				end
			else
				error('optimization:solver:Optimizer:setting', 'The supplied optimization settings are invalid.');
			end
		end
	end
end
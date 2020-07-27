function [x, fval, exitflag, output, lambda, grad, hessian] = optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin)
	%OPTIMIZE call nlopt with global unconstrained algorithm to solve optimization problem
	%	Input:
	%		fun:		objective function to minimize
	%		x_0:		initial value or matrix of initial values or StartPointSet of initial values to start optimization from
	%		A:			matrix of linear inequlity constraints
	%		b:			upper limits for linear inequality constraints
	%		Aeq:		matrix for linear equality constraints
	%		beq:		upper limits for linear equality constraints
	%		lb:			fixed lower bounds on x
	%		ub:			fixed upper bounds on x
	%		nonlcon:	nonlinear inequality and equality constraints
	%		options:	settings for optimization
	%		varargin:	additional arguments for objective function
	%	Output:
	%		x:			optimal value
	%		fval:		function value at optimal value
	%		exitflag:	optimization result indicator
	%		output:		structure with information about optimization
	%		lambda:		langrange multipliers at optimal solution
	%		grad:		objective function gradient at optimal solution
	%		hessian:	objective function hessian at optimal solution
	if isempty(fun)
		error('optimization:solver:nloptuncglobal:input', 'Objective function must not be empty.');
	end
	if ischar(fun)
		fun = str2func(fun);
	end
	if ~isfunctionhandle(fun)
		error('optimization:solver:nloptuncglobal:input', 'Objective function must be a function handle.');
	end
	if isempty(x_0)
		error('optimization:solver:nloptuncglobal:input', 'Initial point must not be empty.');
	end
	if isa(x_0, 'AbstractStartPointSet')
		x_0 = x_0.list().';
	end
	if ~isreal(x_0) || any(isinf(x_0(:))) || any(isnan(x_0(:)))
		error('optimization:solver:nloptuncglobal:input', 'Initial point must be a finite real value.');
	end
	if ~ismatrix(x_0)
		error('optimization:solver:nloptuncglobal:input', 'Initial point must be a column vector or a matrix of column vectors.');
	end
	%if size(x_0, 2) > 1
	%	error('optimization:solver:nloptuncglobal:input', 'Initial point must be a column vector.');
	%end
	usedefaultoption = false;
	defaultsolver = optimization.solver.Optimizer.NLOPTUNCGLOBAL;
	if nargin >= 10 && isa(options, 'optimization.solver.Optimizer')
		defaultsolver = options;
		usedefaultoption = true;
	end
	if nargin <= 9 ||isempty(options)
		options = optimization.options.OptionFactory.instance.options(defaultsolver,...
			'Algorithm',					defaultsolver.getDefaultAlgorithm(),...
			'Display',						'final-detailed',...
			'FunctionTolerance',			1E-6,...
			'StepTolerance',				1E-6,...
			'ConstraintTolerance',			1E-6,...
			'MaxFunctionEvaluations',		1E3,...
			'MaxIterations',				1E3,...
			'SpecifyObjectiveGradient',		true,...
			'SpecifyConstraintGradient',	true,...
			'CheckGradients',				false,...
			'FunValCheck',					false,...
			'Diagnostics',					false...
		);
		usedefaultoption = true;
	end
	if nargin <= 8
		nonlcon = [];
	end
	if nargin <= 7
		ub = Inf(size(x_0, 1), 1);
	end
	if nargin <= 6
		lb = -Inf(size(x_0, 1), 1);
	end
	if nargin <= 5
		beq = zeros(0, 1);
	end
	if nargin <= 4
		Aeq = zeros(0, size(x_0, 1));
	end
	if nargin <= 3
		b = zeros(0, 1);
	end
	if nargin <= 2
		A = zeros(0, size(x_0, 1));
	end
	if isempty(A)
		A = zeros(0, size(x_0, 1));
	end
	if isempty(b)
		b = zeros(0, 1);
	end
	if isempty(Aeq)
		Aeq = zeros(0, size(x_0, 1));
	end
	if isempty(beq)
		beq = zeros(0, 1);
	end
	if isempty(lb)
		lb = -Inf(size(x_0, 1), 1);
	end
	if isempty(ub)
		ub = Inf(size(x_0, 1), 1);
	end
	if ~isreal(A) || any(isnan(A(:)))
		error('optimization:solver:nloptuncglobal:input', 'Inequality matrix must be real.');
	end
	if ~ismatrix(A)
		error('optimization:solver:nloptuncglobal:input', 'Inequality matrix must be a matrix.');
	end
	if size(A, 2) ~= size(x_0, 1)
		error('optimization:solver:nloptuncglobal:input', 'Inequality matrix must have %d columns, not %d.', size(x_0, 1), size(A, 2));
	end
	if ~isreal(b) || any(isnan(b(:)))
		error('optimization:solver:nloptuncglobal:input', 'Inequality bounds must be real.');
	end
	if ~ismatrix(b)
		error('optimization:solver:nloptuncglobal:input', 'Inequality bounds must be a vector.');
	end
	if size(b, 1) ~= size(A, 1)
		error('optimization:solver:nloptuncglobal:input', 'Inequality bounds must have %d rows, not %d.', size(A, 1), size(b, 1));
	end
	if ~isreal(Aeq) || any(isnan(Aeq(:)))
		error('optimization:solver:nloptuncglobal:input', 'Equality matrix must be real.');
	end
	if ~ismatrix(Aeq)
		error('optimization:solver:nloptuncglobal:input', 'Equality matrix must be a matrix.');
	end
	if size(Aeq, 2) ~= size(x_0, 1)
		error('optimization:solver:nloptuncglobal:input', 'Equality matrix must have %d columns, not %d.', size(x_0, 1), size(Aeq, 2));
	end
	if ~isreal(beq) || any(isnan(beq(:)))
		error('optimization:solver:nloptuncglobal:input', 'Equality bounds must be real.');
	end
	if ~ismatrix(beq)
		error('optimization:solver:nloptuncglobal:input', 'Equality bounds must be a vector.');
	end
	if size(beq, 1) ~= size(Aeq, 1)
		error('optimization:solver:nloptuncglobal:input', 'Equality bounds must have %d rows, not %d.', size(Aeq, 1), size(beq, 1));
	end
	if ~isreal(lb) || any(isnan(lb(:)))
		error('optimization:solver:nloptuncglobal:input', 'Lower bounds must be real.');
	end
	if ~ismatrix(lb)
		error('optimization:solver:nloptuncglobal:input', 'Lower bounds must be a vector.');
	end
	if size(lb, 1) ~= size(x_0, 1)
		error('optimization:solver:nloptuncglobal:input', 'Lower bounds must have %d rows, not %d.', size(x_0, 1), size(lb, 1));
	end
	if ~isreal(ub) || any(isnan(ub(:)))
		error('optimization:solver:nloptuncglobal:input', 'Upper bounds must be real.');
	end
	if ~ismatrix(ub)
		error('optimization:solver:nloptuncglobal:input', 'Upper bounds must be a vector.');
	end
	if size(ub, 1) ~= size(x_0, 1)
		error('optimization:solver:nloptuncglobal:input', 'Upper bounds must have %d rows, not %d.', size(x_0, 1), size(ub, 1));
	end
	if nargin >= 9
		if ~isempty(nonlcon)
			if ischar(nonlcon)
				nonlcon = str2func(nonlcon);
			end
			if ~isfunctionhandle(nonlcon)
				error('optimization:solver:nloptuncglobal:input', 'Constraint function must be a function handle.');
			end
		end
	else
		nonlcon = [];
	end
	if ~isa(options, 'optimization.options.Options')
		if isstruct(options)
			options = optimization.options.OptionFactory.instance.options(optimization.solver.Optimizer.NLOPTUNCGLOBAL, options);
		elseif isa(options, 'optim.options.SolverOptions')
			options = optimization.options.OptionFactory.instance.options(optimization.solver.Optimizer.NLOPTUNCGLOBAL, options);
		else
			error('optimization:solver:nloptuncglobal:input', 'Options must be of type optimization.options.Options or optimoptions or optimset.');
		end
	end
	if ~isa(options, 'optimization.options.nloptuncglobal')
		options = optimization.options.OptionFactory.instance.options(optimization.solver.Optimizer.NLOPTUNCGLOBAL, options);
	end
	try
		if ~isempty(nonlcon)
			if nargin >= 11
				if nargin(nonlcon) == 1
					[c, ceq] = callnonlcon(nonlcon, x_0(:, 1));
				else
					[c, ceq] = callnonlcon(nonlcon, x_0(:, 1), varargin{:});
				end
			else
				[c, ceq] = callnonlcon(nonlcon, x_0(:, 1));
			end
		else
			c = [];
			ceq = [];
		end
	catch e
		throwAsCaller(e);
	end
	function [f, grad] = objectivegradientvararg(x)
		[f, grad] = fun(x, varargin{:});
	end
	if nargin >= 11
		if nargout(fun) >= 2
			if nargin(fun) == 1
				Jfun = fun;
			else
				Jfun = @objectivegradientvararg;
			end
			[~, gradJ] = Jfun(x_0(:, 1));
		else
			if nargout(fun) == -1
				try
					if nargin(fun) == 1
						[~, gradJ] = fun(x_0(:, 1));
						Jfun = fun;
					else
						[~, gradJ] = fun(x_0(:, 1), varargin{:});
						Jfun = @objectivegradientvararg;
					end
				catch e
					if ~strcmpi(e.identifier, 'MATLAB:maxlhs') && ~strcmpi(e.identifier, 'MATLAB:TooManyOutputs') && ~strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
						rethrow(e);
					else
						if nargin(fun) == 1
							Jfun = fun;
						else
							Jfun = @(x) fun(x, varargin{:});
						end
						gradJ = NaN(size(x_0, 1), 1);
						if usedefaultoption
							options.SpecifyObjectiveGradient = false;
						end
					end
				end
			else
				if nargin(fun) == 1
					Jfun = fun;
				else
					Jfun = @(x) fun(x, varargin{:});
				end
				gradJ = NaN(size(x_0, 1), 1);
				if usedefaultoption
					options.SpecifyObjectiveGradient = false;
				end
			end
		end
	else
		if nargout(fun) >= 2
			Jfun = fun;
			[~, gradJ] = Jfun(x_0(:, 1));
		else
			if nargout(fun) == -1
				try
					[~, tempgrad] = fun(x_0(:, 1)); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
					Jfun = fun;
					[~, gradJ] = Jfun(x_0(:, 1));
				catch e
					if ~strcmpi(e.identifier, 'MATLAB:maxlhs') && ~strcmpi(e.identifier, 'MATLAB:TooManyOutputs') && ~strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
						rethrow(e);
					else
						Jfun = fun;
						gradJ = NaN(size(x_0, 1), 1);
						if usedefaultoption
							options.SpecifyObjectiveGradient = false;
						end
					end
				end
			else
				Jfun = fun;
				gradJ = NaN(size(x_0, 1), 1);
				if usedefaultoption
					options.SpecifyObjectiveGradient = false;
				end
			end
		end
	end
	if isempty(nonlcon)
		if options.SpecifyObjectiveGradient
			if any(isnan(gradJ(:)))
				gradient = false;
			else
				gradient = true;
			end
		else
			gradient = false;
		end
	else
		if options.SpecifyObjectiveGradient && options.SpecifyConstraintGradient
			if any(isnan(gradJ(:)))
				gradient = false;
			else
				gradient = true;
			end
		else
			gradient = false;
		end
	end
	globalflag = true;
	[algorithm, needsgradient, isglobal, ~, needsbounds] = optimization.options.nlopt.getalgorithm(options.Algorithm, gradient, globalflag, options.ProblemType);
	if isnan(algorithm)
		[algorithm, needsgradient, isglobal, ~, needsbounds] = optimization.options.nlopt.getalgorithm(options.Algorithm, ~gradient, globalflag, options.ProblemType);
	end
	if isnan(algorithm)
		error('optimization:solver:nloptuncglobal:input', 'Current algorithm is not supported.');
	end
	if needsgradient && ~gradient
		error('optimization:solver:nloptuncglobal:input', 'Current algorithm needs gradient information.');
	end
	if needsbounds
		if any(isinf(lb(:))) || any(isinf(ub(:)))
			lb_x_0 = mean(x_0, 2) - 10*max(abs(x_0), [], 2);
			lb(isinf(lb)) = lb_x_0(isinf(lb));
			ub_x_0 = mean(x_0, 2) + 10*max(abs(x_0), [], 2);
			ub(isinf(ub)) = ub_x_0(isinf(ub));
			if any(isinf(lb(:))) || any(isinf(ub(:))) || any(lb > ub)
				error('optimization:solver:nloptuncglobal:input', 'Bounds must be specified for the current algorithm.');
			else
				warning('optimization:solver:nloptuncglobal:input', 'Bounds must be specified for the current algorithm. Using [x_0 - 10*x_0, x_0 + 10*x_0] as bound.');
			end
		end
	end
	if ~isglobal
		error('optimization:solver:nloptuncglobal:input', 'nloptuncglobal is for global unconstrained problems, for local optimization use nloptunc instead.');
	end
	if exist('nlopt_optimize', 'file') == 3
		nloptinterface = true;
	else
		nloptinterface = false;
	end
	if exist('nlopt', 'file') == 3
		optiinterface = true;
	else
		optiinterface = false;
	end
	if ~nloptinterface && ~optiinterface
		error('optimization:solver:nloptconglobal:input', 'No function interface for nlopt could be found.');
	end
	if optiinterface
		nloptinterface = false;
	end
	options.NumberVariables = size(x_0, 1);
	options.NumberConstraintsInequality = size(A, 1) + size(c, 1);
	options.NumberConstraintsEquality = size(Aeq, 1) + size(ceq, 1);
	options.NumberConstraintsBounds = sum(~isinf(lb)) + sum(~isinf(ub));
	solveroptions = options.getpreferred();
	if isfield(solveroptions, 'Algorithm')
		solveroptions = rmfield(solveroptions, 'Algorithm');
	end
	if isfield(solveroptions, 'verbose')
		solveroptions.verbose = str2double(solveroptions.verbose);
	end
	if optiinterface
		solveroptions.display = solveroptions.verbose;
		if isfield(solveroptions, 'maxeval')
			solveroptions.maxfeval = solveroptions.maxeval;
		end
	end
	solveroptions.algorithm = algorithm;
	if ~isempty(options.SubproblemAlgorithm)
		[subalgorithm, subneedsgradient, subisglobal, ~, subneedsbounds] = optimization.options.nlopt.getalgorithm(options.SubproblemAlgorithm, gradient, false, options.ProblemType);
		if isnan(subalgorithm)
			[subalgorithm, subneedsgradient, subisglobal, ~, subneedsbounds] = optimization.options.nlopt.getalgorithm(options.SubproblemAlgorithm, ~gradient, false, options.ProblemType);
		end
		if isnan(subalgorithm) || isempty(subalgorithm)
			error('optimization:solver:nloptuncglobal:input', 'Current subproblem algorithm is not supported.');
		end
		if subneedsgradient && ~gradient
			error('optimization:solver:nloptuncglobal:input', 'Current subproblem algorithm needs gradient information.');
		end
		if subneedsbounds
			if any(isinf(lb(:))) || any(isinf(ub(:)))
				error('optimization:solver:nloptuncglobal:input', 'Bounds must be specified for the current subproblem algorithm.');
			end
		end
		if subisglobal
			error('optimization:solver:nloptuncglobal:input', 'Local algorithm for nloptuncglobal must not be global.');
		end
		if isfield(solveroptions, 'local_optimizer')
			solveroptions = rmfield(solveroptions, 'local_optimizer');
		end
		localsolveroptions = solveroptions;
		localnloptnames = fieldnames(localsolveroptions);
		localremovefields = false(size(localnloptnames, 1), 1);
		for ii = 1:size(localnloptnames, 1) %#ok<FORPF> no parfor for call to mex interface
			if isempty(localsolveroptions.(localnloptnames{ii, 1}))
				localremovefields(ii, 1) = true;
			end
		end
		localsolveroptions = rmfield(localsolveroptions, localnloptnames(localremovefields));
		localsolveroptions.algorithm = subalgorithm;
		solveroptions.local_optimizer = localsolveroptions;
	else
		error('optimization:solver:nloptuncglobal:input', 'Subproblem algorithm must be specified for global optimization.');
	end
	nloptnames = fieldnames(solveroptions);
	removefields = false(size(nloptnames, 1), 1);
	for ii = 1:size(nloptnames, 1) %#ok<FORPF> no parfor for call to mex interface
		if isempty(solveroptions.(nloptnames{ii, 1}))
			removefields(ii, 1) = true;
		end
	end
	solveroptions = rmfield(solveroptions, nloptnames(removefields));
	if nloptinterface
		solveroptions.min_objective = objective_dispatcher(Jfun);
		%solveroptions.fc = {};
		%solveroptions.h = {};
		solveroptions.lower_bounds = lb;
		solveroptions.upper_bounds = ub;
	else
		solveroptions = struct(...
			'objective',	objective_dispatcher(Jfun),...
			'gradient',		objective_dispatcher(Jfun, true),...
			'nlcon',		[],...
			'nlrhs',		zeros(0, 1),...
			'nle',			zeros(0, 1),...
			'nljac',		[],...
			'lb',			lb,...
			'ub',			ub,...
			'options',		solveroptions...
		);
	end
	solvertime = tic;
	retry = 0;
	retryiterations = 0;
	retryfunevals = 0;
	maxTime = options.MaxTime;
	if isempty(maxTime) || isnan(maxTime) || maxTime <= 0
		maxTime = Inf;
	end
	initialvalues = size(x_0, 2);
	useparallel = options.UseParallel;
	nargout3 = nargout >= 3;
	nargout4 = nargout >= 4;
	if nargout4
		alloutputs = cell(max(1, options.Retries), initialvalues);
	end
	while retry < max(1, options.Retries)
		x_opt = NaN(size(x_0));
		f_opt = NaN(1, initialvalues);
		iterations = zeros(1, initialvalues);
		funevals = zeros(1, initialvalues);
		if nargout3 || useparallel
			exitflag = NaN(1, initialvalues);
		end
		if nargout4 || useparallel
			output = cell(1, initialvalues);
			solvertimes = repmat(tic, 1, initialvalues);
			tempoutputs = cell(1, initialvalues);
		end
		if useparallel
			parfor ii = 1:initialvalues
				try
					if nloptinterface
						[x_opt(:, ii), f_opt(1, ii), exitflag(1, ii)] = nlopt_optimize(solveroptions, x_0(:, ii));
					else
						if nargout4
							[x_opt(:, ii), f_opt(1, ii), exitflag(1, ii), output{1, ii}] = nlopt(solveroptions, x_0(:, ii));
						else
							[x_opt(:, ii), f_opt(1, ii), exitflag(1, ii)] = nlopt(solveroptions, x_0(:, ii));
						end
					end
				catch e
					if strcmpi(e.identifier, 'EMLRT:runTime:UserInterrupt')
						x_opt(:, ii) = NaN(size(x_0(:, ii), 1), 1);
						f_opt(1, ii) = NaN;
						exitflag(1, ii) = -5;
						if nargout4
							if nloptinterface
								output{1, ii} = struct('funevals', NaN);
							else
								output{1, ii} = struct(...
									'objective',		NaN,...
									'constraint',		NaN,...
									'gradient',			NaN,...
									'jacobian',			NaN...
								);
							end
						end
					else
						rethrow(e);
					end
				end
				if nargout4
					iterations(1, ii) = 1;
					if nloptinterface
						% TODO: this counts cumulative function evaluations instead of real function evaluations because of persistent in the objective function
						[~, ~, funevals(1, ii)] = solveroptions.min_objective();
						output{1, ii} = struct('funevals', funevals(1, ii));
					else
						funevals(1, ii) = output{1, ii}.objective;
					end
					tempoutputs{1, ii} = options.formatOutput(exitflag(1, ii), toc(solvertimes(1, ii)), x_opt(:, ii), f_opt(1, ii), numel(x_0(:, ii)), iterations(1, ii), funevals(1, ii), retry, output{1, ii}); %#ok<PFBNS> options is an object
				end
			end
		else
			for ii = 1:initialvalues %#ok<FORPF> parallel computing is disabled through option set by user
				solvertimes(1, ii) = tic;
				try
					if nloptinterface
						[x_opt(:, ii), f_opt(1, ii), exitflag(1, ii)] = nlopt_optimize(solveroptions, x_0(:, ii));
					else
						if nargout4
							[x_opt(:, ii), f_opt(1, ii), exitflag(1, ii), output{1, ii}] = nlopt(solveroptions, x_0(:, ii));
						else
							[x_opt(:, ii), f_opt(1, ii), exitflag(1, ii)] = nlopt(solveroptions, x_0(:, ii));
						end
					end
				catch e
					if strcmpi(e.identifier, 'EMLRT:runTime:UserInterrupt')
						x_opt(:, ii) = NaN(size(x_0, 1), 1);
						f_opt(1, ii) = NaN;
						exitflag(1, ii) = -5;
						if nargout4
							if nloptinterface
								output{1, ii} = struct('funevals', NaN);
							else
								output{1, ii} = struct(...
									'objective',		NaN,...
									'constraint',		NaN,...
									'gradient',			NaN,...
									'jacobian',			NaN...
								);
							end
						end
						break;
					else
						rethrow(e);
					end
				end
				if nargout4
					iterations(1, ii) = 1;
					if nloptinterface
						% TODO: this counts cumulative function evaluations instead of real function evaluations because of persistent in the objective function
						[~, ~, funevals(1, ii)] = solveroptions.min_objective();
						output{1, ii} = struct('funevals', funevals(1, ii));
					else
						funevals(1, ii) = output{1, ii}.objective;
					end
					tempoutputs{1, ii} = options.formatOutput(exitflag(1, ii), toc(solvertimes(1, ii)), x_opt(:, ii), f_opt(1, ii), numel(x_0(:, ii)), iterations(1, ii), funevals(1, ii), retry, output{1, ii});
				end
			end
		end
		if any(exitflag == -2)
			x_opt(:, exitflag == -2) = NaN;
			f_opt(1, exitflag == -2) = NaN;
		end
		if nargout >= 4
			alloutputs(retry + 1, :) = tempoutputs;
			retryiterations = retryiterations + sum(iterations);
			retryfunevals = retryfunevals + sum(funevals);
		end
		retry = retry + 1;
		if toc(solvertime) > maxTime
			break;
		end
		x_0 = x_opt;
	end
	if matlab.Version.CURRENT >= matlab.Version.R2015A
		[fval, idx] = min(f_opt, [], 2, 'omitnan');
	else
		[fval, idx] = min(f_opt, [], 2);
	end
	if isempty(idx)
		error('optimization:solver:nloptuncglobal:solution', 'No solution could be found for any initial value.');
	end
	x = x_opt(:, idx);
	if nargout3
		exitflag = exitflag(1, idx);
	end
	if nargout4
		output = output{1, idx};
		switch(exitflag)
			case {1, 2}
				message = 'Optimal';
			case 3
				message = sprintf('Optimal within function tolerance of %G', options.FunctionTolerance);
			case 4
				message = sprintf('Optimal within step tolerance of %G', options.StepTolerance);
			case 5
				message = sprintf('Exceeded Function Evaluations %d', options.MaxFunctionEvaluations);
			case 6
				message = sprintf('Exceeded Maximum Time Limit %f', options.MaxTime);
			case -1
				message = 'Infeasible';
			case -2
				message = 'Incorrect arguments';
			case -4
				message = 'Not enough memory';
			case -5
				message = 'User Exit';
			otherwise
				message = 'NLOPT Error';
		end
		if ~isstruct(output) || isempty(output)
			output = struct(...
				'message',	message...
			);
		else
			output.message = message;
		end
	end
	if nargout >= 5
		lambda = struct(...
			'lower',		[],...
			'upper',		[],...
			'ineqnonlin',	[],...
			'eqnonlin',		[],...
			'ineqlin',		[],...
			'eqlin',		[]...
		);
	end
	if nargout >= 6
		if nargout(Jfun) >= 2
			[~, grad] = Jfun(x);
		else
			if nargout(Jfun) == -1
				try
					[~, grad] = Jfun(x);
					hasgradient = true;
				catch e
					if strcmpi(e.identifier, 'MATLAB:maxlhs') || strcmpi(e.identifier, 'MATLAB:TooManyOutputs') || strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
						hasgradient = false;
					else
						rethrow(e);
					end
				end
			else
				hasgradient = false;
			end
			if ~hasgradient
				grad = zeros(size(x, 1), 1);
				step = eps;
				f = Jfun(x);
				parfor ii = 1:size(x, 1)
					grad(ii) = (Jfun(x + step*(1:size(x, 1) == ii)') - f)/step;
				end
			end
		end
	end
	if nargout >= 7
		hessian = NaN(size(x_0, 1), size(x_0, 1));
	end
	solvertime = toc(solvertime);
	if nargout >= 4
		alloutputs(retry + 1:end, :) = [];
		output = options.formatOutput(exitflag, solvertime, x, fval, numel(x_0(:, 1)), retryiterations, retryfunevals, retry, output, alloutputs);
	end
end

function [c, ceq, gradc, gradceq] = callnonlcon(nonlcon, x, varargin)
	%CALLNONLCON wrapper function for calling nonlinear constraint function for calculating dimensions of constraints
	%	Input:
	%		nonlcon:	function to call
	%		x:			value to evaluate function at
	%		varargin:	additional input arguments for the constraint function
	%	Output:
	%		c:			inequality constraints
	%		ceq:		equality constraints
	%		gradc:		gradient of inequality constraints
	%		gradceq:	gradient of equality constraints
	if nargin >= 3
		if nargout >= 3
			[c, ceq, gradc, gradceq] = nonlcon(x, varargin{:});
		else
			[c, ceq] = nonlcon(x, varargin{:});
		end
	else
		if nargout >= 3
			[c, ceq, gradc, gradceq] = nonlcon(x);
		else
			[c, ceq] = nonlcon(x);
		end
	end
end

function [J] = objective_dispatcher(fun, gradient)
	%OBJECTIVE_DISPATCHER wrapper function for nloptuncglobal to convert the optimization variable to a column vector in case optimization variable is a row vector
	%	Input:
	%		fun:		objective function handle
	%		gradient:	indicator, if gradient function shoulb be split from objective function
	%	Output:
	%		c:		objective function handle
	if nargin <= 1
		gradient = false;
	end
	function [J, gradJ, funevals] = objective(x)
		persistent eval;
		if isempty(eval)
			eval = 0;
		end
		if nargout >= 3 && nargin == 0
			J = [];
			gradJ = [];
			funevals = eval;
			return;
		end
		if ~iscolumn(x)
			x = x';
		end
		eval = eval + 1;
		if nargout >= 2
			[J, gradJ] = fun(x);
		else
			J = fun(x);
		end
	end
	function [gradJ] = objective_gradient(x)
		if ~iscolumn(x)
			x = x';
		end
		[~, gradJ] = fun(x);
	end
	if gradient
		J = @objective_gradient;
	else
		J = @objective;
	end
end
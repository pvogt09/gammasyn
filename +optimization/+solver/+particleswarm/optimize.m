function [x, fval, exitflag, output, lambda, grad, hessian] = optimize(fun, x_0, A, b, Aeq, beq, lb, ub, ~, options, varargin)
	%OPTIMIZE call particleswarm to solve optimization problem
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
		error('optimization:solver:particleswarm:input', 'Objective function must not be empty.');
	end
	if ischar(fun)
		fun = str2func(fun);
	end
	if ~isfunctionhandle(fun)
		error('optimization:solver:particleswarm:input', 'Objective function must be a function handle.');
	end
	if isempty(x_0)
		error('optimization:solver:particleswarm:input', 'Initial point must not be empty.');
	end
	if isa(x_0, 'AbstractStartPointSet')
		x_0 = x_0.list().';
	end
	if ~isreal(x_0) || any(isinf(x_0(:))) || any(isnan(x_0(:)))
		error('optimization:solver:particleswarm:input', 'Initial point must be a finite real value.');
	end
	if ~ismatrix(x_0)
		error('optimization:solver:particleswarm:input', 'Initial point must be a column vector or a matrix of column vectors.');
	end
	%if size(x_0, 2) > 1
	%	error('optimization:solver:particleswarm:input', 'Initial point must be a column vector.');
	%end
	usedefaultoption = false;
	defaultsolver = optimization.solver.Optimizer.PARTICLESWARM;
	if nargin >= 10 && isa(options, 'optimization.solver.Optimizer')
		defaultsolver = options;
		usedefaultoption = true;
	end
	if nargin <= 9 || isempty(options)
		options = optimization.options.OptionFactory.instance.options(defaultsolver,...
			'Algorithm',					'particleswarm',...
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
		error('optimization:solver:particleswarm:input', 'Inequality matrix must be real.');
	end
	if ~ismatrix(A)
		error('optimization:solver:particleswarm:input', 'Inequality matrix must be a matrix.');
	end
	if size(A, 2) ~= size(x_0, 1)
		error('optimization:solver:particleswarm:input', 'Inequality matrix must have %d columns, not %d.', size(x_0, 1), size(A, 2));
	end
	if ~isreal(b) || any(isnan(b(:)))
		error('optimization:solver:particleswarm:input', 'Inequality bounds must be real.');
	end
	if ~ismatrix(b)
		error('optimization:solver:particleswarm:input', 'Inequality bounds must be a vector.');
	end
	if size(b, 1) ~= size(A, 1)
		error('optimization:solver:particleswarm:input', 'Inequality bounds must have %d rows, not %d.', size(A, 1), size(b, 1));
	end
	if ~isreal(Aeq) || any(isnan(Aeq(:)))
		error('optimization:solver:particleswarm:input', 'Equality matrix must be real.');
	end
	if ~ismatrix(Aeq)
		error('optimization:solver:particleswarm:input', 'Equality matrix must be a matrix.');
	end
	if size(Aeq, 2) ~= size(x_0, 1)
		error('optimization:solver:particleswarm:input', 'Equality matrix must have %d columns, not %d.', size(x_0, 1), size(Aeq, 2));
	end
	if ~isreal(beq) || any(isnan(beq(:)))
		error('optimization:solver:particleswarm:input', 'Equality bounds must be real.');
	end
	if ~ismatrix(beq)
		error('optimization:solver:particleswarm:input', 'Equality bounds must be a vector.');
	end
	if size(beq, 1) ~= size(Aeq, 1)
		error('optimization:solver:particleswarm:input', 'Equality bounds must have %d rows, not %d.', size(Aeq, 1), size(beq, 1));
	end
	if ~isreal(lb) || any(isnan(lb(:)))
		error('optimization:solver:particleswarm:input', 'Lower bounds must be real.');
	end
	if ~ismatrix(lb)
		error('optimization:solver:particleswarm:input', 'Lower bounds must be a vector.');
	end
	if size(lb, 1) ~= size(x_0, 1)
		error('optimization:solver:particleswarm:input', 'Lower bounds must have %d rows, not %d.', size(x_0, 1), size(lb, 1));
	end
	if ~isreal(ub) || any(isnan(ub(:)))
		error('optimization:solver:particleswarm:input', 'Upper bounds must be real.');
	end
	if ~ismatrix(ub)
		error('optimization:solver:particleswarm:input', 'Upper bounds must be a vector.');
	end
	if size(ub, 1) ~= size(x_0, 1)
		error('optimization:solver:particleswarm:input', 'Upper bounds must have %d rows, not %d.', size(x_0, 1), size(ub, 1));
	end
	if ~isa(options, 'optimization.options.Options')
		if isstruct(options)
			options = optimization.options.OptionFactory.instance.options(optimization.solver.Optimizer.PARTICLESWARM, options);
		elseif isa(options, 'optim.options.SolverOptions')
			options = optimization.options.OptionFactory.instance.options(optimization.solver.Optimizer.PARTICLESWARM, options);
		else
			error('optimization:solver:particleswarm:input', 'Options must be of type optimization.options.Options or optimoptions or optimset.');
		end
	end
	if ~isa(options, 'optimization.options.particleswarm')
		options = optimization.options.OptionFactory.instance.options(optimization.solver.Optimizer.PARTICLESWARM, options);
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
		else
			if nargout(fun) == -1
				try
					if nargin(fun) == 1
						[~, gradJ] = fun(x_0(:, 1)); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
						Jfun = fun;
					else
						[~, gradJ] = fun(x_0(:, 1), varargin{:}); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
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
				if usedefaultoption
					options.SpecifyObjectiveGradient = false;
				end
			end
		end
	else
		if nargout(fun) >= 2
			Jfun = fun;
		else
			if nargout(fun) == -1
				try
					[~, gradJ] = fun(x_0(:, 1)); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
					Jfun = fun;
				catch e
					if ~strcmpi(e.identifier, 'MATLAB:maxlhs') && ~strcmpi(e.identifier, 'MATLAB:TooManyOutputs') && ~strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
						rethrow(e);
					else
						Jfun = fun;
						if usedefaultoption
							options.SpecifyObjectiveGradient = false;
						end
					end
				end
			else
				Jfun = fun;
				if usedefaultoption
					options.SpecifyObjectiveGradient = false;
				end
			end
		end
	end
	options.NumberVariables = size(x_0, 1);
	options.NumberConstraintsInequality = size(A, 1);
	options.NumberConstraintsEquality = size(Aeq, 1);
	options.NumberConstraintsBounds = sum(~isinf(lb)) + sum(~isinf(ub));
	solveroptions = options.getpreferred();
	solvertime = tic;
	retry = 0;
	retryiterations = 0;
	retryfunevals = 0;
	maxTime = options.MaxTime;
	if isempty(maxTime) || isnan(maxTime) || maxTime <= 0
		maxTime = Inf;
	end
	if nargout >= 4
		alloutputs = cell(max(1, options.Retries), 1);
	end
	while retry < max(1, options.Retries)
		solvertimes = tic;
		try
			if nargout >= 4
				[x, fval, exitflag, output] = particleswarm(objective_dispatcher(Jfun), size(x_0, 1), lb, ub, solveroptions);
			elseif nargout >= 3
				[x, fval, exitflag] = particleswarm(objective_dispatcher(Jfun), size(x_0, 1), lb, ub, solveroptions);
			elseif nargout >= 2
				[x, fval] = particleswarm(objective_dispatcher(Jfun), size(x_0, 1), lb, ub, solveroptions);
			else
				x = particleswarm(objective_dispatcher(Jfun), size(x_0, 1), lb, ub, solveroptions);
			end
		catch e
			rethrow(e);
		end
		x = x';
		if nargout >= 4
			retryiterations = retryiterations + output.iterations;
			retryfunevals = retryfunevals + output.funccount;
			alloutputs{retry + 1, :} = options.formatOutput(exitflag, toc(solvertimes), x, fval, numel(x_0(:, 1)), output.iterations, output.funccount, retry, output);
		end
		retry = retry + 1;
		if toc(solvertime) > maxTime
			break;
		end
		x_0 = x;
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

function [J] = objective_dispatcher(fun)
	%OBJECTIVE_DISPATCHER wrapper function for ga to convert the optimization variable to a column vector in case optimization variable is a row vector
	%	Input:
	%		fun:	objective function handle
	%	Output:
	%		c:		objective function handle
	function [J, gradJ] = objective(x)
		if ~iscolumn(x)
			x = x';
		end
		if nargout >= 2
			[J, gradJ] = fun(x);
		else
			J = fun(x);
		end
	end
	J = @objective;
end
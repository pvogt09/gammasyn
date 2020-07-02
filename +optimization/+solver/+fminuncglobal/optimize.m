function [x, fval, exitflag, output, lambda, grad, hessian] = optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin)
	%OPTIMIZE call global search algorithm with fminunc to solve optimization problem
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
		error('optimization:solver:fminuncglobal:input', 'Objective function must not be empty.');
	end
	if ischar(fun)
		fun = str2func(fun);
	end
	if ~isfunctionhandle(fun)
		error('optimization:solver:fminuncglobal:input', 'Objective function must be a function handle.');
	end
	if isempty(x_0)
		error('optimization:solver:fminuncglobal:input', 'Initial point must not be empty.');
	end
	if isa(x_0, 'AbstractStartPointSet')
		x_0 = x_0.list().';
	end
	if ~isreal(x_0) || any(isinf(x_0(:))) || any(isnan(x_0(:)))
		error('optimization:solver:fminuncglobal:input', 'Initial point must be a finite real value.');
	end
	if ~ismatrix(x_0)
		error('optimization:solver:fminuncglobal:input', 'Initial point must be a column vector or a matrix of column vectors.');
	end
	%if size(x_0, 2) > 1
	%	error('optimization:solver:fminuncglobal:input', 'Initial point must be a column vector.');
	%end
	usedefaultoption = false;
	defaultsolver = optimization.solver.Optimizer.FMINUNCGLOBAL;
	if nargin >= 10 && isa(options, 'optimization.solver.Optimizer')
		defaultsolver = options;
		usedefaultoption = true;
	end
	if nargin <= 9 || isempty(options)
		options = optimization.options.OptionFactory.instance.options(defaultsolver,...
			'Display',						'final',...
			'FunctionTolerance',			1E-6,...
			'StepTolerance',				1E-6,...
			'MaxFunctionEvaluations',		'100*numberOfVariables',...
			'MaxIterations',				400,...
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
		error('optimization:solver:fminuncglobal:input', 'Inequality matrix must be real.');
	end
	if ~ismatrix(A)
		error('optimization:solver:fminuncglobal:input', 'Inequality matrix must be a matrix.');
	end
	if size(A, 2) ~= size(x_0, 1)
		error('optimization:solver:fminuncglobal:input', 'Inequality matrix must have %d columns, not %d.', size(x_0, 1), size(A, 2));
	end
	if ~isreal(b) || any(isnan(b(:)))
		error('optimization:solver:fminuncglobal:input', 'Inequality bounds must be real.');
	end
	if ~ismatrix(b)
		error('optimization:solver:fminuncglobal:input', 'Inequality bounds must be a vector.');
	end
	if size(b, 1) ~= size(A, 1)
		error('optimization:solver:fminuncglobal:input', 'Inequality bounds must have %d rows, not %d.', size(A, 1), size(b, 1));
	end
	if ~isreal(Aeq) || any(isnan(Aeq(:)))
		error('optimization:solver:fminuncglobal:input', 'Equality matrix must be real.');
	end
	if ~ismatrix(Aeq)
		error('optimization:solver:fminuncglobal:input', 'Equality matrix must be a matrix.');
	end
	if size(Aeq, 2) ~= size(x_0, 1)
		error('optimization:solver:fminuncglobal:input', 'Equality matrix must have %d columns, not %d.', size(x_0, 1), size(Aeq, 2));
	end
	if ~isreal(beq) || any(isnan(beq(:)))
		error('optimization:solver:fminuncglobal:input', 'Equality bounds must be real.');
	end
	if ~ismatrix(beq)
		error('optimization:solver:fminuncglobal:input', 'Equality bounds must be a vector.');
	end
	if size(beq, 1) ~= size(Aeq, 1)
		error('optimization:solver:fminuncglobal:input', 'Equality bounds must have %d rows, not %d.', size(Aeq, 1), size(beq, 1));
	end
	if ~isreal(lb) || any(isnan(lb(:)))
		error('optimization:solver:fminuncglobal:input', 'Lower bounds must be real.');
	end
	if ~ismatrix(lb)
		error('optimization:solver:fminuncglobal:input', 'Lower bounds must be a vector.');
	end
	if size(lb, 1) ~= size(x_0, 1)
		error('optimization:solver:fminuncglobal:input', 'Lower bounds must have %d rows, not %d.', size(x_0, 1), size(lb, 1));
	end
	if ~isreal(ub) || any(isnan(ub(:)))
		error('optimization:solver:fminuncglobal:input', 'Upper bounds must be real.');
	end
	if ~ismatrix(ub)
		error('optimization:solver:fminuncglobal:input', 'Upper bounds must be a vector.');
	end
	if size(ub, 1) ~= size(x_0, 1)
		error('optimization:solver:fminuncglobal:input', 'Upper bounds must have %d rows, not %d.', size(x_0, 1), size(ub, 1));
	end
	if nargin >= 9
		if ~isempty(nonlcon)
			if ischar(nonlcon)
				nonlcon = str2func(nonlcon);
			end
			if ~isfunctionhandle(nonlcon)
				error('optimization:solver:fminuncglobal:input', 'Constraint function must be a function handle.');
			end
		end
	else
		nonlcon = [];
	end
	if ~isa(options, 'optimization.options.Options')
		if isstruct(options)
			options = optimization.options.OptionFactory.instance.options(optimization.solver.Optimizer.FMINUNCGLOBAL, options);
		elseif isa(options, 'optim.options.SolverOptions')
			options = optimization.options.OptionFactory.instance.options(optimization.solver.Optimizer.FMINUNCGLOBAL, options);
		else
			error('optimization:solver:fminuncglobal:input', 'Options must be of type optimization.options.Options or optimoptions or optimset.');
		end
	end
	if ~isa(options, 'optimization.options.fminuncglobal')
		options = optimization.options.OptionFactory.instance.options(optimization.solver.Optimizer.FMINUNCGLOBAL, options);
	end
	function [f, grad, hessian] = objectivehessian(x)
		[f, grad, hessian] = fun(x, varargin{:});
	end
	function [f, grad] = objectivegradient(x)
		[f, grad] = fun(x, varargin{:});
	end
	function [f, grad, hessian] = objectivehessianvararg(x)
		[f, grad, hessian] = fun(x, varargin{:});
	end
	function [f, grad] = objectivegradientvararg(x)
		[f, grad] = fun(x, varargin{:});
	end
	needshessian = options.supportsHessian() && options.SpecifyObjectiveHessian;
	hasgradient = false;
	hashessian = false;
	if nargin >= 11
		if nargout(fun) >= 3
			hasgradient = true;
			hashessian = true;
			if nargin(fun) == 1
				Jfun = fun;
			else
				Jfun = @objectivehessian;
			end
		elseif nargout(fun) >= 2
			hasgradient = true;
			if nargin(fun) == 1
				Jfun = fun;
			else
				Jfun = @objectivegradient;
			end
			if usedefaultoption
				options.SpecifyObjectiveHessian = false;
			end
		else
			if nargout(fun) == -1
				try
					if needshessian
						try
							if nargin(fun) == 1
								[~, tempgrad, temphess] = fun(x_0(:, 1)); %#ok<ASGLU> call with three output arguments to check if the third one is present and catch error if not
								Jfun = fun;
							else
								[~, tempgrad, temphess] = fun(x_0(:, 1), varargin{:}); %#ok<ASGLU> call with three output arguments to check if the third one is present and catch error if not
								Jfun = @objectivehessianvararg;
							end
							hasgradient = true;
							hashessian = true;
						catch e
							if strcmpi(e.identifier, 'MATLAB:maxlhs') || strcmpi(e.identifier, 'MATLAB:TooManyOutputs') || strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
								if  usedefaultoption
									options.SpecifyObjectiveHessian = false;
								end
								if nargin(fun) == 1
									[~, tempgrad] = fun(x_0(:, 1)); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
									Jfun = fun;
								else
									[~, tempgrad] = fun(x_0(:, 1), varargin{:}); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
									Jfun = @objectivegradientvararg;
								end
								hasgradient = true;
							else
								if  usedefaultoption
									options.SpecifyObjectiveHessian = false;
								end
								rethrow(e);
							end
						end
					else
						if nargin(fun) == 1
							[~, tempgrad] = fun(x_0(:, 1)); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
							Jfun = fun;
						else
							[~, tempgrad] = fun(x_0(:, 1), varargin{:}); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
							Jfun = @objectivegradientvararg;
						end
						hasgradient = true;
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
					if nargout(fun) >= 3
						Jfun = fun;
						hasgradient = true;
						hashessian = true;
					elseif nargout(fun) >= 2
						hasgradient = true;
						Jfun = fun;
					else
						Jfun = fun;
					end
				else
					if nargout(fun) >= 3
						Jfun = @objectivehessianvararg;
						hasgradient = true;
						hashessian = true;
					elseif nargout(fun) >= 2
						Jfun = @objectivegradientvararg;
						hasgradient = true;
					else
						Jfun = @(x) fun(x, varargin{:});
					end
				end
				if nargout(fun) < 3 && usedefaultoption
					options.SpecifyObjectiveHessian = false;
				end
				if nargout(fun) < 2 && usedefaultoption
					options.SpecifyObjectiveGradient = false;
				end
			end
		end
	else
		Jfun = fun;
		if nargout(fun) >= 3
			hasgradient = true;
			hashessian = true;
		elseif nargout(fun)>= 2
			hasgradient = true;
		end
		if nargout(fun) < 3
			if nargout(fun) == -1
				try
					if needshessian
						try
							if nargin(fun) == 1
								[~, tempgrad, temphess] = fun(x_0(:, 1)); %#ok<ASGLU> call with three output arguments to check if the third one is present and catch error if not
							else
								[~, tempgrad, temphess] = fun(x_0(:, 1), varargin{:}); %#ok<ASGLU> call with three output arguments to check if the third one is present and catch error if not
							end
							hasgradient = true;
							hashessian = true;
						catch e
							if strcmpi(e.identifier, 'MATLAB:maxlhs') || strcmpi(e.identifier, 'MATLAB:TooManyOutputs') || strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
								if  usedefaultoption
									options.SpecifyObjectiveHessian = false;
								end
								if nargin(fun) == 1
									[~, tempgrad] = fun(x_0(:, 1)); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
								else
									[~, tempgrad] = fun(x_0(:, 1), varargin{:}); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
								end
								hasgradient = true;
							else
								if  usedefaultoption
									options.SpecifyObjectiveHessian = false;
								end
								rethrow(e);
							end
						end
					else
						if nargin(fun) == 1
							[~, tempgrad] = fun(x_0(:, 1)); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
						else
							[~, tempgrad] = fun(x_0(:, 1), varargin{:}); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
						end
						hasgradient = true;
					end
				catch e
					if strcmpi(e.identifier, 'MATLAB:maxlhs') || strcmpi(e.identifier, 'MATLAB:TooManyOutputs') || strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
						if  usedefaultoption
							options.SpecifyObjectiveGradient = false;
						end
					else
						rethrow(e);
					end
				end
			else
				if  usedefaultoption
					options.SpecifyObjectiveHessian = false;
					if nargout(fun) < 2
						options.SpecifyObjectiveGradient = false;
					end
				end
			end
		end
	end
	if needshessian && ~hashessian
		error('optimization:solver:fminuncglobal:input', 'User supplied objective function does not return hessian, but calculation of hessian matrix is requested.');
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
	options.NumberVariables = size(x_0, 1);
	options.NumberConstraintsInequality = size(A, 1) + size(c, 1);
	options.NumberConstraintsEquality = size(Aeq, 1) + size(ceq, 1);
	options.NumberConstraintsBounds = sum(~isinf(lb)) + sum(~isinf(ub));
	maxTime = options.MaxTime;
	maxTimeLastvalue = maxTime;
	if isempty(maxTime) || isnan(maxTime) || maxTime <= 0
		maxTime = Inf;
		options.MaxTime = maxTime;
	end
	solveroptions = options.getpreferred();
	if options.SpecifyObjectiveHessian && hashessian% TODO: prevent warning for quasi-newton algorithm with: && strcmpi(solveroptions.Algorithm, 'trust-region')
		if matlab.Version.CURRENT >= matlab.Version.R2016A
			solveroptions.HessianFcn = 'objective';
		else
			solveroptions.Hessian = 'user-supplied';
		end
	end
	solvertime = tic;
	retry = 0;
	retryiterations = 0;
	retryfunevals = 0;
	problem = createOptimProblem('fminunc',...
		'objective',	Jfun,...
		'x0',			x_0(:, 1),...
		'options',		solveroptions...
	);
	gs = MultiStart(...
		'MaxTime',		options.MaxTime,...
		'OutputFcns',	options.OutputFcn,...
		'PlotFcns',		options.PlotFcn,...
		'TolFun',		options.FunctionTolerance,...
		'TolX',			options.StepTolerance...
	);
	options.MaxTime = maxTimeLastvalue;
	if nargout >= 4
		alloutputs = cell(max(1, options.Retries), 1);
	end
	while retry < max(1, options.Retries)
		solvertimes = tic;
		startpoints_random = RandomStartPointSet();
		startpoints_x_0 = CustomStartPointSet(x_0');
		try
			if nargout >= 4
				[x, fval, exitflag, output] = run(gs, problem, {startpoints_x_0, startpoints_random});
			elseif nargout >= 3
				[x, fval, exitflag] = run(gs, problem, {startpoints_x_0, startpoints_random});
			elseif nargout >= 2
				[x, fval] = run(gs, problem, {startpoints_x_0, startpoints_random});
			else
				x = run(gs, problem, {startpoints_x_0, startpoints_random});
			end
		catch e
			rethrow(e);
		end
		if nargout >= 4
			retryiterations = retryiterations + output.localSolverTotal;
			retryfunevals = retryfunevals + output.funcCount;
			alloutputs{retry + 1, :} = options.formatOutput(exitflag, toc(solvertimes), x, fval, numel(x_0(:, 1)), output.localSolverTotal, output.funcCount, retry, output);
		end
		retry = retry + 1;
		if toc(solvertime) > maxTime
			break;
		end
		problem.x_0 = x;
	end
	solvertime = toc(solvertime);
	if nargout >= 4
		alloutputs(retry + 1:end, :) = [];
		output = options.formatOutput(exitflag, solvertime, x, fval, numel(x_0(:, 1)), retryiterations, retryfunevals, retry, output, alloutputs);
	end
	if nargout >= 5
		lambda.lower		= NaN;
		lambda.upper		= NaN;
		lambda.ineqnonlin	= NaN(size(c, 1), 1);
		lambda.eqnonlin		= NaN(size(ceq, 1), 1);
		lambda.ineqlin		= NaN(size(A, 1), 1);
		lambda.eqlin		= NaN(size(Aeq, 1), 1);
	end
	if nargout >= 7
		hessian = NaN(size(x_0, 1), size(x_0, 1));
	end
	if nargout >= 6
		if hasgradient
			if hashessian && nargout >= 7
				[~, grad, hessian] = Jfun(x);
			else
				[~, grad] = Jfun(x);
			end
		else
			grad = zeros(size(x, 1), 1);
			step = eps;
			f = Jfun(x);
			parfor ii = 1:size(x, 1)
				grad(ii) = (Jfun(x + step*(1:size(x, 1) == ii)') - f)/step;
			end
		end
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
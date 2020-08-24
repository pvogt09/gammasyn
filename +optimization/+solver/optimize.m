function [x, fval, exitflag, output, lambda, grad, hessian] = optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin)
	%OPTIMIZE call fmincon to solve optimization problem
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
	%		varargin{:}:	additional arguments for objective function
	%	Output:
	%		x:			optimal value
	%		fval:		function value at optimal value
	%		exitflag:	optimization result indicator
	%		output:		structure with information about optimization
	%		lambda:		langrange multipliers at optimal solution
	%		grad:		objective function gradient at optimal solution
	%		hessian:	objective function hessian at optimal solution
	if isempty(fun)
		error('optimization:solver:optimize:input', 'Objective function must not be empty.');
	end
	if ischar(fun)
		fun = str2func(fun);
	end
	if ~isfunctionhandle(fun)
		error('optimization:solver:optimize:input', 'Objective function must be a function handle.');
	end
	if isempty(x_0)
		error('optimization:solver:optimize:input', 'Initial point must not be empty.');
	end
	if isa(x_0, 'AbstractStartPointSet')
		x_0 = x_0.list().';
	end
	if ~isreal(x_0) || any(isinf(x_0(:))) || any(isnan(x_0(:)))
		error('optimization:solver:optimize:input', 'Initial point must be a finite real value.');
	end
	if ~ismatrix(x_0)
		error('optimization:solver:optimize:input', 'Initial point must be a column vector or a matrix of column vectors.');
	end
	%if size(x_0, 2) > 1
	%	error('optimization:solver:optimize:input', 'Initial point must be a column vector.');
	%end
	usedefaultoption = false;
	defaultsolver = optimization.solver.Optimizer.FMINCON;
	if nargin >= 10 && isa(options, 'optimization.solver.Optimizer')
		defaultsolver = options;
		usedefaultoption = true;
	end
	if nargin <= 9 || isempty(options)
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
		error('optimization:solver:optimize:input', 'Inequality matrix must be real.');
	end
	if ~ismatrix(A)
		error('optimization:solver:optimize:input', 'Inequality matrix must be a matrix.');
	end
	if size(A, 2) ~= size(x_0, 1)
		error('optimization:solver:optimize:input', 'Inequality matrix must have %d columns, not %d.', size(x_0, 1), size(A, 2));
	end
	if ~isreal(b) || any(isnan(b(:)))
		error('optimization:solver:optimize:input', 'Inequality bounds must be real.');
	end
	if ~ismatrix(b)
		error('optimization:solver:optimize:input', 'Inequality bounds must be a vector.');
	end
	if size(b, 1) ~= size(A, 1)
		error('optimization:solver:optimize:input', 'Inequality bounds must have %d rows, not %d.', size(A, 1), size(b, 1));
	end
	if ~isreal(Aeq) || any(isnan(Aeq(:)))
		error('optimization:solver:optimize:input', 'Equality matrix must be real.');
	end
	if ~ismatrix(Aeq)
		error('optimization:solver:optimize:input', 'Equality matrix must be a matrix.');
	end
	if size(Aeq, 2) ~= size(x_0, 1)
		error('optimization:solver:optimize:input', 'Equality matrix must have %d columns, not %d.', size(x_0, 1), size(Aeq, 2));
	end
	if ~isreal(beq) || any(isnan(beq(:)))
		error('optimization:solver:optimize:input', 'Equality bounds must be real.');
	end
	if ~ismatrix(beq)
		error('optimization:solver:optimize:input', 'Equality bounds must be a vector.');
	end
	if size(beq, 1) ~= size(Aeq, 1)
		error('optimization:solver:optimize:input', 'Equality bounds must have %d rows, not %d.', size(Aeq, 1), size(beq, 1));
	end
	if ~isreal(lb) || any(isnan(lb(:)))
		error('optimization:solver:optimize:input', 'Lower bounds must be real.');
	end
	if ~ismatrix(lb)
		error('optimization:solver:optimize:input', 'Lower bounds must be a vector.');
	end
	if size(lb, 1) ~= size(x_0, 1)
		error('optimization:solver:optimize:input', 'Lower bounds must have %d rows, not %d.', size(x_0, 1), size(lb, 1));
	end
	if ~isreal(ub) || any(isnan(ub(:)))
		error('optimization:solver:optimize:input', 'Upper bounds must be real.');
	end
	if ~ismatrix(ub)
		error('optimization:solver:optimize:input', 'Upper bounds must be a vector.');
	end
	if size(ub, 1) ~= size(x_0, 1)
		error('optimization:solver:optimize:input', 'Upper bounds must have %d rows, not %d.', size(x_0, 1), size(ub, 1));
	end
	if nargin >= 9
		if ~isempty(nonlcon)
			if ischar(nonlcon)
				nonlcon = str2func(nonlcon);
			end
			if ~isfunctionhandle(nonlcon)
				error('optimization:solver:optimize:input', 'Constraint function must be a function handle.');
			end
		end
	else
		nonlcon = [];
	end
	if ~isa(options, 'optimization.options.Options')
		if isstruct(options)
			options = optimization.options.OptionFactory.instance.options(optimization.solver.Optimizer.FMINCON, options);
		elseif isa(options, 'optim.options.SolverOptions')
			options = optimization.options.OptionFactory.instance.options(optimization.solver.Optimizer.FMINCON, options);
		else
			error('optimization:solver:optimize:input', 'Options must be of type optimization.options.Options or optimoptions or optimset.');
		end
	end
	switch options.Solver
		case optimization.solver.Optimizer.FMINCON
			if usedefaultoption
				options = [];
			end
			if nargout >= 7
				[x, fval, exitflag, output, lambda, grad, hessian] = optimization.solver.fmincon.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 6
				[x, fval, exitflag, output, lambda, grad] = optimization.solver.fmincon.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 5
				[x, fval, exitflag, output, lambda] = optimization.solver.fmincon.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 4
				[x, fval, exitflag, output] = optimization.solver.fmincon.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 3
				[x, fval, exitflag] = optimization.solver.fmincon.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 2
				[x, fval] = optimization.solver.fmincon.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			else
				x = optimization.solver.fmincon.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			end
		case optimization.solver.Optimizer.FMINCONGLOBAL
			if usedefaultoption
				options = [];
			end
			if nargout >= 7
				[x, fval, exitflag, output, lambda, grad, hessian] = optimization.solver.fminconglobal.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 6
				[x, fval, exitflag, output, lambda, grad] = optimization.solver.fminconglobal.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 5
				[x, fval, exitflag, output, lambda] = optimization.solver.fminconglobal.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 4
				[x, fval, exitflag, output] = optimization.solver.fminconglobal.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 3
				[x, fval, exitflag] = optimization.solver.fminconglobal.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 2
				[x, fval] = optimization.solver.fminconglobal.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			else
				x = optimization.solver.fminconglobal.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			end
		case optimization.solver.Optimizer.FMINIMAX
			if usedefaultoption
				options = [];
			end
			if nargout >= 7
				[x, fval, exitflag, output, lambda, grad, hessian] = optimization.solver.fminimax.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 6
				[x, fval, exitflag, output, lambda, grad] = optimization.solver.fminimax.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 5
				[x, fval, exitflag, output, lambda] = optimization.solver.fminimax.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 4
				[x, fval, exitflag, output] = optimization.solver.fminimax.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 3
				[x, fval, exitflag] = optimization.solver.fminimax.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 2
				[x, fval] = optimization.solver.fminimax.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			else
				x = optimization.solver.fminimax.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			end
		case optimization.solver.Optimizer.FMINSEARCH
			if usedefaultoption
				options = [];
			end
			if nargout >= 7
				[x, fval, exitflag, output, lambda, grad, hessian] = optimization.solver.fminsearch.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 6
				[x, fval, exitflag, output, lambda, grad] = optimization.solver.fminsearch.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 5
				[x, fval, exitflag, output, lambda] = optimization.solver.fminsearch.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 4
				[x, fval, exitflag, output] = optimization.solver.fminsearch.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 3
				[x, fval, exitflag] = optimization.solver.fminsearch.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 2
				[x, fval] = optimization.solver.fminsearch.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			else
				x = optimization.solver.fminsearch.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			end
		case optimization.solver.Optimizer.FMINUNC
			if usedefaultoption
				options = [];
			end
			if nargout >= 7
				[x, fval, exitflag, output, lambda, grad, hessian] = optimization.solver.fminunc.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 6
				[x, fval, exitflag, output, lambda, grad] = optimization.solver.fminunc.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 5
				[x, fval, exitflag, output, lambda] = optimization.solver.fminunc.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 4
				[x, fval, exitflag, output] = optimization.solver.fminunc.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 3
				[x, fval, exitflag] = optimization.solver.fminunc.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 2
				[x, fval] = optimization.solver.fminunc.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			else
				x = optimization.solver.fminunc.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			end
		case optimization.solver.Optimizer.FMINUNCGLOBAL
			if usedefaultoption
				options = [];
			end
			if nargout >= 7
				[x, fval, exitflag, output, lambda, grad, hessian] = optimization.solver.fminuncglobal.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 6
				[x, fval, exitflag, output, lambda, grad] = optimization.solver.fminuncglobal.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 5
				[x, fval, exitflag, output, lambda] = optimization.solver.fminuncglobal.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 4
				[x, fval, exitflag, output] = optimization.solver.fminuncglobal.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 3
				[x, fval, exitflag] = optimization.solver.fminuncglobal.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 2
				[x, fval] = optimization.solver.fminuncglobal.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			else
				x = optimization.solver.fminuncglobal.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			end
		case optimization.solver.Optimizer.GA
			if usedefaultoption
				options = [];
			end
			if nargout >= 7
				[x, fval, exitflag, output, lambda, grad, hessian] = optimization.solver.ga.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 6
				[x, fval, exitflag, output, lambda, grad] = optimization.solver.ga.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 5
				[x, fval, exitflag, output, lambda] = optimization.solver.ga.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 4
				[x, fval, exitflag, output] = optimization.solver.ga.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 3
				[x, fval, exitflag] = optimization.solver.ga.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 2
				[x, fval] = optimization.solver.ga.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			else
				x = optimization.solver.ga.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			end
		case optimization.solver.Optimizer.IPOPT
			if usedefaultoption
				options = [];
			end
			if nargout >= 7
				[x, fval, exitflag, output, lambda, grad, hessian] = optimization.solver.ipopt.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 6
				[x, fval, exitflag, output, lambda, grad] = optimization.solver.ipopt.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 5
				[x, fval, exitflag, output, lambda] = optimization.solver.ipopt.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 4
				[x, fval, exitflag, output] = optimization.solver.ipopt.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 3
				[x, fval, exitflag] = optimization.solver.ipopt.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 2
				[x, fval] = optimization.solver.ipopt.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			else
				x = optimization.solver.ipopt.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			end
		case optimization.solver.Optimizer.NLOPTCON
			if usedefaultoption
				options = [];
			end
			if nargout >= 7
				[x, fval, exitflag, output, lambda, grad, hessian] = optimization.solver.nloptcon.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 6
				[x, fval, exitflag, output, lambda, grad] = optimization.solver.nloptcon.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 5
				[x, fval, exitflag, output, lambda] = optimization.solver.nloptcon.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 4
				[x, fval, exitflag, output] = optimization.solver.nloptcon.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 3
				[x, fval, exitflag] = optimization.solver.nloptcon.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 2
				[x, fval] = optimization.solver.nloptcon.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			else
				x = optimization.solver.nloptcon.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			end
		case optimization.solver.Optimizer.NLOPTCONGLOBAL
			if usedefaultoption
				options = [];
			end
			if nargout >= 7
				[x, fval, exitflag, output, lambda, grad, hessian] = optimization.solver.nloptconglobal.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 6
				[x, fval, exitflag, output, lambda, grad] = optimization.solver.nloptconglobal.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 5
				[x, fval, exitflag, output, lambda] = optimization.solver.nloptconglobal.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 4
				[x, fval, exitflag, output] = optimization.solver.nloptconglobal.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 3
				[x, fval, exitflag] = optimization.solver.nloptconglobal.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 2
				[x, fval] = optimization.solver.nloptconglobal.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			else
				x = optimization.solver.nloptconglobal.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			end
		case optimization.solver.Optimizer.NLOPTUNC
			if usedefaultoption
				options = [];
			end
			if nargout >= 7
				[x, fval, exitflag, output, lambda, grad, hessian] = optimization.solver.nloptunc.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 6
				[x, fval, exitflag, output, lambda, grad] = optimization.solver.nloptunc.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 5
				[x, fval, exitflag, output, lambda] = optimization.solver.nloptunc.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 4
				[x, fval, exitflag, output] = optimization.solver.nloptunc.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 3
				[x, fval, exitflag] = optimization.solver.nloptunc.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 2
				[x, fval] = optimization.solver.nloptunc.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			else
				x = optimization.solver.nloptunc.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			end
		case optimization.solver.Optimizer.NLOPTUNCGLOBAL
			if usedefaultoption
				options = [];
			end
			if nargout >= 7
				[x, fval, exitflag, output, lambda, grad, hessian] = optimization.solver.nloptuncglobal.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 6
				[x, fval, exitflag, output, lambda, grad] = optimization.solver.nloptuncglobal.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 5
				[x, fval, exitflag, output, lambda] = optimization.solver.nloptuncglobal.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 4
				[x, fval, exitflag, output] = optimization.solver.nloptuncglobal.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 3
				[x, fval, exitflag] = optimization.solver.nloptuncglobal.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 2
				[x, fval] = optimization.solver.nloptuncglobal.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			else
				x = optimization.solver.nloptuncglobal.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			end
		case optimization.solver.Optimizer.PARTICLESWARM
			if usedefaultoption
				options = [];
			end
			if nargout >= 7
				[x, fval, exitflag, output, lambda, grad, hessian] = optimization.solver.particleswarm.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 6
				[x, fval, exitflag, output, lambda, grad] = optimization.solver.particleswarm.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 5
				[x, fval, exitflag, output, lambda] = optimization.solver.particleswarm.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 4
				[x, fval, exitflag, output] = optimization.solver.particleswarm.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 3
				[x, fval, exitflag] = optimization.solver.particleswarm.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 2
				[x, fval] = optimization.solver.particleswarm.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			else
				x = optimization.solver.particleswarm.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			end
		case optimization.solver.Optimizer.PATTERNSEARCH
			if usedefaultoption
				options = [];
			end
			if nargout >= 7
				[x, fval, exitflag, output, lambda, grad, hessian] = optimization.solver.patternsearch.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 6
				[x, fval, exitflag, output, lambda, grad] = optimization.solver.patternsearch.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 5
				[x, fval, exitflag, output, lambda] = optimization.solver.patternsearch.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 4
				[x, fval, exitflag, output] = optimization.solver.patternsearch.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 3
				[x, fval, exitflag] = optimization.solver.patternsearch.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 2
				[x, fval] = optimization.solver.patternsearch.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			else
				x = optimization.solver.patternsearch.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			end
		case optimization.solver.Optimizer.PPPBOX
			if usedefaultoption
				options = [];
			end
			if nargout >= 7
				[x, fval, exitflag, output, lambda, grad, hessian] = optimization.solver.pppbox.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 6
				[x, fval, exitflag, output, lambda, grad] = optimization.solver.pppbox.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 5
				[x, fval, exitflag, output, lambda] = optimization.solver.pppbox.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 4
				[x, fval, exitflag, output] = optimization.solver.pppbox.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 3
				[x, fval, exitflag] = optimization.solver.pppbox.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 2
				[x, fval] = optimization.solver.pppbox.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			else
				x = optimization.solver.pppbox.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			end
		case optimization.solver.Optimizer.SIMULANNEAL
			if usedefaultoption
				options = [];
			end
			if nargout >= 7
				[x, fval, exitflag, output, lambda, grad, hessian] = optimization.solver.simulanneal.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 6
				[x, fval, exitflag, output, lambda, grad] = optimization.solver.simulanneal.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 5
				[x, fval, exitflag, output, lambda] = optimization.solver.simulanneal.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 4
				[x, fval, exitflag, output] = optimization.solver.simulanneal.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 3
				[x, fval, exitflag] = optimization.solver.simulanneal.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 2
				[x, fval] = optimization.solver.simulanneal.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			else
				x = optimization.solver.simulanneal.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			end
		case optimization.solver.Optimizer.SNOPT
			if usedefaultoption
				options = [];
			end
			if nargout >= 7
				[x, fval, exitflag, output, lambda, grad, hessian] = optimization.solver.snopt.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 6
				[x, fval, exitflag, output, lambda, grad] = optimization.solver.snopt.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 5
				[x, fval, exitflag, output, lambda] = optimization.solver.snopt.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 4
				[x, fval, exitflag, output] = optimization.solver.snopt.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 3
				[x, fval, exitflag] = optimization.solver.snopt.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 2
				[x, fval] = optimization.solver.snopt.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			else
				x = optimization.solver.snopt.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			end
		case optimization.solver.Optimizer.SLPGS
			if usedefaultoption
				options = [];
			end
			if nargout >= 7
				[x, fval, exitflag, output, lambda, grad, hessian] = optimization.solver.slpgs.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 6
				[x, fval, exitflag, output, lambda, grad] = optimization.solver.slpgs.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 5
				[x, fval, exitflag, output, lambda] = optimization.solver.slpgs.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 4
				[x, fval, exitflag, output] = optimization.solver.slpgs.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 3
				[x, fval, exitflag] = optimization.solver.slpgs.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 2
				[x, fval] = optimization.solver.slpgs.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			else
				x = optimization.solver.slpgs.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			end
		case optimization.solver.Optimizer.SOLVOPT
			if usedefaultoption
				options = [];
			end
			if nargout >= 7
				[x, fval, exitflag, output, lambda, grad, hessian] = optimization.solver.solvopt.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 6
				[x, fval, exitflag, output, lambda, grad] = optimization.solver.solvopt.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 5
				[x, fval, exitflag, output, lambda] = optimization.solver.solvopt.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 4
				[x, fval, exitflag, output] = optimization.solver.solvopt.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 3
				[x, fval, exitflag] = optimization.solver.solvopt.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 2
				[x, fval] = optimization.solver.solvopt.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			else
				x = optimization.solver.solvopt.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			end
		case optimization.solver.Optimizer.SQPGS
			if usedefaultoption
				options = [];
			end
			if nargout >= 7
				[x, fval, exitflag, output, lambda, grad, hessian] = optimization.solver.sqpgs.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 6
				[x, fval, exitflag, output, lambda, grad] = optimization.solver.sqpgs.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 5
				[x, fval, exitflag, output, lambda] = optimization.solver.sqpgs.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 4
				[x, fval, exitflag, output] = optimization.solver.sqpgs.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 3
				[x, fval, exitflag] = optimization.solver.sqpgs.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 2
				[x, fval] = optimization.solver.sqpgs.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			else
				x = optimization.solver.sqpgs.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			end
		case optimization.solver.Optimizer.SCBFGS
			if usedefaultoption
				options = [];
			end
			if nargout >= 7
				[x, fval, exitflag, output, lambda, grad, hessian] = optimization.solver.scbfgs.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 6
				[x, fval, exitflag, output, lambda, grad] = optimization.solver.scbfgs.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 5
				[x, fval, exitflag, output, lambda] = optimization.solver.scbfgs.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 4
				[x, fval, exitflag, output] = optimization.solver.scbfgs.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 3
				[x, fval, exitflag] = optimization.solver.scbfgs.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 2
				[x, fval] = optimization.solver.scbfgs.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			else
				x = optimization.solver.scbfgs.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			end
		case optimization.solver.Optimizer.KSOPT
			if usedefaultoption
				options = [];
			end
			if nargout >= 7
				[x, fval, exitflag, output, lambda, grad, hessian] = optimization.solver.ksopt.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 6
				[x, fval, exitflag, output, lambda, grad] = optimization.solver.ksopt.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 5
				[x, fval, exitflag, output, lambda] = optimization.solver.ksopt.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 4
				[x, fval, exitflag, output] = optimization.solver.ksopt.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 3
				[x, fval, exitflag] = optimization.solver.ksopt.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			elseif nargout >= 2
				[x, fval] = optimization.solver.ksopt.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			else
				x = optimization.solver.ksopt.optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin{:});
			end
		otherwise
			error('optimization:solver:optimize:input', 'Undefined optimizer.');
	end
end
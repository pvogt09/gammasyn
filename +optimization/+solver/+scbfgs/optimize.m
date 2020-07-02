function [x, fval, exitflag, output, lambda, grad, hessian] = optimize(fun, x_0, A, b, Aeq, beq, lb, ub, ~, options, varargin)
	%OPTIMIZE call scbfgs to solve optimization problem
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
		error('optimization:solver:scbfgs:input', 'Objective function must not be empty.');
	end
	if ischar(fun)
		fun = str2func(fun);
	end
	if ~isfunctionhandle(fun)
		error('optimization:solver:scbfgs:input', 'Objective function must be a function handle.');
	end
	if isempty(x_0)
		error('optimization:solver:scbfgs:input', 'Initial point must not be empty.');
	end
	if isa(x_0, 'AbstractStartPointSet')
		x_0 = x_0.list().';
	end
	if ~isreal(x_0) || any(isinf(x_0(:))) || any(isnan(x_0(:)))
		error('optimization:solver:scbfgs:input', 'Initial point must be a finite real value.');
	end
	if ~ismatrix(x_0)
		error('optimization:solver:scbfgs:input', 'Initial point must be a column vector or a matrix of column vectors.');
	end
	%if size(x_0, 2) > 1
	%	error('optimization:solver:scbfgs:input', 'Initial point must be a column vector.');
	%end
	usedefaultoption = false;
	defaultsolver = optimization.solver.Optimizer.SCBFGS;
	if nargin >= 10 && isa(options, 'optimization.solver.Optimizer')
		defaultsolver = options;
		usedefaultoption = true;
	end
	if nargin <= 9 || isempty(options)
		options = optimization.options.OptionFactory.instance.options(defaultsolver,...
			'Algorithm',					defaultsolver.getDefaultAlgorithm(),...
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
		if nargin == 3 && isa(A, 'optimization.options.Options')
			options = A;
		end
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
		error('optimization:solver:scbfgs:input', 'Inequality matrix must be real.');
	end
	if ~ismatrix(A)
		error('optimization:solver:scbfgs:input', 'Inequality matrix must be a matrix.');
	end
	if size(A, 2) ~= size(x_0, 1)
		error('optimization:solver:scbfgs:input', 'Inequality matrix must have %d columns, not %d.', size(x_0, 1), size(A, 2));
	end
	if ~isreal(b) || any(isnan(b(:)))
		error('optimization:solver:scbfgs:input', 'Inequality bounds must be real.');
	end
	if ~ismatrix(b)
		error('optimization:solver:scbfgs:input', 'Inequality bounds must be a vector.');
	end
	if size(b, 1) ~= size(A, 1)
		error('optimization:solver:scbfgs:input', 'Inequality bounds must have %d rows, not %d.', size(A, 1), size(b, 1));
	end
	if ~isreal(Aeq) || any(isnan(Aeq(:)))
		error('optimization:solver:scbfgs:input', 'Equality matrix must be real.');
	end
	if ~ismatrix(Aeq)
		error('optimization:solver:scbfgs:input', 'Equality matrix must be a matrix.');
	end
	if size(Aeq, 2) ~= size(x_0, 1)
		error('optimization:solver:scbfgs:input', 'Equality matrix must have %d columns, not %d.', size(x_0, 1), size(Aeq, 2));
	end
	if ~isreal(beq) || any(isnan(beq(:)))
		error('optimization:solver:scbfgs:input', 'Equality bounds must be real.');
	end
	if ~ismatrix(beq)
		error('optimization:solver:scbfgs:input', 'Equality bounds must be a vector.');
	end
	if size(beq, 1) ~= size(Aeq, 1)
		error('optimization:solver:scbfgs:input', 'Equality bounds must have %d rows, not %d.', size(Aeq, 1), size(beq, 1));
	end
	if ~isreal(lb) || any(isnan(lb(:)))
		error('optimization:solver:scbfgs:input', 'Lower bounds must be real.');
	end
	if ~ismatrix(lb)
		error('optimization:solver:scbfgs:input', 'Lower bounds must be a vector.');
	end
	if size(lb, 1) ~= size(x_0, 1)
		error('optimization:solver:scbfgs:input', 'Lower bounds must have %d rows, not %d.', size(x_0, 1), size(lb, 1));
	end
	if ~isreal(ub) || any(isnan(ub(:)))
		error('optimization:solver:scbfgs:input', 'Upper bounds must be real.');
	end
	if ~ismatrix(ub)
		error('optimization:solver:scbfgs:input', 'Upper bounds must be a vector.');
	end
	if size(ub, 1) ~= size(x_0, 1)
		error('optimization:solver:scbfgs:input', 'Upper bounds must have %d rows, not %d.', size(x_0, 1), size(ub, 1));
	end
	if ~isa(options, 'optimization.options.Options')
		if isstruct(options)
			options = optimization.options.OptionFactory.instance.options(optimization.solver.Optimizer.SCBFGS, options);
		elseif isa(options, 'optim.options.SolverOptions')
			options = optimization.options.OptionFactory.instance.options(optimization.solver.Optimizer.SCBFGS, options);
		else
			error('optimization:solver:scbfgs:input', 'Options must be of type optimization.options.Options or optimoptions or optimset.');
		end
	end
	if ~isa(options, 'optimization.options.scbfgs')
		options = optimization.options.OptionFactory.instance.options(optimization.solver.Optimizer.SCBFGS, options);
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
					[~, gradJ] = fun(x_0(:, 1));
					Jfun = fun;
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
	if options.SpecifyObjectiveGradient
		if any(isnan(gradJ(:)))
			gradient = false;
		else
			gradient = true;
		end
	else
		gradient = false;
	end
	options.NumberVariables = size(x_0, 1);
	options.NumberConstraintsInequality = 0;
	options.NumberConstraintsEquality = 0;
	options.NumberConstraintsBounds = sum(~isinf(lb)) + sum(~isinf(ub));
	%solveroptions = options.getpreferred();
	if isempty(options.FiniteDifferenceType)
		FinDiffType = optimization.general.FinDiffType.FORWARD;
	else
		FinDiffType = optimization.general.FinDiffType.fromname(options.FiniteDifferenceType);
	end
	if isscalar(options.FiniteDifferenceStepSize)
		FinDiffStepSize = options.FiniteDifferenceStepSize*ones(size(x_0, 1), 1);
	else
		FinDiffStepSize = options.FiniteDifferenceStepSize;
	end
	Jfun = function_dispatcher(Jfun, gradient, FinDiffType, FinDiffStepSize, options.TypicalX, options.HonorBounds);
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
	nargout6 = nargout >= 6;
	nargout7 = nargout >= 7;
	if nargout4
		alloutputs = cell(max(1, options.Retries), initialvalues);
	end
	max_iter = options.MaxIterations;
	a0    = 1/16;      % stepsize constants
	a1    = 0;         % ... for the formula alpha = 1/(a0 + k*a1)
	b     = 64;        % batch size
	dmax  = b*max_iter;% accessed data point (ADP) limit
	eta   = 1/4;       % BFGS displacement vector constants
	theta = 4;         % ... equation (9) in reference paper
	C     = 0;         % max iterations in SC-BFGS-sub in reference paper
	rho   = 1/8;       % input parameters
	tau   = 8;         % ... for SC-BFGS-sub in reference paper
	M     = 0;         % L-BFGS history length (0 ~ full BFGS)
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
		if nargout6 || useparallel
			grad = cell(1, initialvalues);
		end
		if nargout7 || useparallel
			hessian = cell(1, initialvalues);
		end
		if useparallel
			parfor ii = 1:initialvalues
				iterpar = 0;
				functionevalspar = 0;
				steplengthpar = 0;
				try
					if nargout6
						[x_opt(:, ii), ~, f_opt(1, ii), iterpar, functionevalspar, steplengthpar, grad{1, ii}] = scbfgs.scBFGS(Jfun, x_0(:, ii), dmax, a0, a1, eta, theta, C, rho, tau, M, b);
					elseif nargout4
						[x_opt(:, ii), ~, f_opt(1, ii), iterpar, functionevalspar, steplengthpar] = scbfgs.scBFGS(Jfun, x_0(:, ii), dmax, a0, a1, eta, theta, C, rho, tau, M, b);
					else
						[x_opt(:, ii), ~, f_opt(1, ii)] = scbfgs.scBFGS(Jfun, x_0(:, ii), dmax, a0, a1, eta, theta, C, rho, tau, M, b);
					end
					if nargout3
						exitflag(1, ii) = 1;
					end
					if nargout4
						output{1, ii} = struct(...
							'iterations',	iterpar,...
							'funcCount',	functionevalspar,...
							'lssteplength',	steplengthpar...
						);
					end
				catch e
					rethrow(e);
				end
				if nargout4
					iterations(1, ii) = output{1, ii}.iterations;
					funevals(1, ii) = output{1, ii}.funcCount;
					tempoutputs{1, ii} = options.formatOutput(exitflag(1, ii), toc(solvertimes(1, ii)), x_opt(:, ii), f_opt(1, ii), numel(x_0(:, ii)), iterations(1, ii), funevals(1, ii), retry, output{1, ii}); %#ok<PFBNS> options is an object
				end
			end
		else
			for ii = 1:initialvalues %#ok<FORPF> parallel computing is disabled through option set by user
				solvertimes(1, ii) = tic;
				try
					if nargout6
						[x_opt(:, ii), ~, f_opt(1, ii), iter, functionevals, steplength, grad{1, ii}] = scbfgs.scBFGS(Jfun, x_0(:, ii), dmax, a0, a1, eta, theta, C, rho, tau, M, b);
					elseif nargout4
						[x_opt(:, ii), ~, f_opt(1, ii), iter, functionevals, steplength] = scbfgs.scBFGS(Jfun, x_0(:, ii), dmax, a0, a1, eta, theta, C, rho, tau, M, b);
					else
						[x_opt(:, ii), ~, f_opt(1, ii)] = scbfgs.scBFGS(Jfun, x_0(:, ii), dmax, a0, a1, eta, theta, C, rho, tau, M, b);
					end
					if nargout3
						exitflag(1, ii) = 1;
					end
					if nargout4
						output{1, ii} = struct(...
							'iterations',	iter,...
							'funcCount',	functionevals,...
							'lssteplength',	steplength...
						);
					end
				catch e
					rethrow(e);
				end
				if nargout4
					iterations(1, ii) = output{1, ii}.iterations;
					funevals(1, ii) = output{1, ii}.funcCount;
					tempoutputs{1, ii} = options.formatOutput(exitflag(1, ii), toc(solvertimes(1, ii)), x_opt(:, ii), f_opt(1, ii), numel(x_0(:, ii)), iterations(1, ii), funevals(1, ii), retry, output{1, ii});
				end
			end
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
	% HINT: constraints are not incorporated in this algorithm, so minimal objective value is the only criterion for best value
	if matlab.Version.CURRENT >= matlab.Version.R2015A
		[fval, idx] = min(f_opt, [], 2, 'omitnan');
	else
		[fval, idx] = min(f_opt, [], 2);
	end
	if isempty(idx)
		error('optimization:solver:fminunc:solution', 'No solution could be found for any initial value.');
	end
	x = x_opt(:, idx);
	if nargout3
		exitflag = exitflag(1, idx);
	end
	if nargout4
		output = output{1, idx};
	end
	if nargout6
		grad = grad{1, idx};
	end
	if nargout7
		hessian = NaN(size(x_0, 1), size(x_0, 1));
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
	solvertime = toc(solvertime);
	if nargout >= 4
		alloutputs(retry + 1:end, :) = [];
		output = options.formatOutput(exitflag, solvertime, x, fval, numel(x_0(:, 1)), retryiterations, retryfunevals, retry, output, alloutputs);
	end
end

function [fun] = function_dispatcher(obj, gradientJ, FinDiffType, FinDiffRelStep, TypicalX, AlwaysHonorBounds)
	%FUNCTION_DISPATCHER wrapper function for scbfgs to convert the objective function in a function with return type argument
	%	Input:
	%		obj:				objective function handle
	%		gradientJ:			indicator, if gradient for objective function is supplied
	%		FinDiffType:		finite difference type to use
	%		FinDiffRelStep:		relative finite difference step
	%		TypicalX:			typical values for optimization variable
	%		AlwaysHonorBounds:	indicator if bounds are honored during finite difference calculation
	%	Output:
	%		fun:				objective function handle for use with SC-BFGS
	function [val] = f(x, type)
		if type == 0% Objective function value
			val = obj(x);
		elseif type == 1% Objective gradient value (as column vector)
			if gradientJ
				[~, val] = obj(x);
			else
				val = optimization.general.finitedifference(x, obj, NaN, FinDiffType, 1, [], [], FinDiffRelStep, TypicalX, AlwaysHonorBounds, true).';
			end
		else% This error is not supposed to happen!
			error('optimization:solver:scbfgs:input', 'Option not given to problem function evaluator.');
		end
	end
	fun = @f;
end
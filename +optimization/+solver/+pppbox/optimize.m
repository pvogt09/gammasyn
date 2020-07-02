function [x, fval, exitflag, output, lambda, grad, hessian] = optimize(fun, x_0, A, b, Aeq, beq, lb, ub, ~, options, varargin)
	%OPTIMIZE call pppbox with constrained algorithm to solve optimization problem
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
		error('optimization:solver:pppbox:input', 'Objective function must not be empty.');
	end
	if ischar(fun)
		fun = str2func(fun);
	end
	if ~isfunctionhandle(fun)
		error('optimization:solver:pppbox:input', 'Objective function must be a function handle.');
	end
	if isempty(x_0)
		error('optimization:solver:pppbox:input', 'Initial point must not be empty.');
	end
	if isa(x_0, 'AbstractStartPointSet')
		x_0 = x_0.list().';
	end
	if ~isreal(x_0) || any(isinf(x_0(:))) || any(isnan(x_0(:)))
		error('optimization:solver:pppbox:input', 'Initial point must be a finite real value.');
	end
	if ~ismatrix(x_0)
		error('optimization:solver:pppbox:input', 'Initial point must be a column vector or a matrix of column vectors.');
	end
	%if size(x_0, 2) > 1
	%	error('optimization:solver:pppbox:input', 'Initial point must be a column vector.');
	%end
	usedefaultoption = false;
	defaultsolver = optimization.solver.Optimizer.PPPBOX;
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
		error('optimization:solver:pppbox:input', 'Inequality matrix must be real.');
	end
	if ~ismatrix(A)
		error('optimization:solver:pppbox:input', 'Inequality matrix must be a matrix.');
	end
	if size(A, 2) ~= size(x_0, 1)
		error('optimization:solver:pppbox:input', 'Inequality matrix must have %d columns, not %d.', size(x_0, 1), size(A, 2));
	end
	if ~isreal(b) || any(isnan(b(:)))
		error('optimization:solver:pppbox:input', 'Inequality bounds must be real.');
	end
	if ~ismatrix(b)
		error('optimization:solver:pppbox:input', 'Inequality bounds must be a vector.');
	end
	if size(b, 1) ~= size(A, 1)
		error('optimization:solver:pppbox:input', 'Inequality bounds must have %d rows, not %d.', size(A, 1), size(b, 1));
	end
	if ~isreal(Aeq) || any(isnan(Aeq(:)))
		error('optimization:solver:pppbox:input', 'Equality matrix must be real.');
	end
	if ~ismatrix(Aeq)
		error('optimization:solver:pppbox:input', 'Equality matrix must be a matrix.');
	end
	if size(Aeq, 2) ~= size(x_0, 1)
		error('optimization:solver:pppbox:input', 'Equality matrix must have %d columns, not %d.', size(x_0, 1), size(Aeq, 2));
	end
	if ~isreal(beq) || any(isnan(beq(:)))
		error('optimization:solver:pppbox:input', 'Equality bounds must be real.');
	end
	if ~ismatrix(beq)
		error('optimization:solver:pppbox:input', 'Equality bounds must be a vector.');
	end
	if size(beq, 1) ~= size(Aeq, 1)
		error('optimization:solver:pppbox:input', 'Equality bounds must have %d rows, not %d.', size(Aeq, 1), size(beq, 1));
	end
	if ~isreal(lb) || any(isnan(lb(:)))
		error('optimization:solver:pppbox:input', 'Lower bounds must be real.');
	end
	if ~ismatrix(lb)
		error('optimization:solver:pppbox:input', 'Lower bounds must be a vector.');
	end
	if size(lb, 1) ~= size(x_0, 1)
		error('optimization:solver:pppbox:input', 'Lower bounds must have %d rows, not %d.', size(x_0, 1), size(lb, 1));
	end
	if ~isreal(ub) || any(isnan(ub(:)))
		error('optimization:solver:pppbox:input', 'Upper bounds must be real.');
	end
	if ~ismatrix(ub)
		error('optimization:solver:pppbox:input', 'Upper bounds must be a vector.');
	end
	if size(ub, 1) ~= size(x_0, 1)
		error('optimization:solver:pppbox:input', 'Upper bounds must have %d rows, not %d.', size(x_0, 1), size(ub, 1));
	end
	if ~isa(options, 'optimization.options.Options')
		if isstruct(options)
			options = optimization.options.OptionFactory.instance.options(optimization.solver.Optimizer.PPPBOX, options);
		elseif isa(options, 'optim.options.SolverOptions')
			options = optimization.options.OptionFactory.instance.options(optimization.solver.Optimizer.PPPBOX, options);
		else
			error('optimization:solver:pppbox:input', 'Options must be of type optimization.options.Options or optimoptions or optimset.');
		end
	end
	if ~isa(options, 'optimization.options.pppbox')
		options = optimization.options.OptionFactory.instance.options(optimization.solver.Optimizer.PPPBOX, options);
	end
	function [f, grad] = objectivegradientvararg(x)
		[f, grad] = fun(x, varargin{:});
	end
	isvarargfun = false;
	if nargin >= 11
		if nargout(fun) >= 2
			if nargin(fun) == 1
				Jfun = fun;
			else
				Jfun = @objectivegradientvararg;
				isvarargfun = true;
			end
		else
			if nargout(fun) == -1
				try
					if nargin(fun) == 1
						[~, tempgrad] = fun(x_0(:, 1)); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
						Jfun = fun;
					else
						[~, tempgrad] = fun(x_0(:, 1), varargin{:}); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
						Jfun = @objectivegradientvararg;
						isvarargfun = true;
					end
				catch e
					if ~strcmpi(e.identifier, 'MATLAB:maxlhs') && ~strcmpi(e.identifier, 'MATLAB:TooManyOutputs') && ~strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
						rethrow(e);
					else
						if nargin(fun) == 1
							Jfun = fun;
						else
							Jfun = @(x) fun(x, varargin{:});
							isvarargfun = true;
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
					isvarargfun = true;
				end
				if usedefaultoption
					options.SpecifyObjectiveGradient = false;
				end
			end
		end
	else
		Jfun = fun;
		isvarargfun = nargin(fun) > 1;
		if usedefaultoption && nargout(fun) < 2
			if nargout(fun) == -1
				try
					if nargin(fun) == 1
						[~, tempgrad] = fun(x_0(:, 1)); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
					else
						[~, tempgrad] = fun(x_0(:, 1), varargin{:}); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
						isvarargfun = true;
					end
				catch e
					if ~strcmpi(e.identifier, 'MATLAB:maxlhs') && ~strcmpi(e.identifier, 'MATLAB:TooManyOutputs') && ~strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
						rethrow(e);
					else
						options.SpecifyObjectiveGradient = false;
					end
				end
			else
				options.SpecifyObjectiveGradient = false;
			end
		end
	end
	options.NumberVariables = size(x_0, 1);
	options.NumberConstraintsInequality = size(A, 1) + 0;
	options.NumberConstraintsEquality = size(Aeq, 1) + 0;
	options.NumberConstraintsBounds = sum(~isinf(lb)) + sum(~isinf(ub));
	solveroptions = options.getpreferred();
	multiobjective = strcmpi(options.Algorithm, 'multi');
	if nargin >= 11
		if nargin(fun) == 1
			fval = fun(x_0(:, 1));
		else
			fval = fun(x_0(:, 1), varargin{:});
		end
	else
		fval = fun(x_0(:, 1));
	end
	if ~multiobjective && ~isscalar(fval)
		error('optimization:solver:pppbox:input', 'Objective function must be scalar for PPPBox Algorithm ''single''.');
	end
	objective_number = size(fval, 1);
	if isfield(solveroptions, 'Display')
		solveroptions.verbose = upper(solveroptions.Display);
	end
	pppboxnames = fieldnames(solveroptions);
	removefields = false(size(pppboxnames, 1), 1);
	parfor ii = 1:size(pppboxnames, 1)
		if isempty(solveroptions.(pppboxnames{ii, 1}))
			removefields(ii, 1) = true;
		end
	end
	solveroptions = rmfield(solveroptions, pppboxnames(removefields));
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
					if multiobjective
						if nargout4
							if isvarargfun
								[x_opt(:, ii), f_opt(1, ii), exitflag(1, ii), output{1, ii}] = pppNonSmooth(fun, x_0(:, ii), solveroptions, varargin{:});
							else
								[x_opt(:, ii), f_opt(1, ii), exitflag(1, ii), output{1, ii}] = pppNonSmooth(fun, x_0(:, ii), solveroptions);
							end
						elseif nargout3
							if isvarargfun
								[x_opt(:, ii), f_opt(1, ii), exitflag(1, ii)] = pppNonSmooth(fun, x_0(:, ii), solveroptions, varargin{:});
							else
								[x_opt(:, ii), f_opt(1, ii), exitflag(1, ii)] = pppNonSmooth(fun, x_0(:, ii), solveroptions);
							end
						else
							if isvarargfun
								[x_opt(:, ii), f_opt(1, ii)] = pppNonSmooth(fun, x_0(:, ii), solveroptions, varargin{:});
							else
								[x_opt(:, ii), f_opt(1, ii)] = pppNonSmooth(fun, x_0(:, ii), solveroptions);
							end
						end
					else
						if nargout4
							if isvarargfun
								[x_opt(:, ii), f_opt(1, ii), exitflag(1, ii), output{1, ii}] = pppBox(fun, x_0(:, ii), solveroptions, A, b, varargin{:});
							else
								[x_opt(:, ii), f_opt(1, ii), exitflag(1, ii), output{1, ii}] = pppBox(fun, x_0(:, ii), solveroptions, A, b);
							end
						elseif nargout3
							if isvarargfun
								[x_opt(:, ii), f_opt(1, ii), exitflag(1, ii)] = pppBox(fun, x_0(:, ii), solveroptions, A, b, varargin{:});
							else
								[x_opt(:, ii), f_opt(1, ii), exitflag(1, ii)] = pppBox(fun, x_0(:, ii), solveroptions, A, b);
							end
						else
							if isvarargfun
								[x_opt(:, ii), f_opt(1, ii)] = pppBox(fun, x_0(:, ii), solveroptions, A, b, varargin{:});
							else
								[x_opt(:, ii), f_opt(1, ii)] = pppBox(fun, x_0(:, ii), solveroptions, A, b);
							end
						end
					end
				catch e
					if strcmpi(e.identifier, 'EMLRT:runTime:UserInterrupt')
						x_opt(:, ii) = NaN(size(x_0(:, ii), 1), 1);
						f_opt(1, ii) = NaN;
						exitflag(1, ii) = -5;
						if nargout4
							output{1, ii} = struct(...
								'iterations',		0,...
								'funcCount',		1,...
								'firstorderopt',	0,...
								'message',			'',...
								'time',				0,...
								'trace',			struct()...
							);
						end
					else
						rethrow(e);
					end
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
					if multiobjective
						if nargout4
							if isvarargfun
								[x_opt(:, ii), f_opt(1, ii), exitflag(1, ii), output{1, ii}] = pppNonSmooth(fun, x_0(:, ii), solveroptions, varargin{:});
							else
								[x_opt(:, ii), f_opt(1, ii), exitflag(1, ii), output{1, ii}] = pppNonSmooth(fun, x_0(:, ii), solveroptions);
							end
						elseif nargout3
							if isvarargfun
								[x_opt(:, ii), f_opt(1, ii), exitflag(1, ii)] = pppNonSmooth(fun, x_0(:, ii), solveroptions, varargin{:});
							else
								[x_opt(:, ii), f_opt(1, ii), exitflag(1, ii)] = pppNonSmooth(fun, x_0(:, ii), solveroptions);
							end
						else
							if isvarargfun
								[x_opt(:, ii), f_opt(1, ii)] = pppNonSmooth(fun, x_0(:, ii), solveroptions, varargin{:});
							else
								[x_opt(:, ii), f_opt(1, ii)] = pppNonSmooth(fun, x_0(:, ii), solveroptions);
							end
						end
					else
						if nargout4
							if isvarargfun
								[x_opt(:, ii), f_opt(1, ii), exitflag(1, ii), output{1, ii}] = pppBox(fun, x_0(:, ii), solveroptions, A, b, varargin{:});
							else
								[x_opt(:, ii), f_opt(1, ii), exitflag(1, ii), output{1, ii}] = pppBox(fun, x_0(:, ii), solveroptions, A, b);
							end
						elseif nargout3
							if isvarargfun
								[x_opt(:, ii), f_opt(1, ii), exitflag(1, ii)] = pppBox(fun, x_0(:, ii), solveroptions, A, b, varargin{:});
							else
								[x_opt(:, ii), f_opt(1, ii), exitflag(1, ii)] = pppBox(fun, x_0(:, ii), solveroptions, A, b);
							end
						else
							if isvarargfun
								[x_opt(:, ii), f_opt(1, ii)] = pppBox(fun, x_0(:, ii), solveroptions, A, b, varargin{:});
							else
								[x_opt(:, ii), f_opt(1, ii)] = pppBox(fun, x_0(:, ii), solveroptions, A, b);
							end
						end
					end
				catch e
					if strcmpi(e.identifier, 'EMLRT:runTime:UserInterrupt')
						x_opt(:, ii) = NaN(size(x_0, 1), 1);
						f_opt(1, ii) = NaN;
						exitflag(1, ii) = -5;
						if nargout4
							output{1, ii} = struct(...
								'iterations',		0,...
								'funcCount',		1,...
								'firstorderopt',	0,...
								'message',			'',...
								'time',				0,...
								'trace',			struct()...
							);
						end
						break;
					else
						rethrow(e);
					end
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
	if matlab.Version.CURRENT >= matlab.Version.R2015A
		[fval, idx] = min(f_opt, [], 2, 'omitnan');
	else
		[fval, idx] = min(f_opt, [], 2);
	end
	if isempty(idx)
		error('optimization:solver:pppbox:solution', 'No solution could be found for any initial value.');
	end
	x = x_opt(:, idx);
	if nargout3
		exitflag = exitflag(1, idx);
	end
	if nargout4
		output = output{1, idx};
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
			[f, grad] = Jfun(x);
			[~, fvalidx] = min(f, [], 1, 'omitnan');
			grad = grad(:, fvalidx);
		else
			if nargout(Jfun) == -1
				try
					[f, grad] = Jfun(x);
					hasgradient = true;
					[~, fvalidx] = min(f, [], 1, 'omitnan');
					grad = grad(:, fvalidx);
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
				grad = zeros(size(x, 1), objective_number);
				step = eps;
				if nargin >= 11 && nargin(Jfun) ~= 1
					f = Jfun(x, varargin{:});
					parfor ii = 1:size(x, 1)
						grad(ii, :) = (Jfun(x + step*(1:size(x, 1) == ii)', varargin{:}) - f)/step; %#ok<PFBNS> fun is a function handle
					end
				else
					f = Jfun(x);
					parfor ii = 1:size(x, 1)
						grad(ii, :) = (Jfun(x + step*(1:size(x, 1) == ii)') - f)/step;
					end
				end
				[~, fvalidx] = min(f, [], 1, 'omitnan');
				grad = grad(:, fvalidx);
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
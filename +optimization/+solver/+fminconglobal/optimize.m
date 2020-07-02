function [x, fval, exitflag, output, lambda, grad, hessian] = optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin)
	%OPTIMIZE call global search algorithm with fmincon to solve optimization problem
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
		error('optimization:solver:fminconglobal:input', 'Objective function must not be empty.');
	end
	if ischar(fun)
		fun = str2func(fun);
	end
	if ~isfunctionhandle(fun)
		error('optimization:solver:fminconglobal:input', 'Objective function must be a function handle.');
	end
	if isempty(x_0)
		error('optimization:solver:fminconglobal:input', 'Initial point must not be empty.');
	end
	if isa(x_0, 'AbstractStartPointSet')
		x_0 = x_0.list().';
	end
	if ~isreal(x_0) || any(isinf(x_0(:))) || any(isnan(x_0(:)))
		error('optimization:solver:fminconglobal:input', 'Initial point must be a finite real value.');
	end
	if ~ismatrix(x_0)
		error('optimization:solver:fminconglobal:input', 'Initial point must be a column vector or a matrix of column vectors.');
	end
	%if size(x_0, 2) > 1
	%	error('optimization:solver:fminconglobal:input', 'Initial point must be a column vector.');
	%end
	usedefaultoption = false;
	defaultsolver = optimization.solver.Optimizer.FMINCONGLOBAL;
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
	isemptynonlcon = false;
	if nargin <= 8
		nonlcon = [];
		isemptynonlcon = true;
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
		error('optimization:solver:fminconglobal:input', 'Inequality matrix must be real.');
	end
	if ~ismatrix(A)
		error('optimization:solver:fminconglobal:input', 'Inequality matrix must be a matrix.');
	end
	if size(A, 2) ~= size(x_0, 1)
		error('optimization:solver:fminconglobal:input', 'Inequality matrix must have %d columns, not %d.', size(x_0, 1), size(A, 2));
	end
	if ~isreal(b) || any(isnan(b(:)))
		error('optimization:solver:fminconglobal:input', 'Inequality bounds must be real.');
	end
	if ~ismatrix(b)
		error('optimization:solver:fminconglobal:input', 'Inequality bounds must be a vector.');
	end
	if size(b, 1) ~= size(A, 1)
		error('optimization:solver:fminconglobal:input', 'Inequality bounds must have %d rows, not %d.', size(A, 1), size(b, 1));
	end
	if ~isreal(Aeq) || any(isnan(Aeq(:)))
		error('optimization:solver:fminconglobal:input', 'Equality matrix must be real.');
	end
	if ~ismatrix(Aeq)
		error('optimization:solver:fminconglobal:input', 'Equality matrix must be a matrix.');
	end
	if size(Aeq, 2) ~= size(x_0, 1)
		error('optimization:solver:fminconglobal:input', 'Equality matrix must have %d columns, not %d.', size(x_0, 1), size(Aeq, 2));
	end
	if ~isreal(beq) || any(isnan(beq(:)))
		error('optimization:solver:fminconglobal:input', 'Equality bounds must be real.');
	end
	if ~ismatrix(beq)
		error('optimization:solver:fminconglobal:input', 'Equality bounds must be a vector.');
	end
	if size(beq, 1) ~= size(Aeq, 1)
		error('optimization:solver:fminconglobal:input', 'Equality bounds must have %d rows, not %d.', size(Aeq, 1), size(beq, 1));
	end
	if ~isreal(lb) || any(isnan(lb(:)))
		error('optimization:solver:fminconglobal:input', 'Lower bounds must be real.');
	end
	if ~ismatrix(lb)
		error('optimization:solver:fminconglobal:input', 'Lower bounds must be a vector.');
	end
	if size(lb, 1) ~= size(x_0, 1)
		error('optimization:solver:fminconglobal:input', 'Lower bounds must have %d rows, not %d.', size(x_0, 1), size(lb, 1));
	end
	if ~isreal(ub) || any(isnan(ub(:)))
		error('optimization:solver:fminconglobal:input', 'Upper bounds must be real.');
	end
	if ~ismatrix(ub)
		error('optimization:solver:fminconglobal:input', 'Upper bounds must be a vector.');
	end
	if size(ub, 1) ~= size(x_0, 1)
		error('optimization:solver:fminconglobal:input', 'Upper bounds must have %d rows, not %d.', size(x_0, 1), size(ub, 1));
	end
	if nargin >= 9
		if ~isempty(nonlcon)
			if ischar(nonlcon)
				nonlcon = str2func(nonlcon);
			end
			if ~isfunctionhandle(nonlcon)
				error('optimization:solver:fminconglobal:input', 'Constraint function must be a function handle.');
			end
		else
			isemptynonlcon = true;
		end
	else
		nonlcon = [];
		isemptynonlcon = true;
	end
	if ~isa(options, 'optimization.options.Options')
		if isstruct(options)
			options = optimization.options.OptionFactory.instance.options(optimization.solver.Optimizer.FMINCONGLOBAL, options);
		elseif isa(options, 'optim.options.SolverOptions')
			options = optimization.options.OptionFactory.instance.options(optimization.solver.Optimizer.FMINCONGLOBAL, options);
		else
			error('optimization:solver:fminconglobal:input', 'Options must be of type optimization.options.Options or optimoptions or optimset.');
		end
	end
	if ~isa(options, 'optimization.options.fminconglobal')
		options = optimization.options.OptionFactory.instance.options(optimization.solver.Optimizer.FMINCONGLOBAL, options);
	end
	function [f, grad, hessian] = objectivehessianvararg(x)
		[f, grad, hessian] = fun(x, varargin{:});
	end
	function [f, grad] = objectivegradientvararg(x)
		[f, grad] = fun(x, varargin{:});
	end
	needshessian = options.supportsHessian() && options.SpecifyObjectiveHessian && options.SpecifyConstraintHessian;
	hasobjectivehessian = false;
	hasconstrainthessian = false;
	if nargin >= 11
		if nargout(fun) >= 3
			hasobjectivehessian = true;
			if nargin(fun) == 1
				Jfun = fun;
			else
				Jfun = @objectivehessianvararg;
			end
		elseif nargout(fun) >= 2
			if nargin(fun) == 1
				Jfun = fun;
			else
				Jfun = @objectivegradientvararg;
			end
		else
			if nargout(fun) == -1
				try
					try
						if nargin(fun) == 1
							[~, tempgrad, temphess] = fun(x_0(:, 1)); %#ok<ASGLU> call with three output arguments to check if the third one is present and catch error if not
							Jfun = fun;
							hasobjectivehessian = true;
						else
							[~, tempgrad, temphess] = fun(x_0(:, 1), varargin{:}); %#ok<ASGLU> call with three output arguments to check if the third one is present and catch error if not
							Jfun = @objectivehessianvararg;
							hasobjectivehessian = true;
						end
					catch e
						if strcmpi(e.identifier, 'MATLAB:maxlhs') || strcmpi(e.identifier, 'MATLAB:TooManyOutputs') || strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
							if usedefaultoption
								options.SpecifyObjectiveHessian = false;
							end
							if nargin(fun) == 1
								[~, tempgrad] = fun(x_0(:, 1)); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
								Jfun = fun;
							else
								[~, tempgrad] = fun(x_0(:, 1), varargin{:}); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
								Jfun = @objectivegradientvararg;
							end
						else
							rethrow(e);
						end
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
							options.SpecifyObjectiveHessian = false;
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
					options.SpecifyObjectiveHessian = false;
				end
			end
		end
		if ~isemptynonlcon
			if nargout(nonlcon) >= 6
				if nargin(nonlcon) == 1
					cfun = nonlcon;
				else
					cfun = constr_dispatcher('constrhess', nonlcon, varargin{:});
				end
				hasconstrainthessian = true;
			elseif nargout(nonlcon) >= 4
				if nargin(nonlcon) == 1
					cfun = nonlcon;
				else
					cfun = constr_dispatcher('constrgrad', nonlcon, varargin{:});
				end
			else
				if nargout(nonlcon) == -1
					try
						if needshessian
							try
								if nargin(nonlcon) == 1
									[~, ~, tempgradc, tempgradceq, temphessc, temphessceq] = callnonlcon(nonlcon, x_0(:, 1)); %#ok<ASGLU> call with six output arguments to check if the fifth and sixth one are present and catch error if not
									cfun = constr_dispatcher('constrhess', nonlcon);
								else
									[~, ~, tempgradc, tempgradceq, temphessc, temphessceq] = callnonlcon(nonlcon, x_0(:, 1), varargin{:}); %#ok<ASGLU> call with six output arguments to check if the fifth and sixth one are present and catch error if not
									cfun = constr_dispatcher('constrhess', nonlcon, varargin{:});
								end
								hasconstrainthessian = true;
							catch e
								if ~strcmpi(e.identifier, 'MATLAB:maxlhs') && ~strcmpi(e.identifier, 'MATLAB:TooManyOutputs') && ~strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
									rethrow(e);
								else
									if nargin(nonlcon) == 1
										[~, ~, tempgradc, tempgradceq] = callnonlcon(nonlcon, x_0(:, 1)); %#ok<ASGLU> call with four output arguments to check if the third and fourth one are present and catch error if not
										cfun = constr_dispatcher('constrgrad', nonlcon);
									else
										[~, ~, tempgradc, tempgradceq] = callnonlcon(nonlcon, x_0(:, 1), varargin{:}); %#ok<ASGLU> call with four output arguments to check if the third and fourth one are present and catch error if not
										cfun = constr_dispatcher('constrgrad', nonlcon, varargin{:});
									end
									if usedefaultoption
										options.SpecifyConstraintHessian = false;
									end
								end
							end
						else
							if nargin(nonlcon) == 1
								[~, ~, tempgradc, tempgradceq] = callnonlcon(nonlcon, x_0(:, 1)); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
								cfun = constr_dispatcher('constrgrad', nonlcon);
							else
								[~, ~, tempgradc, tempgradceq] = callnonlcon(nonlcon, x_0(:, 1), varargin{:}); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
								cfun = constr_dispatcher('constrgrad', nonlcon, varargin{:});
							end
						end
					catch e
						if ~strcmpi(e.identifier, 'MATLAB:maxlhs') && ~strcmpi(e.identifier, 'MATLAB:TooManyOutputs') && ~strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
							rethrow(e);
						else
							if nargin(nonlcon) == 1
								cfun = nonlcon;
							else
								cfun = constr_dispatcher('constr', nonlcon, varargin{:});
							end
							if usedefaultoption
								options.SpecifyConstraintGradient = false;
								options.SpecifyConstraintHessian = false;
							end
						end
					end
				else
					if nargin(nonlcon) == 1
						cfun = nonlcon;
					else
						cfun = constr_dispatcher('constr', nonlcon, varargin{:});
					end
					if usedefaultoption
						options.SpecifyConstraintGradient = false;
						options.SpecifyConstraintHessian = false;
					end
				end
			end
		else
			cfun = nonlcon;
			hasconstrainthessian = true;
		end
	else
		Jfun = fun;
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
							hasobjectivehessian = true;
						catch e
							if strcmpi(e.identifier, 'MATLAB:maxlhs') || strcmpi(e.identifier, 'MATLAB:TooManyOutputs') || strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
								if usedefaultoption
									options.SpecifyObjectiveHessian = false;
								end
								if nargin(fun) == 1
									[~, tempgrad] = fun(x_0(:, 1)); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
								else
									[~, tempgrad] = fun(x_0(:, 1), varargin{:}); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
								end
							else
								rethrow(e);
							end
						end
					else
						if nargin(fun) == 1
							[~, tempgrad] = fun(x_0(:, 1)); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
						else
							[~, tempgrad] = fun(x_0(:, 1), varargin{:}); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
						end
					end
				catch e
					if strcmpi(e.identifier, 'MATLAB:maxlhs') || strcmpi(e.identifier, 'MATLAB:TooManyOutputs') || strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
						if usedefaultoption
							options.SpecifyObjectiveGradient = false;
							options.SpecifyObjectiveHessian = false;
						end
					else
						rethrow(e);
					end
				end
			else
				if usedefaultoption
					options.SpecifyObjectiveGradient = false;
					options.SpecifyObjectiveHessian = false;
				end
			end
		else
			hasobjectivehessian = true;
		end
		cfun = nonlcon;
		if ~isemptynonlcon && nargout(nonlcon) < 6
			if nargout(nonlcon) == -1
				try
					if needshessian
						try
							if nargin(nonlcon) == 1
								[~, ~, tempgradc, tempgradceq, temphessc, temphessceq] = callnonlcon(nonlcon, x_0(:, 1)); %#ok<ASGLU> call with six output arguments to check if the fifth and sixth one are present and catch error if not
							else
								[~, ~, tempgradc, tempgradceq, temphessc, temphessceq] = callnonlcon(nonlcon, x_0(:, 1), varargin{:}); %#ok<ASGLU> call with six output arguments to check if the fifth and sixth one are present and catch error if not
							end
							hasconstrainthessian = true;
						catch e
							if strcmpi(e.identifier, 'MATLAB:maxlhs') || strcmpi(e.identifier, 'MATLAB:TooManyOutputs') || strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
								if usedefaultoption
									options.SpecifyConstraintHessian = false;
								end
								if nargin(nonlcon) == 1
									[~, ~, tempgradc, tempgradceq] = callnonlcon(nonlcon, x_0(:, 1)); %#ok<ASGLU> call with four output arguments to check if the third and fourth one are present and catch error if not
								else
									[~, ~, tempgradc, tempgradceq] = callnonlcon(nonlcon, x_0(:, 1), varargin{:}); %#ok<ASGLU> call with four output arguments to check if the third and fourth one are present and catch error if not
								end
							else
								rethrow(e);
							end
						end
					else
						if nargin(nonlcon) == 1
							[~, ~, tempgradc, tempgradceq] = callnonlcon(nonlcon, x_0(:, 1)); %#ok<ASGLU> call with four output arguments to check if the third and fourth one are present and catch error if not
						else
							[~, ~, tempgradc, tempgradceq] = callnonlcon(nonlcon, x_0(:, 1), varargin{:}); %#ok<ASGLU> call with four output arguments to check if the third and fourth one are present and catch error if not
						end
					end
				catch e
					if strcmpi(e.identifier, 'MATLAB:maxlhs') || strcmpi(e.identifier, 'MATLAB:TooManyOutputs') || strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
						if usedefaultoption
							options.SpecifyConstraintGradient = false;
							options.SpecifyConstraintHessian = false;
						end
					else
						rethrow(e);
					end
				end
			else
				if usedefaultoption
					if nargout(nonlcon) < 4
						options.SpecifyConstraintGradient = false;
					end
					options.SpecifyConstraintHessian = false;
				end
			end
		elseif ~isemptynonlcon && nargout(nonlcon) >= 6
			hasconstrainthessian = true;
		elseif isemptynonlcon
			hasconstrainthessian = true;
		end
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
	if needshessian && (~hasobjectivehessian || ~hasconstrainthessian)
		error('optimization:solver:fminconglobal:input', 'User supplied objective and constraint function does not return hessian, but calculation of hessian matrix is requested.');
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
	if options.SpecifyObjectiveHessian && hasobjectivehessian && options.SpecifyObjectiveHessian && hasconstrainthessian
		if matlab.Version.CURRENT >= matlab.Version.R2016A
			if strcmpi(solveroptions.Algorithm, 'interior-point')
				solveroptions.Hessian = 'user-supplied';
				solveroptions.HessianFcn = hessian_dispatcher(Jfun, nonlcon, varargin{:});
			elseif strcmpi(solveroptions.Algorithm, 'trust-region-reflective')
				solveroptions.Hessian = 'user-supplied';
			end
		else
			if strcmpi(solveroptions.Algorithm, 'interior-point')
				solveroptions.Hessian = 'user-supplied';
				solveroptions.HessFcn = hessian_dispatcher(Jfun, nonlcon, varargin{:});
			elseif strcmpi(solveroptions.Algorithm, 'trust-region-reflective')
				solveroptions.Hessian = 'user-supplied';
			end
		end
	end
	solvertime = tic;
	retry = 0;
	retryiterations = 0;
	retryfunevals = 0;
	problem = createOptimProblem('fmincon',...
		'objective',	Jfun,...
		'x0',			x_0(:, 1),...
		'Aineq',		A,...
		'bineq',		b,...
		'Aeq',			Aeq,...
		'beq',			beq,...
		'lb',			lb,...
		'ub',			ub,...
		'nonlcon',		cfun,...
		'options',		solveroptions...
	);
	gs = GlobalSearch(...
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
		try
			if nargout >= 4
				[x, fval, exitflag, output] = run(gs, problem);
			elseif nargout >= 3
				[x, fval, exitflag] = run(gs, problem);
			elseif nargout >= 2
				[x, fval] = run(gs, problem);
			else
				x = run(gs, problem);
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
		if hasobjectivehessian && (isemptynonlcon || hasconstrainthessian)
			% TODO: lambda is filled with NaN because globalsearch does not return langrange multiplier information
			hessfun = hessian_dispatcher(Jfun, nonlcon, varargin{:});
			hessian = hessfun(x, lambda);
		else
			hessian = NaN(size(x_0, 1), size(x_0, 1));
		end
	end
end

function [c, ceq, gradc, gradceq, hessianc, hessianceq] = callnonlcon(nonlcon, x, varargin)
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
		if nargout >= 5
			[c, ceq, gradc, gradceq, hessianc, hessianceq] = nonlcon(x, varargin{:});
		elseif nargout >= 3
			[c, ceq, gradc, gradceq] = nonlcon(x, varargin{:});
		else
			[c, ceq] = nonlcon(x, varargin{:});
		end
	else
		if nargout >= 5
			[c, ceq, gradc, gradceq, hessianc, hessianceq] = nonlcon(x);
		elseif nargout >= 3
			[c, ceq, gradc, gradceq] = nonlcon(x);
		else
			[c, ceq] = nonlcon(x);
		end
	end
end

function [cfun] = constr_dispatcher(type, nonlcon, varargin)
	%CONSTR_DISPATCHER wrapper function to convert a constraint function that needs additional arguments to a constraint function with one argument
	%	Input:
	%		type:		type of function to return
	%		nonlcon:	nonlinear constraint function
	%		varargin:	additional input arguments for the objective function
	%	Output:
	%		cfun:		constraint function
	function [c, ceq] = constrfunvararg(x)
		[c, ceq] = nonlcon(x, varargin{:});
	end
	function [c, ceq, gradc, gradceq] = constrgradfunvararg(x)
		if nargout >= 3
			[c, ceq, gradc, gradceq] = nonlcon(x, varargin{:});
		else
			[c, ceq] = nonlcon(x, varargin{:});
		end
	end
	function [c, ceq, gradc, gradceq, hessianc, hessianceq] = constrhessfunvararg(x)
		if nargout >= 5
			[c, ceq, gradc, gradceq, hessianc, hessianceq] = nonlcon(x, varargin{:});
		elseif nargout >= 3
			[c, ceq, gradc, gradceq] = nonlcon(x, varargin{:});
		else
			[c, ceq] = nonlcon(x, varargin{:});
		end
	end
	if strcmpi(type, 'constr')
		cfun = @constrfunvararg;
	elseif strcmpi(type, 'constrgrad')
		cfun = @constrgradfunvararg;
	elseif strcmpi(type, 'constrhess')
		cfun = @constrhessfunvararg;
	else
		error('optimization:solver:fminconglobal:input', 'Undefined constraint function type.');
	end
end

function [hfun] = hessian_dispatcher(fun, nonlcon, varargin)
	function [H] = hessian_constr_wrapper(x, lambda)
		%HESSIAN_CONSTR_WRAPPER warpper function for calculation of hessian matrix with constraints
		%	Input:
		%		x:		optimization variable
		%		lambda:	structure with lagrange multipliers
		%	Output:
		%		H:		hessian matrix at x
		H_constraint = zeros(size(x, 1), size(x, 1));
		if nargin(fun) >= 2
			[~, ~, H_objective] = fun(x, varargin{:});
		else
			[~, ~, H_objective] = fun(x);
		end
		if ~isempty(lambda) || ~isempty(nonlcon)
			if nargin(nonlcon) >= 2
				[c, ceq, ~, ~, hessianc, hessianceq] = nonlcon(x, varargin{:});
			else
				[c, ceq, ~, ~, hessianc, hessianceq] = nonlcon(x);
			end
			if ~isempty(c) && ~isempty(lambda.ineqnonlin)
				if length(c) ~= length(lambda.ineqnonlin)
					error('optimization:solver:fminconglobal:hessian', 'Number of constraints does not match number of langrange multipliers for inequality constraints.');
				end
				if length(c) ~= size(hessianc, 3)
					error('optimization:solver:fminconglobal:hessian', 'Incorrect number of hessians for inequality constraints.');
				end
				for ii = 1:size(hessianc, 3) %#ok<FORPF> no parfor because of dependent iterations
					H_constraint = H_constraint + hessianc(:, :, ii)*lambda.ineqnonlin(ii, 1);
				end
			else
				if (~isempty(c) && isempty(lambda.ineqnonlin)) || (isempty(c) && ~isempty(lambda.ineqnonlin))
					error('optimization:solver:fminconglobal:hessian', 'Inequality constraints and lagrange multipliers must be specified.');
				end
			end
			if ~isempty(ceq) && ~isempty(lambda.eqnonlin)
				if length(ceq) ~= length(lambda.eqnonlin)
					error('optimization:solver:fminconglobal:hessian', 'Number of constraints does not match number of langrange multipliers for equality constraints.');
				end
				if length(ceq) ~= size(hessianceq, 3)
					error('optimization:solver:fminconglobal:hessian', 'Incorrect number of hessians for equality constraints.');
				end
				for jj = 1:size(hessianceq, 3) %#ok<FORPF> no parfor because of dependent iterations
					H_constraint = H_constraint + hessianceq(:, :, jj)*lambda.eqnonlin(jj, 1);
				end
			else
				if (~isempty(c) && isempty(lambda.ineqnonlin)) || (isempty(c) && ~isempty(lambda.ineqnonlin))
					error('optimization:solver:fminconglobal:hessian', 'Equality constraints and lagrange multipliers must be specified.');
				end
			end
		end
		H = H_objective + H_constraint;
	end
	function [H] = hessian_wrapper(x, ~)% should only be needed for interior-point algorithm, because objective function itself can already be used otherwise
		%HESSIAN_WRAPPER warpper function for calculation of hessian matrix without constraints
		%	Input:
		%		x:		optimization variable
		%		lambda:	structure with lagrange multipliers
		%	Output:
		%		H:		hessian matrix at x
		if nargin(fun) >= 2
			[~, ~, H] = fun(x, varargin{:});
		else
			[~, ~, H] = fun(x);
		end
	end
	if isempty(fun)
		error('optimization:solver:fminconglobal:hessian', 'Objective function must be specified.');
	end
	if isempty(nonlcon)
		hfun = @hessian_wrapper;
	else
		hfun = @hessian_constr_wrapper;
	end
end
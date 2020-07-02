function [x, fval, exitflag, output, lambda, grad, hessian] = optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin)
	%OPTIMIZE call ga to solve optimization problem
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
		error('optimization:solver:ga:input', 'Objective function must not be empty.');
	end
	if ischar(fun)
		fun = str2func(fun);
	end
	if ~isfunctionhandle(fun)
		error('optimization:solver:ga:input', 'Objective function must be a function handle.');
	end
	if isempty(x_0)
		error('optimization:solver:ga:input', 'Initial point must not be empty.');
	end
	if isa(x_0, 'AbstractStartPointSet')
		x_0 = x_0.list().';
	end
	if ~isreal(x_0) || any(isinf(x_0(:))) || any(isnan(x_0(:)))
		error('optimization:solver:ga:input', 'Initial point must be a finite real value.');
	end
	if ~ismatrix(x_0)
		error('optimization:solver:ga:input', 'Initial point must be a column vector or a matrix of column vectors.');
	end
	%if size(x_0, 2) > 1
	%	error('optimization:solver:ga:input', 'Initial point must be a column vector.');
	%end
	usedefaultoption = false;
	defaultsolver = optimization.solver.Optimizer.GA;
	if nargin >= 10 && isa(options, 'optimization.solver.Optimizer')
		defaultsolver = options;
		usedefaultoption = true;
	end
	if nargin <= 9 || isempty(options)
		options = optimization.options.OptionFactory.instance.options(defaultsolver,...
			'Algorithm',					'ga',...
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
		error('optimization:solver:ga:input', 'Inequality matrix must be real.');
	end
	if ~ismatrix(A)
		error('optimization:solver:ga:input', 'Inequality matrix must be a matrix.');
	end
	if size(A, 2) ~= size(x_0, 1)
		error('optimization:solver:ga:input', 'Inequality matrix must have %d columns, not %d.', size(x_0, 1), size(A, 2));
	end
	if ~isreal(b) || any(isnan(b(:)))
		error('optimization:solver:ga:input', 'Inequality bounds must be real.');
	end
	if ~ismatrix(b)
		error('optimization:solver:ga:input', 'Inequality bounds must be a vector.');
	end
	if size(b, 1) ~= size(A, 1)
		error('optimization:solver:ga:input', 'Inequality bounds must have %d rows, not %d.', size(A, 1), size(b, 1));
	end
	if ~isreal(Aeq) || any(isnan(Aeq(:)))
		error('optimization:solver:ga:input', 'Equality matrix must be real.');
	end
	if ~ismatrix(Aeq)
		error('optimization:solver:ga:input', 'Equality matrix must be a matrix.');
	end
	if size(Aeq, 2) ~= size(x_0, 1)
		error('optimization:solver:ga:input', 'Equality matrix must have %d columns, not %d.', size(x_0, 1), size(Aeq, 2));
	end
	if ~isreal(beq) || any(isnan(beq(:)))
		error('optimization:solver:ga:input', 'Equality bounds must be real.');
	end
	if ~ismatrix(beq)
		error('optimization:solver:ga:input', 'Equality bounds must be a vector.');
	end
	if size(beq, 1) ~= size(Aeq, 1)
		error('optimization:solver:ga:input', 'Equality bounds must have %d rows, not %d.', size(Aeq, 1), size(beq, 1));
	end
	if ~isreal(lb) || any(isnan(lb(:)))
		error('optimization:solver:ga:input', 'Lower bounds must be real.');
	end
	if ~ismatrix(lb)
		error('optimization:solver:ga:input', 'Lower bounds must be a vector.');
	end
	if size(lb, 1) ~= size(x_0, 1)
		error('optimization:solver:ga:input', 'Lower bounds must have %d rows, not %d.', size(x_0, 1), size(lb, 1));
	end
	if ~isreal(ub) || any(isnan(ub(:)))
		error('optimization:solver:ga:input', 'Upper bounds must be real.');
	end
	if ~ismatrix(ub)
		error('optimization:solver:ga:input', 'Upper bounds must be a vector.');
	end
	if size(ub, 1) ~= size(x_0, 1)
		error('optimization:solver:ga:input', 'Upper bounds must have %d rows, not %d.', size(x_0, 1), size(ub, 1));
	end
	if nargin >= 9
		if ~isempty(nonlcon)
			if ischar(nonlcon)
				nonlcon = str2func(nonlcon);
			end
			if ~isfunctionhandle(nonlcon)
				error('optimization:solver:ga:input', 'Constraint function must be a function handle.');
			end
		end
	else
		nonlcon = [];
	end
	if ~isa(options, 'optimization.options.Options')
		if isstruct(options)
			options = optimization.options.OptionFactory.instance.options(optimization.solver.Optimizer.GA, options);
		elseif isa(options, 'optim.options.SolverOptions')
			options = optimization.options.OptionFactory.instance.options(optimization.solver.Optimizer.GA, options);
		else
			error('optimization:solver:ga:input', 'Options must be of type optimization.options.Options or optimoptions or optimset.');
		end
	end
	if ~isa(options, 'optimization.options.ga')
		options = optimization.options.OptionFactory.instance.options(optimization.solver.Optimizer.GA, options);
	end
	if nargin >= 11
		if nargout(fun) >= 2
			if nargin(fun) == 1
				Jfun = objective_dispatcher('objgrad', fun);
			else
				Jfun = objective_dispatcher('objgradvararg', fun, varargin{:});
			end
			fval = Jfun(x_0(:, 1));
		else
			if nargout(fun) == -1
				try
					if nargin(fun) == 1
						[fval, gradJ] = fun(x_0(:, 1)); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
						Jfun = objective_dispatcher('objgrad', fun);
					else
						[fval, gradJ] = fun(x_0(:, 1), varargin{:}); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
						Jfun = objective_dispatcher('objgradvararg', fun, varargin{:});
					end
				catch e
					if ~strcmpi(e.identifier, 'MATLAB:maxlhs') && ~strcmpi(e.identifier, 'MATLAB:TooManyOutputs') && ~strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
						rethrow(e);
					else
						if nargin(fun) == 1
							Jfun = objective_dispatcher('obj', fun);
						else
							Jfun = objective_dispatcher('objvararg', fun, varargin{:});
						end
						fval = Jfun(x_0(:, 1));
						if usedefaultoption
							options.SpecifyObjectiveGradient = false;
						end
					end
				end
			else
				if nargin(fun) == 1
					Jfun = objective_dispatcher('obj', fun);
				else
					Jfun = objective_dispatcher('objvararg', fun, varargin{:});
				end
				fval = Jfun(x_0(:, 1));
				if usedefaultoption
					options.SpecifyObjectiveGradient = false;
				end
			end
		end
		if ~isempty(nonlcon)
			if nargout(nonlcon) >= 3
				if nargin(nonlcon) == 1
					cfun = constraint_dispatcher('constrgrad', nonlcon);
				else
					cfun = constraint_dispatcher('constrgradvararg', nonlcon, varargin{:});
				end
				[c, ceq] = cfun(x_0(:, 1));
			else
				if nargout(nonlcon) == -1
					try
						if nargin(nonlcon) == 1
							[c, ceq, gradc, gradceq] = callnonlcon(nonlcon, x_0(:, 1)); %#ok<ASGLU> call with four output arguments to check if the fourth one is present and catch error if not
							cfun = constraint_dispatcher('constrgrad', nonlcon);
						else
							[c, ceq, gradc, gradceq] = callnonlcon(nonlcon, x_0(:, 1), varargin{:}); %#ok<ASGLU> call with four output arguments to check if the fourth one is present and catch error if not
							cfun = constraint_dispatcher('constrgradvararg', nonlcon, varargin{:});
						end
					catch e
						if ~strcmpi(e.identifier, 'MATLAB:maxlhs') && ~strcmpi(e.identifier, 'MATLAB:TooManyOutputs') && ~strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
							rethrow(e);
						else
							if nargin(nonlcon) == 1
								cfun = constraint_dispatcher('constr', nonlcon);
							else
								cfun = constraint_dispatcher('constrvararg', nonlcon, varargin{:});
							end
							[c, ceq] = cfun(x_0(:, 1));
							if usedefaultoption
								options.SpecifyConstraintGradient = false;
							end
						end
					end
				else
					if nargin(nonlcon) == 1
						cfun = constraint_dispatcher('constr', nonlcon);
					else
						cfun = constraint_dispatcher('constrvararg', nonlcon, varargin{:});
					end
					[c, ceq] = cfun(x_0(:, 1));
					if usedefaultoption
						options.SpecifyConstraintGradient = false;
					end
				end
			end
		else
			cfun = constraint_dispatcher('constrgrad', nonlcon);
			c = [];
			ceq = [];
		end
	else
		if nargout(fun) >= 2
			Jfun = objective_dispatcher('objgrad', fun);
			fval = Jfun(x_0(:, 1));
		else
			if nargout(fun) == -1
				try
					[fval, gradJ] = fun(x_0(:, 1)); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
					Jfun = objective_dispatcher('objgrad', fun);
				catch e
					if ~strcmpi(e.identifier, 'MATLAB:maxlhs') && ~strcmpi(e.identifier, 'MATLAB:TooManyOutputs') && ~strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
						rethrow(e);
					else
						Jfun = objective_dispatcher('obj', fun);
						fval = Jfun(x_0(:, 1));
					end
					if usedefaultoption
						options.SpecifyObjectiveGradient = false;
					end
				end
			else
				Jfun = objective_dispatcher('obj', fun);
				fval = Jfun(x_0(:, 1));
				if usedefaultoption
					options.SpecifyObjectiveGradient = false;
				end
			end
		end
		if ~isempty(nonlcon)
			if nargout(nonlcon) >= 3
				cfun = constraint_dispatcher('constrgrad', nonlcon);
				[c, ceq] = cfun(x_0(:, 1));
			else
				if nargout(nonlcon) == -1
					try
						[c, ceq, gradc, gradceq] = callnonlcon(nonlcon, x_0(:, 1)); %#ok<ASGLU> call with four output arguments to check if the fourth one is present and catch error if not
						cfun = constraint_dispatcher('constrgrad', nonlcon);
					catch e
						if ~strcmpi(e.identifier, 'MATLAB:maxlhs') && ~strcmpi(e.identifier, 'MATLAB:TooManyOutputs') && ~strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
							rethrow(e);
						else
							cfun = constraint_dispatcher('constr', nonlcon);
							[c, ceq] = cfun(x_0(:, 1));
						end
						if usedefaultoption
							options.SpecifyConstraintGradient = false;
						end
					end
				else
					cfun = constraint_dispatcher('constr', nonlcon);
					[c, ceq] = cfun(x_0(:, 1));
					if usedefaultoption
						options.SpecifyConstraintGradient = false;
					end
				end
			end
		else
			cfun = constraint_dispatcher('constr', nonlcon);
			c = [];
			ceq = [];
		end
	end
	multiobjective = strcmpi(options.Algorithm, 'gamulti');
	if usedefaultoption
		if ~isscalar(fval)
			options.Algorithm = 'gamulti';
			multiobjective = true;
		end
	end
	if ~multiobjective && ~isscalar(fval)
		error('optimization:solver:ga:input', 'Objective function must be scalar for ga Algorithm ''ga''.');
	end
	objective_number = size(fval, 1);
	options.NumberVariables = size(x_0, 1);
	options.NumberConstraintsInequality = size(A, 1) + size(c, 1);
	options.NumberConstraintsEquality = size(Aeq, 1) + size(ceq, 1);
	options.NumberConstraintsBounds = sum(~isinf(lb)) + sum(~isinf(ub));
	solveroptions = options.getpreferred();
	if options.NumberConstraintsInequality + options.NumberConstraintsEquality + options.NumberConstraintsBounds > 0
		if isfield(solveroptions, 'MutationFcn') && ((iscell(solveroptions.MutationFcn) && isequal(solveroptions.MutationFcn{1}, @mutationgaussian)) || isfunctionhandle(solveroptions.MutationFcn) && isequal(solveroptions.MutationFcn, @mutationgaussian))
			if iscell(solveroptions.MutationFcn)
				if length(solveroptions.MutationFcn) > 1
					solveroptions.MutationFcn = [{@mutationadaptfeasible}, solveroptions.MutationFcn(2:end)];
				else
					solveroptions.MutationFcn = {@mutationadaptfeasible};
				end
			else
				solveroptions.MutationFcn = @mutationadaptfeasible;
			end
		end
	end
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
		if multiobjective
			errorretry = 1;
			maxerrorretry = 5;
			x = NaN(size(x_0, 1), 1);
			while errorretry <= maxerrorretry
				try
					if nargout >= 4
						[x, fval, exitflag, output] = gamultiobj(Jfun, size(x_0, 1), A, b, Aeq, beq, lb, ub, cfun, solveroptions);
						break;
					elseif nargout >= 3
						[x, fval, exitflag] = gamultiobj(Jfun, size(x_0, 1), A, b, Aeq, beq, lb, ub, cfun, solveroptions);
						break;
					elseif nargout >= 2
						[x, fval] = gamultiobj(Jfun, size(x_0, 1), A, b, Aeq, beq, lb, ub, cfun, solveroptions);
						break;
					else
						x = gamultiobj(Jfun, size(x_0, 1), A, b, Aeq, beq, lb, ub, cfun, solveroptions);
						break;
					end
				catch e
					% TODO: gamultiobj seems to be broken somehow, because in matlabroot/toolbox/globaloptim/globaloptim/crossoverscattered.m it sets parent to zero in some/all? cases and tries to index with this variable which breaks the optimization run
					if strcmpi(e.identifier, 'MATLAB:badsubscript') && isstruct(e.stack) && ~isempty(e.stack) && strcmpi(e.stack(1).name, 'crossoverscattered') && (e.stack(1).line == 45 || e.stack(1).line == 43)
						if errorretry == maxerrorretry
							rethrow(e);
						else
							warning(e.identifier, e.message);
							errorretry = errorretry + 1;
						end
					else
						rethrow(e);
					end
				end
			end
			if all(isnan(x(:)))
				error('optimization:solver:ga:input', 'gamultiobj crashed on the problem, try to find out why.');
			end
			[fval, foptidx] = min(min(fval, [], 2), [], 1);
			x = x(:, foptidx);
		else
			try
				if nargout >= 4
					[x, fval, exitflag, output] = ga(Jfun, size(x_0, 1), A, b, Aeq, beq, lb, ub, cfun, [], solveroptions);
				elseif nargout >= 3
					[x, fval, exitflag] = ga(Jfun, size(x_0, 1), A, b, Aeq, beq, lb, ub, cfun, [], solveroptions);
				elseif nargout >= 2
					[x, fval] = ga(Jfun, size(x_0, 1), A, b, Aeq, beq, lb, ub, cfun, [], solveroptions);
				else
					x = ga(Jfun, size(x_0, 1), A, b, Aeq, beq, lb, ub, cfun, [], solveroptions);
				end
			catch e
				rethrow(e);
			end
		end
		x = x';
		if nargout >= 4
			retryiterations = retryiterations + output.generations;
			retryfunevals = retryfunevals + output.funccount;
			alloutputs{retry + 1, :} = options.formatOutput(exitflag, toc(solvertimes), x, fval, numel(x_0(:, 1)), output.generations, output.funccount, retry, output);
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
			[f, grad] = Jfun(x);
			f = f.';
			if matlab.Version.CURRENT >= matlab.Version.R2015A
				[~, fvalidx] = min(f, [], 1, 'omitnan');
			else
				[~, fvalidx] = min(f, [], 1);
			end
			grad = grad(:, fvalidx);
		else
			if nargout(Jfun) == -1
				try
					[f, grad] = Jfun(x);
					f = f.';
					if matlab.Version.CURRENT >= matlab.Version.R2015A
						[~, fvalidx] = min(f, [], 1, 'omitnan');
					else
						[~, fvalidx] = min(f, [], 1);
					end
					grad = grad(:, fvalidx);
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
				grad = zeros(size(x, 1), objective_number);
				step = eps;
				if nargin >= 11 && nargin(Jfun) ~= 1
					f = Jfun(x, varargin{:});
					f = f.';
					parfor ii = 1:size(x, 1)
						grad(ii, :) = (Jfun(x + step*(1:size(x, 1) == ii)', varargin{:}).' - f)/step; %#ok<PFBNS> fun is a function handle
					end
				else
					f = Jfun(x).';
					parfor ii = 1:size(x, 1)
						grad(ii, :) = (Jfun(x + step*(1:size(x, 1) == ii)').' - f)/step; %#ok<PFBNS> fun is a function handle
					end
				end
				if matlab.Version.CURRENT >= matlab.Version.R2015A
					[~, fvalidx] = min(f, [], 1, 'omitnan');
				else
					[~, fvalidx] = min(f, [], 1);
				end
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

function [J] = objective_dispatcher(type, fun, varargin)
	%OBJECTIVE_DISPATCHER wrapper function for ga to convert the optimization variable to a column vector in case optimization variable is a row vector and pass varargin
	%	Input:
	%		type:		type of function to return
	%		fun:		objective function handle
	%		varargin:	additional input arguments for the objective function
	%	Output:
	%		J:			objective function handle
	function [J, gradJ] = objectivegrad(x)
		if ~iscolumn(x)
			x = x';
		end
		if nargout >= 2
			[J, gradJ] = fun(x);
		else
			J = fun(x);
		end
		J = J.';
	end
	function [J, gradJ] = objectivegradvararg(x)
		if ~iscolumn(x)
			x = x';
		end
		if nargout >= 2
			[J, gradJ] = fun(x, varargin{:});
		else
			J = fun(x, varargin{:});
		end
		J = J.';
	end
	function [J] = objective(x)
		if ~iscolumn(x)
			x = x';
		end
		J = fun(x);
		J = J.';
	end
	function [J] = objectivevararg(x)
		if ~iscolumn(x)
			x = x';
		end
		J = fun(x, varargin{:});
		J = J.';
	end
	if isempty(fun)
		J = fun;
		return;
	end
	if strcmpi(type, 'obj')
		J = @objective;
	elseif strcmpi(type, 'objgrad')
		J = @objectivegrad;
	elseif strcmpi(type, 'objvararg')
		J = @objectivevararg;
	elseif strcmpi(type, 'objgradvararg')
		J = @objectivegradvararg;
	else
		error('optimization:solver:ga:input', 'Undefined objective function type.');
	end
end

function [c] = constraint_dispatcher(type, fun, varargin)
	%CONSTRAINT_DISPATCHER wrapper function for ga to convert the optimization variable to a column vector in case optimization variable is a row vector and pass varargin
	%	Input:
	%		type:		type of function to return
	%		fun:		constraint function handle
	%		varargin:	additional input arguments for the constraint function
	%	Output:
	%		c:			constraint function handle
	function [c, ceq, gradc, gradceq] = constraintgrad(x)
		if ~iscolumn(x)
			x = x';
		end
		if nargout >= 3
			[c, ceq, gradc, gradceq] = fun(x);
		else
			[c, ceq] = fun(x);
		end
	end
	function [c, ceq, gradc, gradceq] = constraintgradvararg(x)
		if ~iscolumn(x)
			x = x';
		end
		if nargout >= 3
			[c, ceq, gradc, gradceq] = fun(x, varargin{:});
		else
			[c, ceq] = fun(x, varargin{:});
		end
	end
	function [c, ceq] = constraint(x)
		if ~iscolumn(x)
			x = x';
		end
		[c, ceq] = fun(x);
	end
	function [c, ceq] = constraintvararg(x)
		if ~iscolumn(x)
			x = x';
		end
		[c, ceq] = fun(x, varargin{:});
	end
	if isempty(fun)
		c = fun;
		return;
	end
	if strcmpi(type, 'constr')
		c = @constraint;
	elseif strcmpi(type, 'constrgrad')
		c = @constraintgrad;
	elseif strcmpi(type, 'constrvararg')
		c = @constraintvararg;
	elseif strcmpi(type, 'constrgradvararg')
		c = @constraintgradvararg;
	else
		error('optimization:solver:ga:input', 'Undefined constraint function type.');
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
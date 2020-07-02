function [x, fval, exitflag, output, lambda, grad, hessian] = optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin)
	%OPTIMIZE call fminimax to solve optimization problem
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
		error('optimization:solver:fminimax:input', 'Objective function must not be empty.');
	end
	if ischar(fun)
		fun = str2func(fun);
	end
	if ~isfunctionhandle(fun)
		error('optimization:solver:fminimax:input', 'Objective function must be a function handle.');
	end
	if isempty(x_0)
		error('optimization:solver:fminimax:input', 'Initial point must not be empty.');
	end
	if isa(x_0, 'AbstractStartPointSet')
		x_0 = x_0.list().';
	end
	if ~isreal(x_0) || any(isinf(x_0(:))) || any(isnan(x_0(:)))
		error('optimization:solver:fminimax:input', 'Initial point must be a finite real value.');
	end
	if ~ismatrix(x_0)
		error('optimization:solver:fminimax:input', 'Initial point must be a column vector or a matrix of column vectors.');
	end
	%if size(x_0, 2) > 1
	%	error('optimization:solver:fminimax:input', 'Initial point must be a column vector.');
	%end
	usedefaultoption = false;
	defaultsolver = optimization.solver.Optimizer.FMINIMAX;
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
		error('optimization:solver:fminimax:input', 'Inequality matrix must be real.');
	end
	if ~ismatrix(A)
		error('optimization:solver:fminimax:input', 'Inequality matrix must be a matrix.');
	end
	if size(A, 2) ~= size(x_0, 1)
		error('optimization:solver:fminimax:input', 'Inequality matrix must have %d columns, not %d.', size(x_0, 1), size(A, 2));
	end
	if ~isreal(b) || any(isnan(b(:)))
		error('optimization:solver:fminimax:input', 'Inequality bounds must be real.');
	end
	if ~ismatrix(b)
		error('optimization:solver:fminimax:input', 'Inequality bounds must be a vector.');
	end
	if size(b, 1) ~= size(A, 1)
		error('optimization:solver:fminimax:input', 'Inequality bounds must have %d rows, not %d.', size(A, 1), size(b, 1));
	end
	if ~isreal(Aeq) || any(isnan(Aeq(:)))
		error('optimization:solver:fminimax:input', 'Equality matrix must be real.');
	end
	if ~ismatrix(Aeq)
		error('optimization:solver:fminimax:input', 'Equality matrix must be a matrix.');
	end
	if size(Aeq, 2) ~= size(x_0, 1)
		error('optimization:solver:fminimax:input', 'Equality matrix must have %d columns, not %d.', size(x_0, 1), size(Aeq, 2));
	end
	if ~isreal(beq) || any(isnan(beq(:)))
		error('optimization:solver:fminimax:input', 'Equality bounds must be real.');
	end
	if ~ismatrix(beq)
		error('optimization:solver:fminimax:input', 'Equality bounds must be a vector.');
	end
	if size(beq, 1) ~= size(Aeq, 1)
		error('optimization:solver:fminimax:input', 'Equality bounds must have %d rows, not %d.', size(Aeq, 1), size(beq, 1));
	end
	if ~isreal(lb) || any(isnan(lb(:)))
		error('optimization:solver:fminimax:input', 'Lower bounds must be real.');
	end
	if ~ismatrix(lb)
		error('optimization:solver:fminimax:input', 'Lower bounds must be a vector.');
	end
	if size(lb, 1) ~= size(x_0, 1)
		error('optimization:solver:fminimax:input', 'Lower bounds must have %d rows, not %d.', size(x_0, 1), size(lb, 1));
	end
	if ~isreal(ub) || any(isnan(ub(:)))
		error('optimization:solver:fminimax:input', 'Upper bounds must be real.');
	end
	if ~ismatrix(ub)
		error('optimization:solver:fminimax:input', 'Upper bounds must be a vector.');
	end
	if size(ub, 1) ~= size(x_0, 1)
		error('optimization:solver:fminimax:input', 'Upper bounds must have %d rows, not %d.', size(x_0, 1), size(ub, 1));
	end
	if nargin >= 9
		if ~isempty(nonlcon)
			if ischar(nonlcon)
				nonlcon = str2func(nonlcon);
			end
			if ~isfunctionhandle(nonlcon)
				error('optimization:solver:fminimax:input', 'Constraint function must be a function handle.');
			end
		end
	else
		nonlcon = [];
	end
	if ~isa(options, 'optimization.options.Options')
		if isstruct(options)
			options = optimization.options.OptionFactory.instance.options(optimization.solver.Optimizer.FMINIMAX, options);
		elseif isa(options, 'optim.options.SolverOptions')
			options = optimization.options.OptionFactory.instance.options(optimization.solver.Optimizer.FMINIMAX, options);
		else
			error('optimization:solver:fminimax:input', 'Options must be of type optimization.options.Options or optimoptions or optimset.');
		end
	end
	if ~isa(options, 'optimization.options.fminimax')
		options = optimization.options.OptionFactory.instance.options(optimization.solver.Optimizer.FMINIMAX, options);
	end
	function [f, grad] = objectivegradient(x, varargin) %#ok<VANUS> varargin is only defined for compatibility with a constraint function with additional arguments
		[f, grad] = fun(x);
	end
	try
		if ~isempty(nonlcon)
			if nargin >= 11
				if nargin(nonlcon) ~= 1
					if nargout(nonlcon) == -1
						try
							[c, ceq, tempgradc, tempgradceq] = callnonlcon(nonlcon, x_0(:, 1), varargin{:}); %#ok<ASGLU> call with four output arguments to check if the third and fourth are present and catch error if not
						catch e
							if ~strcmpi(e.identifier, 'MATLAB:maxlhs') && ~strcmpi(e.identifier, 'MATLAB:TooManyOutputs') && ~strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
								rethrow(e);
							else
								[c, ceq] = callnonlcon(nonlcon, x_0(:, 1), varargin{:});
								if usedefaultoption
									options.SpecifyConstraintGradient = false;
								end
							end
						end
					elseif nargout(nonlcon) < 4
						[c, ceq] = callnonlcon(nonlcon, x_0(:, 1), varargin{:});
						if usedefaultoption
							options.SpecifyConstraintGradient = false;
						end
					else
						[c, ceq] = callnonlcon(nonlcon, x_0(:, 1), varargin{:});
					end
				else
					[c, ceq] = callnonlcon(nonlcon, x_0(:, 1));
					if nargout(nonlcon) >= 3
						nonlcon = constr_dispatcher('constrgrad', nonlcon);
					else
						if nargout(nonlcon) == -1
							try
								if nargin(nonlcon) == 1
									[~, ~, tempgradc, tempgradceq] = callnonlcon(nonlcon, x_0(:, 1)); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
									nonlcon = constr_dispatcher('constrgrad', nonlcon);
								end
							catch e
								if ~strcmpi(e.identifier, 'MATLAB:maxlhs') && ~strcmpi(e.identifier, 'MATLAB:TooManyOutputs') && ~strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
									rethrow(e);
								else
									if nargin(nonlcon) == 1
										nonlcon = constr_dispatcher('constr', nonlcon);
									end
									if usedefaultoption
										options.SpecifyConstraintGradient = false;
									end
								end
							end
						else
							if nargin(nonlcon) == 1
								nonlcon = constr_dispatcher('constr', nonlcon);
							end
							if usedefaultoption
								options.SpecifyConstraintGradient = false;
							end
						end
					end
				end
				if nargout(fun) >= 2
					if nargin(fun) == 1
						Jfun = @objectivegradient;
						fval = fun(x_0(:, 1));
					else
						Jfun = fun;
						fval = fun(x_0(:, 1), varargin{:});
					end
				else
					if nargout(fun) == -1
						try
							if nargin(fun) == 1
								[fval, tempgrad] = fun(x_0(:, 1)); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
								Jfun = @objectivegradient;
							else
								[fval, tempgrad] = fun(x_0(:, 1), varargin{:}); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
								Jfun = fun;
							end
						catch e
							if strcmpi(e.identifier, 'MATLAB:maxlhs') || strcmpi(e.identifier, 'MATLAB:TooManyOutputs') || strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
								if nargin(fun) == 1
									Jfun = @(x, varargin) fun(x);
									fval = fun(x_0(:, 1));
								else
									Jfun = fun;
									fval = fun(x_0(:, 1), varargin{:});
								end
								if usedefaultoption
									options.SpecifyObjectiveGradient = false;
								end
							else
								rethrow(e);
							end
						end
					else
						if nargin(fun) == 1
							Jfun = @(x, varargin) fun(x);
							fval = fun(x_0(:, 1));
						else
							Jfun = fun;
							fval = fun(x_0(:, 1), varargin{:});
						end
						if usedefaultoption
							options.SpecifyObjectiveGradient = false;
						end
					end
				end
			else
				Jfun = fun;
				if nargout(fun) == -1
					try
						if nargin(fun) == 1
							[fval, tempgrad] = fun(x_0(:, 1)); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
						else
							[fval, tempgrad] = fun(x_0(:, 1), varargin{:}); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
						end
					catch e
						if strcmpi(e.identifier, 'MATLAB:maxlhs') || strcmpi(e.identifier, 'MATLAB:TooManyOutputs') || strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
							if nargin(fun) == 1
								fval = fun(x_0(:, 1));
							else
								fval = fun(x_0(:, 1), varargin{:});
							end
							if usedefaultoption
								options.SpecifyObjectiveGradient = false;
							end
						else
							rethrow(e);
						end
					end
				else
					if nargin(fun) == 1
						fval = fun(x_0(:, 1));
					else
						fval = fun(x_0(:, 1), varargin{:});
					end
					if nargout(fun) < 2 && usedefaultoption
						options.SpecifyObjectiveGradient = false;
					end
				end
				if nargout(nonlcon) == -1
					try
						if nargin(nonlcon) == 1
							[c, ceq, tempgradc, tempgradceq] = callnonlcon(nonlcon, x_0(:, 1)); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
						else
							[c, ceq, tempgradc, tempgradceq] = callnonlcon(nonlcon, x_0(:, 1), varargin{:}); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
						end
					catch e
						if strcmpi(e.identifier, 'MATLAB:maxlhs') || strcmpi(e.identifier, 'MATLAB:TooManyOutputs') || strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
							if nargin(nonlcon) ~= 1
								[c, ceq] = callnonlcon(nonlcon, x_0(:, 1), varargin{:});
							else
								[c, ceq] = callnonlcon(nonlcon, x_0(:, 1));
							end
							if usedefaultoption
								options.SpecifyConstraintGradient = false;
							end
						else
							rethrow(e);
						end
					end
				else
					if nargin(nonlcon) ~= 1
						[c, ceq] = callnonlcon(nonlcon, x_0(:, 1), varargin{:});
					else
						[c, ceq] = callnonlcon(nonlcon, x_0(:, 1));
					end
					if nargout(nonlcon) < 4 && usedefaultoption
						options.SpecifyConstraintGradient = false;
					end
				end
			end
		else
			if nargin >= 11
				if nargout(fun) == -1
					try
						if nargin(fun) == 1
							[fval, tempgrad] = fun(x_0(:, 1)); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
							Jfun = @objectivegradient;
						else
							[fval, tempgrad] = fun(x_0(:, 1), varargin{:}); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
							Jfun = fun;
						end
					catch e
						if strcmpi(e.identifier, 'MATLAB:maxlhs') || strcmpi(e.identifier, 'MATLAB:TooManyOutputs') || strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
							if nargin(fun) == 1
								Jfun = @(x, varargin) fun(x);
								fval = fun(x_0(:, 1));
							else
								Jfun = fun;
								fval = fun(x_0(:, 1), varargin{:});
							end
							if usedefaultoption
								options.SpecifyObjectiveGradient = false;
							end
						else
							rethrow(e);
						end
					end
				else
					if nargin(fun) == 1
						Jfun = @(x, varargin) fun(x);
						fval = fun(x_0(:, 1));
					else
						Jfun = fun;
						fval = fun(x_0(:, 1), varargin{:});
					end
					if usedefaultoption
						options.SpecifyObjectiveGradient = false;
					end
				end
			else
				Jfun = fun;
				if nargout(fun) == -1
					try
						[fval, tempgrad] = fun(x_0(:, 1)); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
					catch e
						if strcmpi(e.identifier, 'MATLAB:maxlhs') || strcmpi(e.identifier, 'MATLAB:TooManyOutputs') || strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
							fval = fun(x_0(:, 1), varargin{:});
							if usedefaultoption
								options.SpecifyObjectiveGradient = false;
							end
						else
							rethrow(e);
						end
					end
				else
					fval = fun(x_0(:, 1), varargin{:});
					if nargout(fun) < 2 && usedefaultoption
						options.SpecifyObjectiveGradient = false;
					end
				end
			end
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
	solveroptions = options.getpreferred();
	if isscalar(fval)
		if isfield(solveroptions, 'MeritFunction') || isprop(solveroptions, 'MeritFunction')
			solveroptions.MeritFunction = 'singleobj';
		end
	end
	objective_number = size(fval, 1);
	solvertime = tic;
	retry = 0;
	retryiterations = 0;
	retryfunevals = 0;
	maxTime = options.MaxTime;
	if isempty(maxTime) || isnan(maxTime) || maxTime <= 0
		maxTime = Inf;
	end
	tolCon = options.ConstraintTolerance;
	if isempty(tolCon) || isnan(tolCon) || tolCon <= 0
		tolCon = 0;
	end
	initialvalues = size(x_0, 2);
	useparallel = options.UseParallel;
	nargout3 = nargout >= 3;
	nargout4 = nargout >= 4;
	nargout5 = nargout >= 5;
	nargout6 = nargout >= 6;
	nargout7 = nargout >= 7;
	if nargout4
		alloutputs = cell(max(1, options.Retries), initialvalues);
	end
	while retry < max(1, options.Retries)
		x_opt = NaN(size(x_0));
		f_opt = NaN(1, initialvalues);
		infeas_opt = NaN(1, initialvalues);
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
		if nargout5 || useparallel
			lambda = cell(1, initialvalues);
		end
		if nargout6 || useparallel
			grad = cell(1, initialvalues);
		end
		if nargout7 || useparallel
			hessian = cell(1, initialvalues);
		end
		if useparallel
			parfor ii = 1:initialvalues
				try
					if nargout5
						[x_opt(:, ii), ~, f_opt(1, ii), exitflag(1, ii), output{1, ii}, lambda{1, ii}] = fminimax(Jfun, x_0(:, ii), A, b, Aeq, beq, lb, ub, nonlcon, solveroptions, varargin{:});
						infeas_opt(1, ii) = output{1, ii}.constrviolation;
					elseif nargout4
						[x_opt(:, ii), ~, f_opt(1, ii), exitflag(1, ii), output{1, ii}] = fminimax(Jfun, x_0(:, ii), A, b, Aeq, beq, lb, ub, nonlcon, solveroptions, varargin{:});
						infeas_opt(1, ii) = output{1, ii}.constrviolation;
					elseif nargout3
						[x_opt(:, ii), ~, f_opt(1, ii), exitflag(1, ii), temp] = fminimax(Jfun, x_0(:, ii), A, b, Aeq, beq, lb, ub, nonlcon, solveroptions, varargin{:});
						infeas_opt(1, ii) = temp.constrviolation;
					else
						[x_opt(:, ii), ~, f_opt(1, ii), ~, temp] = fminimax(Jfun, x_0(:, ii), A, b, Aeq, beq, lb, ub, nonlcon, solveroptions, varargin{:});
						infeas_opt(1, ii) = temp.constrviolation;
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
					if nargout5
						[x_opt(:, ii), ~, f_opt(1, ii), exitflag(1, ii), output{1, ii}, lambda{1, ii}] = fminimax(Jfun, x_0(:, ii), A, b, Aeq, beq, lb, ub, nonlcon, solveroptions, varargin{:});
						infeas_opt(1, ii) = output{1, ii}.constrviolation;
					elseif nargout4
						[x_opt(:, ii), ~, f_opt(1, ii), exitflag(1, ii), output{1, ii}] = fminimax(Jfun, x_0(:, ii), A, b, Aeq, beq, lb, ub, nonlcon, solveroptions, varargin{:});
						infeas_opt(1, ii) = output{1, ii}.constrviolation;
					elseif nargout3
						[x_opt(:, ii), ~, f_opt(1, ii), exitflag(1, ii), temp] = fminimax(Jfun, x_0(:, ii), A, b, Aeq, beq, lb, ub, nonlcon, solveroptions, varargin{:});
						infeas_opt(1, ii) = temp.constrviolation;
					else
						[x_opt(:, ii), ~, f_opt(1, ii), ~, temp] = fminimax(Jfun, x_0(:, ii), A, b, Aeq, beq, lb, ub, nonlcon, solveroptions, varargin{:});
						infeas_opt(1, ii) = temp.constrviolation;
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
	infeas_opt(isnan(infeas_opt)) = Inf;
	feasible = infeas_opt <= tolCon;
	infeas_opt = max([
		infeas_opt;
		zeros(size(infeas_opt))
	], [], 1);
	isMatlab2015A = matlab.Version.CURRENT >= matlab.Version.R2015A;
	if any(feasible)
		if isMatlab2015A
			minval = min(infeas_opt(1, feasible), [], 2, 'omitnan');
		else
			minval = min(infeas_opt(1, feasible), [], 2);
		end
	else
		if isMatlab2015A
			minval = min(infeas_opt, [], 2, 'omitnan');
		else
			minval = min(infeas_opt, [], 2);
		end
	end
	feasidx = infeas_opt <= minval + tolCon;
	linidx = 1:size(feasible, 2);
	feasidx = linidx(feasidx);
	if isMatlab2015A
		[fval, fvalidx] = min(f_opt(:, feasidx), [], 2, 'omitnan');
	else
		[fval, fvalidx] = min(f_opt(:, feasidx), [], 2);
	end
	if isempty(fvalidx)
		error('optimization:solver:fminimax:solution', 'No solution could be found for any initial value.');
	end
	idx = feasidx(fvalidx);
	if isempty(idx)
		error('optimization:solver:fminimax:solution', 'No solution could be found for any initial value.');
	end
	x = x_opt(:, idx);
	if nargout3
		exitflag = exitflag(1, idx);
	end
	if nargout4
		output = output{1, idx};
	end
	if nargout5
		lambda = lambda{1, idx};
	end
	if nargout >= 6
		if nargout(Jfun) >= 2
			if nargin >= 11 && nargin(Jfun) ~= 1
				[f, grad] = Jfun(x, varargin{:});
			else
				[f, grad] = Jfun(x);
			end
			if isMatlab2015A
				[~, fvalidx] = min(f, [], 1, 'omitnan');
			else
				[~, fvalidx] = min(f, [], 1);
			end
			grad = grad(:, fvalidx);
		else
			if nargout(Jfun) == -1
				try
					if nargin >= 11 && nargin(Jfun) ~= 1
						[f, grad] = Jfun(x, varargin{:});
					else
						[f, grad] = Jfun(x);
					end
					if isMatlab2015A
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
					parfor ii = 1:size(x, 1)
						grad(ii, :) = (Jfun(x + step*(1:size(x, 1) == ii)', varargin{:}) - f)/step; %#ok<PFBNS> Jfun is a function handle
					end
				else
					f = Jfun(x);
					parfor ii = 1:size(x, 1)
						grad(ii, :) = (Jfun(x + step*(1:size(x, 1) == ii)') - f)/step;
					end
				end
				if isMatlab2015A
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

function [cfun] = constr_dispatcher(type, nonlcon)
	function [c, ceq] = constrfunvararg(x, varargin) %#ok<VANUS> varargin is only defined for compatibility with an objective function with additional arguments
		[c, ceq] = nonlcon(x);
	end
	function [c, ceq, gradc, gradceq] = constrgradfunvararg(x, varargin) %#ok<VANUS> varargin is only defined for compatibility with an objective function with additional arguments
		if nargout >= 3
			[c, ceq, gradc, gradceq] = nonlcon(x);
		else
			[c, ceq] = nonlcon(x);
		end
	end
	if isempty(nonlcon)
		cfun = [];
	elseif strcmpi(type, 'constr')
		cfun = @constrfunvararg;
	elseif strcmpi(type, 'constrgrad')
		cfun = @constrgradfunvararg;
	else
		error('optimization:solver:fminimax:input', 'Undefined constraint function type.');
	end
end
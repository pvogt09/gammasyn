function [x, fval, exitflag, output, lambda, grad, hessian] = optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin)
	%OPTIMIZE call solvopt to solve optimization problem
	%	Input:
	%		fun:		objective function to minimize
	%		x_0:		initial value or matrix of initial values or StartPointSet of initial values to start optimization from
	%		A:			matrix of linear inequality constraints
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
		error('optimization:solver:solvopt:input', 'Objective function must not be empty.');
	end
	if ischar(fun)
		fun = str2func(fun);
	end
	if ~isfunctionhandle(fun)
		error('optimization:solver:solvopt:input', 'Objective function must be a function handle.');
	end
	if isempty(x_0)
		error('optimization:solver:solvopt:input', 'Initial point must not be empty.');
	end
	if isa(x_0, 'AbstractStartPointSet')
		x_0 = x_0.list().';
	end
	if ~isreal(x_0) || any(isinf(x_0(:))) || any(isnan(x_0(:)))
		error('optimization:solver:solvopt:input', 'Initial point must be a finite real value.');
	end
	if ~ismatrix(x_0)
		error('optimization:solver:solvopt:input', 'Initial point must be a column vector or a matrix of column vectors.');
	end
	%if size(x_0, 2) > 1
	%	error('optimization:solver:solvopt:input', 'Initial point must be a column vector.');
	%end
	isemptynonlcon = false;
	usedefaultoption = false;
	defaultsolver = optimization.solver.Optimizer.SOLVOPT;
	if nargin >= 10 && isa(options, 'optimization.solver.Optimizer')
		defaultsolver = options;
		usedefaultoption = true;
	end
	if nargin <= 9 || isempty(options)
		options = optimization.options.OptionFactory.instance.options(defaultsolver,...
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
		nonlcon = @emptynonlcon;
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
		error('optimization:solver:solvopt:input', 'Inequality matrix must be real.');
	end
	if ~ismatrix(A)
		error('optimization:solver:solvopt:input', 'Inequality matrix must be a matrix.');
	end
	if size(A, 2) ~= size(x_0, 1)
		error('optimization:solver:solvopt:input', 'Inequality matrix must have %d columns, not %d.', size(x_0, 1), size(A, 2));
	end
	if ~isreal(b) || any(isnan(b(:)))
		error('optimization:solver:solvopt:input', 'Inequality bounds must be real.');
	end
	if ~ismatrix(b)
		error('optimization:solver:solvopt:input', 'Inequality bounds must be a vector.');
	end
	if size(b, 1) ~= size(A, 1)
		error('optimization:solver:solvopt:input', 'Inequality bounds must have %d rows, not %d.', size(A, 1), size(b, 1));
	end
	if ~isreal(Aeq) || any(isnan(Aeq(:)))
		error('optimization:solver:solvopt:input', 'Equality matrix must be real.');
	end
	if ~ismatrix(Aeq)
		error('optimization:solver:solvopt:input', 'Equality matrix must be a matrix.');
	end
	if size(Aeq, 2) ~= size(x_0, 1)
		error('optimization:solver:solvopt:input', 'Equality matrix must have %d columns, not %d.', size(x_0, 1), size(Aeq, 2));
	end
	if ~isreal(beq) || any(isnan(beq(:)))
		error('optimization:solver:solvopt:input', 'Equality bounds must be real.');
	end
	if ~ismatrix(beq)
		error('optimization:solver:solvopt:input', 'Equality bounds must be a vector.');
	end
	if size(beq, 1) ~= size(Aeq, 1)
		error('optimization:solver:solvopt:input', 'Equality bounds must have %d rows, not %d.', size(Aeq, 1), size(beq, 1));
	end
	if ~isreal(lb) || any(isnan(lb(:)))
		error('optimization:solver:solvopt:input', 'Lower bounds must be real.');
	end
	if ~ismatrix(lb)
		error('optimization:solver:solvopt:input', 'Lower bounds must be a vector.');
	end
	if size(lb, 1) ~= size(x_0, 1)
		error('optimization:solver:solvopt:input', 'Lower bounds must have %d rows, not %d.', size(x_0, 1), size(lb, 1));
	end
	if ~isreal(ub) || any(isnan(ub(:)))
		error('optimization:solver:solvopt:input', 'Upper bounds must be real.');
	end
	if ~ismatrix(ub)
		error('optimization:solver:solvopt:input', 'Upper bounds must be a vector.');
	end
	if size(ub, 1) ~= size(x_0, 1)
		error('optimization:solver:solvopt:input', 'Upper bounds must have %d rows, not %d.', size(x_0, 1), size(ub, 1));
	end
	if nargin >= 9
		if ~isempty(nonlcon)
			if ischar(nonlcon)
				nonlcon = str2func(nonlcon);
			end
			if ~isfunctionhandle(nonlcon)
				error('optimization:solver:solvopt:input', 'Constraint function must be a function handle.');
			end
		else
			nonlcon = @emptynonlcon;
			isemptynonlcon = true;
		end
	else
		nonlcon = @emptynonlcon;
		isemptynonlcon = true;
	end
	function [c, ceq] = constrfun(x)
		[c, ceq] = nonlcon(x, varargin{:});
	end
	function [c, ceq, gradc, gradceq] = constrgradfun(x)
		if nargout >= 3
			[c, ceq, gradc, gradceq] = nonlcon(x, varargin{:});
		else
			[c, ceq] = nonlcon(x, varargin{:});
		end
	end
	function [f, grad] = objectivegradient(x)
		[f, grad] = fun(x, varargin{:});
	end
	isnargoutconstraintfunwithgradient = false;
	if nargin >= 11
		if nargout(fun) >= 2
			if nargin(fun) == 1
				Jfun = fun;
			else
				Jfun = @objectivegradient;
			end
			hasobjectivegradient = true;
		else
			if nargout(fun) == -1
				try
					if nargin(fun) == 1
						[~, tempgrad] = fun(x_0(:, 1)); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
						Jfun = fun;
					else
						[~, tempgrad] = fun(x_0(:, 1), varargin{:}); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
						Jfun = @objectivegradient;
					end
					hasobjectivegradient = true;
				catch e
					if strcmpi(e.identifier, 'MATLAB:maxlhs') || strcmpi(e.identifier, 'MATLAB:TooManyOutputs') || strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
						hasobjectivegradient = false;
						if nargin(fun) == 1
							Jfun = fun;
						else
							Jfun = @(x) fun(x, varargin{:});
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
					Jfun = fun;
				else
					Jfun = @(x) fun(x, varargin{:});
				end
				hasobjectivegradient = false;
				if usedefaultoption
					options.SpecifyObjectiveGradient = false;
				end
			end
		end
		if ~isemptynonlcon
			if nargout(nonlcon) >= 3
				if nargin(nonlcon) == 1
					cfun = nonlcon;
				else
					cfun = @constrgradfun;
				end
				hasconstraintgradient = true;
			else
				if nargout(nonlcon) == -1
					try
						if nargin(nonlcon) == 1
							[~, ~, tempgradc, tempgradceq] = nonlcon(x_0(:, 1)); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
							cfun = nonlcon;
							isnargoutconstraintfunwithgradient = true;
						else
							[~, ~, tempgradc, tempgradceq] = nonlcon(x_0(:, 1), varargin{:}); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
							cfun = @constrgradfun;
							isnargoutconstraintfunwithgradient = true;
						end
						hasconstraintgradient = true;
					catch e
						if strcmpi(e.identifier, 'MATLAB:maxlhs') || strcmpi(e.identifier, 'MATLAB:TooManyOutputs') || strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
							hasconstraintgradient = false;
							if nargin(nonlcon) == 1
								cfun = nonlcon;
							else
								cfun = @constrfun;
							end
							if usedefaultoption
								options.SpecifyConstraintGradient = false;
							end
						else
							rethrow(e);
						end
					end
				else
					if nargin(nonlcon) == 1
						cfun = nonlcon;
					else
						cfun = @constrfun;
					end
					hasconstraintgradient = false;
					if usedefaultoption
						options.SpecifyConstraintGradient = false;
					end
				end
			end
		else
			cfun = nonlcon;
			hasconstraintgradient = false;
		end
	else
		if nargout(fun) >= 2
			hasobjectivegradient = true;
		else
			if nargout(fun) == -1
				try
					[~, tempgrad] = fun(x_0(:, 1)); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
					hasobjectivegradient = true;
				catch e
					if strcmpi(e.identifier, 'MATLAB:maxlhs') || strcmpi(e.identifier, 'MATLAB:TooManyOutputs') || strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
						hasobjectivegradient = false;
						if usedefaultoption
							options.SpecifyObjectiveGradient = false;
						end
					else
						rethrow(e);
					end
				end
			else
				hasobjectivegradient = false;
				if usedefaultoption
					options.SpecifyObjectiveGradient = false;
				end
			end
		end
		Jfun = fun;
		if nargout(nonlcon) >= 3
			hasconstraintgradient = true;
		else
			if nargout(nonlcon) == -1
				try
					[~, ~, tempgradc, tempgradceq] = nonlcon(x_0(:, 1)); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
					hasconstraintgradient = true;
					isnargoutconstraintfunwithgradient = true;
				catch e
					if strcmpi(e.identifier, 'MATLAB:maxlhs') || strcmpi(e.identifier, 'MATLAB:TooManyOutputs') || strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
						hasconstraintgradient = false;
						if usedefaultoption
							options.SpecifyConstraintGradient = false;
						end
					else
						rethrow(e);
					end
				end
			else
				hasconstraintgradient = false;
				if usedefaultoption
					options.SpecifyConstraintGradient = false;
				end
			end
		end
		cfun = nonlcon;
	end
	hasconstraintgradient = hasconstraintgradient & (nargout(cfun) >= 4 || (nargout(cfun) == -1 && isnargoutconstraintfunwithgradient));
	if ~isa(options, 'optimization.options.Options')
		if isstruct(options)
			options = optimization.options.OptionFactory.instance.options(optimization.solver.Optimizer.SOLVOPT, options);
		elseif isa(options, 'optim.options.SolverOptions')
			options = optimization.options.OptionFactory.instance.options(optimization.solver.Optimizer.SOLVOPT, options);
		else
			error('optimization:solver:solvopt:input', 'Options must be of type optimization.options.Options or optimoptions or optimset.');
		end
	end
	if ~isa(options, 'optimization.options.solvopt')
		options = optimization.options.OptionFactory.instance.options(optimization.solver.Optimizer.SOLVOPT, options);
	end
	model = struct(...
		'f',		Jfun,...
		'c',		cfun,...
		'A',		A,...
		'b',		b,...
		'Aeq',		Aeq,...
		'beq',		beq,...
		'lb',		lb,...
		'ub',		ub,...
		'gradfun',	hasobjectivegradient & options.SpecifyObjectiveGradient,...
		'gradcon',	hasconstraintgradient & options.SpecifyConstraintGradient...
	);
	J = @callback_objective;
	gradJ = @callback_objective_gradient;
	constr = @callback_constraint;
	gradconstr = @callback_constraint_gradient;
	useparallel = options.UseParallel;
	if useparallel
		pool = gcp('nocreate');
		if ~isempty(pool)
			parfor ii = 1:pool.NumWorkers
				callback([], model);
			end
		end
	end
	callback([], model);
	[c, ceq] = cfun(x_0(:, 1));
	xmul = zeros(size(x_0, 1), 1);
	Fmul = zeros(1 + size(c, 1) + size(ceq, 1) + size(A, 1) + size(Aeq, 1), 1);
	options.NumberVariables = size(x_0, 1);
	options.NumberConstraintsInequality = size(A, 1) + size(c, 1);
	options.NumberConstraintsEquality = size(Aeq, 1) + size(ceq, 1);
	options.NumberConstraintsBounds = sum(~isinf(lb)) + sum(~isinf(ub));
	solveroptions = options.getpreferred();
	solveroptions = solveroptions2options(solveroptions);
	solvertime = tic;
	retry = 0;
	retryiterations = 0;
	retryfunevals = 0;
	outputstr = '';
	maxTime = options.MaxTime;
	if isempty(maxTime) || isnan(maxTime) || maxTime <= 0
		maxTime = Inf;
	end
	tolCon = options.ConstraintTolerance;
	if isempty(tolCon) || isnan(tolCon) || tolCon <= 0
		tolCon = 0;
	end
	initialvalues = size(x_0, 2);
	%useparallel = options.UseParallel;
	nargout3 = nargout >= 3;
	nargout4 = nargout >= 4;
	nargout5 = nargout >= 5;
	nargin11 = nargin >= 11;
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
			solvertimes = repmat(tic, 1, initialvalues);
			tempoutputs = cell(1, initialvalues);
			outstr = cell(1, initialvalues);
		end
		if nargout5 || useparallel
			lambda = cell(1, initialvalues);
		end
		if useparallel
			parfor ii = 1:initialvalues
				try
					[x_opt(:, ii), f_opt(1, ii), exitoptions] = solvopt(x_0(:, ii)',...
						J, gradJ,...
						solveroptions,...
						constr, gradconstr...
					);
					if exitoptions(9) <= 0
						exitflag(1, ii) = exitoptions(9);
						iterations(1, ii) = 0;
					else
						exitflag(1, ii) = 0;
						iterations(1, ii) = exitoptions(9);
					end
					funevals(1, ii) = exitoptions(10);
					if nargout5
						lambda{1, ii} = struct(...
							'lower',		xmul,...
							'upper',		xmul,...
							'ineqnonlin',	Fmul(2:size(c, 1) + 1),...
							'eqnonlin',		Fmul(size(c, 1) + 2:size(c, 1) + size(ceq, 1) + 1),...
							'ineqlin',		Fmul(size(c, 1) + size(ceq, 1) + 2:size(c, 1) + size(ceq, 1) + size(A, 1) + 1),...
							'eqlin',		Fmul(size(c, 1) + size(ceq, 1) + size(A, 1) + 2:size(c, 1) + size(ceq, 1) + size(A, 1) + size(Aeq, 1) + 1)...
						);
					end
				catch e
					rethrow(e);
				end
				if ~isempty(nonlcon)
					if nargin11
						if nargin(cfun) == 1
							[ccurrent, ceqcurrent] = cfun(x_opt(:, ii));
						else
							[ccurrent, ceqcurrent] = cfun(x_opt(:, ii), varargin{:});
						end
					else
						[ccurrent, ceqcurrent] = cfun(x_opt(:, ii));
					end
				else
					ccurrent = zeros(0, 1);
					ceqcurrent = zeros(0, 1);
				end
				infeas_opt(1, ii) = max([
					ccurrent;
					ceqcurrent;
					A*x_opt(:, ii) - b;
					Aeq*x_opt(:, ii) - beq;
					0
				]);
				if nargout4
					outstr{1, ii} = solvopt_status2message(exitflag(1, ii));
					tempoutputs{1, ii} = options.formatOutput(exitflag(1, ii), toc(solvertimes(1, ii)), x_opt(:, ii), f_opt(1, ii), numel(x_0(:, ii)), iterations(1, ii), funevals(1, ii), retry, struct(...
						'iter',			iterations(1, ii),...
						'funevals',		funevals(1, ii),...
						'fval',			f_opt(1, ii),...
						'constrviol',	infeas_opt(1, ii),...
						'outputstr',	outstr{1, ii}...
					));
				end
			end
		else
			for ii = 1:initialvalues %#ok<FORPF> parallel computing is disabled through option set by user
				try
					[x_opt(:, ii), f_opt(1, ii), exitoptions] = solvopt(x_0(:, ii)',...
						J, gradJ,...
						solveroptions,...
						constr, gradconstr...
					);
					if exitoptions(9) <= 0
						exitflag(1, ii) = exitoptions(9);
						iterations(1, ii) = 0;
					else
						exitflag(1, ii) = 0;
						iterations(1, ii) = exitoptions(9);
					end
					funevals(1, ii) = exitoptions(10);
					if nargout5
						lambda{1, ii} = struct(...
							'lower',		xmul,...
							'upper',		xmul,...
							'ineqnonlin',	Fmul(2:size(c, 1) + 1),...
							'eqnonlin',		Fmul(size(c, 1) + 2:size(c, 1) + size(ceq, 1) + 1),...
							'ineqlin',		Fmul(size(c, 1) + size(ceq, 1) + 2:size(c, 1) + size(ceq, 1) + size(A, 1) + 1),...
							'eqlin',		Fmul(size(c, 1) + size(ceq, 1) + size(A, 1) + 2:size(c, 1) + size(ceq, 1) + size(A, 1) + size(Aeq, 1) + 1)...
						);
					end
				catch e
					rethrow(e);
				end
				if ~isempty(nonlcon)
					if nargin11
						if nargin(cfun) == 1
							[ccurrent, ceqcurrent] = cfun(x_opt(:, ii));
						else
							[ccurrent, ceqcurrent] = cfun(x_opt(:, ii), varargin{:});
						end
					else
						[ccurrent, ceqcurrent] = cfun(x_opt(:, ii));
					end
				else
					ccurrent = zeros(0, 1);
					ceqcurrent = zeros(0, 1);
				end
				infeas_opt(1, ii) = max([
					ccurrent;
					ceqcurrent;
					A*x_opt(:, ii) - b;
					Aeq*x_opt(:, ii) - beq;
					0
				]);
				if nargout4
					outstr{1, ii} = solvopt_status2message(exitflag(1, ii));
					tempoutputs{1, ii} = options.formatOutput(exitflag(1, ii), toc(solvertimes(1, ii)), x_opt(:, ii), f_opt(1, ii), numel(x_0(:, ii)), iterations(1, ii), funevals(1, ii), retry, struct(...
						'iter',			iterations(1, ii),...
						'funevals',		funevals(1, ii),...
						'fval',			f_opt(1, ii),...
						'constrviol',	infeas_opt(1, ii),...
						'outputstr',	outstr{1, ii}...
					));
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
			outputstr = sprintf('Time limit %fs exceeded', maxTime);
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
		error('optimization:solver:snopt:solution', 'No solution could be found for any initial value.');
	end
	idx = feasidx(fvalidx);
	if isempty(idx)
		error('optimization:solver:snopt:solution', 'No solution could be found for any initial value.');
	end
	x = x_opt(:, idx);
	if nargout3
		exitflag = exitflag(1, idx);
	end
	if nargout5
		lambda = lambda{1, idx};
	end
	if nargout >= 6
		if model.gradfun
			grad = gradJ(x);
			grad = grad.';
		else
			grad = zeros(size(x, 1), 1);
			step = eps;
			f = J(x);
			parfor ii = 1:size(x, 1)
				grad(ii) = (J(x + step*(1:size(x, 1) == ii)') - f)/step;
			end
		end
	end
	if nargout >= 7
		hessian = NaN(size(x_0, 1), size(x_0, 1));
	end
	solvertime = toc(solvertime);
	if nargout >= 4
		if strcmpi(outputstr, '')
			outputstr = outstr{1, idx};
		end
		alloutputs(retry + 1:end, :) = [];
		output = options.formatOutput(exitflag, solvertime, x, fval, numel(x_0(:, 1)), retryiterations, retryfunevals, retry, struct(...
			'iter',			iterations(1, idx),...
			'funevals',		funevals(1, idx),...
			'fval',			fval,...
			'constrviol',	infeas_opt(1, idx),...
			'outputstr',	outputstr,...
			'time',			solvertime...
		), alloutputs);
	end
end

function [c, ceq] = emptynonlcon(~)
	%EMPTYNONLCON dummy function for not supplied nonlinear constraints
	%	Input:
	%		x:		current value
	%	Output:
	%		c:		inequality constraints
	%		ceq:	equality constraints
	c = [];
	ceq = [];
end

function [options] = solveroptions2options(solveroptions)
	%SOLVEROPTIONS2OPTIONS convert solveroptions to option vector
	%	Input:
	%		solveroptions:	options to set
	%	Output:
	%		options:		vecor with options
	options = NaN(8, 1);
	options(1, 1) = -1;% minimization problem
	options(2, 1) = solveroptions.StepTolerance;
	options(3, 1) = solveroptions.FunctionTolerance;
	options(4, 1) = solveroptions.MaxIterations;
	if any(strcmpi(solveroptions.Display, {'off', 'none'}))
		options(5, 1) = -1;
	elseif any(strcmpi(solveroptions.Display, {'final', 'final-detailed'}))
		options(5, 1) = 0;
	else
		options(5, 1) = 10;
	end
	options(6, 1) = solveroptions.ConstraintTolerance;
	if ~isempty(solveroptions.FiniteDifferenceStepSize)
		options(8, 1) = solveroptions.FiniteDifferenceStepSize;
	end
end
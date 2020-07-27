function [x, fval, exitflag, output, lambda, grad, hessian] = optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin)
	%OPTIMIZE call sqpgs with constrained algorithm to solve optimization problem
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
		error('optimization:solver:slqpgs:input', 'Objective function must not be empty.');
	end
	if ischar(fun)
		fun = str2func(fun);
	end
	if ~isfunctionhandle(fun)
		error('optimization:solver:slqpgs:input', 'Objective function must be a function handle.');
	end
	if isempty(x_0)
		error('optimization:solver:slqpgs:input', 'Initial point must not be empty.');
	end
	if isa(x_0, 'AbstractStartPointSet')
		x_0 = x_0.list().';
	end
	if ~isreal(x_0) || any(isinf(x_0(:))) || any(isnan(x_0(:)))
		error('optimization:solver:slqpgs:input', 'Initial point must be a finite real value.');
	end
	if ~ismatrix(x_0)
		error('optimization:solver:slqpgs:input', 'Initial point must be a column vector or a matrix of column vectors.');
	end
	%if size(x_0, 2) > 1
	%	error('optimization:solver:slqpgs:input', 'Initial point must be a column vector.');
	%end
	usedefaultoption = false;
	defaultsolver = optimization.solver.Optimizer.SQPGS;
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
		error('optimization:solver:slqpgs:input', 'Inequality matrix must be real.');
	end
	if ~ismatrix(A)
		error('optimization:solver:slqpgs:input', 'Inequality matrix must be a matrix.');
	end
	if size(A, 2) ~= size(x_0, 1)
		error('optimization:solver:slqpgs:input', 'Inequality matrix must have %d columns, not %d.', size(x_0, 1), size(A, 2));
	end
	if ~isreal(b) || any(isnan(b(:)))
		error('optimization:solver:slqpgs:input', 'Inequality bounds must be real.');
	end
	if ~ismatrix(b)
		error('optimization:solver:slqpgs:input', 'Inequality bounds must be a vector.');
	end
	if size(b, 1) ~= size(A, 1)
		error('optimization:solver:slqpgs:input', 'Inequality bounds must have %d rows, not %d.', size(A, 1), size(b, 1));
	end
	if ~isreal(Aeq) || any(isnan(Aeq(:)))
		error('optimization:solver:slqpgs:input', 'Equality matrix must be real.');
	end
	if ~ismatrix(Aeq)
		error('optimization:solver:slqpgs:input', 'Equality matrix must be a matrix.');
	end
	if size(Aeq, 2) ~= size(x_0, 1)
		error('optimization:solver:slqpgs:input', 'Equality matrix must have %d columns, not %d.', size(x_0, 1), size(Aeq, 2));
	end
	if ~isreal(beq) || any(isnan(beq(:)))
		error('optimization:solver:slqpgs:input', 'Equality bounds must be real.');
	end
	if ~ismatrix(beq)
		error('optimization:solver:slqpgs:input', 'Equality bounds must be a vector.');
	end
	if size(beq, 1) ~= size(Aeq, 1)
		error('optimization:solver:slqpgs:input', 'Equality bounds must have %d rows, not %d.', size(Aeq, 1), size(beq, 1));
	end
	if ~isreal(lb) || any(isnan(lb(:)))
		error('optimization:solver:slqpgs:input', 'Lower bounds must be real.');
	end
	if ~ismatrix(lb)
		error('optimization:solver:slqpgs:input', 'Lower bounds must be a vector.');
	end
	if size(lb, 1) ~= size(x_0, 1)
		error('optimization:solver:slqpgs:input', 'Lower bounds must have %d rows, not %d.', size(x_0, 1), size(lb, 1));
	end
	if ~isreal(ub) || any(isnan(ub(:)))
		error('optimization:solver:slqpgs:input', 'Upper bounds must be real.');
	end
	if ~ismatrix(ub)
		error('optimization:solver:slqpgs:input', 'Upper bounds must be a vector.');
	end
	if size(ub, 1) ~= size(x_0, 1)
		error('optimization:solver:slqpgs:input', 'Upper bounds must have %d rows, not %d.', size(x_0, 1), size(ub, 1));
	end
	if nargin >= 9
		if ~isempty(nonlcon)
			if ischar(nonlcon)
				nonlcon = str2func(nonlcon);
			end
			if ~isfunctionhandle(nonlcon)
				error('optimization:solver:slqpgs:input', 'Constraint function must be a function handle.');
			end
		end
	else
		nonlcon = [];
	end
	if ~isa(options, 'optimization.options.Options')
		if isstruct(options)
			options = optimization.options.OptionFactory.instance.options(optimization.solver.Optimizer.SQPGS, options);
		elseif isa(options, 'optim.options.SolverOptions')
			options = optimization.options.OptionFactory.instance.options(optimization.solver.Optimizer.SQPGS, options);
		else
			error('optimization:solver:slqpgs:input', 'Options must be of type optimization.options.Options or optimoptions or optimset.');
		end
	end
	if ~isa(options, 'optimization.options.sqpgs')
		options = optimization.options.OptionFactory.instance.options(optimization.solver.Optimizer.SQPGS, options);
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
		if ~isempty(nonlcon)
			if nargout(nonlcon) >= 3
				if nargin(nonlcon) == 1
					cfun = nonlcon;
				else
					cfun = constr_dispatcher('constrgrad', nonlcon, varargin{:});
				end
				[~, ~, gradc, gradceq] = cfun(x_0(:, 1));
			else
				if nargout(nonlcon) == -1
					try
						if nargin(nonlcon) == 1
							[~, ~, gradc, gradceq] = callnonlcon(nonlcon, x_0(:, 1));
							cfun = nonlcon;
						else
							[~, ~, gradc, gradceq] = callnonlcon(nonlcon, x_0(:, 1), varargin{:});
							cfun = constr_dispatcher('constrgrad', nonlcon, varargin{:});
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
							gradc = NaN(size(c, 1), size(x_0, 1));
							gradceq = NaN(size(ceq, 1), size(x_0, 1));
							if usedefaultoption
								options.SpecifyConstraintGradient = false;
							end
						end
					end
				else
					if nargin(nonlcon) == 1
						cfun = nonlcon;
					else
						cfun = constr_dispatcher('constr', nonlcon, varargin{:});
					end
					gradc = NaN(size(c, 1), size(x_0, 1));
					gradceq = NaN(size(ceq, 1), size(x_0, 1));
					if usedefaultoption
						options.SpecifyConstraintGradient = false;
					end
				end
			end
		else
			cfun = constr_dispatcher('constrgrad', nonlcon, varargin{:});
			gradc = NaN(size(c, 1), size(x_0, 1));
			gradceq = NaN(size(ceq, 1), size(x_0, 1));
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
		if ~isempty(nonlcon)
			if nargout(nonlcon) >= 3
				cfun = nonlcon;
				[~, ~, gradc, gradceq] = cfun(x_0(:, 1));
			else
				if nargout(nonlcon) == -1
					try
						[~, ~, gradc, gradceq] = callnonlcon(nonlcon, x_0(:, 1));
						cfun = nonlcon;
					catch e
						if ~strcmpi(e.identifier, 'MATLAB:maxlhs') && ~strcmpi(e.identifier, 'MATLAB:TooManyOutputs') && ~strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
							rethrow(e);
						else
							cfun = nonlcon;
							gradc = NaN(size(c, 1), size(x_0, 1));
							gradceq = NaN(size(ceq, 1), size(x_0, 1));
							if usedefaultoption
								options.SpecifyConstraintGradient = false;
							end
						end
					end
				else
					cfun = nonlcon;
					gradc = NaN(size(c, 1), size(x_0, 1));
					gradceq = NaN(size(ceq, 1), size(x_0, 1));
					if usedefaultoption
						options.SpecifyConstraintGradient = false;
					end
				end
			end
		else
			cfun = nonlcon;
			gradc = NaN(size(c, 1), size(x_0, 1));
			gradceq = NaN(size(ceq, 1), size(x_0, 1));
		end
	end
	if isempty(nonlcon)
		if options.SpecifyObjectiveGradient
			if any(isnan(gradJ(:)))
				gradientJ = false;
			else
				gradientJ = true;
			end
		else
			gradientJ = false;
		end
		gradientc = true;
	else
		if options.SpecifyObjectiveGradient && options.SpecifyConstraintGradient
			if any(isnan(gradJ(:)))
				gradientJ = false;
				if any(isnan(gradc(:))) || any(isnan(gradceq(:)))
					gradientc = false;
				else
					gradientc = true;
				end
			else
				gradientJ = true;
				if any(isnan(gradc(:))) || any(isnan(gradceq(:)))
					gradientc = false;
				else
					gradientc = true;
				end
			end
		else
			gradientJ = false;
			gradientc = false;
		end
	end
	options.NumberVariables = size(x_0, 1);
	options.NumberConstraintsInequality = size(A, 1) + size(c, 1);
	options.NumberConstraintsEquality = size(Aeq, 1) + size(ceq, 1);
	options.NumberConstraintsBounds = sum(~isinf(lb)) + sum(~isinf(ub));
	solveroptions = options.getpreferred();
	solveroptions.algorithm = 0;
	solveroptions.sp_solver = solveroptions.SubproblemAlgorithm;
	if isempty(solveroptions.sp_solver)
		solveroptions.sp_solver = 'quadprog';
	end
	if ~any(strcmpi(solveroptions.sp_solver, {'quadprog', 'gurobi'}))
		error('optimization:solver:slqpgs:input', 'Algorithm slqpgs only supports subproblem algorithms ''quadprog'' and ''gurobi''.');
	end
	solveroptions.eq_tol = solveroptions.ineq_tol;
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
	solveroptions.f = function_dispatcher(Jfun, cfun, lb, ub, A, b, Aeq, beq, gradientJ, gradientc, FinDiffType, FinDiffStepSize, options.TypicalX, options.HonorBounds);
	solveroptions.nV = options.NumberVariables;
	solveroptions.nE = options.NumberConstraintsEquality;
	solveroptions.nI = options.NumberConstraintsInequality + options.NumberConstraintsBounds;
	solveroptions.output = str2double(solveroptions.Display);
	solveroptions.pO = 2*options.NumberVariables;
	solveroptions.pE = 2*ones(options.NumberConstraintsEquality, 1);
	solveroptions.pI = 2*ones(options.NumberConstraintsInequality + options.NumberConstraintsBounds, 1);
	sqpgsnames = fieldnames(solveroptions);
	removefields = false(size(sqpgsnames, 1), 1);
	for ii = 1:size(sqpgsnames, 1) %#ok<FORPF> no parfor because of structure modification
		if isempty(solveroptions.(sqpgsnames{ii, 1}))
			removefields(ii, 1) = true;
		end
	end
	solveroptions = rmfield(solveroptions, sqpgsnames(removefields));
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
				currentoptions = solveroptions;
				currentoptions.x = x_0(:, ii);
				try
					opt = slqpgs.SLQPGS(currentoptions);
					opt.optimize();
					if nargout7
						[x_opt(:, ii), f_opt(1, ii), exitflag(1, ii), output{1, ii}, lambda{1, ii}, grad{1, ii}, hessian{1, ii}] = opt.getResult();
						infeas_opt(1, ii) = output{1, ii}.constrviolation;
					elseif nargout6
						[x_opt(:, ii), f_opt(1, ii), exitflag(1, ii), output{1, ii}, lambda{1, ii}, grad{1, ii}] = opt.getResult();
						infeas_opt(1, ii) = output{1, ii}.constrviolation;
					elseif nargout5
						[x_opt(:, ii), f_opt(1, ii), exitflag(1, ii), output{1, ii}, lambda{1, ii}] = opt.getResult();
						infeas_opt(1, ii) = output{1, ii}.constrviolation;
					elseif nargout4
						[x_opt(:, ii), f_opt(1, ii), exitflag(1, ii), output{1, ii}] = opt.getResult();
						infeas_opt(1, ii) = output{1, ii}.constrviolation;
					elseif nargout3
						[x_opt(:, ii), f_opt(1, ii), exitflag(1, ii), temp] = opt.getResult();
						infeas_opt(1, ii) = temp.constrviolation;
					else
						[x_opt(:, ii), f_opt(1, ii), ~, temp] = opt.getResult();
						infeas_opt(1, ii) = temp.constrviolation;
					end
				catch e
					if strcmpi(e.identifier, 'EMLRT:runTime:UserInterrupt')
						x_opt(:, ii) = NaN(numel(x_0(:, ii)), 1);
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
							if nargout5
								lambda{1, ii} = struct(...
									'lower',		[],...
									'upper',		[],...
									'ineqnonlin',	[],...
									'eqnonlin',		[],...
									'ineqlin',		[],...
									'eqlin',		[]...
								);
								if nargout6
									grad{1, ii} = NaN(numel(x_0(:, ii)), 1);
									if nargout7
										hessian{1, ii} = NaN(numel(x_0(:, ii)), numel(x_0(:, ii)));
									end
								end
							end
						end
					else
						rethrow(e);
					end
				end
				clearfun = solveroptions.f;
				clearfun(x_0(:, ii), [], [], -1);
				if nargout4
					iterations(1, ii) = output{1, ii}.iterations;
					funevals(1, ii) = output{1, ii}.funcCount;
					tempoutputs{1, ii} = options.formatOutput(exitflag(1, ii), toc(solvertimes(1, ii)), x_opt(:, ii), f_opt(1, ii), numel(x_0(:, ii)), iterations(1, ii), funevals(1, ii), retry, output{1, ii}); %#ok<PFBNS> options is an object
				end
			end
		else
			for ii = 1:initialvalues %#ok<FORPF> parallel computing is disabled through option set by user
				solvertimes(1, ii) = tic;
				currentoptions = solveroptions;
				currentoptions.x = x_0(:, ii);
				try
					opt = slqpgs.SLQPGS(currentoptions);
					opt.optimize();
					if nargout7
						[x_opt(:, ii), f_opt(1, ii), exitflag(1, ii), output{1, ii}, lambda{1, ii}, grad{1, ii}, hessian{1, ii}] = opt.getResult();
						infeas_opt(1, ii) = output{1, ii}.constrviolation;
					elseif nargout6
						[x_opt(:, ii), f_opt(1, ii), exitflag(1, ii), output{1, ii}, lambda{1, ii}, grad{1, ii}] = opt.getResult();
						infeas_opt(1, ii) = output{1, ii}.constrviolation;
					elseif nargout5
						[x_opt(:, ii), f_opt(1, ii), exitflag(1, ii), output{1, ii}, lambda{1, ii}] = opt.getResult();
						infeas_opt(1, ii) = output{1, ii}.constrviolation;
					elseif nargout4
						[x_opt(:, ii), f_opt(1, ii), exitflag(1, ii), output{1, ii}] = opt.getResult();
						infeas_opt(1, ii) = output{1, ii}.constrviolation;
					elseif nargout3
						[x_opt(:, ii), f_opt(1, ii), exitflag(1, ii), temp] = opt.getResult();
						infeas_opt(1, ii) = temp.constrviolation;
					else
						[x_opt(:, ii), f_opt(1, ii), ~, temp] = opt.getResult();
						infeas_opt(1, ii) = temp.constrviolation;
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
							if nargout5
								lambda{1, ii} = struct(...
									'lower',		[],...
									'upper',		[],...
									'ineqnonlin',	[],...
									'eqnonlin',		[],...
									'ineqlin',		[],...
									'eqlin',		[]...
								);
								if nargout6
									grad{1, ii} = NaN(size(x_0, 1), 1);
									if nargout7
										hessian{1, ii} = NaN(size(x_0, 1), size(x_0, 1));
									end
								end
							end
						end
						break;
					else
						rethrow(e);
					end
				end
				clearfun = solveroptions.f;
				clearfun(x_0(:, ii), [], [], -1);
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
		error('optimization:solver:sqpgs:solution', 'No solution could be found for any initial value.');
	end
	idx = feasidx(fvalidx);
	if isempty(idx)
		error('optimization:solver:sqpgs:solution', 'No solution could be found for any initial value.');
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
	if nargout6
		grad = grad{1, idx};
	end
	if nargout7
		hessian = hessian{1, idx};
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
	if isempty(nonlcon)
		c = [];
		ceq = [];
		if nargout >= 3
			gradc = zeros(0, size(x, 1));
			gradceq = zeros(0, size(x, 1));
		end
		return;
	end
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
	if isempty(nonlcon)
		cfun = [];
	elseif strcmpi(type, 'constr')
		cfun = @constrfunvararg;
	elseif strcmpi(type, 'constrgrad')
		cfun = @constrgradfunvararg;
	else
		error('optimization:solver:nloptcon:input', 'Undefined constraint function type.');
	end
end

function [fun] = function_dispatcher(obj, con, lb, ub, A, b, Aeq, beq, gradientJ, gradientc, FinDiffType, FinDiffRelStep, TypicalX, AlwaysHonorBounds)
	%FUNCTION_DISPATCHER wrapper function for slpgs to convert the objective function into a function with return type argument
	%	Input:
	%		obj:				objective function handle
	%		con:				constraint function handle
	%		lb:					lower bounds
	%		ub:					upper bounds
	%		A:					matrix of linear inequality constraints
	%		b:					upper bound for linear inequality constraints
	%		Aeq:				matrix of linear equality constraints
	%		beq:				upper bound for linear equality constraints
	%		gradientJ:			indicator, if gradient for objective function is supplied
	%		gradientc:			indicator, if gradient for constraint function is supplied
	%		FinDiffType:		finite difference type to use
	%		FinDiffRelStep:		relative finite difference step
	%		TypicalX:			typical values for optimization variable
	%		AlwaysHonorBounds:	indicator if bounds are honored during finite difference calculation
	%	Output:
	%		fun:				objective function handle for use with SLP-GS
	function [val] = f(x, ~, index, type)
		persistent lastx;
		persistent c;
		persistent ceq;
		persistent gradc;
		persistent gradceq;
		persistent firstrun;
		if type == 0% Objective function value
			val = obj(x);
		elseif type == 1% Objective gradient value (as column vector)
			if gradientJ
				[~, val] = obj(x);
			else
				val = optimization.general.finitedifference(x, obj, NaN, FinDiffType, 1, lb, ub, FinDiffRelStep, TypicalX, AlwaysHonorBounds, true).';
			end
		elseif any(type == [2, 3, 4, 5])
			if isempty(lastx) || isempty(firstrun) || firstrun || any(lastx(:) ~= x(:))
				if isempty(con)
					c = [];
					ceq = [];
					gradc = zeros(size(x, 1), 0);
					gradceq = zeros(size(x, 1), 0);
				else
					if gradientc
						[c, ceq, gradc, gradceq] = con(x);
					else
						[c, ceq] = con(x);
						gradc = optimization.general.finitedifference(x, con, size(c, 1), FinDiffType, 1, lb, ub, FinDiffRelStep, TypicalX, AlwaysHonorBounds, true).';
						gradceq = optimization.general.finitedifference(x, con, size(ceq, 1), FinDiffType, 2, lb, ub, FinDiffRelStep, TypicalX, AlwaysHonorBounds, true).';
					end
				end
				if isempty(ceq)
					ceq = Aeq*x - beq;
				else
					ceq = [
						ceq;
						Aeq*x - beq;
					];
				end
				gradceq = [
					gradceq';
					Aeq
				];
				if isempty(c)
					c = [
						A*x - b;
						-x(~isinf(lb)) + lb(~isinf(lb));
						x(~isinf(ub)) - ub(~isinf(ub))
					];
				else
					c = [
						c;
						A*x - b;
						-x(~isinf(lb)) + lb(~isinf(lb));
						x(~isinf(ub)) - ub(~isinf(ub))
					];
				end
				I = eye(size(x, 1));
				gradc = [
					gradc';
					A;
					-I(~isinf(lb), :);
					I(~isinf(ub), :)
				];
				firstrun = false;
			end
			if type == 2% j-th equality constraint value
				if isempty(ceq)
					val = [];
				else
					val = ceq(index);
				end
			elseif type == 3% j-th equality constraint gradient value (as row vector)
				if isempty(gradceq)
					val = [];
				else
					val = gradceq(index, :);
				end
			elseif type == 4% j-th inequality constraint value
				if isempty(c)
					val = [];
				else
					val = c(index);
				end
			elseif type == 5% j-th inequality constraint gradient value (as row vector)
				if isempty(gradc)
					val = [];
				else
					val = gradc(index, :);
				end
			else
				error('optimization:solver:slqpgs:input', 'Option not given to problem function evaluator.');
			end
		elseif -1% clear persistent variables
				lastx  = [];
				c = [];
				ceq = [];
				gradc = [];
				gradceq = [];
				firstrun = true;
		else% This error is not supposed to happen!
			error('optimization:solver:slqpgs:input', 'Option not given to problem function evaluator.');
		end
		if type ~= -1
			lastx = x;
		end
	end
	fun = @f;
end
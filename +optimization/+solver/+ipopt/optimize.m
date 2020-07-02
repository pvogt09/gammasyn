function [x, fval, exitflag, output, lambda, grad, hessian] = optimize(fun, x_0, A, b, Aeq, beq, lb, ub, nonlcon, options, varargin)
	%OPTIMIZE call ipopt to solve optimization problem
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
		error('optimization:solver:ipopt:input', 'Objective function must not be empty.');
	end
	if ischar(fun)
		fun = str2func(fun);
	end
	if ~isfunctionhandle(fun)
		error('optimization:solver:ipopt:input', 'Objective function must be a function handle.');
	end
	if isempty(x_0)
		error('optimization:solver:ipopt:input', 'Initial point must not be empty.');
	end
	if isa(x_0, 'AbstractStartPointSet')
		x_0 = x_0.list().';
	end
	if ~isreal(x_0) || any(isinf(x_0(:))) || any(isnan(x_0(:)))
		error('optimization:solver:ipopt:input', 'Initial point must be a finite real value.');
	end
	if ~ismatrix(x_0)
		error('optimization:solver:ipopt:input', 'Initial point must be a column vector or a matrix of column vectors.');
	end
	%if size(x_0, 2) > 1
	%	error('optimization:solver:ipopt:input', 'Initial point must be a column vector.');
	%end
	isemptynonlcon = false;
	usedefaultoption = false;
	defaultsolver = optimization.solver.Optimizer.FMINCONGLOBAL;
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
		error('optimization:solver:ipopt:input', 'Inequality matrix must be real.');
	end
	if ~ismatrix(A)
		error('optimization:solver:ipopt:input', 'Inequality matrix must be a matrix.');
	end
	if size(A, 2) ~= size(x_0, 1)
		error('optimization:solver:ipopt:input', 'Inequality matrix must have %d columns, not %d.', size(x_0, 1), size(A, 2));
	end
	if ~isreal(b) || any(isnan(b(:)))
		error('optimization:solver:ipopt:input', 'Inequality bounds must be real.');
	end
	if ~ismatrix(b)
		error('optimization:solver:ipopt:input', 'Inequality bounds must be a vector.');
	end
	if size(b, 1) ~= size(A, 1)
		error('optimization:solver:ipopt:input', 'Inequality bounds must have %d rows, not %d.', size(A, 1), size(b, 1));
	end
	if ~isreal(Aeq) || any(isnan(Aeq(:)))
		error('optimization:solver:ipopt:input', 'Equality matrix must be real.');
	end
	if ~ismatrix(Aeq)
		error('optimization:solver:ipopt:input', 'Equality matrix must be a matrix.');
	end
	if size(Aeq, 2) ~= size(x_0, 1)
		error('optimization:solver:ipopt:input', 'Equality matrix must have %d columns, not %d.', size(x_0, 1), size(Aeq, 2));
	end
	if ~isreal(beq) || any(isnan(beq(:)))
		error('optimization:solver:ipopt:input', 'Equality bounds must be real.');
	end
	if ~ismatrix(beq)
		error('optimization:solver:ipopt:input', 'Equality bounds must be a vector.');
	end
	if size(beq, 1) ~= size(Aeq, 1)
		error('optimization:solver:ipopt:input', 'Equality bounds must have %d rows, not %d.', size(Aeq, 1), size(beq, 1));
	end
	if ~isreal(lb) || any(isnan(lb(:)))
		error('optimization:solver:ipopt:input', 'Lower bounds must be real.');
	end
	if ~ismatrix(lb)
		error('optimization:solver:ipopt:input', 'Lower bounds must be a vector.');
	end
	if size(lb, 1) ~= size(x_0, 1)
		error('optimization:solver:ipopt:input', 'Lower bounds must have %d rows, not %d.', size(x_0, 1), size(lb, 1));
	end
	if ~isreal(ub) || any(isnan(ub(:)))
		error('optimization:solver:ipopt:input', 'Upper bounds must be real.');
	end
	if ~ismatrix(ub)
		error('optimization:solver:ipopt:input', 'Upper bounds must be a vector.');
	end
	if size(ub, 1) ~= size(x_0, 1)
		error('optimization:solver:ipopt:input', 'Upper bounds must have %d rows, not %d.', size(x_0, 1), size(ub, 1));
	end
	if nargin >= 9
		if ~isempty(nonlcon)
			if ischar(nonlcon)
				nonlcon = str2func(nonlcon);
			end
			if ~isfunctionhandle(nonlcon)
				error('optimization:solver:ipopt:input', 'Constraint function must be a function handle.');
			end
		else
			nonlcon = @emptynonlcon;
			isemptynonlcon = true;
		end
	else
		nonlcon = @emptynonlcon;
		isemptynonlcon = true;
	end
	if ~isa(options, 'optimization.options.Options')
		if isstruct(options)
			options = optimization.options.OptionFactory.instance.options(optimization.solver.Optimizer.IPOPT, options);
		elseif isa(options, 'optim.options.SolverOptions')
			options = optimization.options.OptionFactory.instance.options(optimization.solver.Optimizer.IPOPT, options);
		else
			error('optimization:solver:ipopt:input', 'Options must be of type optimization.options.Options or optimoptions or optimset.');
		end
	end
	if ~isa(options, 'optimization.options.ipopt')
		options = optimization.options.OptionFactory.instance.options(optimization.solver.Optimizer.IPOPT, options);
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
	function [c, ceq, gradc, gradceq, hessianc, hessianceq] = constrhessfun(x)
		if nargout >= 5
			[c, ceq, gradc, gradceq, hessianc, hessianceq] = nonlcon(x, varargin{:});
		elseif nargout >= 3
			[c, ceq, gradc, gradceq] = nonlcon(x, varargin{:});
		else
			[c, ceq] = nonlcon(x, varargin{:});
		end
	end
	function [f, grad] = objectivegradientvararg(x)
		[f, grad] = fun(x, varargin{:});
	end
	function [f, grad, hessian] = objectivehessianvararg(x)
		[f, grad, hessian] = fun(x, varargin{:});
	end
	isnargoutconstraintfunwithgradient = false;
	needshessian = options.supportsHessian() && options.SpecifyObjectiveHessian && options.SpecifyConstraintHessian;
	hasobjectivehessian = false;
	hasconstrainthessian = false;
	if nargin >= 11
		if nargout(fun) >= 3
			if nargin(fun) == 1
				Jfun = fun;
			else
				Jfun = @objectivehessianvararg;
			end
			hasobjectivegradient = true;
			hasobjectivehessian = true;
		elseif nargout(fun) >= 2
			if nargin(fun) == 1
				Jfun = fun;
			else
				Jfun = @objectivegradientvararg;
			end
			hasobjectivegradient = true;
		else
			if nargout(fun) == -1
				if needshessian
					try
						try
							if nargin(fun) == 1
								[~, tempgrad, temphess] = fun(x_0(:, 1)); %#ok<ASGLU> call with three output arguments to check if the third one is present and catch error if not
								hasobjectivegradient = true;
								hasobjectivehessian = true;
								Jfun = fun;
							else
								[~, tempgrad, temphess] = fun(x_0(:, 1), varargin{:}); %#ok<ASGLU> call with three output arguments to check if the third one is present and catch error if not
								hasobjectivegradient = true;
								hasobjectivehessian = true;
								Jfun = @objectivehessianvararg;
							end
						catch e
							if strcmpi(e.identifier, 'MATLAB:maxlhs') || strcmpi(e.identifier, 'MATLAB:TooManyOutputs') || strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
								if usedefaultoption
									options.SpecifyObjectiveHessian = false;
								end
								if nargin(fun) == 1
									[~, tempgrad] = fun(x_0(:, 1)); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
									hasobjectivegradient = true;
									Jfun = fun;
								else
									[~, tempgrad] = fun(x_0(:, 1), varargin{:}); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
									hasobjectivegradient = true;
									Jfun = @objectivegradientvararg;
								end
							else
								rethrow(e);
							end
						end
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
					try
						if nargin(fun) == 1
							[~, tempgrad] = fun(x_0(:, 1)); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
							hasobjectivegradient = true;
							Jfun = fun;
						else
							[~, tempgrad] = fun(x_0(:, 1), varargin{:}); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
							hasobjectivegradient = true;
							Jfun = @objectivegradientvararg;
						end
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
					options.SpecifyObjectiveHessian = false;
				end
			end
		end
		if ~isemptynonlcon
			if nargout(nonlcon) >= 6
				if nargin(nonlcon) == 1
					cfun = nonlcon;
				else
					cfun = @constrhessfun;
				end
				hasconstraintgradient = true;
				hasconstrainthessian = true;
			elseif nargout(nonlcon) >= 4
				if nargin(nonlcon) == 1
					cfun = nonlcon;
				else
					cfun = @constrgradfun;
				end
				hasconstraintgradient = true;
			else
				if nargout(nonlcon) == -1
					if needshessian
						try
							try
								if nargin(nonlcon) == 1
									[~, ~, tempgradc, tempgradceq, temphessc, temphessceq] = nonlcon(x_0(:, 1)); %#ok<ASGLU> call with six output arguments to check if the fifth and sixth one are present and catch error if not
									hasconstraintgradient = true;
									hasconstrainthessian = true;
									isnargoutconstraintfunwithgradient = true;
									cfun = nonlcon;
								else
									[~, ~, tempgradc, tempgradceq, temphessc, temphessceq] = nonlcon(x_0(:, 1), varargin{:}); %#ok<ASGLU> call with six output arguments to check if the fifth and sixth one are present and catch error if not
									hasconstraintgradient = true;
									hasconstrainthessian = true;
									isnargoutconstraintfunwithgradient = true;
									cfun = @constrhessfun;
								end
							catch e
								if strcmpi(e.identifier, 'MATLAB:maxlhs') || strcmpi(e.identifier, 'MATLAB:TooManyOutputs') || strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
									if nargin(nonlcon) == 1
										[~, ~, tempgradc, tempgradceq] = nonlcon(x_0(:, 1)); %#ok<ASGLU> call with four output arguments to check if the third and fourth one are present and catch error if not
										hasconstraintgradient = true;
										isnargoutconstraintfunwithgradient = true;
										cfun = nonlcon;
									else
										[~, ~, tempgradc, tempgradceq] = nonlcon(x_0(:, 1), varargin{:}); %#ok<ASGLU> call with three output arguments to check if the third and fourth one are present and catch error if not
										hasconstraintgradient = true;
										isnargoutconstraintfunwithgradient = true;
										cfun = @constrgradfun;
									end
								else
									rethrow(e);
								end
							end
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
						try
							if nargin(nonlcon) == 1
								[~, ~, tempgradc, tempgradceq] = nonlcon(x_0(:, 1)); %#ok<ASGLU> call with four output arguments to check if the third and fourth one are present and catch error if not
								hasconstraintgradient = true;
								isnargoutconstraintfunwithgradient = true;
								cfun = nonlcon;
							else
								[~, ~, tempgradc, tempgradceq] = nonlcon(x_0(:, 1), varargin{:}); %#ok<ASGLU> call with three output arguments to check if the third and fourth one are present and catch error if not
								hasconstraintgradient = true;
								isnargoutconstraintfunwithgradient = true;
								cfun = @constrgradfun;
							end
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
						options.SpecifyConstraintHessian = false;
					end
				end
			end
		else
			cfun = nonlcon;
			hasconstraintgradient = false;
			hasconstrainthessian = false;
		end
	else
		if nargout(fun) >= 3
			hasobjectivegradient = true;
			hasobjectivehessian = true;
		elseif nargout(fun) >= 2
			hasobjectivegradient = true;
		else
			if nargout(fun) == -1
				if needshessian
					try
						try
							[~, tempgrad, temphess] = fun(x_0(:, 1)); %#ok<ASGLU> call with three output arguments to check if the third one is present and catch error if not
							hasobjectivegradient = true;
							hasobjectivehessian = true;
						catch e
							if strcmpi(e.identifier, 'MATLAB:maxlhs') || strcmpi(e.identifier, 'MATLAB:TooManyOutputs') || strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
								[~, tempgrad] = fun(x_0(:, 1)); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
								hasobjectivegradient = true;
							else
								rethrow(e);
							end
						end
					catch e
						if strcmpi(e.identifier, 'MATLAB:maxlhs') || strcmpi(e.identifier, 'MATLAB:TooManyOutputs') || strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
							hasobjectivegradient = false;
							if usedefaultoption
								options.SpecifyObjectiveGradient = false;
								options.SpecifyObjectiveHessian = false;
							end
						else
							rethrow(e);
						end
					end
				else
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
				end
			else
				hasobjectivegradient = false;
				if usedefaultoption
					options.SpecifyObjectiveGradient = false;
					options.SpecifyObjectiveHessian = false;
				end
			end
		end
		Jfun = fun;
		if nargout(nonlcon) >= 6
			hasconstraintgradient = true;
			hasconstrainthessian = true;
		elseif nargout(nonlcon) >= 3
			hasconstraintgradient = true;
		else
			if nargout(nonlcon) == -1
				if needshessian
					try
						try
							[~, ~, tempgradc, tempgradceq, temphessc, temphessceq] = nonlcon(x_0(:, 1)); %#ok<ASGLU> call with six output arguments to check if the fifth and sixth one are present and catch error if not
							hasconstraintgradient = true;
							hasconstrainthessian = true;
							isnargoutconstraintfunwithgradient = true;
						catch e
							if strcmpi(e.identifier, 'MATLAB:maxlhs') || strcmpi(e.identifier, 'MATLAB:TooManyOutputs') || strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
								[~, ~, tempgradc, tempgradceq] = nonlcon(x_0(:, 1)); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
								hasconstraintgradient = true;
								isnargoutconstraintfunwithgradient = true;
							else
								rethrow(e);
							end
						end
					catch e
						if strcmpi(e.identifier, 'MATLAB:maxlhs') || strcmpi(e.identifier, 'MATLAB:TooManyOutputs') || strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
							hasconstraintgradient = false;
							if usedefaultoption
								options.SpecifyConstraintGradient = false;
								options.SpecifyConstraintHessian = false;
							end
						else
							rethrow(e);
						end
					end
				else
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
				end
			else
				hasconstraintgradient = false;
				if usedefaultoption
					options.SpecifyConstraintGradient = false;
					options.SpecifyConstraintHessian = false;
				end
			end
		end
		cfun = nonlcon;
	end
	hasconstraintgradient = hasconstraintgradient & (nargout(cfun) >= 4 || (nargout(cfun) == -1 && isnargoutconstraintfunwithgradient));
	if needshessian && (~hasobjectivehessian || (~hasconstrainthessian && ~isemptynonlcon))
		error('optimization:solver:ipopt:input', 'User supplied objective and constraint function does not return hessian, but calculation of hessian matrix is requested.');
	end
	model = struct(...
		'f',			Jfun,...
		'c',			cfun,...
		'A',			A,...
		'b',			b,...
		'Aeq',			Aeq,...
		'beq',			beq,...
		'gradfun',		hasobjectivegradient && options.SpecifyObjectiveGradient,...
		'gradcon',		hasconstraintgradient && options.SpecifyConstraintGradient,...
		'hessfun',		hasobjectivehessian && options.SpecifyObjectiveHessian,...
		'hesscon',		hasconstrainthessian && options.SpecifyConstraintHessian,...
		'emptynonlcon',	isemptynonlcon...
	);
	% TODO: remove error after implementation of finite difference gradient in private/callback
	if ~model.gradfun || (~isemptynonlcon && ~model.gradcon)
		error('optimization:solver:ipopt:input', 'User supplied objective function does not return gradient, but IPOPT needs gradient information.');
	end
	if ~isemptynonlcon && ~model.gradcon
		error('optimization:solver:ipopt:input', 'User supplied constraint function does not return gradient, but IPOPT needs gradient information.');
	end
	if model.hessfun && model.hesscon
		modelfunctions = struct(...
			'objective',			@callback_objective,...
			'gradient',				@callback_objective_gradient,...
			'constraints',			@callback_constraint,...
			'jacobian',				@callback_constraint_gradient,...
			'jacobianstructure',	@callback_constraint_gradient_structure,...
			'hessian',				@callback_hessian,...
			'hessianstructure',		@callback_hessian_structure...
		);
	else
		modelfunctions = struct(...
			'objective',			@callback_objective,...
			'gradient',				@callback_objective_gradient,...
			'constraints',			@callback_constraint,...
			'jacobian',				@callback_constraint_gradient,...
			'jacobianstructure',	@callback_constraint_gradient_structure...
		);
	end
	J = @callback_objective;
	callback([], model);
	%J([], model);
	%constr([], model);
	[c, ceq] = cfun(x_0(:, 1));
	Fupp = [
		zeros(size(c, 1), 1);
		zeros(size(ceq, 1), 1);
		zeros(size(model.b, 1), 1);
		zeros(size(model.beq, 1), 1)
	];
	Flow = [
		-Inf*ones(size(c, 1), 1);
		zeros(size(ceq, 1), 1);
		-Inf*ones(size(model.b, 1), 1);
		zeros(size(model.beq, 1), 1)
	];
	if isempty(Flow)
		Flow = [];
		Fupp = [];
	end
	if any(lb > ub)
		x = NaN(size(x_0, 1), 1);
		if nargout >= 2
			fval = NaN;
		end
		if nargout >= 3
			exitflag = -1;
		end
		if nargout >= 4
			output = struct(...
				'iterations',		NaN,...
				'funcCount',		NaN,...
				'lssteplength',		NaN,...
				'constrviolation',	NaN,...
				'stepsize',			NaN,...
				'algorithm',		'IPOPT',...
				'cgiterations',		NaN,...
				'firstorderopt',	NaN,...
				'message',			'Lower bound is larger than upper bound.'...
			);
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
			grad = NaN(size(x_0, 1), 1);
		end
		if nargout >= 7
			hessian = NaN(size(x_0, 1), size(x_0, 1));
		end
		return;
	end
	solveroptions = struct(...
		'lb',	lb,...
		'ub',	ub,...
		'cl',	Flow,...
		'cu',	Fupp...
	);
	options.NumberVariables = size(x_0, 1);
	options.NumberConstraintsInequality = size(A, 1) + size(c, 1);
	options.NumberConstraintsEquality = size(Aeq, 1) + size(ceq, 1);
	options.NumberConstraintsBounds = sum(~isinf(lb)) + sum(~isinf(ub));
	solveroptions.ipopt = options.getpreferred();
	solveroptions.ipopt.print_level = str2double(solveroptions.ipopt.Display);
	solveroptions.ipopt = rmfield(solveroptions.ipopt, 'Display');
	ipoptnames = fieldnames(solveroptions.ipopt);
	removefields = false(size(ipoptnames, 1), 1);
	for ii = 1:length(ipoptnames) %#ok<FORPF> no parfor for call to mex interface
		if ~isempty(solveroptions.ipopt.(ipoptnames{ii}))
			if islogical(solveroptions.ipopt.(ipoptnames{ii}))
				solveroptions.ipopt.(ipoptnames{ii}) = logicaltoyesno(solveroptions.ipopt.(ipoptnames{ii}));
			end
		else
			removefields(ii, 1) = true;
		end
	end
	solveroptions.ipopt = rmfield(solveroptions.ipopt, ipoptnames(removefields));
	% TODO: use HessianFcn etc. if supplied
	if ~isfield(solveroptions.ipopt, 'linear_solver')
		solveroptions.iptopt.linear_solver = 'ma97';
	end
	if model.hessfun && model.hesscon
		solveroptions.ipopt.hessian_approximation = 'exact';
	else
		solveroptions.ipopt.hessian_approximation = 'limited-memory';
	end
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
	useparallel = options.UseParallel;
	nargout3 = nargout >= 3;
	nargout4 = nargout >= 4;
	nargout5 = nargout >= 5;
	if nargout4
		alloutputs = cell(max(1, options.Retries), initialvalues);
	end
	while retry < max(1, options.Retries)
		x_opt = NaN(size(x_0));
		f_opt = NaN(1, initialvalues);
		infeas_opt = NaN(1, initialvalues);
		iterations = zeros(1, initialvalues);
		funevals = zeros(1, initialvalues);
		info = cell(1, initialvalues);
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
				% initialize model structure in every parfor-worker
				callback([], model);
				try
					[x_opt(:, ii), info{1, ii}] = ipopt(x_0(:, ii), modelfunctions, solveroptions);
					if nargout3
						exitflag(1, ii) = info{1, ii}.status;
					end
					f_opt(1, ii) = J(x_opt(:, ii));
					if nargout5
						if isempty(info{1, ii}.zl)
							zl = NaN(size(lb, 1), 1);
						else
							zl = info{1, ii}.zl;
						end
						if isempty(info{1, ii}.zu)
							zu = NaN(size(ub, 1), 1);
						else
							zu = info{1, ii}.zu;
						end
						if isempty(info{1, ii}.lambda)
							ineqnonlin = NaN(size(c, 1), 1);
							eqnonlin = NaN(size(ceq, 1), 1);
							ineqlin = NaN(size(b, 1), 1);
							eqlin = NaN(size(b, 1), 1);
						else
							ineqnonlin = info{1, ii}.lambda(1:size(c, 1));
							eqnonlin = info{1, ii}.lambda(size(c, 1) + 1:size(c, 1) + size(ceq, 1));
							ineqlin = info{1, ii}.lambda(size(c, 1) + size(ceq, 1) + 1:size(c, 1) + size(ceq, 1) + size(A, 1));
							eqlin = info{1, ii}.lambda(size(c, 1) + size(ceq, 1) + size(A, 1) + 1:size(c, 1) + size(ceq, 1) + size(A, 1) + size(Aeq, 1));
						end
						lambda{1, ii} = struct(...
							'lower',		zl,...
							'upper',		zu,...
							'ineqnonlin',	ineqnonlin,...
							'eqnonlin',		eqnonlin,...
							'ineqlin',		ineqlin,...
							'eqlin',		eqlin...
						);
					end
				catch e
					rethrow(e);
				end
				if ~isempty(nonlcon)
					if nargin(cfun) == 1
						[ccurrent, ceqcurrent] = cfun(x_opt(:, ii));
					else
						[ccurrent, ceqcurrent] = cfun(x_opt(:, ii), varargin{:});
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
					iterations(1, ii) = info{1, ii}.iter;
					funevals(1, ii) = info{1, ii}.eval.objective;
					outstr{1, ii} = ipopt_status2message(info{1, ii}.status);
					tempoutputs{1, ii} = options.formatOutput(exitflag(1, ii), toc(solvertimes(1, ii)), x_opt(:, ii), f_opt(1, ii), numel(x_0(:, ii)), iterations(1, ii), funevals(1, ii), retry, struct(...
						'iter',				info{1, ii}.iter,...
						'funevals',			info{1, ii}.eval.objective,...
						'infeas',			NaN,...
						'primalinfeas',		NaN,...
						'fval',				f_opt(1, ii),...
						'constrviol',		infeas_opt(1, ii),...
						'outputstr',		outstr{1, ii},...
						'time',				info{1, ii}.cpu...
					)); %#ok<PFBNS> options is an object
				end
			end
		else
			for ii = 1:initialvalues %#ok<FORPF> parallel computing is disabled through option set by user
				solvertimes(1, ii) = tic;
				try
					[x_opt(:, ii), info{1, ii}] = ipopt(x_0(:, ii), modelfunctions, solveroptions);
					if nargout3
						exitflag(1, ii) = info{1, ii}.status;
					end
					f_opt(1, ii) = J(x_opt(:, ii));
					if nargout5
						if isempty(info{1, ii}.zl)
							zl = NaN(size(lb, 1), 1);
						else
							zl = info{1, ii}.zl;
						end
						if isempty(info{1, ii}.zu)
							zu = NaN(size(ub, 1), 1);
						else
							zu = info{1, ii}.zu;
						end
						if isempty(info{1, ii}.lambda)
							ineqnonlin = NaN(size(c, 1), 1);
							eqnonlin = NaN(size(ceq, 1), 1);
							ineqlin = NaN(size(b, 1), 1);
							eqlin = NaN(size(b, 1), 1);
						else
							ineqnonlin = info{1, ii}.lambda(1:size(c, 1));
							eqnonlin = info{1, ii}.lambda(size(c, 1) + 1:size(c, 1) + size(ceq, 1));
							ineqlin = info{1, ii}.lambda(size(c, 1) + size(ceq, 1) + 1:size(c, 1) + size(ceq, 1) + size(A, 1));
							eqlin = info{1, ii}.lambda(size(c, 1) + size(ceq, 1) + size(A, 1) + 1:size(c, 1) + size(ceq, 1) + size(A, 1) + size(Aeq, 1));
						end
						lambda{1, ii} = struct(...
							'lower',		zl,...
							'upper',		zu,...
							'ineqnonlin',	ineqnonlin,...
							'eqnonlin',		eqnonlin,...
							'ineqlin',		ineqlin,...
							'eqlin',		eqlin...
						);
					end
				catch e
					rethrow(e);
				end
				if ~isempty(nonlcon)
					if nargin(cfun) == 1
						[ccurrent, ceqcurrent] = cfun(x_opt(:, ii));
					else
						[ccurrent, ceqcurrent] = cfun(x_opt(:, ii), varargin{:});
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
					iterations(1, ii) = info{1, ii}.iter;
					funevals(1, ii) = info{1, ii}.eval.objective;
					outstr{1, ii} = ipopt_status2message(info{1, ii}.status);
					tempoutputs{1, ii} = options.formatOutput(exitflag(1, ii), toc(solvertimes(1, ii)), x_opt(:, ii), f_opt(1, ii), numel(x_0(:, ii)), iterations(1, ii), funevals(1, ii), retry, struct(...
						'iter',				info{1, ii}.iter,...
						'funevals',			info{1, ii}.eval.objective,...
						'infeas',			NaN,...
						'primalinfeas',		NaN,...
						'fval',				f_opt(1, ii),...
						'constrviol',		infeas_opt(1, ii),...
						'outputstr',		outstr{1, ii},...
						'time',				info{1, ii}.cpu...
					));
				end
			end
		end
		if nargout4
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
		error('optimization:solver:ipopt:solution', 'No solution could be found for any initial value.');
	end
	idx = feasidx(fvalidx);
	if isempty(idx)
		error('optimization:solver:ipopt:solution', 'No solution could be found for any initial value.');
	end
	x = x_opt(:, idx);
	if nargout3
		exitflag = exitflag(1, idx);
	end
	info = info{1, idx};
	if nargout5
		lambda = lambda{1, idx};
	end
	if nargout >= 6
		if model.gradfun
			grad = callback_objective_gradient(x).';
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
		if model.hessfun && (model.hesscon || model.emptynonlcon)
			hessian = full(callback_hessian(x, 1, info.lambda));
			hessian = hessian + tril(hessian, -1).';
		else
			% TODO: calculate finite difference gradient from gradient or two times from function values
			hessian = NaN(size(x_0, 1), size(x_0, 1));
		end
	end
	solvertime = toc(solvertime);
	if nargout >= 4
		if strcmpi(outputstr, '')
			outputstr = outstr{1, idx};
		end
		alloutputs(retry + 1:end, :) = [];
		output = options.formatOutput(exitflag, solvertime, x, fval, numel(x_0(:, 1)), retryiterations, retryfunevals, retry, struct(...
			'iter',				info.iter,...
			'funevals',			info.eval.objective,...
			'infeas',			NaN,...
			'primalinfeas',		NaN,...
			'fval',				fval,...
			'constrviol',		infeas_opt(1, idx),...
			'outputstr',		outputstr,...
			'time',				info.cpu...
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
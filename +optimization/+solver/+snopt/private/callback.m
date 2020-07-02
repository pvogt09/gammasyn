function [fun, grad] = callback(x, optimmodel)
	%CALLBACK callback function for one function handle callback type of snopt
	%	Input:
	%		x:			current optimization variable
	%		optimmodel:	structure with problem functions and matrices (only used once to initialize problem)
	%	Output:
	%		fun:		function value of objective function and constraint functions
	%		grad:		gradient of objective function and constraint functions
	persistent model;
	if nargin > 1
		if ~isfield(optimmodel, 'f') || ~isfield(optimmodel, 'gradfun')
			error('optimization:solver:snopt:callback', 'Optimization problem is defined incorrectly.')
		end
		if ischar(optimmodel.f)
			optimmodel.f = str2func(optimmodel.f);
		end
		if ~isfunctionhandle(optimmodel.f)
			error('optimization:solver:snopt:callback', 'Callback function must be a function handle, not a %s.', class(optimmodel.f));
		end
		if ~isfield(optimmodel, 'c') || ~isfield(optimmodel, 'gradcon')
			error('optimization:solver:snopt:callback', 'Optimization problem is defined incorrectly.')
		end
		if ischar(optimmodel.c)
			optimmodel.f = str2func(optimmodel.c);
		end
		if ~isfunctionhandle(optimmodel.c)
			error('optimization:solver:snopt:callback', 'Constraint function must be a function handle, not a %s.', class(optimmodel.c));
		end
		if ~isfield(optimmodel, 'A') || ~isfield(optimmodel, 'b')
			error('optimization:solver:snopt:callback', 'Optimization problem is defined incorrectly, A and b are missing.')
		end
		if ~isfield(optimmodel, 'Aeq') || ~isfield(optimmodel, 'beq')
			error('optimization:solver:snopt:callback', 'Optimization problem is defined incorrectly, Aeq and beq are missing.')
		end
		model = optimmodel;
		return;
	end
	fun_pointer = model.f;
	constr_pointer = model.c;
	if model.gradfun && nargout >= 2
		[J, gradJ] = fun_pointer(x);
		if ~isempty(gradJ)
			gradJ = gradJ.';
		else
			gradJ = NaN(1, size(x, 1));
		end
	else
		J = fun_pointer(x);
		if nargout >= 2
			gradJ = NaN(1, size(x, 1));
		end
	end
	if model.gradcon && nargout >= 2
		[c, ceq, gradc, gradceq] = constr_pointer(x);
		if ~isempty(gradc)
			gradc = gradc.';
		else
			gradc = zeros(0, size(x, 1));
		end
		if ~isempty(gradceq)
			gradceq = gradceq.';
		else
			gradceq = zeros(0, size(x, 1));
		end
	else
		[c, ceq] = constr_pointer(x);
		if nargout >= 3
			if ~isempty(c)
				gradc = NaN(size(c, 1), size(x, 1));
			else
				gradc = zeros(0, size(x, 1));
			end
			if ~isempty(ceq)
				gradceq = NaN(size(ceq, 1), size(x, 1));
			else
				gradceq = zeros(0, size(x, 1));
			end
		end
	end
	fun = [
		J;
		c;
		ceq;
		zeros(size(model.A, 1), 1);
		zeros(size(model.Aeq, 1), 1)
	];
	if nargout >= 2
		grad = [
			gradJ;
			gradc;
			gradceq;
			zeros(size(model.A, 1), size(x, 1));
			zeros(size(model.Aeq, 1), size(x, 1))
		];
	end
end
function [c, ceq, gradc, gradceq] = callback_constraint(x, optimmodel)
	%CALLBACK_CONSTRAINT callback function for contraint function handle for snopt
	%	Input:
	%		x:			current optimization variable
	%		optimmodel:	structure with problem functions and matrices (only used once to initialize problem)
	%	Output:
	%		c:			inequality constraint function value
	%		ceq:		equaltity constrint function value
	%		gradc:		gradient of inequality constraint
	%		gradceq:	gradient of equality constraint
	persistent model;
	if nargin > 1
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
	constr_pointer = model.c;
	if model.gradcon && nargout >= 3
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
end
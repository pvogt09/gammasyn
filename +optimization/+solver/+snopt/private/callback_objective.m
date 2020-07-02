function [J, gradJ] = callback_objective(x, optimmodel)
	%CALLBACK_OBJECTIVE callback function for objective function handle for snopt
	%	Input:
	%		x:			current optimization variable
	%		optimmodel:	structure with problem functions and matrices (only used once to initialize problem)
	%	Output:
	%		J:			function value of objective function
	%		gradJ:		gradient of objective function
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
		model = optimmodel;
		return;
	end
	fun_pointer = model.f;
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
end
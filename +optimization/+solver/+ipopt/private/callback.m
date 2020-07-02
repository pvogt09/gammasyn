function [fun] = callback(x, optimmodel, sigma, lambda)
	%CALLBACK callback function for one function handle callback type of ipopt
	%	Input:
	%		x:			current optimization variable
	%		optimmodel:	structure with problem functions and matrices (only used once to initialize problem)
	%	Output:
	%		fun:		function value of objective function and constraint functions
	persistent model;
	if nargin > 1 && ~ischar(optimmodel)
		if ~isfield(optimmodel, 'f') || ~isfield(optimmodel, 'gradfun') || ~isfield(optimmodel, 'hessfun')
			error('optimization:solver:ipopt:callback', 'Optimization problem is defined incorrectly.')
		end
		if ischar(optimmodel.f)
			optimmodel.f = str2func(optimmodel.f);
		end
		if ~isfunctionhandle(optimmodel.f)
			error('optimization:solver:ipopt:callback', 'Callback function must be a function handle, not a %s.', class(optimmodel.f));
		end
		if ~isfield(optimmodel, 'c') || ~isfield(optimmodel, 'gradcon') || ~isfield(optimmodel, 'hesscon')
			error('optimization:solver:ipopt:callback', 'Optimization problem is defined incorrectly.')
		end
		if ischar(optimmodel.c)
			optimmodel.f = str2func(optimmodel.c);
		end
		if ~isfunctionhandle(optimmodel.c)
			error('optimization:solver:ipopt:callback', 'Constraint function must be a function handle, not a %s.', class(optimmodel.c));
		end
		if ~isfield(optimmodel, 'A') || ~isfield(optimmodel, 'b')
			error('optimization:solver:ipopt:callback', 'Optimization problem is defined incorrectly, A and b are missing.')
		end
		if ~isfield(optimmodel, 'Aeq') || ~isfield(optimmodel, 'beq')
			error('optimization:solver:ipopt:callback', 'Optimization problem is defined incorrectly, Aeq and beq are missing.')
		end
		if ~isfield(optimmodel, 'emptynonlcon')
			error('optimization:solver:ipopt:callback', 'Optimization problem is defined incorrectly, emptynonlcon is missing.')
		end
		model = optimmodel;
		return;
	elseif nargin <= 1
		optimmodel = 'fun';
	elseif nargin > 1 && ischar(optimmodel)
	else
		error('optimization:solver:ipopt:callback', 'Optimization problem is defined incorrectly, only ''fun'', ''con'', ''gradfun'', ''gradcon'', ''gradconstruct'', ''hess'' and ''hessstruct'' are allowed.');
	end
	fun = strcmpi(optimmodel, 'fun');
	con = strcmpi(optimmodel, 'con');
	gradfun = strcmpi(optimmodel, 'gradfun');
	gradcon = strcmpi(optimmodel, 'gradcon');
	gradconstruct = strcmpi(optimmodel, 'gradconstruct');
	hess = strcmpi(optimmodel, 'hess');
	hessstruct = strcmpi(optimmodel, 'hessstruct');
	if fun || gradfun
		fun_pointer = model.f;
		if model.gradfun && (nargout >= 2 || gradfun)
			[J, gradJ] = fun_pointer(x);
			if ~isempty(gradJ)
				gradJ = gradJ.';
			else
				gradJ = NaN(1, size(x, 1));
			end
		else
			J = fun_pointer(x);
			if nargout >= 2 || gradfun
				gradJ = NaN(1, size(x, 1));
			end
		end
		if gradfun
			fun = gradJ;
		else
			fun = J;
		end
	elseif con || gradcon || gradconstruct
		constr_pointer = model.c;
		if gradconstruct && isempty(x)
			x = NaN(size(model.A, 2), 1);
		end
		if model.gradcon && (nargout >= 2 || gradcon || gradconstruct)
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
			if nargout >= 2 || gradcon || gradconstruct
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
		if gradcon
			fun = [
				gradc;
				gradceq;
				model.A;
				model.Aeq
			];
		elseif gradconstruct
			fun = sparse([
				ones(size(gradc));
				ones(size(gradceq));
				model.A;
				model.Aeq
			]);
		else
			fun = [
				c;
				ceq;
				model.A*x - model.b;
				model.Aeq*x - model.beq
			];
		end
	elseif hess || hessstruct
		fun_pointer = model.f;
		constr_pointer = model.c;
		if hessstruct && isempty(x)
			x = NaN(size(model.A, 2), 1);
		end
		if model.hessfun
			[~, ~, H_objective] = fun_pointer(x);
			if isempty(H_objective)
				H_objective = NaN(size(x, 1), size(x, 1));
			end
		else
			error('optimization:solver:ipopt:callback', 'Hessian information not supplied by objective function handle.');
		end
		if hess && (nargin <= 2 || nargin <= 3)
			error('optimization:solver:ipopt:callback', 'Lagrange multipliers have to be supplied for hessian calculation.');
		end
		H_constraint = zeros(size(x, 1), size(x, 1));
		if hessstruct || (nargin >= 4 && ~isempty(lambda))
			if model.hesscon
				[~, ~, ~, ~, hessianc, hessianceq] = constr_pointer(x);
				if isempty(hessianc)
					hessianc = zeros(size(x, 1), size(x, 1), 0);
				end
				if isempty(hessianceq)
					hessianceq = zeros(size(x, 1), size(x, 1), 0);
				end
				if hessstruct && nargin <= 3
					lambda = ones(size(hessianc, 3) + size(hessianceq, 3), 1);
				end
				if hessstruct && nargin <= 2
					sigma = 1;
				end
				for ii = 1:size(hessianc, 3) %#ok<FORPF> no parfor because of dependent iterations
					H_constraint = H_constraint + hessianc(:, :, ii)*lambda(ii, 1);
				end
				for jj = 1:size(hessianceq, 3) %#ok<FORPF> no parfor because of dependent iterations
					H_constraint = H_constraint + hessianceq(:, :, jj)*lambda(ii + jj, 1);
				end
				% no hessian for linear constraints
			elseif model.emptynonlcon
			else
				error('optimization:solver:ipopt:callback', 'Hessian information not supplied by constraint function handle.');
			end
		end
		H = sigma*H_objective + H_constraint;
		if hessstruct
			fun = sparse(tril(real(double(isnan(H) | ~isnan(H)))));
		else
			fun = sparse(tril(H));
		end
	else
		error('optimization:solver:ipopt:callback', 'Optimization problem is defined incorrectly, only ''fun'', ''con'', ''gradfun'', ''gradcon'', ''gradconstruct'', ''hess'' and ''hessstruct'' are allowed.');
	end
end
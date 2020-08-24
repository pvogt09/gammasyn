function [fun] = callback(x, optimmodel, sigma, lambda)
	%CALLBACK callback function for one function handle callback type of ipopt
	%	Input:
	%		x:			current optimization variable
	%		optimmodel:	structure with problem functions and matrices (only used once to initialize problem)
	%	Output:
	%		fun:		function value of objective function and constraint functions
	persistent model;
	persistent last_f;
	persistent last_gradf;
	persistent last_hessf;
	persistent last_c;
	persistent last_gradc;
	persistent last_hessc;
	persistent last_ceq;
	persistent last_gradceq;
	persistent last_hessceq;
	persistent last_x_f;
	persistent last_x_c;
	persistent last_x_hess;
	persistent initialize_f;
	persistent initialize_gradf;
	persistent initialize_c;
	persistent initialize_gradc;
	persistent initialize_hessf;
	persistent initialize_hessc;
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
		last_f = [];
		last_gradf = [];
		last_c = [];
		last_gradc = [];
		last_ceq = [];
		last_gradceq = [];
		last_x_f = [];
		last_x_c = [];
		last_x_hess = [];
		initialize_f = false;
		initialize_gradf = false;
		initialize_c = false;
		initialize_gradc = false;
		initialize_hessf = false;
		initialize_hessc = false;
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
			if isempty(last_x_f) || ~initialize_gradf || ~isequal(x, last_x_f)
				[J, gradJ] = fun_pointer(x);
				initialize_f = true;
				initialize_gradf = true;
				last_f = J;
				last_gradf = gradJ;
			else
				J = last_f;
				gradJ = last_gradf;
			end
			if ~isempty(gradJ)
				gradJ = gradJ.';
			else
				gradJ = NaN(1, size(x, 1));
			end
		else
			if isempty(last_x_f) || (~initialize_f && ~initialize_gradf) || ~isequal(x, last_x_f)
				J = fun_pointer(x);
				initialize_f = true;
				initialize_gradf = false;
				last_f = J;
			else
				J = last_f;
			end
			if nargout >= 2 || gradfun
				gradJ = NaN(1, size(x, 1));
			end
		end
		if gradfun
			fun = gradJ;
		else
			fun = J;
		end
		last_x_f = x;
	elseif con || gradcon || gradconstruct
		constr_pointer = model.c;
		if gradconstruct && isempty(x)
			x = NaN(size(model.A, 2), 1);
		end
		if model.gradcon && (nargout >= 2 || gradcon || gradconstruct)
			if isempty(last_x_c) || ~initialize_gradc || ~isequal(x, last_x_c)
				[c, ceq, gradc, gradceq] = constr_pointer(x);
				initialize_c = true;
				initialize_gradc = true;
				last_c = c;
				last_gradc = gradc;
				last_ceq = ceq;
				last_gradceq = gradceq;
			else
				c = last_c;
				gradc = last_gradc;
				ceq = last_ceq;
				gradceq = last_gradceq;
			end
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
			if isempty(last_x_c) || (~initialize_c && ~initialize_gradc) || ~isequal(x, last_x_c)
				[c, ceq] = constr_pointer(x);
				initialize_c = true;
				initialize_gradc = false;
				last_c = c;
				last_ceq = ceq;
			else
				c = last_c;
				ceq = last_ceq;
			end
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
		last_x_c = x;
	elseif hess || hessstruct
		fun_pointer = model.f;
		constr_pointer = model.c;
		if hessstruct && isempty(x)
			x = NaN(size(model.A, 2), 1);
		end
		if model.hessfun
			if isempty(last_x_hess) || ~initialize_hessf || ~isequal(x, last_x_hess)
				[~, ~, H_objective] = fun_pointer(x);
				initialize_hessf = true;
				last_hessf = H_objective;
			else
				H_objective = last_hessf;
			end
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
				if isempty(last_x_hess) || ~initialize_hessc || ~isequal(x, last_x_hess)
					[~, ~, ~, ~, hessianc, hessianceq] = constr_pointer(x);
					initialize_hessc = true;
					last_hessc = hessianc;
					last_hessceq = hessianceq;
				else
					hessianc = last_hessc;
					hessianceq = last_hessceq;
				end
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
		last_x_hess = x;
	else
		error('optimization:solver:ipopt:callback', 'Optimization problem is defined incorrectly, only ''fun'', ''con'', ''gradfun'', ''gradcon'', ''gradconstruct'', ''hess'' and ''hessstruct'' are allowed.');
	end
end
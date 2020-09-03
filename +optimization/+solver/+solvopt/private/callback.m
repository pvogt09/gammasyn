function [fun] = callback(x, optimmodel)
	%CALLBACK callback function for one function handle callback type of solvopt
	%	Input:
	%		x:			current optimization variable
	%		optimmodel:	structure with problem functions and matrices (only used once to initialize problem)
	%	Output:
	%		fun:		function value of objective function and constraint functions
	persistent model;
	persistent last_f;
	persistent last_gradf;
	persistent last_c;
	persistent last_gradc;
	persistent last_ceq;
	persistent last_gradceq;
	persistent last_x_f;
	persistent last_x_c;
	persistent initialize_f;
	persistent initialize_gradf;
	persistent initialize_c;
	persistent initialize_gradc;
	if nargin > 1 && ~ischar(optimmodel)
		if ~isfield(optimmodel, 'f') || ~isfield(optimmodel, 'gradfun')
			error('optimization:solver:solvopt:callback', 'Optimization problem is defined incorrectly.')
		end
		if ischar(optimmodel.f)
			optimmodel.f = str2func(optimmodel.f);
		end
		if ~isfunctionhandle(optimmodel.f)
			error('optimization:solver:solvopt:callback', 'Callback function must be a function handle, not a %s.', class(optimmodel.f));
		end
		if ~isfield(optimmodel, 'c') || ~isfield(optimmodel, 'gradcon')
			error('optimization:solver:solvopt:callback', 'Optimization problem is defined incorrectly.')
		end
		if ischar(optimmodel.c)
			optimmodel.f = str2func(optimmodel.c);
		end
		if ~isfunctionhandle(optimmodel.c)
			error('optimization:solver:solvopt:callback', 'Constraint function must be a function handle, not a %s.', class(optimmodel.c));
		end
		if ~isfield(optimmodel, 'A') || ~isfield(optimmodel, 'b')
			error('optimization:solver:solvopt:callback', 'Optimization problem is defined incorrectly, A and b are missing.')
		end
		if ~isfield(optimmodel, 'Aeq') || ~isfield(optimmodel, 'beq')
			error('optimization:solver:solvopt:callback', 'Optimization problem is defined incorrectly, Aeq and beq are missing.')
		end
		if ~isfield(optimmodel, 'lb') || ~isfield(optimmodel, 'ub')
			error('optimization:solver:solvopt:callback', 'Optimization problem is defined incorrectly, lb and ub are missing.')
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
		initialize_f = false;
		initialize_gradf = false;
		initialize_c = false;
		initialize_gradc = false;
		return;
	elseif nargin <= 1
		optimmodel = 'fun';
	elseif nargin > 1 && ischar(optimmodel)
	else
		error('optimization:solver:solvopt:callback', 'Optimization problem is defined incorrectly, only ''fun'', ''con'', ''gradfun'' and ''gradcon'' are allowed.');
	end
	fun = strcmpi(optimmodel, 'fun');
	con = strcmpi(optimmodel, 'con');
	gradfun = strcmpi(optimmodel, 'gradfun');
	gradcon = strcmpi(optimmodel, 'gradcon');
	x = x';
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
	elseif con || gradcon
		constr_pointer = model.c;
		if model.gradcon && (nargout >= 2 || gradcon)
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
			if nargout >= 2 || gradcon
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
		c_ineq = [
			c;
			model.A*x - model.b;
			model.lb - x;
			x - model.ub
		];
		c_ineq = max([
			c_ineq, zeros(size(c_ineq, 1), 1)
		], [], 2);
		c_eq = abs([
			ceq;
			model.Aeq*x - model.beq
		]);
		[c_max, idxc_max] = max(c_ineq);
		[ceq_max, idxceq_max] = max(c_eq);
		if gradcon
			if isempty(c_max) || isempty(ceq_max)
				if isempty(c_max) && isempty(ceq_max)
					fun = gradc;
				elseif isempty(ceq_max)
					gradc_ineq = [
						gradc;
						model.A;
						-eye(size(x, 1));
						eye(size(x, 1))
					];
					fun = gradc_ineq(idxc_max, :);
				else
					gradc_eq = [
						gradceq;
						model.Aeq
					];
					fun = gradc_eq(idxceq_max, :);
				end
			elseif c_max > ceq_max
				gradc_ineq = [
					gradc;
					model.A;
					-eye(size(x, 1));
					eye(size(x, 1))
				];
				fun = gradc_ineq(idxc_max, :);
			else
				gradc_eq = [
					gradceq;
					model.Aeq
				];
				fun = gradc_eq(idxceq_max, :);
			end
		else
			fun = max([c_max, ceq_max]);
		end
		last_x_c = x;
	else
		error('optimization:solver:solvopt:callback', 'Optimization problem is defined incorrectly, only ''fun'', ''con'', ''gradfun'' and ''gradcon'' are allowed.');
	end
end
function [F_opt, info] = prefilter(R_opt, C_stat, systems, T, F_fixed, options, F_bounds, F_nonlin)
	%PREFILTER calculate constrained prefilter
	%	Input:
	%		R_opt:		optimal gain matrix for closing control loop with
	%		C_stat:		matrix for choosing the stationary outputs from all outputs of the system
	%		systems:	systems to take into consideration for prefilter calculation
	%		T:			sampling time for the systems
	%		F_fixed:	cell array with indicator matrix for prefilter elements that should be fixed and the values the fixed elements have, empty if no fixed elements are needed. For linear dependent prefilter components the first element is a 3D matrix with relation in the third dimension and the second element a vector of constant constraint elements
	%		options:	structure with options
	%		F_bounds:	cell array with indicator matrix for prefilter elements that should be bounded and the values the elements are bounded by, empty if no bounded elements are needed. For linear dependent prefilter components the first element is a 3D matrix with relation in the third dimension and the second element a vector of upper bound constraint elements
	%		F_nonlin:	function pointer to a function of nonlinear inequality and equality constraints on prefilter elements with signature [c_F, ceq_F, gradc_F, gradceq_F] = constraint(F)
	%	Output:
	%		F_opt:		optimal prefilter matrix for the systems
	%		info:		structure with information about the optimization
	if nargin <= 2
		T = -1;
	end
	if nargin <= 4
		F_fixed = [];
	end
	if nargin <= 5
		options = struct(...
			'weight',	1,...
			'options',	optimoptions(@lsqlin, 'Algorithm', 'interior-point', 'Display', 'iter-detailed', 'MaxIter', 1000)...
		);
	end
	if nargin <= 6
		F_bounds = [];
	end
	if nargin <= 7
		F_nonlin = [];
	end
	if ~isempty(F_nonlin)
		% TODO: implement nonlinear constraints of prefilter matrix with general constrained optimization or external least squares algorithm
		error('control:design:gamma:prefilter:dimension', 'Nonlinear prefilter constraints must not be supplied.');
	end
	if ~isstruct(options)
		if isa(options, 'optim.options.Lsqlin')
			options = struct(...
				'weight',	1,...
				'options',	options...
			);
		else
			error('control:design:gamma:prefilter:dimension', 'Options must be of type ''struct'' or ''optimoptions''.');
		end
	else
		if optimization.options.isoptimset(options)
			options = struct(...
				'weight',	1,...
				'options',	options...
			);
		end
		if ~isfield(options, 'weight')
			options.weight = 1;
		end
		if ~isfield(options, 'options')
			options.options = optimoptions(@lsqlin, 'Algorithm', 'interior-point', 'Display', 'iter-detailed', 'MaxIter', 1000);
		end
	end
	isdiscrete = control.design.outputfeedback.OutputFeedback.isdiscreteT(T);
	[system, ~, ~, number_controls, number_measurements, number_measurements_xdot, ~, descriptor] = checkandtransformsystems(systems);
	if descriptor
		% TODO: allow for descriptor systems and take derivative measurements into account
		error('control:design:gamma:prefilter:dimension', 'Prefilter can not be calculated for descriptor systems.');
	end
	if isempty(C_stat)
		C_stat = eye(number_measurements);
	end
	if ~isnumeric(C_stat)
		error('control:design:gamma:prefilter:dimension', 'Stationary measurment matrix must be numeric.');
	end
	if size(C_stat, 2) ~= number_measurements
		error('control:design:gamma:prefilter:dimension', 'Stationary measurment matrix must have %d columns.', number_measurements);
	end
	number_references = size(C_stat, 1);
	if ~isempty(F_fixed)
		if isnumeric(F_fixed)
			if ~ismatrix(F_fixed)
				if size(F_fixed, 3) > 2
					F_fixed = {F_fixed, zeros(size(F_fixed, 3), 1)};
				else
					F_fixed = {logical(F_fixed(:, :, 1)), F_fixed(:, :, 2)};
				end
			end
		elseif islogical(F_fixed)
		elseif iscell(F_fixed)
			if numel(F_fixed) >= 2
				if islogical(F_fixed{1}) && isnumeric(F_fixed{2})
				elseif islogical(F_fixed{2}) && isnumeric(F_fixed{1})
					F_fixed = F_fixed';
				else
					if ~isnumeric(F_fixed{1}) || ~isnumeric(F_fixed{2})
						error('control:design:gamma:dimension', 'Fixed proportional gain must contain a logical and a numeric matrix or a numerical constraint system, not a ''%s''.', class(F_fixed{1}));
					end
				end
			else
				error('control:design:gamma:dimension', 'Fixed proportional gain constraint system must not contain more than 2 matrices.');
			end
		else
			error('control:design:gamma:dimension', 'Fixed proportional gain positions must be two %dX%d matrices with positions and fixed values.', number_controls, number_references);
		end
		if isnumeric(F_fixed)
			if size(F_fixed, 1) ~= number_controls || size(F_fixed, 2) ~= number_references
				error('control:design:gamma:prefilter:dimension', 'Fixed prefilter values must be a %dX%d matrix.', number_controls, number_references);
			end
			F_fixed_NaN = ~isnan(F_fixed);
			rg = sum(F_fixed_NaN(:));
			if rg >= number_controls*number_references
				error('control:design:gamma:prefilter:dimension', 'At least one prefilter component must be unconstrained.');
			end
			constraint_system = zeros(rg, number_controls*number_references);
			F_fixed_row_logical = reshape(F_fixed_NaN, 1, number_controls*number_references);
			F_fixed_row = reshape(F_fixed, 1, number_controls*number_references);
			idx = find(F_fixed_row_logical);
			constraint_border = F_fixed_row(idx).';
			parfor ii = 1:size(idx, 2)
				constraint_system(ii, :) = (1:number_controls*number_references) == idx(ii);
			end
			Aeq = constraint_system;
			beq = constraint_border;
		elseif iscell(F_fixed)
			if islogical(F_fixed{1})
				if size(F_fixed{2}, 1) ~= number_controls || size(F_fixed{2}, 2) ~= number_references
					error('control:design:gamma:prefilter:dimension', 'Fixed prefilter values must be a %dX%d matrix.', number_controls, number_references);
				end
				rg = sum(F_fixed{1}(:));
				if rg >= number_controls*number_references
					error('control:design:gamma:prefilter:dimension', 'At least one prefilter component must be unconstrained.');
				end
				constraint_system = zeros(rg, number_controls*number_references);
				F_fixed_row_logical = reshape(F_fixed{1}, 1, number_controls*number_references);
				F_fixed_row = reshape(F_fixed{2}, 1, number_controls*number_references);
				idx = find(F_fixed_row_logical);
				constraint_border = F_fixed_row(idx).';
				parfor ii = 1:size(idx, 2)
					constraint_system(ii, :) = (1:number_controls*number_references) == idx(ii);
				end
			else
				if size(F_fixed{1}, 3) ~= size(F_fixed{2}, 1) || size(F_fixed{2}, 2) ~= 1
					error('control:design:gamma:prefilter:dimension', 'Fixed prefilter constraint system must be a %d vector of bounds.', size(F_fixed{1}, 3));
				end
				if size(F_fixed{1}, 3) >= number_controls*number_references
					error('control:design:gamma:prefilter:dimension', 'At least one prefilter component must be unconstrained.');
				end
				constraint_system = zeros(size(F_fixed{1}, 3), number_controls*number_references);
				temp = F_fixed{1};
				parfor ii = 1:size(temp, 3)
					constraint_system(ii, :) = reshape(temp(:, :, ii), 1, number_controls*number_references);
				end
				constraint_border = F_fixed{2};
				rg = rank(constraint_system);
				if rg ~= size(constraint_system, 1)
					error('control:design:gamma:prefilter:dimension', 'Fixed prefilter constraint system must have column rank %d.', rg);
				end
			end
				Aeq = constraint_system;
				beq = constraint_border;
		else
			error('control:design:gamma:prefilter:dimension', 'Fixed prefilter constraints must be numeric or a cell array of constraints.');
		end
	else
		Aeq = [];
		beq = [];
	end
	if ~isempty(F_bounds)
		onlybounds_F = false;
		if isempty(F_bounds)
			F_bounds = {-Inf(number_controls, number_references), Inf(number_controls, number_references)};
		end
		if iscell(F_bounds)
			if numel(F_bounds) >= 2
				if ~isnumeric(F_bounds{1}) || ~isnumeric(F_bounds{2})
					error('control:design:gamma:dimension', 'Bounded proportional gain must contain a logical and a numeric matrix or a numerical constraint system, not a ''%s''.', class(F_bounds{1}));
				end
			else
				error('control:design:gamma:dimension', 'Bounded proportional gain constraint system must not contain more than 2 matrices.');
			end
		elseif isnumeric(F_bounds)
			if ~ismatrix(F_bounds)
				if size(F_bounds, 3) > 2
					F_bounds = {F_bounds, zeros(size(F_bounds, 3), 1)};
				else
					F_bounds = {F_bounds(:, :, 1), F_bounds(:, :, 2)};
					onlybounds_F = true;
				end
			else
				error('control:design:gamma:dimension', 'Bounded gain must be a %dX%dX2 matrix.', number_controls, number_references);
			end
		else
			error('control:design:gamma:dimension', 'Bounded proportional gain positions must be two %dX%d matrices with positions and fixed values.', number_controls, number_references);
		end
		if size(F_bounds{1}, 1) ~= number_controls || size(F_bounds{1}, 2) ~= number_references
			error('control:design:gamma:dimension', 'Bounded proportional gain positions must be a %dX%d matrix.', number_controls, number_references);
		end
		if size(F_bounds{2}, 1) == number_controls && size(F_bounds{2}, 2) == number_references
			onlybounds_F = true;
		end
		if onlybounds_F
			if size(F_bounds{2}, 1) ~= number_controls && size(F_bounds{2}, 2) ~= number_references
				error('control:design:gamma:dimension', 'Bounded proportional gain values must be a %dX%d matrix.', number_controls, number_references);
			end
			F_bounds{1}(isnan(F_bounds{1})) = -Inf;
			F_bounds{2}(isnan(F_bounds{2})) = Inf;
			F_bounds_lower = reshape(F_bounds{1}, number_controls*number_references, 1);
			F_bounds_upper = reshape(F_bounds{2}, number_controls*number_references, 1);
			if any(F_bounds_lower > F_bounds_upper)
				error('control:design:gamma:dimension', 'Proportional gain lower bounds must be smaller than upper bounds.');
			end
			if any(F_bounds_lower == F_bounds_upper)
				error('control:design:gamma:dimension', 'Proportional gain lower bounds must not equal upper bounds, use fixed proportional gain instead.');
			end
			lb = F_bounds_lower;
			ub = F_bounds_upper;
			A = [];
			b = [];
		else
			if size(F_bounds{1}, 3) ~= size(F_bounds{2}, 1) || size(F_bounds{2}, 2) ~= 1
				error('control:design:gamma:dimension', 'Bounded proportional gain constraint system must be a %d vector of upper bounds.', size(F_bounds{1}, 3));
			end
			bound_system = zeros(size(F_bounds{1}, 3), number_controls*number_references);
			temp = F_bounds{1};
			parfor ii = 1:size(temp, 3)
				bound_system(ii, :) = reshape(temp(:, :, ii), 1, number_controls*number_references);
			end
			bound_border = F_bounds{2};
			A = bound_system;
			b = bound_border;
			rg_bounds = rank(bound_system);
			if rg_bounds ~= size(bound_system, 1)
				error('control:design:gamma:dimension', 'Fixed proportional gain constraint system must have column rank %d.', rg_bounds);
			end
			F_bounds_lower = -Inf(number_controls*number_references, 1);
			F_bounds_upper = Inf(number_controls*number_references, 1);
			bound_positions = logical(bound_system);
			if all(sum(bound_positions, 2) == 1)
				for ii = 1:size(F_bounds{1}, 3) %#ok<FORPF> no parfor because of sub2ind
					[idxrow, idxcol] = find(F_bounds{1}(:, :, ii), 1, 'first');
					if F_bounds{1}(idxrow, idxcol, ii) > 0
						F_bounds_upper(sub2ind(size(F_bounds{1}), idxrow, idxcol), 1) = F_bounds{2}(ii, 1)/F_bounds{1}(idxrow, idxcol, ii);
					else
						F_bounds_lower(sub2ind(size(F_bounds{1}), idxrow, idxcol), 1) = -F_bounds{2}(ii, 1)/F_bounds{1}(idxrow, idxcol, ii);
					end
				end
				lb = reshape(F_bounds_lower, number_controls, number_references);
				ub = reshape(F_bounds_upper, number_controls, number_references);
			else
				lb = [];
				ub = [];
			end
		end
	else
		A = [];
		b = [];
		lb = [];
		ub = [];
	end
	C = sparse(size(systems, 1)*number_references*number_references, number_references*number_controls);
	d = sparse(size(systems, 1)*number_references*number_references, 1);
	weight = options.weight;
	for ii = 1:size(system, 1)
		Asys = system(ii).A;
		Bsys = system(ii).B;
		Csys = system(ii).C;
		%Dsys = system(ii).D;
		Acl = Asys - Bsys*R_opt*Csys;
		if isdiscrete
			Ftemp = Csys*((eye(size(Acl, 1)) - Acl)\Bsys);
		else
			Ftemp = -Csys*(Acl\Bsys);
		end
		Ftemp = repmat({C_stat*Ftemp}, 1, number_references);
		for jj = 1:size(Ftemp, 2)
			Ftemp{1, jj}(jj, :) = weight*Ftemp{1, jj}(jj, :);
		end
		C((1:number_references*number_references) + number_references*number_references*(ii - 1), :) = blkdiag(Ftemp{:});
		d((1:number_references*number_references) + number_references*number_references*(ii - 1), 1) = reshape(weight*eye(number_references), number_references*number_references, 1);
	end
	x0 = [];
	[x, resnorm, residual, exitflag, output, lambda] = lsqlin(C, d, A, b, Aeq, beq, lb, ub, x0, options.options);
	F_opt = reshape(x, number_controls, number_references);
	if nargout >= 2
		info = struct(...
			'resnorm',	resnorm,...
			'residual',	residual,...
			'exitflag',	exitflag,...
			'output',	output,...
			'lambda',	lambda...
		);
	end
end
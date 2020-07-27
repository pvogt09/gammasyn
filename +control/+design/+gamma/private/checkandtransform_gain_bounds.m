function [R_bounds, bound_system, bound_border, rg_bounds, hasbounds_R, onlybounds_R, bound_system_hadamard] = checkandtransform_gain_bounds(R_bounds, number_controls, number_measurements, gaintype)
	%CHECKANDTRANSFORM_GAIN_BOUNDS check and convert bound gain arguments for gammasyn with different datatypes and meanings to an uniform constraint description for further use
	%	Input:
	%		R_bounds:				cell array with lower and upper bounds for proportional gain elements that should be bounded or 3D matrix where lower bounds are in the first plane in the third dimension and upper bounds in the second or a cell array with a 3D matrix of linear dependent bound expressions A*x <= b and the upper bounds in the second dimension of the cell array or (expreimental) cell array with symbolic expression for the gain coefficients in the first dimension and equation system in the second
	%		number_controls:		number of controls
	%		number_measurements:	number of measurements (usual or derivative depending on constraint matrix)
	%		gaintype:				'proportional' for proportional gain, 'derivative' for derivative gain, 'prefilter' for prefilter gain and 'combined' for combined gain
	%	Output:
	%		R_bounds:				uniform gain bound system
	%		bound_system:			matrix of bound system in the form A*vec(R) <= b
	%		bound_border:			border of bound system in the form A*vec(R) <= b
	%		rg_bounds:				rank of bound system
	%		hasbounds_R:			indicator, if gain matrix has bounded elements
	%		onlybounds_R:			indicator, if gain matrix has only bounded elements
	%		bound_system_hadamard:	bound system in the form sum(A.*K) <= b
	if ~isnumeric(number_controls) || ~isscalar(number_controls)
		error('control:design:gamma:dimension', 'Number of controls must be a numeric scalar.');
	end
	if ~isnumeric(number_measurements) || ~isscalar(number_measurements)
		error('control:design:gamma:dimension', 'Number of measurements must be a numeric scalar.');
	end
	if ~any(strcmpi(gaintype, {'proportional', 'derivative', 'prefilter', 'combined'}))
		error('control:design:gamma:dimension', 'Gain type must be ''proportional'', ''derivative'', ''prefilter'' or ''combined''.');
	end
	hasbounds_R = false;
	onlybounds_R = false;
	if isempty(R_bounds)
		R_bounds = {-Inf(number_controls, number_measurements), Inf(number_controls, number_measurements)};
	end
	if iscell(R_bounds)
		if numel(R_bounds) >= 2
			if ~isnumeric(R_bounds{1}) || ~isnumeric(R_bounds{2})
				if isa(R_bounds{1}, 'sym') || isa(R_bounds{2}, 'sym')
					if isa(R_bounds{1}, 'sym')
						isequation1 = sym.isequation(R_bounds{1}, 'any');
					else
						if size(R_bounds{1}, 1) == size(R_bounds{2}, 1) && size(R_bounds{1}, 2) == size(R_bounds{2}, 2)
							isequation1 = isa(R_bounds{2}, 'sym');
							R_bounds{1} = R_bounds{2} <= R_bounds{1};
						else
							isequation1 = false;
						end
					end
					if isa(R_bounds{2}, 'sym')
						isequation2 = sym.isequation(R_bounds{2}, 'any');
					else
						if size(R_bounds{1}, 1) == size(R_bounds{2}, 1) && size(R_bounds{1}, 2) == size(R_bounds{2}, 2)
							isequation2 = isa(R_bounds{1}, 'sym');
							R_bounds{2} = R_bounds{1} <= R_bounds{2};
						else
							isequation2 = false;
						end
					end
					if any(isequation1(:)) && ~any(isequation2(:))
						R_bounds_sym = R_bounds{1};
						R_bounds_var = R_bounds{2};
					elseif any(isequation2(:)) && ~any(isequation1(:))
						R_bounds_sym = R_bounds{2};
						R_bounds_var = R_bounds{1};
					else
						error('control:design:gamma:dimension', 'Symbolic %s gain inequality constraints must consist of constraint system and a mapping matrix of symbolic variables to gain matrix positions.', gaintype);
					end
					if ndims(R_bounds_var) > 2 %#ok<ISMAT> compatibility with Octave
						error('control:design:gamma:dimension', 'Symbolic %s gain constraint mapping must be a matrix.', gaintype);
					end
					if size(R_bounds_var, 1) ~= number_controls
						error('control:design:gamma:dimension', 'Symbolic %s gain inequality constraint mapping must have %d rows, not %d.', gaintype, number_controls, size(R_bounds_var, 1));
					end
					if size(R_bounds_var, 2) ~= number_measurements
						error('control:design:gamma:dimension', 'Symbolic %s gain inequality constraint mapping must have %d columns, not %d.', gaintype, number_measurements, size(R_bounds_var, 2));
					end
					for ii = 1:size(R_bounds_sym, 1)
						for jj = 1:size(R_bounds_sym, 2)
							if ~isnan(R_bounds_sym(ii, jj))
								if sym.isequation(R_bounds_sym(ii, jj), {'<=', '<', '>=', '>'})
									temp = children(R_bounds_sym(ii, jj));
									if numel(temp) > 2
										error('control:design:gamma:dimension', 'Symbolic linear %s gain matrix ineuality constraints must be an equation.', gaintype);
									end
									coefficient = gradient(temp(1), symvar(R_bounds_var));
									if ~isempty(symvar(coefficient))
										error('control:design:gamma:dimension', 'Symbolic linear %s gain matrix ineuality constraints must be linear, but left hand side of element (%d,%d) is nonlinear.', gaintype, ii, jj);
									end
									coefficient = gradient(temp(2), symvar(R_bounds_var));
									if ~isempty(symvar(coefficient))
										error('control:design:gamma:dimension', 'Symbolic linear %s gain matrix inequality constraints must be linear, but right hand side of element (%d,%d) is nonlinear.', gaintype, ii, jj);
									end
								else
									error('control:design:gamma:dimension', 'Symbolic linear %s gain matrix inequality constraints must be inequality constraints, but element (%d,%d) is not.', gaintype, ii, jj);
								end
							end
						end
					end
					% symbolic gain variable
					names = unique(cellfun(@char, num2cell(symvar(R_bounds_var)), 'UniformOutput', false));
					uniquename = ['gammasyn_', names{randi([1, numel(names)])}];
					r_gamma = sym(uniquename, [number_controls, number_measurements]);
					r_gamma_vec = reshape(r_gamma, number_controls*number_measurements, 1);
					% calculate relationship between gain coefficients  and symbolic variables in position matrix
					equation_system = r_gamma == R_bounds_var;
					% TODO: handle assumtions?
					%z = assumptions(R_bounds_var);
					[A, b] = equationsToMatrix(equation_system, r_gamma_vec);
					currentsymvars = symvar(b);
					lastsymvars = currentsymvars;
					while (~isempty(currentsymvars))
						replaceidx = [];
						replacevar = [];
						for ii = 1:size(b, 1)
							if ~isempty(symvar(b(ii, 1)))
								replaceidx = ii;
								replacevar = b(ii, 1);
								break;
							end
						end
						if ~isempty(replaceidx)
							equation_system = subs(equation_system, replacevar, A(replaceidx, :)*r_gamma_vec);
							R_bounds_sym = subs(R_bounds_sym, replacevar, A(replaceidx, :)*r_gamma_vec);
							[A, b] = equationsToMatrix(equation_system, r_gamma_vec);
							currentsymvars = symvar(b);
							if ~isequal(lastsymvars, currentsymvars)
								lastsymvars = currentsymvars;
							else
								error('control:design:gamma:dimension', 'Symbolic linear %s gain matrix constraints can not be reduced further to conain not symbolic expressions.', gaintype);
							end
						else
							error('control:design:gamma:dimension', 'Symbolic linear %s gain matrix constraints can not be reduced further to conain not symbolic expressions.', gaintype);
						end
					end
					% return inequality constraint system
					[A, b] = equationsToMatrixIneq(R_bounds_sym, r_gamma_vec);
					Atemp = zeros(number_controls, number_measurements, size(A, 1));
					parfor ii = 1:size(Atemp, 3)
						Atemp(:, :, ii) = reshape(A(ii, :), number_controls, number_measurements);
					end
					R_bounds = {double(Atemp), double(b)};
				else
					error('control:design:gamma:dimension', 'Bounded %s gain must contain a logical and a numeric matrix or a numerical constraint system, not a ''%s''.', gaintype, class(R_bounds{1}));
				end
			else
				if ~isnumeric(R_bounds{1}) && ~isnumeric(R_bounds{2})
					error('control:design:gamma:dimension', 'Bounded %s gain must contain a logical and a numeric matrix or a numerical constraint system, not a ''%s''.', gaintype, class(R_bounds{1}));
				end
			end
		else
			error('control:design:gamma:dimension', 'Bounded %s gain constraint system must not contain more than 2 matrices.', gaintype);
		end
	elseif isnumeric(R_bounds)
		if ~ismatrix(R_bounds)
			if size(R_bounds, 3) > 2
				R_bounds = {R_bounds, zeros(size(R_bounds, 3), 1)};
			else
				R_bounds = {R_bounds(:, :, 1), R_bounds(:, :, 2)};
				onlybounds_R = true;
			end
		else
			error('control:design:gamma:dimension', 'Bounded %s gain must be a %dX%dX2 matrix.', gaintype, number_controls, number_measurements);
		end
	elseif isa(R_bounds, 'sym')
		error('control:design:gamma:dimension', 'Matlab does not support converting symbolic inqeualities to matrix form.');
	else
		error('control:design:gamma:dimension', 'Bounded %s gain positions must be two %dX%d matrices with positions and fixed values.', gaintype, number_controls, number_measurements);
	end
	if size(R_bounds{1}, 1) ~= number_controls || size(R_bounds{1}, 2) ~= number_measurements
		error('control:design:gamma:dimension', 'Bounded %s gain positions must be a %dX%d matrix.', gaintype, number_controls, number_measurements);
	end
	if size(R_bounds{2}, 1) == number_controls && size(R_bounds{2}, 2) == number_measurements
		onlybounds_R = true;
	end
	if onlybounds_R
		if size(R_bounds{2}, 1) ~= number_controls && size(R_bounds{2}, 2) ~= number_measurements
			error('control:design:gamma:dimension', 'Bounded %s gain values must be a %dX%d matrix.', gaintype, number_controls, number_measurements);
		end
		if any(imag(R_bounds{1}(:)) ~= 0)
			error('control:design:gamma:dimension', 'Bounded %s gain value must not be complex.', gaintype);
		end
		if any(imag(R_bounds{2}(:)) ~= 0)
			error('control:design:gamma:dimension', 'Bounded %s gain value must not be complex.', gaintype);
		end
		R_bounds{1}(isnan(R_bounds{1})) = -Inf;
		R_bounds{2}(isnan(R_bounds{2})) = Inf;
		rg_bounds = number_controls*number_measurements;
		%if rg >= number_controls*number_measurements
		%	error('control:design:gamma:dimension', 'At least one proportional gain component must be unconstrained.');% TODO K und D können unabhängig beschränkt sein und zusammen muss mindestens eine freie Variable übrigbleiben
		%end
		R_bounds_lower = reshape(R_bounds{1}, number_controls*number_measurements, 1);
		R_bounds_upper = reshape(R_bounds{2}, number_controls*number_measurements, 1);
		if any(R_bounds_lower > R_bounds_upper)
			error('control:design:gamma:dimension', '%s gain lower bounds must be smaller than upper bounds.', ucfirst(gaintype));
		end
		if any(R_bounds_lower == R_bounds_upper)
			error('control:design:gamma:dimension', '%s gain lower bounds must not equal upper bounds, use fixed proportional gain instead.', ucfirst(gaintype));
		end
		bound_system = [
			-eye(number_controls*number_measurements);
			eye(number_controls*number_measurements)
		];
		bound_border = [
			-R_bounds_lower;
			R_bounds_upper
		];
	else
		if size(R_bounds{1}, 3) ~= size(R_bounds{2}, 1) || size(R_bounds{2}, 2) ~= 1
			error('control:design:gamma:dimension', 'Bounded %s gain constraint system must be a %d vector of upper bounds.', gaintype, size(R_bounds{1}, 3));
		end
		%if size(R_bounds{1}, 3) >= number_controls*number_measurements
		%	error('control:design:gamma:dimension', 'At least one proportional gain component must be unconstrained.');
		%end
		if any(imag(R_bounds{1}(:)) ~= 0)
			error('control:design:gamma:dimension', 'Bounded %s gain constraint system must not be complex.', gaintype);
		end
		if any(imag(R_bounds{2}(:)) ~= 0)
			error('control:design:gamma:dimension', 'Bounded %s gain value must not be complex.', gaintype);
		end
		bound_system = zeros(size(R_bounds{1}, 3), number_controls*number_measurements);
		temp = R_bounds{1};
		parfor ii = 1:size(temp, 3)
			bound_system(ii, :) = reshape(temp(:, :, ii), 1, number_controls*number_measurements);
		end
		bound_border = R_bounds{2};
		rg_bounds = rank(bound_system);
		if rg_bounds ~= size(bound_system, 1)% TODO: rank condition has to be replaced by solvability condition for inequality systems
			%error('control:design:gamma:dimension', 'Bounded %s gain constraint system must have column rank %d.', gaintype, rg_bounds);
		end
		hasbounds_R = size(bound_system, 1) > 0;
		R_bounds_lower = -Inf(number_controls*number_measurements, 1);
		R_bounds_upper = Inf(number_controls*number_measurements, 1);
		bound_positions = logical(bound_system);
		if all(sum(bound_positions, 2) == 1)
			for ii = 1:size(R_bounds{1}, 3) %#ok<FORPF> no parfor because of sub2ind
				[idxrow, idxcol] = find(R_bounds{1}(:, :, ii), 1, 'first');
				if R_bounds{1}(idxrow, idxcol, ii) > 0
					R_bounds_upper(sub2ind(size(R_bounds{1}), idxrow, idxcol), 1) = R_bounds{2}(ii, 1)/R_bounds{1}(idxrow, idxcol, ii);
				else
					R_bounds_lower(sub2ind(size(R_bounds{1}), idxrow, idxcol), 1) = R_bounds{2}(ii, 1)/R_bounds{1}(idxrow, idxcol, ii);
				end
			end
			R_bounds = {reshape(R_bounds_lower, number_controls, number_measurements), reshape(R_bounds_upper, number_controls, number_measurements)};
			onlybounds_R = true;
		else
			R_bounds = {-Inf(number_controls, number_measurements), Inf(number_controls, number_measurements)};
		end
	end
	if nargout >= 7
		bound_system_hadamard = zeros(number_controls, number_measurements, size(bound_border, 1));
		parfor ii = 1:size(bound_border, 1)
			bound_system_hadamard(:, :, ii) = reshape(bound_system(ii, :), number_controls, number_measurements);
		end
		bound_system_hadamard = {
			bound_system_hadamard,	bound_border
		};
	end
end
function [R_fixed, constraint_system, constraint_border, rg, T, T_inv, hasfixed_R, onlyfixed_R, allfixed, constraint_system_hadamard, isforced2zero] = checkandtransform_gain_fixed(R_fixed, number_controls, number_measurements, gaintype, zerocoefficients)
	%CHECKANDTRANSFORM_GAIN_FIXED check and convert fixed gain arguments for gammasyn with different datatypes and meanings to an uniform constraint description for further use
	%	Input:
	%		R_fixed:					cell array with indicator matrix for proportional gain elements that should be fixed and the values the fixed gains have or 3D matrix where fixed elements are the non zero elements in the first and second dimension or (experimental) cell array with symbolic expression for the gain coefficients in the first dimension and equation system in the second
	%		number_controls:			number of controls
	%		number_measurements:		number of measurements (usual or derivative depending on constraint matrix)
	%		gaintype:					'proportional' for proportional gain, 'derivative' for derivative gain, 'prefilter' for prefilter gain and 'combined' for combined gain
	%		zerocoefficients:			indicator matrix for coefficient that should be forced to zero
	%	Output:
	%		R_fixed:					uniform gain constraint system
	%		constraint_system:			matrix of constraint system in the form A*vec(R) = b
	%		constraint_border:			border of constraint system in the form A*vec(R) = b
	%		rg:							rank of constraint system
	%		T:							transformation from gain coefficients to constrained gain coefficients
	%		T_inv:						inverse transformation from constrained gain coefficients back to gain coefficients
	%		hasfixed_R:					indicator, if gain matrix has fixed elements
	%		onlyfixed_R:				indicator, if gain matrix has only fixed elements
	%		allfixed:					indicator, if all coefficients of gain matrix are fixed
	%		constraint_system_hadamard:	constraint system in the form sum(A.*K) = b
	%		isforced2zero:				indicator if supplied constraint force K == 0
	if ~isnumeric(number_controls) || ~isscalar(number_controls)
		error('control:design:gamma:dimension', 'Number of controls must be a numeric scalar.');
	end
	if ~isnumeric(number_measurements) || ~isscalar(number_measurements)
		error('control:design:gamma:dimension', 'Number of measurements must be a numeric scalar.');
	end
	if ~any(strcmpi(gaintype, {'proportional', 'derivative', 'prefilter', 'combined'}))
		error('control:design:gamma:dimension', 'Gain type must be ''proportional'', ''derivative'', ''prefilter'' or ''combined''.');
	end
	if nargin <= 4
		zerocoefficients = true(number_controls, number_measurements);
	end
	if ~islogical(zerocoefficients)
		error('control:design:gamma:dimension', 'Fixed %s gain zero coefficients must be of type ''logical''.', gaintype);
	end
	if size(zerocoefficients, 1) ~= number_controls || size(zerocoefficients, 2) ~= number_measurements
		error('control:design:gamma:dimension', 'Fixed %s gain zero coefficients must be a %dX%d matrix.', gaintype, number_controls, number_measurements);
	end
	hasfixed_R = false;
	onlyfixed_R = false;
	allfixed = false;
	isforced2zero = false;
	if isempty(R_fixed)
		R_fixed = {false(number_controls, number_measurements), zeros(number_controls, number_measurements)};
	end
	if iscell(R_fixed)
		if isempty(R_fixed{1}) || (numel(R_fixed) >= 2 && isempty(R_fixed{2}))
			R_fixed = {false(number_controls, number_measurements), zeros(number_controls, number_measurements)};
		end
		if numel(R_fixed) >= 2
			if islogical(R_fixed{1}) && isnumeric(R_fixed{2})
				onlyfixed_R = true;
			elseif islogical(R_fixed{2}) && isnumeric(R_fixed{1})
				R_fixed = {
					R_fixed{2},	R_fixed{1}
				};
				onlyfixed_R = true;
			else
				if ~isnumeric(R_fixed{1}) || ~isnumeric(R_fixed{2})
					if isa(R_fixed{1}, 'sym') && isa(R_fixed{2}, 'sym')
						dim1 = size(R_fixed{1}, 1) == number_controls && size(R_fixed{1}, 2) == number_measurements;
						dim2 = size(R_fixed{2}, 1) == number_controls && size(R_fixed{2}, 2) == number_measurements;
						if dim1 || dim2
							fixed_vars = [
								symvar(R_fixed{1}), symvar(R_fixed{2})
							];
							isequation1 = false(size(R_fixed{1}));
							for ii = 1:size(R_fixed{1}, 1)
								for jj = 1:size(R_fixed{1}, 2)
									if ~isnan(R_fixed{1}(ii, jj))
										eq = char(R_fixed{1}(ii, jj));
										if ~isempty(strfind(eq, '<'))
											error('control:design:gamma:dimension', 'Symbolic linear %s gain matrix coefficients are invalid because element (%d,%d) contains ''<''.', gaintype, ii, jj);
										end
										if ~isempty(strfind(eq, '<='))
											error('control:design:gamma:dimension', 'Symbolic linear %s gain matrix coefficients are invalid because element (%d,%d) contains ''<=''.', gaintype, ii, jj);
										end
										if ~isempty(strfind(eq, '>'))
											error('control:design:gamma:dimension', 'Symbolic linear %s gain matrix coefficients are invalid because element (%d,%d) contains ''>''.', gaintype, ii, jj);
										end
										if ~isempty(strfind(eq, '>='))
											error('control:design:gamma:dimension', 'Symbolic linear %s gain matrix coefficients are invalid because element (%d,%d) contains ''>=''.', gaintype, ii, jj);
										end
										eqfound = strfind(eq, '==');
										if ~isempty(eqfound)
											if numel(eqfound) > 1
												error('control:design:gamma:dimension', 'Symbolic linear %s gain matrix coefficients are invalid because element (%d,%d) contains multiple ''==''.', gaintype, ii, jj);
											end
											isequation1(ii, jj) = true;
										end
										if isequation1(ii, jj)
											temp = children(R_fixed{1}(ii, jj));
											if numel(temp) > 2
												error('control:design:gamma:dimension', 'Symbolic linear %s gain matrix constraints must be an equation.', gaintype);
											end
											coefficient = gradient(temp(1), fixed_vars);
											if ~isempty(symvar(coefficient))
												error('control:design:gamma:dimension', 'Symbolic linear %s gain matrix constraints must be linear, but left hand side of element (%d,%d) is nonlinear.', gaintype, ii, jj);
											end
											coefficient = gradient(temp(2), fixed_vars);
											if ~isempty(symvar(coefficient))
												error('control:design:gamma:dimension', 'Symbolic linear %s gain matrix constraints must be linear, but right hand side of element (%d,%d) is nonlinear.', gaintype, ii, jj);
											end
										else
											coefficient = gradient(R_fixed{1}(ii, jj), fixed_vars);
											if ~isempty(symvar(coefficient))
												error('control:design:gamma:dimension', 'Symbolic linear %s gain matrix constraints must be linear, but element (%d,%d) is nonlinear.', gaintype, ii, jj);
											end
										end
									end
								end
							end
							isequation2 = false(size(R_fixed{2}));
							for ii = 1:size(R_fixed{2}, 1)
								for jj = 1:size(R_fixed{2}, 2)
									if ~isnan(R_fixed{2}(ii, jj))
										eq = char(R_fixed{2}(ii, jj));
										if ~isempty(strfind(eq, '<'))
											error('control:design:gamma:dimension', 'Symbolic linear %s gain matrix coefficients are invalid because element (%d,%d) contains ''<''.', gaintype, ii, jj);
										end
										if ~isempty(strfind(eq, '<='))
											error('control:design:gamma:dimension', 'Symbolic linear %s gain matrix coefficients are invalid because element (%d,%d) contains ''<=''.', gaintype, ii, jj);
										end
										if ~isempty(strfind(eq, '>'))
											error('control:design:gamma:dimension', 'Symbolic linear %s gain matrix coefficients are invalid because element (%d,%d) contains ''>''.', gaintype, ii, jj);
										end
										if ~isempty(strfind(eq, '>='))
											error('control:design:gamma:dimension', 'Symbolic linear %s gain matrix coefficients are invalid because element (%d,%d) contains ''>=''.', gaintype, ii, jj);
										end
										eqfound = strfind(eq, '==');
										if ~isempty(eqfound)
											if numel(eqfound) > 1
												error('control:design:gamma:dimension', 'Symbolic linear %s gain matrix coefficients are invalid because element (%d,%d) contains multiple ''==''.', gaintype, ii, jj);
											end
											isequation2(ii, jj) = true;
										end
										if isequation2(ii, jj)
											temp = children(R_fixed{2}(ii, jj));
											if numel(temp) > 2
												error('control:design:gamma:dimension', 'Symbolic linear %s gain matrix constraints must be an equation.', gaintype);
											end
											coefficient = gradient(temp(1), fixed_vars);
											if ~isempty(symvar(coefficient))
												error('control:design:gamma:dimension', 'Symbolic linear %S gain matrix constraints must be linear, but left hand side of element (%d,%d) is nonlinear.', gaintype, ii, jj);
											end
											coefficient = gradient(temp(2), fixed_vars);
											if ~isempty(symvar(coefficient))
												error('control:design:gamma:dimension', 'Symbolic linear %s gain matrix constraints must be linear, but right hand side of element (%d,%d) is nonlinear.', gaintype, ii, jj);
											end
										else
											coefficient = gradient(R_fixed{2}(ii, jj), fixed_vars);
											if ~isempty(symvar(coefficient))
												error('control:design:gamma:dimension', 'Symbolic linear %s gain matrix constraints must be linear, but element (%d,%d) is nonlinear.', gaintype, ii, jj);
											end
										end
									end
								end
							end
							if all(isequation1(:)) && ~any(isequation2(:))
								R_fixed_sym = R_fixed{2};
								R_fixed_eq = R_fixed{1};
							elseif all(isequation2(:)) && ~any(isequation1(:))
								R_fixed_sym = R_fixed{1};
								R_fixed_eq = R_fixed{2};
							else
								error('control:design:gamma:dimension', 'Fixed %s gain must contain a symbolic gain matrix and a symbolic constraint system.', gaintype);
							end
							if size(R_fixed_sym, 1) ~= number_controls || size(R_fixed_sym, 2) ~= number_measurements
								error('control:design:gamma:dimension', 'Fixed %s gain must contain a %dX%d matrix.', gaintype, number_controls, number_measurements);
							end
							names = unique(cellfun(@char, num2cell(fixed_vars), 'UniformOutput', false));
							uniquename = ['gammasyn_', names{randi([1, numel(names)])}];
							r_gamma = sym(uniquename, [number_controls, number_measurements]);
							r_gamma_vec = reshape(r_gamma, number_controls*number_measurements, 1);
							equation_system = [
								reshape(r_gamma == R_fixed_sym, [], 1);
								reshape(R_fixed_eq, [], 1)
							];
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
									equation_system = subs(equation_system, replacevar, A(replaceidx, :)*reshape(r_gamma_vec, [], 1));
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
							freevariables = all(A == 0, 2);
							Atemp = zeros(number_controls, number_measurements, sum(~freevariables));
							Afixed = A(~freevariables, :);
							parfor ii = 1:size(Atemp, 3)
								Atemp(:, :, ii) = reshape(Afixed(ii, :), number_controls, number_measurements);
							end
							R_fixed = {double(Atemp), double(b(~freevariables))};
						else
							error('control:design:gamma:dimension', 'Symbolic linear %s gain matrix constraints must be either a gain matrix or a constraint system.', gaintype);
						end
						%equationsystem = R_fixed{1} == R_fixed{2};
						%fixed_vars = symvar(equationsystem);
						%[A, b] = equationsToMatrix(equationsystem, fixed_vars);
					else
						%if size(R_fixed{2}, 2) == 1
						%	R = sym('r_gamma', number_controls, number_measurements);
						%	K = sym('d_gamma', number_controls, number_measurements_xdot);
						%else
						error('control:design:gamma:dimension', 'Fixed %s gain must contain a symbolic gain matrix and a symbolic constraint system, not a ''%s''.', gaintype, class(R_fixed{1}));
						%end
					end
				else
					if ~isnumeric(R_fixed{1}) && ~isnumeric(R_fixed{2})
						error('control:design:gamma:dimension', 'Fixed %s gain must contain a logical and a numeric matrix or a numerical constraint system, not a ''%s''.', gaintype, class(R_fixed{1}));
					end
				end
				%end
			end
		else
			error('control:design:gamma:dimension', 'Fixed %s gain constraint system must not contain more than 2 matrices.', gaintype);
		end
	elseif isnumeric(R_fixed)
		if ~ismatrix(R_fixed)
			if size(R_fixed, 3) > 2
				R_fixed = {R_fixed, zeros(size(R_fixed, 3), 1)};
			else
				if any(any(isnan(R_fixed(:, :, 1))))
					R_fixed = {~isnan(R_fixed(:, :, 1)), R_fixed(:, :, 2)};
				else
					R_fixed = {logical(R_fixed(:, :, 1)), R_fixed(:, :, 2)};
				end
				onlyfixed_R = true;
			end
		else
			if any(any(isnan(R_fixed)))
				R_fixed = {~isnan(R_fixed), R_fixed};
			else
				R_fixed = {logical(R_fixed), R_fixed};
			end
			onlyfixed_R = true;
			%error('control:design:gamma:dimension', 'Fixed gain must be a %dX%dX2 matrix.', number_controls, number_measurements);
		end
	elseif islogical(R_fixed)
		R_fixed = {R_fixed, zeros(size(R_fixed))};
		onlyfixed_R = true;
	elseif isa(R_fixed, 'sym')
		if ndims(R_fixed) >= 3 %#ok<ISMAT> compatibility with Octave
			error('control:design:gamma:dimension', 'Fixed %s gain constraint must be a matrix.', gaintype);
		end
		R_fixed_sym = R_fixed;
		fixed_vars = symvar(R_fixed_sym);
		if isempty(fixed_vars)
			R_fixed_sym = double(R_fixed_sym);
			if any(any(isnan(R_fixed_sym)))
				R_fixed = {~isnan(R_fixed_sym), R_fixed_sym};
			else
				R_fixed = {logical(R_fixed_sym), R_fixed_sym};
			end
			onlyfixed_R = true;
		else
			%coefficient_matrix = NaN([size(R_fixed_sym), size(fixed_vars, 2)]);
			isequation = false(size(R_fixed_sym));
			for ii = 1:size(R_fixed_sym, 1)
				for jj = 1:size(R_fixed_sym, 2)
					if ~isnan(R_fixed_sym(ii, jj))
						eq = char(R_fixed_sym(ii, jj));
						if ~isempty(strfind(eq, '<'))
							error('control:design:gamma:dimension', 'Symbolic linear %s gain matrix coefficients are invalid because element (%d,%d) contains ''<''.', gaintype, ii, jj);
						end
						if ~isempty(strfind(eq, '<='))
							error('control:design:gamma:dimension', 'Symbolic linear %s gain matrix coefficients are invalid because element (%d,%d) contains ''<=''.', gaintype, ii, jj);
						end
						if ~isempty(strfind(eq, '>'))
							error('control:design:gamma:dimension', 'Symbolic linear %s gain matrix coefficients are invalid because element (%d,%d) contains ''>''.', gaintype, ii, jj);
						end
						if ~isempty(strfind(eq, '>='))
							error('control:design:gamma:dimension', 'Symbolic linear %s gain matrix coefficients are invalid because element (%d,%d) contains ''>=''.', gaintype, ii, jj);
						end
						eqfound = strfind(eq, '==');
						if ~isempty(eqfound)
							if numel(eqfound) > 1
								error('control:design:gamma:dimension', 'Symbolic linear %s gain matrix coefficients are invalid because element (%d,%d) contains multiple ''==''.', gaintype, ii, jj);
							end
							isequation(ii, jj) = true;
						end
						if isequation(ii, jj)
							temp = children(R_fixed_sym(ii, jj));
							if numel(temp) > 2
								error('control:design:gamma:dimension', 'Symbolic linear %s gain matrix constraints must be an equation.', gaintype);
							end
							coefficient = gradient(temp(1), fixed_vars);
							if ~isempty(symvar(coefficient))
								error('control:design:gamma:dimension', 'Symbolic linear %s gain matrix constraints must be linear, but left hand side of element (%d,%d) is nonlinear.', gaintype, ii, jj);
							end
							coefficient = gradient(temp(2), fixed_vars);
							if ~isempty(symvar(coefficient))
								error('control:design:gamma:dimension', 'Symbolic linear %s gain matrix constraints must be linear, but right hand side of element (%d,%d) is nonlinear.', gaintype, ii, jj);
							end
						else
							coefficient = gradient(R_fixed_sym(ii, jj), fixed_vars);
							if ~isempty(symvar(coefficient))
								error('control:design:gamma:dimension', 'Symbolic linear %s gain matrix constraints must be linear, but element (%d,%d) is nonlinear.', gaintype, ii, jj);
							end
						end
						%coefficient_matrix(ii, jj, :) = double(coefficient);
						%if any(coefficient ~= 0)
						%	coefficient_matrix(ii, jj) = coefficient(coefficient ~= 0);
						%else
						%	coefficient_matrix(ii, jj) = 0;
						%end
					end
				end
			end
			if ~any(isequation(:)) && size(R_fixed_sym, 1) == number_controls && size(R_fixed_sym, 2) == number_measurements
				% symbolic gain variable
				names = unique(cellfun(@char, num2cell(fixed_vars), 'UniformOutput', false));
				uniquename = ['gammasyn_', names{randi([1, numel(names)])}];
				r_gamma = sym(uniquename, [number_controls, number_measurements]);
				r_gamma_vec = reshape(r_gamma, number_controls*number_measurements, 1);
				equation_system = r_gamma == R_fixed_sym;
				% TODO: handle assumtions?
				%z = assumptions(fixed_vars);
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
				freevariables = all(A == 0, 2);
				Atemp = zeros(number_controls, number_measurements, sum(~freevariables));
				Afixed = A(~freevariables, :);
				parfor ii = 1:size(Atemp, 3)
					Atemp(:, :, ii) = reshape(Afixed(ii, :), number_controls, number_measurements);
				end
				R_fixed = {double(Atemp), double(b(~freevariables))};
			elseif all(isequation(:))
				warning('control:design:gamma:dimension', 'Symbolic linear %s gain matrix is built on the assumption K = reshape(symvar(...), %d, %d), proceed with caution.', gaintype, number_controls*number_measurements);
				% symbolic equation system
				allvars = symvar(R_fixed_sym);
				% TODO: position of allvars in gain matrix?
				if numel(allvars) ~= number_controls*number_measurements
					error('control:design:gamma:dimension', 'Symbolic linear %s gain matrix constraint system must contain %d symbolic variables.', gaintype, number_controls*number_measurements);
				end
				[A, b] = equationsToMatrix(R_fixed_sym, allvars);
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
						equation_system = subs(equation_system, replacevar, A(replaceidx, :)*reshape(allvars, [], 1));
						[A, b] = equationsToMatrix(equation_system, allvars);
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
				freevariables = all(A == 0, 2);
				Atemp = zeros(number_controls, number_measurements, sum(~freevariables));
				Afixed = A(~freevariables, :);
				parfor ii = 1:size(Atemp, 3)
					Atemp(:, :, ii) = reshape(Afixed(ii, :), number_controls, number_measurements);
				end
				R_fixed = {double(Atemp), double(b(~freevariables))};
			else
				error('control:design:gamma:dimension', 'Symbolic linear %s gain matrix constraints must be either a gain matrix or a constraint system.', gaintype);
			end
		end
	else
		error('control:design:gamma:dimension', 'Fixed %s gain positions must be two %dX%d matrices with positions and fixed values.', gaintype, number_controls, number_measurements);
	end
	if size(R_fixed{1}, 1) ~= number_controls || size(R_fixed{1}, 2) ~= number_measurements
		error('control:design:gamma:dimension', 'Fixed %s gain positions must be a %dX%d matrix.', gaintype, number_controls, number_measurements);
	end
	if onlyfixed_R
		if size(R_fixed{2}, 1) ~= number_controls || size(R_fixed{2}, 2) ~= number_measurements
			error('control:design:gamma:dimension', 'Fixed %s gain values must be a %dX%d matrix.', gaintype, number_controls, number_measurements);
		end
		if any(imag(R_fixed{2}(:)) ~= 0)
			error('control:design:gamma:dimension', 'Fixed %s gain value must not be complex.', gaintype);
		end
		rg = sum(R_fixed{1}(:));
		if rg > 0 && rg > number_controls*number_measurements
			error('control:design:gamma:dimension', 'At least one %s gain component must be unconstrained.', gaintype);
		end
		if rg > 0 && rg == number_controls*number_measurements
			allfixed = true;
		end
		constraint_system = zeros(rg, number_controls*number_measurements);
		constraint_border = zeros(rg, 1);
		R_fixed_row_logical = reshape(R_fixed{1}, 1, number_controls*number_measurements);
		R_fixed_row = reshape(R_fixed{2}, 1, number_controls*number_measurements);
		idx = find(R_fixed_row_logical);
		for ii = 1:size(idx, 2)
			constraint_system(ii, :) = (1:size(constraint_system, 2)) == idx(ii);
			constraint_border(ii, 1) = R_fixed_row(1, idx(ii));
		end
		T = [
			constraint_system;
			null(constraint_system)'
		];
		T_inv = inv(T);
		if rg > 0
			hasfixed_R = true;
		end
	else
		if size(R_fixed{1}, 3) ~= size(R_fixed{2}, 1) || size(R_fixed{2}, 2) ~= 1
			error('control:design:gamma:dimension', 'Fixed %s gain constraint system must be a %d vector of bounds.', gaintype, size(R_fixed{1}, 3));
		end
		if size(R_fixed{1}, 3) > number_controls*number_measurements
			error('control:design:gamma:dimension', 'At least one %s gain component must be unconstrained.', gaintype);
		end
		if size(R_fixed{1}, 3) == number_controls*number_measurements
			allfixed = true;
		end
		if any(imag(R_fixed{1}(:)) ~= 0)
			error('control:design:gamma:dimension', 'Fixed %s gain constraint system must not be complex.', gaintype);
		end
		if any(imag(R_fixed{2}(:)) ~= 0)
			error('control:design:gamma:dimension', 'Fixed %s gain border must not be complex.', gaintype);
		end
		constraint_system = zeros(size(R_fixed{1}, 3), number_controls*number_measurements);
		temp = R_fixed{1};
		parfor ii = 1:size(temp, 3)
			constraint_system(ii, :) = reshape(temp(:, :, ii), 1, number_controls*number_measurements);
		end
		constraint_border = R_fixed{2};
		rg = rank(constraint_system);
		if rg ~= size(constraint_system, 1)
			error('control:design:gamma:dimension', 'Fixed %s gain constraint system must have column rank %d.', gaintype, rg);
		end
		T = [
			constraint_system;
			null(constraint_system)'
		];
		T_inv = inv(T);
		hasfixed_R = size(constraint_system, 1) > 0;
		constraint_positions = logical(constraint_system);
		if all(sum(constraint_positions, 2) == 1)
			temp = false(number_controls, number_measurements);
			tempval = zeros(number_controls, number_measurements);
			for ii = 1:size(R_fixed{1}, 3)
				temp(logical(R_fixed{1}(:, :, ii))) = true;
				tempval(logical(R_fixed{1}(:, :, ii))) = R_fixed{2}(ii);
			end
			R_fixed = {logical(temp), tempval};
			onlyfixed_R = true;
		else
			R_fixed = {false(number_controls, number_measurements), zeros(number_controls, number_measurements)};
		end
	end
	if any(isnan(constraint_border)) || any(isinf(constraint_border))
		error('control:design:gamma:dimension', 'Fixed %s gain constraint system border must not contain NaN or Inf.', gaintype);
	end
	if nargout >= 10
		constraint_system_hadamard = zeros(number_controls, number_measurements, size(constraint_border, 1));
		parfor ii = 1:size(constraint_border, 1)
			constraint_system_hadamard(:, :, ii) = reshape(constraint_system(ii, :), number_controls, number_measurements);
		end
		constraint_system_hadamard = {
			constraint_system_hadamard,	constraint_border
		};
	end
	if nargout >= 11
		isforced2zero = checkandtransform_gain_fixed_forced2zero(constraint_system_hadamard{1}, constraint_system_hadamard{2}, number_controls, number_measurements, gaintype, zerocoefficients);
	end
end
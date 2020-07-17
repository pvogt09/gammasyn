function [R_nonlin, number_c_R, number_ceq_R, number_c_K, number_ceq_K, number_c_F, number_ceq_F, hasnonlinfun, hasnonlingrad, hasnonlinhessian] = checkandtransform_gain_nonlin(R_nonlin, number_controls, number_measurements, number_measurements_xdot, number_references, T_nonlin)
	%CHECKANDTRANSFORM_GAIN_NONLIN check and convert nonlinear gain arguments for gammasyn with different datatypes and meanings to an uniform constraint description for further use
	%	Input:
	%		R_nonlin:					function pointer with signature [c_R, ceq_R, c_K, ceq_K, c_F, ceq_F, gradc_R, gradceq_R, gradc_K, gradceq_K, gradc_F, gradceq_F] = constraint(R, K, F)
	%		number_controls:			number of controls
	%		number_measurements:		number of measurements
	%		number_measurements_xdot:	number of derivative measurements
	%		number_references:			number of reference inputs
	%		T_nonlin:					structure with data for performing transformations of K, D, F for calculating nonlinear constraints
	%	Output:
	%		R_nonlin:					function pointer with signature [c_R, ceq_R, c_K, ceq_K, c_F, ceq_F, gradc_R, gradceq_R, gradc_K, gradceq_K, gradc_F, gradceq_F] = constraint(R, K, F)
	%		number_c_R:					number of nonlinear inequality constraints of proportional gain matrix
	%		number_ceq_R:				number of nonlinear equality constraints of proportional gain matrix
	%		number_c_K:					number of nonlinear inequality constraints of derivative gain matrix
	%		number_ceq_K:				number of nonlinear equality constraints of derivative gain matrix
	%		number_c_F:					number of nonlinear inequality constraints of prefilter matrix
	%		number_ceq_F:				number of nonlinear equality constraints of prefilter matrix
	%		hasnonlinfun:				indicator, if nonlinear constraints are present
	%		hasnonlingrad:				indicator, if gradient information is supplied
	%		hasnonlinhessian:			indicator, if hessian information is supplied

	%% check T_nonlin
	if nargin <= 5 || isempty(T_nonlin)
		T_nonlin = struct(...
			'number_states_original',	number_measurements,...
			'number_controls_original',	number_controls ...
		);
	end
	%% check general
	if ~isstruct(T_nonlin)
		error('control:design:gamma:input', 'T_nonlin must be of type ''struct''.');
	end
	if ~isfield(T_nonlin, 'number_states_original') || ~isfield(T_nonlin, 'number_controls_original')
		error('control:design:gamma:input', 'T_nonlin must have the fields ''number_states_original'' and ''number_controls_original''.');
	else
		number_states_original	= T_nonlin.number_states_original;
		number_controls_original = T_nonlin.number_controls_original;
	end
	if isfield(T_nonlin, 'transform')
		if ~islogical(T_nonlin.transform)
			error('control:design:gamma:input', 'T_nonlin.transform must be of type ''logical''.');
		end
	else
		T_nonlin.transform = false;
	end

	%% check K_A and K_b
	default_K_A = zeros(number_states_original*number_controls_original, number_measurements*number_controls);
	min_dim_K	=   min([number_states_original*number_controls_original, number_measurements*number_controls]);
	default_K_A(1:min_dim_K, 1:min_dim_K) = eye(min_dim_K);
	default_K_b = zeros(number_states_original*number_controls_original, 1);

	T_nonlin = check_T_nonlin_Ab(T_nonlin, 'K_A', 'K_b', number_states_original*number_controls_original, number_measurements*number_controls, default_K_A, default_K_b);
	%% check D_A and D_b

	default_D_A =   eye(number_controls_original*number_measurements_xdot);
	default_D_b = zeros(number_controls_original*number_measurements_xdot, 1);

	T_nonlin = check_T_nonlin_Ab(T_nonlin, 'D_A', 'D_b', number_controls_original*number_measurements_xdot, number_controls*number_measurements_xdot, default_D_A, default_D_b);

	%% check F_A and F_b

	default_F_A = zeros(number_controls_original^2, number_controls^2);
	min_dim_F	=  min([number_controls_original^2, number_controls^2]);
	default_F_A(1:min_dim_F, 1:min_dim_F) = eye(min_dim_F);
	default_F_b = zeros(number_controls_original^2, 1);

	T_nonlin = check_T_nonlin_Ab(T_nonlin, 'F_A', 'F_b', number_controls_original^2, number_controls^2, default_F_A, default_F_b);

	%% check the rest
	if ~isnumeric(number_controls) || ~isscalar(number_controls)
		error('control:design:gamma:dimension', 'Number of controls must be a numeric scalar.');
	end
	if ~isnumeric(number_measurements) || ~isscalar(number_measurements)
		error('control:design:gamma:dimension', 'Number of measurements must be a numeric scalar.');
	end
	if ~isnumeric(number_measurements_xdot) || ~isscalar(number_measurements_xdot)
		error('control:design:gamma:dimension', 'Number of derivative measurements must be a numeric scalar.');
	end
	if ~isnumeric(number_references) || ~isscalar(number_references)
		error('control:design:gamma:dimension', 'Number of references must be a numeric scalar.');
	end
	if isempty(R_nonlin)
		R_nonlin = [];
		number_c_R = 0;
		number_ceq_R = 0;
		number_c_K = 0;
		number_ceq_K = 0;
		number_c_F = 0;
		number_ceq_F = 0;
		hasnonlinfun = false;
		hasnonlingrad = false;
		hasnonlinhessian = false;
		return;
	else
		%[c_R, ceq_R, c_K, ceq_K, c_F, ceq_F, gradc_R, gradceq_R, gradc_K, gradceq_K, gradc_F, gradceq_F] = constraint(R, K, F)
		% gradc_R = cat(3, [
		%	dc_1/dR_11, dc_1/dR_12...;
		%	dc_1/dR_21, dc_1/dR_22...;
		% ], [
		%	dc_2/dR_11, dc_2/dR_12...;
		%	dc_2/dR_21, dc_2/dR_22...;
		% ], ...)
		% TODO: check if nonlinear constraints are feasible when linear equality constraints are already applied?
		% TODO: add hessian check
		hasnonlinhessian = false;
		if iscell(R_nonlin)
			if numel(R_nonlin) == 2
				% R, K or F and equation system supplied
				isequation1 = sym.isequation(R_nonlin{1}, 'any');
				isequation2 = sym.isequation(R_nonlin{2}, 'any');
				if any(isequation1(:)) && ~any(isequation2(:))
					R_nonlin_sym = R_nonlin{1};
					R_nonlin_var = R_nonlin{2};
				elseif any(isequation2(:)) && ~any(isequation1(:))
					R_nonlin_sym = R_nonlin{2};
					R_nonlin_var = R_nonlin{1};
				else
					error('control:design:gamma:dimension', 'Symbolic nonlinear gain constraints must consist of constraint system and a mapping matrix of symbolic variables to gain matrix positions.');
				end
				if ndims(R_nonlin_var) > 2 %#ok<ISMAT> compatibility with Octave
					error('control:design:gamma:dimension', 'Symbolic nonlinear gain constraint mapping must be a matrix.');
				end
				if size(R_nonlin_var, 1) ~= number_controls
					error('control:design:gamma:dimension', 'Symbolic nonlinear gain constraint mapping must have %d rows, not %d.', number_controls, size(R_nonlin_var, 1));
				end
				if number_measurements == number_measurements_xdot
					if number_measurements == number_references
						diffgain = false;
						prefiltergain = false;
						if size(R_nonlin_var, 2) ~= number_measurements
							error('control:design:gamma:dimension', 'Symbolic nonlinear gain constraint mapping must have %d columns, not %d.', number_measurements, size(R_nonlin_var, 2));
						end
					else
						if size(R_nonlin_var, 2) == number_measurements
							diffgain = false;
							prefiltergain = false;
						elseif  size(R_nonlin_var, 2) == number_references
							diffgain = false;
							prefiltergain = true;
						else
							error('control:design:gamma:dimension', 'Symbolic nonlinear gain constraint mapping must have %d or %d columns, not %d.', number_measurements, number_references, size(R_nonlin_var, 2));
						end
					end
				else
					if number_measurements == number_references
						diffgain = false;
						prefiltergain = false;
						if size(R_nonlin_var, 2) ~= number_measurements
							error('control:design:gamma:dimension', 'Symbolic nonlinear gain constraint mapping must have %d columns, not %d.', number_measurements, size(R_nonlin_var, 2));
						end
					else
						if size(R_nonlin_var, 2) == number_measurements
							diffgain = false;
							prefiltergain = false;
						elseif size(R_nonlin_var, 2) == number_measurements_xdot
							diffgain = true;
							prefiltergain = false;
						elseif  size(R_nonlin_var, 2) == number_references
							diffgain = false;
							prefiltergain = true;
						else
							error('control:design:gamma:dimension', 'Symbolic nonlinear gain constraint mapping must have %d or %d columns, not %d.', number_measurements, number_references, size(R_nonlin_var, 2));
						end
					end
				end
				nonlin_vars_R = reshape(symvar(R_nonlin_var), [], 1);
				if ~diffgain && ~prefiltergain && numel(nonlin_vars_R) > number_controls*number_measurements
					error('control:design:gamma:dimension', 'Symbolic nonlinear gain constraint mapping must not contain more than %d symbolic variables.', number_controls*number_measurements);
				end
				if diffgain && ~prefiltergain && numel(nonlin_vars_R) > number_controls*number_measurements_xdot
					error('control:design:gamma:dimension', 'Symbolic nonlinear gain constraint mapping must not contain more than %d symbolic variables.', number_controls*number_measurements_xdot);
				end
				if ~diffgain && prefiltergain && numel(nonlin_vars_R) > number_controls*number_references
					error('control:design:gamma:dimension', 'Symbolic nonlinear gain constraint mapping must not contain more than %d symbolic variables.', number_controls*number_references);
				end
				isvar = false(size(R_nonlin_var));
				for ii = 1:size(R_nonlin_var, 1)
					for jj = 1:size(R_nonlin_var, 2)
						if numel(symvar(R_nonlin_var(ii, jj))) > 1
							error('control:design:gamma:dimension', 'Symbolic nonlinear gain constraint mapping must contain only one symbolic variable for every component of the gain matrix.');
						end
						grad = gradient(R_nonlin_var(ii, jj), nonlin_vars_R);
						if ~isempty(symvar(grad))
							error('control:design:gamma:dimension', 'Symbolic nonlinear gain constraint mapping must be linear a linear expression.');
						end
						grad = double(grad) ~= 0;
						isvar(ii, jj) = sum(grad(:)) == 1;
						if sum(grad(:)) > 1
							error('control:design:gamma:dimension', 'Relation between symbolic nonlinear gain constraint mapping and gain matrix coefficients must be one to one.');
						end
					end
				end
				if ~any(isvar(:))
					error('control:design:gamma:dimension', 'Symbolic nonlinear gain constraint mapping must at least bepend on one variable.');
				end
				for ii = 1:size(R_nonlin_sym, 1)
					for jj = 1:size(R_nonlin_sym, 2)
						if ~isnan(R_nonlin_sym(ii, jj))
							vars = reshape(symvar(R_nonlin_sym(ii, jj)), [], 1);
							if numel(vars) > size(nonlin_vars_R, 1)
								error('control:design:gamma:dimension', 'Symbolic nonlinear gain constraints must not depend on more variables than specified.');
							end
							dep = (sum(sym.depends(R_nonlin_sym(ii, jj), nonlin_vars_R), 1) ~= 0).';
							if all(~dep(:)) || any(sum(sym.depends(nonlin_vars_R(dep, 1), vars), 1) == 0)
								error('control:design:gamma:dimension', 'Symbolic nonlinear gain constraints must not depend on more variables than specified.');
							end
							eq = char(R_nonlin_sym(ii, jj));
							if numel(strfind(eq, '<')) > 1
								error('control:design:gamma:dimension', 'Symbolic nonlinear constraints are invalid because element (%d,%d) contains ''<''.', ii, jj);
							end
							if numel(strfind(eq, '<=')) > 1
								error('control:design:gamma:dimension', 'Symbolic nonlinear constraints are invalid because element (%d,%d) contains ''<=''.', ii, jj);
							end
							if numel(strfind(eq, '>')) > 1
								error('control:design:gamma:dimension', 'Symbolic nonlinear constraints are invalid because element (%d,%d) contains ''>''.', ii, jj);
							end
							if numel(strfind(eq, '>=')) > 1
								error('control:design:gamma:dimension', 'Symbolic nonlinear constraints are invalid because element (%d,%d) contains ''>=''.', ii, jj);
							end
							eqfound = strfind(eq, '==');
							if ~isempty(eqfound)
								if numel(eqfound) > 1
									error('control:design:gamma:dimension', 'Symbolic linear proportional gain matrix coefficients are invalid because element (%d,%d) contains multiple ''==''.', ii, jj);
								end
								%isequ(ii, jj) = true;
							end
% 							if isequation(ii, jj)
% 								temp = children(R_fixed_sym(ii, jj));
% 								if numel(temp) > 2
% 									error('control:design:gamma:dimension', 'Symbolic linear proportional gain matrix constraints must be an equation.');
% 								end
% 								coefficient = gradient(temp(1), fixed_vars);
% 								if ~isempty(symvar(coefficient))
% 									error('control:design:gamma:dimension', 'Symbolic linear proportional gain matrix constraints must be linear, but left hand side of element (%d,%d) is nonlinear.', ii, jj);
% 								end
% 								coefficient = gradient(temp(2), fixed_vars);
% 								if ~isempty(symvar(coefficient))
% 									error('control:design:gamma:dimension', 'Symbolic linear proportional gain matrix constraints must be linear, but right hand side of element (%d,%d) is nonlinear.', ii, jj);
% 								end
% 							else
% 								coefficient = gradient(R_fixed_sym(ii, jj), fixed_vars);
% 								if ~isempty(symvar(coefficient))
% 									error('control:design:gamma:dimension', 'Symbolic linear proportional gain matrix constraints must be linear, but element (%d,%d) is nonlinear.', ii, jj);
% 								end
% 							end
							%coefficient_matrix(ii, jj, :) = double(coefficient);
							%if any(coefficient ~= 0)
							%	coefficient_matrix(ii, jj) = coefficient(coefficient ~= 0);
							%else
							%	coefficient_matrix(ii, jj) = 0;
							%end
						end
					end
				end
				R_nonlin_sym = reshape(R_nonlin_sym, [], 1);
				iseq = sym.isequation(R_nonlin_sym, '==');
				isleq = sym.isequation(R_nonlin_sym, {'<=', '<'});
				isgeq = sym.isequation(R_nonlin_sym, {'>=', '>'});
				noneq = ~(iseq | isleq | isgeq);
				isineq = isleq | isgeq;
				if any(noneq(:))
					error('control:design:gamma:dimension', 'All symbolic nonlinear gain constraints must be symbolic equations.');
				end
				diffeq = ~xor(iseq, isineq);
				if any(diffeq(:))
					error('control:design:gamma:dimension', 'All symbolic nonlinear gain constraints must be symbolic equations.');
				end
				for ii = 1:size(R_nonlin_sym, 1)
					temp = children(R_nonlin_sym(ii, 1));
					if numel(temp) ~= 2
						error('equationsToMatrixIneq:input', 'Symbolic expression must be an equation.');
					end
					if isgeq(ii, 1)
						lhs = -temp(1) + temp(2);
					else
						lhs = temp(1) - temp(2);
					end
					R_nonlin_sym(ii, 1) = lhs;
				end
				tempiseq = sym.isequation(R_nonlin_sym, 'any');
				if any(tempiseq(:))
					error('control:design:gamma:dimension', 'After reformulation, some symbolic nonlinear gain constraints are still symbolic equations, but should be simple expressions.');
				end
				if diffgain
					ceq_R = [];
					c_R = [];
					gradceq_R = [];
					gradc_R = [];
					if any(iseq)
						ceq_K = matlabFunction(R_nonlin_sym(iseq, :), 'Vars', reshape(R_nonlin_var(isvar), [], 1));
						gradceq_K = matlabFunction(jacobian(R_nonlin_sym(iseq, :), reshape(R_nonlin_var(isvar), [], 1)), 'Vars', reshape(R_nonlin_var(isvar), [], 1));
					else
						ceq_K = [];
						gradceq_K = [];
					end
					if any(isineq)
						c_K = matlabFunction(R_nonlin_sym(isineq, :), 'Vars', reshape(R_nonlin_var(isvar), [], 1));
						gradc_K = matlabFunction(jacobian(R_nonlin_sym(isineq, :), reshape(R_nonlin_var(isvar), [], 1)), 'Vars', reshape(R_nonlin_var(isvar), [], 1));
					else
						c_K = [];
						gradc_K = [];
					end
					ceq_F = [];
					c_F = [];
					gradceq_F = [];
					gradc_F = [];
					isvar_R = [];
					isvar_K = isvar;
					isvar_F = [];
				elseif prefiltergain
					ceq_R = [];
					c_R = [];
					gradceq_R = [];
					gradc_R = [];
					ceq_K = [];
					c_K = [];
					gradceq_K = [];
					gradc_K = [];
					if any(iseq)
						ceq_F = matlabFunction(R_nonlin_sym(iseq, :), 'Vars', reshape(R_nonlin_var(isvar), [], 1));
						gradceq_F = matlabFunction(jacobian(R_nonlin_sym(iseq, :), reshape(R_nonlin_var(isvar), [], 1)), 'Vars', reshape(R_nonlin_var(isvar), [], 1));
					else
						ceq_F = [];
						gradceq_F = [];
					end
					if any(isineq)
						c_F = matlabFunction(R_nonlin_sym(isineq, :), 'Vars', reshape(R_nonlin_var(isvar), [], 1));
						gradc_F = matlabFunction(jacobian(R_nonlin_sym(isineq, :), reshape(R_nonlin_var(isvar), [], 1)), 'Vars', reshape(R_nonlin_var(isvar), [], 1));
					else
						c_F = [];
						gradc_F = [];
					end
					isvar_R = [];
					isvar_K = [];
					isvar_F = isvar;
				else
					if any(iseq)
						ceq_R = matlabFunction(R_nonlin_sym(iseq, :), 'Vars', reshape(R_nonlin_var(isvar), [], 1));
						gradceq_R = matlabFunction(jacobian(R_nonlin_sym(iseq, :), reshape(R_nonlin_var(isvar), [], 1)), 'Vars', reshape(R_nonlin_var(isvar), [], 1));
					else
						ceq_R = [];
						gradceq_R = [];
					end
					if any(isineq)
						c_R = matlabFunction(R_nonlin_sym(isineq, :), 'Vars', reshape(R_nonlin_var(isvar), [], 1));
						gradc_R = matlabFunction(jacobian(R_nonlin_sym(isineq, :), reshape(R_nonlin_var(isvar), [], 1)), 'Vars', reshape(R_nonlin_var(isvar), [], 1));
					else
						c_R = [];
						gradc_R = [];
					end
					ceq_K = [];
					c_K = [];
					gradceq_K = [];
					gradc_K = [];
					ceq_F = [];
					c_F = [];
					gradceq_F = [];
					gradc_F = [];
					isvar_R = isvar;
					isvar_K = [];
					isvar_F = [];
				end
				R_nonlin = convertsymfun(c_R, gradc_R, ceq_R, gradceq_R, c_K, gradc_K, ceq_K, gradceq_K, c_F, gradc_F, ceq_F, gradceq_F, isvar_R, isvar_K, isvar_F);
			elseif numel(R_nonlin) == 3
				% two of R, K and F and equation system supplied
				isequation1 = sym.isequation(R_nonlin{1}, 'any');
				isequation2 = sym.isequation(R_nonlin{2}, 'any');
				isequation3 = sym.isequation(R_nonlin{3}, 'any');
				if any(isequation1(:)) && ~any(isequation2(:)) && ~any(isequation3(:))
					R_nonlin_sym = R_nonlin{1};
					R_nonlin_var = R_nonlin([2, 3]);
				elseif ~any(isequation1(:)) && any(isequation2(:)) && ~any(isequation3(:))
					R_nonlin_sym = R_nonlin{2};
					R_nonlin_var = R_nonlin([1, 3]);
				elseif ~any(isequation1(:)) && ~any(isequation2(:)) && any(isequation3(:))
					R_nonlin_sym = R_nonlin{3};
					R_nonlin_var = R_nonlin([1, 2]);
				else
					error('control:design:gamma:dimension', 'Symbolic nonlinear gain constraints must consist of constraint system and a mapping matrix of symbolic variables to gain matrix positions.');
				end
				if number_measurements == number_measurements_xdot
					if size(R_nonlin_var{1}, 2) == number_measurements && size(R_nonlin_var{2}, 2) == number_measurements_xdot
						K_nonlin_var = R_nonlin_var{2};
						R_nonlin_var = R_nonlin_var{1};
						F_nonlin_var = zeros(number_control, number_references);
						proportionalgain = true;
						diffgain = true;
						prefiltergain = false;
					else
						error('control:design:gamma:dimension', 'Symbolic nonlinear gain constraints must consist of constraint system and a mapping matrix of symbolic variables to gain matrix positions.');
					end
				else
					if size(R_nonlin_var{1}, 2) == number_measurements && size(R_nonlin_var{2}, 2) == number_measurements_xdot
						F_nonlin_var = zeros(number_controls, number_references);
						K_nonlin_var = R_nonlin_var{2};
						R_nonlin_var = R_nonlin_var{1};
						proportionalgain = true;
						diffgain = true;
						prefiltergain = false;
					elseif size(R_nonlin_var{2}, 2) == number_measurements && size(R_nonlin_var{1}, 2) == number_measurements_xdot
						F_nonlin_var = zeros(number_controls, number_references);
						K_nonlin_var = R_nonlin_var{1};
						R_nonlin_var = R_nonlin_var{2};
						proportionalgain = true;
						diffgain = true;
						prefiltergain = false;
					elseif size(R_nonlin_var{1}, 2) == number_measurements && size(R_nonlin_var{2}, 2) == number_references
						F_nonlin_var = R_nonlin_var{2};
						K_nonlin_var = zeros(number_controls, number_measurements_xdot);
						R_nonlin_var = R_nonlin_var{1};
						proportionalgain = true;
						diffgain = false;
						prefiltergain = true;
					elseif size(R_nonlin_var{1}, 2) == number_measurements_xdot && size(R_nonlin_var{2}, 2) == number_references
						F_nonlin_var = R_nonlin_var{2};
						K_nonlin_var = R_nonlin_var{1};
						R_nonlin_var = zeros(number_controls, number_measurements);
						proportionalgain = false;
						diffgain = true;
						prefiltergain = true;
					elseif size(R_nonlin_var{2}, 2) == number_measurements_xdot && size(R_nonlin_var{1}, 2) == number_references
						F_nonlin_var = R_nonlin_var{1};
						K_nonlin_var = R_nonlin_var{2};
						R_nonlin_var = zeros(number_controls, number_measurements);
						proportionalgain = false;
						diffgain = true;
						prefiltergain = true;
					elseif size(R_nonlin_var{2}, 2) == number_measurements && size(R_nonlin_var{1}, 2) == number_references
						F_nonlin_var = R_nonlin_var{1};
						K_nonlin_var = zeros(number_controls, number_measurements_xdot);
						R_nonlin_var = R_nonlin_var{2};
						proportionalgain = true;
						diffgain = false;
						prefiltergain = true;
					else
						error('control:design:gamma:dimension', 'Symbolic nonlinear gain constraints must consist of constraint system and a mapping matrix of symbolic variables to gain matrix positions.');
					end
				end
				if ndims(R_nonlin_var) > 2 %#ok<ISMAT> compatibility with Octave
					error('control:design:gamma:dimension', 'Symbolic nonlinear proportional gain constraint mapping must be a matrix.');
				end
				if ndims(K_nonlin_var) > 2 %#ok<ISMAT> compatibility with Octave
					error('control:design:gamma:dimension', 'Symbolic nonlinear differential gain constraint mapping must be a matrix.');
				end
				if ndims(F_nonlin_var) > 2 %#ok<ISMAT> compatibility with Octave
					error('control:design:gamma:dimension', 'Symbolic nonlinear prefilter constraint mapping must be a matrix.');
				end
				if size(R_nonlin_var, 1) ~= number_controls
					error('control:design:gamma:dimension', 'Symbolic nonlinear proportional gain constraint mapping must have %d rows, not %d.', number_controls, size(R_nonlin_var, 1));
				end
				if size(K_nonlin_var, 1) ~= number_controls
					error('control:design:gamma:dimension', 'Symbolic nonlinear differential gain constraint mapping must have %d rows, not %d.', number_controls, size(K_nonlin_var, 1));
				end
				if size(F_nonlin_var, 1) ~= number_controls
					error('control:design:gamma:dimension', 'Symbolic nonlinear prefilter constraint mapping must have %d rows, not %d.', number_controls, size(F_nonlin_var, 1));
				end
				if size(R_nonlin_var, 2) ~= number_measurements
					error('control:design:gamma:dimension', 'Symbolic nonlinear proportional gain constraint mapping must have %d columns, not %d.', number_measurements, size(R_nonlin_var, 2));
				end
				if size(K_nonlin_var, 2) ~= number_measurements_xdot
					error('control:design:gamma:dimension', 'Symbolic nonlinear differential gain constraint mapping must have %d columns, not %d.', number_measurements_xdot, size(K_nonlin_var, 2));
				end
				if size(F_nonlin_var, 2) ~= number_references
					error('control:design:gamma:dimension', 'Symbolic nonlinear prefilter constraint mapping must have %d columns, not %d.', number_references, size(F_nonlin_var, 2));
				end
				if proportionalgain
					nonlin_vars_R = reshape(symvar(R_nonlin_var), [], 1);
				else
					nonlin_vars_R = reshape(R_nonlin_var, [], 1);
				end
				if diffgain
					nonlin_vars_K = reshape(symvar(K_nonlin_var), [], 1);
				else
					nonlin_vars_K = reshape(K_nonlin_var, [], 1);
				end
				if prefiltergain
					nonlin_vars_F = reshape(symvar(F_nonlin_var), [], 1);
				else
					nonlin_vars_F = reshape(F_nonlin_var, [], 1);
				end
				if numel(nonlin_vars_R) > number_controls*number_measurements
					error('control:design:gamma:dimension', 'Symbolic nonlinear proportional gain constraint mapping must not contain more than %d symbolic variables.', number_controls*number_measurements);
				end
				if numel(nonlin_vars_K) > number_controls*number_measurements_xdot
					error('control:design:gamma:dimension', 'Symbolic nonlinear derivative gain constraint mapping must not contain more than %d symbolic variables.', number_controls*number_measurements_xdot);
				end
				if numel(nonlin_vars_F) > number_controls*number_references
					error('control:design:gamma:dimension', 'Symbolic nonlinear prefilter constraint mapping must not contain more than %d symbolic variables.', number_controls*number_references);
				end
				isvar_R = false(size(R_nonlin_var));
				if proportionalgain
					for ii = 1:size(R_nonlin_var, 1)
						for jj = 1:size(R_nonlin_var, 2)
							if numel(symvar(R_nonlin_var(ii, jj))) > 1
								error('control:design:gamma:dimension', 'Symbolic nonlinear proportional gain constraint mapping must contain only one symbolic variable for every component of the gain matrix.');
							end
							grad = gradient(R_nonlin_var(ii, jj), nonlin_vars_R);
							if ~isempty(symvar(grad))
								error('control:design:gamma:dimension', 'Symbolic nonlinear proportional gain constraint mapping must be a linear expression.');
							end
							grad = double(grad) ~= 0;
							isvar_R(ii, jj) = sum(grad(:)) == 1;
							if sum(grad(:)) > 1
								error('control:design:gamma:dimension', 'Relation between symbolic nonlinear proportional gain constraint mapping and gain matrix coefficients must be one to one.');
							end
						end
					end
				end
				if proportionalgain && ~any(isvar_R(:))
					error('control:design:gamma:dimension', 'Symbolic nonlinear proportional gain constraint mapping must at least depend on one variable.');
				end
				isvar_K = false(size(K_nonlin_var));
				if diffgain
					for ii = 1:size(K_nonlin_var, 1)
						for jj = 1:size(K_nonlin_var, 2)
							if numel(symvar(K_nonlin_var(ii, jj))) > 1
								error('control:design:gamma:dimension', 'Symbolic nonlinear differential gain constraint mapping must contain only one symbolic variable for every component of the gain matrix.');
							end
							grad = gradient(K_nonlin_var(ii, jj), nonlin_vars_K);
							if ~isempty(symvar(grad))
								error('control:design:gamma:dimension', 'Symbolic nonlinear differential gain constraint mapping must be a linear expression.');
							end
							grad = double(grad) ~= 0;
							isvar_K(ii, jj) = sum(grad(:)) == 1;
							if sum(grad(:)) > 1
								error('control:design:gamma:dimension', 'Relation between symbolic nonlinear differential gain constraint mapping and gain matrix coefficients must be one to one.');
							end
						end
					end
				end
				if diffgain && ~any(isvar_K(:))
					error('control:design:gamma:dimension', 'Symbolic nonlinear differential gain constraint mapping must at least depend on one variable.');
				end
				isvar_F = false(size(F_nonlin_var));
				if prefiltergain
					for ii = 1:size(F_nonlin_var, 1)
						for jj = 1:size(F_nonlin_var, 2)
							if numel(symvar(F_nonlin_var(ii, jj))) > 1
								error('control:design:gamma:dimension', 'Symbolic nonlinear prefilter constraint mapping must contain only one symbolic variable for every component of the gain matrix.');
							end
							grad = gradient(F_nonlin_var(ii, jj), nonlin_vars_F);
							if ~isempty(symvar(grad))
								error('control:design:gamma:dimension', 'Symbolic nonlinear prefilter constraint mapping must be a linear expression.');
							end
							grad = double(grad) ~= 0;
							isvar_F(ii, jj) = sum(grad(:)) == 1;
							if sum(grad(:)) > 1
								error('control:design:gamma:dimension', 'Relation between symbolic nonlinear prefilter constraint mapping and gain matrix coefficients must be one to one.');
							end
						end
					end
				end
				if prefiltergain && ~any(isvar_F(:))
					error('control:design:gamma:dimension', 'Symbolic nonlinear prefilter constraint mapping must at least depend on one variable.');
				end
				if ~any(isvar_R(:)) && ~any(isvar_K(:)) && ~any(isvar_F(:))
					error('control:design:gamma:dimension', 'Symbolic nonlinear constraint mapping must at least depend on one variable.');
				end
				R_nonlin_depends = false(size(R_nonlin_sym, 1), size(R_nonlin_sym, 2), 3);
				for ii = 1:size(R_nonlin_sym, 1)
					for jj = 1:size(R_nonlin_sym, 2)
						if ~isnan(R_nonlin_sym(ii, jj))
							vars = reshape(symvar(R_nonlin_sym(ii, jj)), [], 1);
							if numel(vars) > size(nonlin_vars_R, 1) + size(nonlin_vars_K, 1) + size(nonlin_vars_F, 1)
								error('control:design:gamma:dimension', 'Symbolic nonlinear gain constraints must not depend on more variables than specified.');
							end
							if proportionalgain
								dep_R = (sum(sym.depends(R_nonlin_sym(ii, jj), nonlin_vars_R), 1) ~= 0).';
							else
								dep_R = false(size(nonlin_vars_R, 1), 1);
							end
							if diffgain
								dep_K = (sum(sym.depends(R_nonlin_sym(ii, jj), nonlin_vars_K), 1) ~= 0).';
							else
								dep_K = false(size(nonlin_vars_K, 1), 1);
							end
							if prefiltergain
								dep_F = (sum(sym.depends(R_nonlin_sym(ii, jj), nonlin_vars_F), 1) ~= 0).';
							else
								dep_F = false(size(nonlin_vars_F, 1), 1);
							end
							R_nonlin_depends(ii, jj, :) = [
								any(dep_R);
								any(dep_K);
								any(dep_F)
							];
							if (all(~dep_R(:)) && all(~dep_K(:)) && all(~dep_F(:))) || any(sum(sym.depends([
								nonlin_vars_R(dep_R, 1);
								nonlin_vars_K(dep_K, 1);
								nonlin_vars_F(dep_F, 1)
							], vars), 1) == 0)
								error('control:design:gamma:dimension', 'Symbolic nonlinear gain constraints must not depend on more proportional variables than specified.');
							end
							if (all(~dep_R(:)) && all(~dep_K(:)) && all(~dep_F(:))) || any(sum(sym.depends([
								nonlin_vars_R(dep_R, 1);
								nonlin_vars_K(dep_K, 1);
								nonlin_vars_F(dep_F, 1)
							], vars), 1) == 0)
								error('control:design:gamma:dimension', 'Symbolic nonlinear gain constraints must not depend on more differential variables than specified.');
							end
							if (all(~dep_R(:)) && all(~dep_K(:)) && all(~dep_F(:))) || any(sum(sym.depends([
								nonlin_vars_R(dep_R, 1);
								nonlin_vars_K(dep_K, 1);
								nonlin_vars_F(dep_F, 1)
							], vars), 1) == 0)
								error('control:design:gamma:dimension', 'Symbolic nonlinear gain constraints must not depend on more prefilter variables than specified.');
							end
							eq = char(R_nonlin_sym(ii, jj));
							if numel(strfind(eq, '<')) > 1
								error('control:design:gamma:dimension', 'Symbolic nonlinear constraints are invalid because element (%d,%d) contains ''<''.', ii, jj);
							end
							if numel(strfind(eq, '<=')) > 1
								error('control:design:gamma:dimension', 'Symbolic nonlinear constraints are invalid because element (%d,%d) contains ''<=''.', ii, jj);
							end
							if numel(strfind(eq, '>')) > 1
								error('control:design:gamma:dimension', 'Symbolic nonlinear constraints are invalid because element (%d,%d) contains ''>''.', ii, jj);
							end
							if numel(strfind(eq, '>=')) > 1
								error('control:design:gamma:dimension', 'Symbolic nonlinear constraints are invalid because element (%d,%d) contains ''>=''.', ii, jj);
							end
							eqfound = strfind(eq, '==');
							if ~isempty(eqfound)
								if numel(eqfound) > 1
									error('control:design:gamma:dimension', 'Symbolic linear proportional gain matrix coefficients are invalid because element (%d,%d) contains multiple ''==''.', ii, jj);
								end
								%isequ(ii, jj) = true;
							end
% 							if isequation(ii, jj)
% 								temp = children(R_fixed_sym(ii, jj));
% 								if numel(temp) > 2
% 									error('control:design:gamma:dimension', 'Symbolic linear proportional gain matrix constraints must be an equation.');
% 								end
% 								coefficient = gradient(temp(1), fixed_vars);
% 								if ~isempty(symvar(coefficient))
% 									error('control:design:gamma:dimension', 'Symbolic linear proportional gain matrix constraints must be linear, but left hand side of element (%d,%d) is nonlinear.', ii, jj);
% 								end
% 								coefficient = gradient(temp(2), fixed_vars);
% 								if ~isempty(symvar(coefficient))
% 									error('control:design:gamma:dimension', 'Symbolic linear proportional gain matrix constraints must be linear, but right hand side of element (%d,%d) is nonlinear.', ii, jj);
% 								end
% 							else
% 								coefficient = gradient(R_fixed_sym(ii, jj), fixed_vars);
% 								if ~isempty(symvar(coefficient))
% 									error('control:design:gamma:dimension', 'Symbolic linear proportional gain matrix constraints must be linear, but element (%d,%d) is nonlinear.', ii, jj);
% 								end
% 							end
							%coefficient_matrix(ii, jj, :) = double(coefficient);
							%if any(coefficient ~= 0)
							%	coefficient_matrix(ii, jj) = coefficient(coefficient ~= 0);
							%else
							%	coefficient_matrix(ii, jj) = 0;
							%end
						end
					end
				end
				R_nonlin_sym = reshape(R_nonlin_sym, [], 1);
				dep_R = reshape(R_nonlin_depends(:, :, 1), [], 1);
				dep_K = reshape(R_nonlin_depends(:, :, 2), [], 1);
				dep_F = reshape(R_nonlin_depends(:, :, 3), [], 1);
				iseq = sym.isequation(R_nonlin_sym, '==');
				isleq = sym.isequation(R_nonlin_sym, {'<=', '<'});
				isgeq = sym.isequation(R_nonlin_sym, {'>=', '>'});
				noneq = ~(iseq | isleq | isgeq);
				isineq = isleq | isgeq;
				if any(noneq(:))
					error('control:design:gamma:dimension', 'All symbolic nonlinear gain constraints must be symbolic equations.');
				end
				diffeq = ~xor(iseq, isineq);
				if any(diffeq(:))
					error('control:design:gamma:dimension', 'All symbolic nonlinear gain constraints must be symbolic equations.');
				end
				for ii = 1:size(R_nonlin_sym, 1)
					temp = children(R_nonlin_sym(ii, 1));
					if numel(temp) ~= 2
						error('equationsToMatrixIneq:input', 'Symbolic expression must be an equation.');
					end
					if isgeq(ii, 1)
						lhs = -temp(1) + temp(2);
					else
						lhs = temp(1) - temp(2);
					end
					R_nonlin_sym(ii, 1) = lhs;
				end
				tempiseq = sym.isequation(R_nonlin_sym, 'any');
				if any(tempiseq(:))
					error('control:design:gamma:dimension', 'After reformulation, some symbolic nonlinear gain constraints are still symbolic equations, but should be simple expressions.');
				end
				funvars = [
					reshape(R_nonlin_var(isvar_R), [], 1);
					reshape(K_nonlin_var(isvar_K), [], 1);
					reshape(F_nonlin_var(isvar_F), [], 1)
				];
				if proportionalgain
					if any(iseq & dep_R)
						ceq_R = matlabFunction(R_nonlin_sym(iseq & dep_R, :), 'Vars', funvars);
						gradceq_R = matlabFunction(jacobian(R_nonlin_sym(iseq & dep_R, :), reshape(R_nonlin_var(isvar_R), [], 1)), 'Vars', funvars);
					else
						ceq_R = [];
						gradceq_R = [];
					end
					if any(isineq & dep_R)
						c_R = matlabFunction(R_nonlin_sym(isineq & dep_R, :), 'Vars', funvars);
						gradc_R = matlabFunction(jacobian(R_nonlin_sym(isineq & dep_R, :), reshape(R_nonlin_var(isvar_R), [], 1)), 'Vars', funvars);
					else
						c_R = [];
						gradc_R = [];
					end
				else
					ceq_R = [];
					c_R = [];
					gradc_R = [];
					gradceq_R = [];
				end
				if diffgain
					if any(iseq & dep_K)
						ceq_K = matlabFunction(R_nonlin_sym(iseq & dep_K, :), 'Vars', funvars);
						gradceq_K = matlabFunction(jacobian(R_nonlin_sym(isineq & dep_K, :), reshape(K_nonlin_var(isvar_K), [], 1)), 'Vars', funvars);
					else
						ceq_K = [];
						gradceq_K = [];
					end
					if any(isineq & dep_K)
						c_K = matlabFunction(R_nonlin_sym(isineq & dep_K, :), 'Vars', funvars);
						gradc_K = matlabFunction(jacobian(R_nonlin_sym(isineq & dep_K, :), reshape(K_nonlin_var(isvar_K), [], 1)), 'Vars', funvars);
					else
						c_K = [];
						gradc_K = [];
					end
				else
					c_K = [];
					ceq_K = [];
					gradc_K = [];
					gradceq_K = [];
				end
				if proportionalgain
					if any(iseq & dep_F)
						ceq_F = matlabFunction(R_nonlin_sym(iseq & dep_F, :), 'Vars', funvars);
						gradceq_F = matlabFunction(jacobian(R_nonlin_sym(iseq & dep_F, :), reshape(R_nonlin_var(isvar_F), [], 1)), 'Vars', funvars);
					else
						ceq_F = [];
						gradceq_F = [];
					end
					if any(isineq & dep_F)
						c_F = matlabFunction(R_nonlin_sym(isineq & dep_F, :), 'Vars', funvars);
						gradc_F = matlabFunction(jacobian(R_nonlin_sym(isineq & dep_F, :), reshape(R_nonlin_var(isvar_F), [], 1)), 'Vars', funvars);
					else
						c_F = [];
						gradc_F = [];
					end
				else
					c_F = [];
					ceq_F = [];
					gradc_F = [];
					gradceq_F = [];
				end
				R_nonlin = convertsymfun(c_R, gradc_R, ceq_R, gradceq_R, c_K, gradc_K, ceq_K, gradceq_K, c_F, gradc_F, ceq_F, gradceq_F, isvar_R, isvar_K, isvar_F);
			elseif numel(R_nonlin) == 4
				% R, K and F and equation system supplied
				isequation1 = sym.isequation(R_nonlin{1}, 'any');
				isequation2 = sym.isequation(R_nonlin{2}, 'any');
				isequation3 = sym.isequation(R_nonlin{3}, 'any');
				isequation4 = sym.isequation(R_nonlin{4}, 'any');
				if any(isequation1(:)) && ~any(isequation2(:)) && ~any(isequation3(:)) && ~any(isequation4(:))
					R_nonlin_sym = R_nonlin{1};
					R_nonlin_var = R_nonlin([2, 3, 4]);
				elseif ~any(isequation1(:)) && any(isequation2(:)) && ~any(isequation3(:)) && ~any(isequation4(:))
					R_nonlin_sym = R_nonlin{2};
					R_nonlin_var = R_nonlin([1, 3, 4]);
				elseif ~any(isequation1(:)) && ~any(isequation2(:)) && any(isequation3(:)) && ~any(isequation4(:))
					R_nonlin_sym = R_nonlin{3};
					R_nonlin_var = R_nonlin([1, 2, 4]);
				elseif ~any(isequation1(:)) && ~any(isequation2(:)) && ~any(isequation3(:)) && any(isequation4(:))
					R_nonlin_sym = R_nonlin{4};
					R_nonlin_var = R_nonlin([1, 2, 3]);
				else
					error('control:design:gamma:dimension', 'Symbolic nonlinear gain constraints must consist of constraint system and a mapping matrix of symbolic variables to gain matrix positions.');
				end
				if number_measurements == number_measurements_xdot
					if size(R_nonlin_var{1}, 2) == number_measurements && size(R_nonlin_var{2}, 2) == number_measurements_xdot
						K_nonlin_var = R_nonlin_var{2};
						R_nonlin_var = R_nonlin_var{1};
					else
						error('control:design:gamma:dimension', 'Symbolic nonlinear gain constraints must consist of constraint system and a mapping matrix of symbolic variables to gain matrix positions.');
					end
				else
					if size(R_nonlin_var{1}, 2) == number_measurements && size(R_nonlin_var{2}, 2) == number_measurements_xdot && size(R_nonlin_var{3}, 2) == number_references
						F_nonlin_var = R_nonlin_var{3};
						K_nonlin_var = R_nonlin_var{2};
						R_nonlin_var = R_nonlin_var{1};
					elseif size(R_nonlin_var{2}, 2) == number_measurements && size(R_nonlin_var{1}, 2) == number_measurements_xdot && size(R_nonlin_var{3}, 2) == number_references
						F_nonlin_var = R_nonlin_var{3};
						K_nonlin_var = R_nonlin_var{1};
						R_nonlin_var = R_nonlin_var{2};
					elseif size(R_nonlin_var{1}, 2) == number_measurements && size(R_nonlin_var{3}, 2) == number_measurements_xdot && size(R_nonlin_var{2}, 2) == number_references
						F_nonlin_var = R_nonlin_var{2};
						K_nonlin_var = R_nonlin_var{3};
						R_nonlin_var = R_nonlin_var{1};
					elseif size(R_nonlin_var{3}, 2) == number_measurements && size(R_nonlin_var{1}, 2) == number_measurements_xdot && size(R_nonlin_var{2}, 2) == number_references
						F_nonlin_var = R_nonlin_var{2};
						K_nonlin_var = R_nonlin_var{1};
						R_nonlin_var = R_nonlin_var{3};
					elseif size(R_nonlin_var{3}, 2) == number_measurements && size(R_nonlin_var{2}, 2) == number_measurements_xdot && size(R_nonlin_var{1}, 2) == number_references
						F_nonlin_var = R_nonlin_var{1};
						K_nonlin_var = R_nonlin_var{2};
						R_nonlin_var = R_nonlin_var{3};
					elseif size(R_nonlin_var{2}, 2) == number_measurements && size(R_nonlin_var{3}, 2) == number_measurements_xdot && size(R_nonlin_var{1}, 2) == number_references
						F_nonlin_var = R_nonlin_var{1};
						K_nonlin_var = R_nonlin_var{3};
						R_nonlin_var = R_nonlin_var{2};
					else
						error('control:design:gamma:dimension', 'Symbolic nonlinear gain constraints must consist of constraint system and a mapping matrix of symbolic variables to gain matrix positions.');
					end
				end
				if ndims(R_nonlin_var) > 2 %#ok<ISMAT> compatibility with Octave
					error('control:design:gamma:dimension', 'Symbolic nonlinear proportional gain constraint mapping must be a matrix.');
				end
				if ndims(K_nonlin_var) > 2 %#ok<ISMAT> compatibility with Octave
					error('control:design:gamma:dimension', 'Symbolic nonlinear differential gain constraint mapping must be a matrix.');
				end
				if ndims(F_nonlin_var) > 2 %#ok<ISMAT> compatibility with Octave
					error('control:design:gamma:dimension', 'Symbolic nonlinear prefilter constraint mapping must be a matrix.');
				end
				if size(R_nonlin_var, 1) ~= number_controls
					error('control:design:gamma:dimension', 'Symbolic nonlinear proportional gain constraint mapping must have %d rows, not %d.', number_controls, size(R_nonlin_var, 1));
				end
				if size(K_nonlin_var, 1) ~= number_controls
					error('control:design:gamma:dimension', 'Symbolic nonlinear differential gain constraint mapping must have %d rows, not %d.', number_controls, size(K_nonlin_var, 1));
				end
				if size(F_nonlin_var, 1) ~= number_controls
					error('control:design:gamma:dimension', 'Symbolic nonlinear prefilter constraint mapping must have %d rows, not %d.', number_controls, size(F_nonlin_var, 1));
				end
				if size(R_nonlin_var, 2) ~= number_measurements
					error('control:design:gamma:dimension', 'Symbolic nonlinear proportional gain constraint mapping must have %d columns, not %d.', number_measurements, size(R_nonlin_var, 2));
				end
				if size(K_nonlin_var, 2) ~= number_measurements_xdot
					error('control:design:gamma:dimension', 'Symbolic nonlinear differential gain constraint mapping must have %d columns, not %d.', number_measurements_xdot, size(K_nonlin_var, 2));
				end
				if size(F_nonlin_var, 2) ~= number_references
					error('control:design:gamma:dimension', 'Symbolic nonlinear prefilter constraint mapping must have %d columns, not %d.', number_references, size(F_nonlin_var, 2));
				end
				nonlin_vars_R = reshape(symvar(R_nonlin_var), [], 1);
				nonlin_vars_K = reshape(symvar(K_nonlin_var), [], 1);
				nonlin_vars_F = reshape(symvar(F_nonlin_var), [], 1);
				if numel(nonlin_vars_R) > number_controls*number_measurements
					error('control:design:gamma:dimension', 'Symbolic nonlinear proportional gain constraint mapping must not contain more than %d symbolic variables.', number_controls*number_measurements);
				end
				if numel(nonlin_vars_K) > number_controls*number_measurements_xdot
					error('control:design:gamma:dimension', 'Symbolic nonlinear derivative gain constraint mapping must not contain more than %d symbolic variables.', number_controls*number_measurements_xdot);
				end
				if numel(nonlin_vars_F) > number_controls*number_references
					error('control:design:gamma:dimension', 'Symbolic nonlinear prefilter constraint mapping must not contain more than %d symbolic variables.', number_controls*number_references);
				end
				isvar_R = false(size(R_nonlin_var));
				for ii = 1:size(R_nonlin_var, 1)
					for jj = 1:size(R_nonlin_var, 2)
						if numel(symvar(R_nonlin_var(ii, jj))) > 1
							error('control:design:gamma:dimension', 'Symbolic nonlinear proportional gain constraint mapping must contain only one symbolic variable for every component of the gain matrix.');
						end
						grad = gradient(R_nonlin_var(ii, jj), nonlin_vars_R);
						if ~isempty(symvar(grad))
							error('control:design:gamma:dimension', 'Symbolic nonlinear proportional gain constraint mapping must be a linear expression.');
						end
						grad = double(grad) ~= 0;
						isvar_R(ii, jj) = sum(grad(:)) == 1;
						if sum(grad(:)) > 1
							error('control:design:gamma:dimension', 'Relation between symbolic nonlinear proportional gain constraint mapping and gain matrix coefficients must be one to one.');
						end
					end
				end
				if ~any(isvar_R(:))
					error('control:design:gamma:dimension', 'Symbolic nonlinear proportional gain constraint mapping must at least depend on one variable.');
				end
				isvar_K = false(size(K_nonlin_var));
				for ii = 1:size(K_nonlin_var, 1)
					for jj = 1:size(K_nonlin_var, 2)
						if numel(symvar(K_nonlin_var(ii, jj))) > 1
							error('control:design:gamma:dimension', 'Symbolic nonlinear differential gain constraint mapping must contain only one symbolic variable for every component of the gain matrix.');
						end
						grad = gradient(K_nonlin_var(ii, jj), nonlin_vars_K);
						if ~isempty(symvar(grad))
							error('control:design:gamma:dimension', 'Symbolic nonlinear differential gain constraint mapping must be a linear expression.');
						end
						grad = double(grad) ~= 0;
						isvar_K(ii, jj) = sum(grad(:)) == 1;
						if sum(grad(:)) > 1
							error('control:design:gamma:dimension', 'Relation between symbolic nonlinear differential gain constraint mapping and gain matrix coefficients must be one to one.');
						end
					end
				end
				if ~any(isvar_K(:))
					error('control:design:gamma:dimension', 'Symbolic nonlinear differential gain constraint mapping must at least depend on one variable.');
				end
				isvar_F = false(size(F_nonlin_var));
				for ii = 1:size(F_nonlin_var, 1)
					for jj = 1:size(F_nonlin_var, 2)
						if numel(symvar(F_nonlin_var(ii, jj))) > 1
							error('control:design:gamma:dimension', 'Symbolic nonlinear prefilter constraint mapping must contain only one symbolic variable for every component of the gain matrix.');
						end
						grad = gradient(F_nonlin_var(ii, jj), nonlin_vars_F);
						if ~isempty(symvar(grad))
							error('control:design:gamma:dimension', 'Symbolic nonlinear prefilter constraint mapping must be a linear expression.');
						end
						grad = double(grad) ~= 0;
						isvar_F(ii, jj) = sum(grad(:)) == 1;
						if sum(grad(:)) > 1
							error('control:design:gamma:dimension', 'Relation between symbolic nonlinear prefilter constraint mapping and gain matrix coefficients must be one to one.');
						end
					end
				end
				if ~any(isvar_F(:))
					error('control:design:gamma:dimension', 'Symbolic nonlinear prefilter constraint mapping must at least depend on one variable.');
				end
				R_nonlin_depends = false(size(R_nonlin_sym, 1), size(R_nonlin_sym, 2), 3);
				for ii = 1:size(R_nonlin_sym, 1)
					for jj = 1:size(R_nonlin_sym, 2)
						if ~isnan(R_nonlin_sym(ii, jj))
							vars = reshape(symvar(R_nonlin_sym(ii, jj)), [], 1);
							if numel(vars) > size(nonlin_vars_R, 1) + size(nonlin_vars_K, 1) + size(nonlin_vars_F, 1)
								error('control:design:gamma:dimension', 'Symbolic nonlinear gain constraints must not depend on more variables than specified.');
							end
							dep_R = (sum(sym.depends(R_nonlin_sym(ii, jj), nonlin_vars_R), 1) ~= 0).';
							dep_K = (sum(sym.depends(R_nonlin_sym(ii, jj), nonlin_vars_K), 1) ~= 0).';
							dep_F = (sum(sym.depends(R_nonlin_sym(ii, jj), nonlin_vars_F), 1) ~= 0).';
							R_nonlin_depends(ii, jj, :) = [
								any(dep_R);
								any(dep_K);
								any(dep_F)
							];
							if (all(~dep_R(:)) && all(~dep_K(:)) && all(~dep_F(:))) || any(sum(sym.depends([
								nonlin_vars_R(dep_R, 1);
								nonlin_vars_K(dep_K, 1);
								nonlin_vars_F(dep_F, 1)
							], vars), 1) == 0)
								error('control:design:gamma:dimension', 'Symbolic nonlinear gain constraints must not depend on more proportional variables than specified.');
							end
							if (all(~dep_R(:)) && all(~dep_K(:)) && all(~dep_F(:))) || any(sum(sym.depends([
								nonlin_vars_R(dep_R, 1);
								nonlin_vars_K(dep_K, 1);
								nonlin_vars_F(dep_F, 1)
							], vars), 1) == 0)
								error('control:design:gamma:dimension', 'Symbolic nonlinear gain constraints must not depend on more differential variables than specified.');
							end
							if (all(~dep_R(:)) && all(~dep_K(:)) && all(~dep_F(:))) || any(sum(sym.depends([
								nonlin_vars_R(dep_R, 1);
								nonlin_vars_K(dep_K, 1);
								nonlin_vars_F(dep_F, 1)
							], vars), 1) == 0)
								error('control:design:gamma:dimension', 'Symbolic nonlinear gain constraints must not depend on more prefilter variables than specified.');
							end
							eq = char(R_nonlin_sym(ii, jj));
							if numel(strfind(eq, '<')) > 1
								error('control:design:gamma:dimension', 'Symbolic nonlinear constraints are invalid because element (%d,%d) contains ''<''.', ii, jj);
							end
							if numel(strfind(eq, '<=')) > 1
								error('control:design:gamma:dimension', 'Symbolic nonlinear constraints are invalid because element (%d,%d) contains ''<=''.', ii, jj);
							end
							if numel(strfind(eq, '>')) > 1
								error('control:design:gamma:dimension', 'Symbolic nonlinear constraints are invalid because element (%d,%d) contains ''>''.', ii, jj);
							end
							if numel(strfind(eq, '>=')) > 1
								error('control:design:gamma:dimension', 'Symbolic nonlinear constraints are invalid because element (%d,%d) contains ''>=''.', ii, jj);
							end
							eqfound = strfind(eq, '==');
							if ~isempty(eqfound)
								if numel(eqfound) > 1
									error('control:design:gamma:dimension', 'Symbolic linear proportional gain matrix coefficients are invalid because element (%d,%d) contains multiple ''==''.', ii, jj);
								end
								%isequ(ii, jj) = true;
							end
% 							if isequation(ii, jj)
% 								temp = children(R_fixed_sym(ii, jj));
% 								if numel(temp) > 2
% 									error('control:design:gamma:dimension', 'Symbolic linear proportional gain matrix constraints must be an equation.');
% 								end
% 								coefficient = gradient(temp(1), fixed_vars);
% 								if ~isempty(symvar(coefficient))
% 									error('control:design:gamma:dimension', 'Symbolic linear proportional gain matrix constraints must be linear, but left hand side of element (%d,%d) is nonlinear.', ii, jj);
% 								end
% 								coefficient = gradient(temp(2), fixed_vars);
% 								if ~isempty(symvar(coefficient))
% 									error('control:design:gamma:dimension', 'Symbolic linear proportional gain matrix constraints must be linear, but right hand side of element (%d,%d) is nonlinear.', ii, jj);
% 								end
% 							else
% 								coefficient = gradient(R_fixed_sym(ii, jj), fixed_vars);
% 								if ~isempty(symvar(coefficient))
% 									error('control:design:gamma:dimension', 'Symbolic linear proportional gain matrix constraints must be linear, but element (%d,%d) is nonlinear.', ii, jj);
% 								end
% 							end
							%coefficient_matrix(ii, jj, :) = double(coefficient);
							%if any(coefficient ~= 0)
							%	coefficient_matrix(ii, jj) = coefficient(coefficient ~= 0);
							%else
							%	coefficient_matrix(ii, jj) = 0;
							%end
						end
					end
				end
				R_nonlin_sym = reshape(R_nonlin_sym, [], 1);
				dep_R = reshape(R_nonlin_depends(:, :, 1), [], 1);
				dep_K = reshape(R_nonlin_depends(:, :, 2), [], 1);
				dep_F = reshape(R_nonlin_depends(:, :, 3), [], 1);
				iseq = sym.isequation(R_nonlin_sym, '==');
				isleq = sym.isequation(R_nonlin_sym, {'<=', '<'});
				isgeq = sym.isequation(R_nonlin_sym, {'>=', '>'});
				noneq = ~(iseq | isleq | isgeq);
				isineq = isleq | isgeq;
				if any(noneq(:))
					error('control:design:gamma:dimension', 'All symbolic nonlinear gain constraints must be symbolic equations.');
				end
				diffeq = ~xor(iseq, isineq);
				if any(diffeq(:))
					error('control:design:gamma:dimension', 'All symbolic nonlinear gain constraints must be symbolic equations.');
				end
				for ii = 1:size(R_nonlin_sym, 1)
					temp = children(R_nonlin_sym(ii, 1));
					if numel(temp) ~= 2
						error('equationsToMatrixIneq:input', 'Symbolic expression must be an equation.');
					end
					if isgeq(ii, 1)
						lhs = -temp(1) + temp(2);
					else
						lhs = temp(1) - temp(2);
					end
					R_nonlin_sym(ii, 1) = lhs;
				end
				tempiseq = sym.isequation(R_nonlin_sym, 'any');
				if any(tempiseq(:))
					error('control:design:gamma:dimension', 'After reformulation, some symbolic nonlinear gain constraints are still symbolic equations, but should be simple expressions.');
				end
				funvars = [
					reshape(R_nonlin_var(isvar_R), [], 1);
					reshape(K_nonlin_var(isvar_K), [], 1);
					reshape(F_nonlin_var(isvar_F), [], 1)
				];
				if any(iseq & dep_R)
					ceq_R = matlabFunction(R_nonlin_sym(iseq & dep_R, :), 'Vars', funvars);
					gradceq_R = matlabFunction(jacobian(R_nonlin_sym(iseq & dep_R, :), reshape(R_nonlin_var(isvar_R), [], 1)), 'Vars', funvars);
				else
					ceq_R = [];
					gradceq_R = [];
				end
				if any(isineq & dep_R)
					c_R = matlabFunction(R_nonlin_sym(isineq & dep_R, :), 'Vars', funvars);
					gradc_R = matlabFunction(jacobian(R_nonlin_sym(isineq & dep_R, :), reshape(R_nonlin_var(isvar_R), [], 1)), 'Vars', funvars);
				else
					c_R = [];
					gradceq_R = [];
				end
				if any(iseq & dep_K)
					ceq_K = matlabFunction(R_nonlin_sym(iseq & dep_K, :), 'Vars', funvars);
					gradceq_K = matlabFunction(jacobian(R_nonlin_sym(isineq & dep_K, :), reshape(K_nonlin_var(isvar_K), [], 1)), 'Vars', funvars);
				else
					ceq_K = [];
					gradceq_K = [];
				end
				if any(isineq & dep_K)
					c_K = matlabFunction(R_nonlin_sym(isineq, :), 'Vars', funvars);
					gradc_K = matlabFunction(jacobian(R_nonlin_sym(isineq, :), reshape(K_nonlin_var(isvar_K), [], 1)), 'Vars', funvars);
				else
					c_K = [];
					gradc_K = [];
				end
				if any(iseq & dep_F)
					ceq_F = matlabFunction(R_nonlin_sym(iseq, :), 'Vars', funvars);
					gradceq_F = matlabFunction(jacobian(R_nonlin_sym(iseq, :), reshape(R_nonlin_var(isvar_F), [], 1)), 'Vars', funvars);
				else
					ceq_F = [];
					gradceq_F = [];
				end
				if any(isineq & dep_F)
					c_F = matlabFunction(R_nonlin_sym(isineq, :), 'Vars', funvars);
					gradc_F = matlabFunction(jacobian(R_nonlin_sym(isineq, :), reshape(R_nonlin_var(isvar_F), [], 1)), 'Vars', funvars);
				else
					c_F = [];
					gradc_F = [];
				end
				R_nonlin = convertsymfun(c_R, gradc_R, ceq_R, gradceq_R, c_K, gradc_K, ceq_K, gradceq_K, c_F, gradc_F, ceq_F, gradceq_F, isvar_R, isvar_K, isvar_F);
			else
				error('control:design:gamma:dimension', 'Symbolic nonlinear gain constraints must consist of constraint system and a mapping matrix of symbolic variables to proportional and differential gain and prefilter matrix positions.');
			end
		end
		if ~isfunctionhandle(R_nonlin)
			error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must be a function handle.');
		end
		R_test = zeros(number_controls, number_measurements);
		K_test = zeros(number_controls, number_measurements_xdot);
		F_test = zeros(number_controls, number_references);
		if nargin(R_nonlin) == 2
			if nargout(R_nonlin) == 8
				try
					[c_R, ceq_R, c_K, ceq_K, gradc_R, gradceq_R, gradc_K, gradceq_K] = callnonlcon2(R_nonlin, R_test, K_test);
					hasnonlingrad = true;
				catch e
					ex = MException('control:design:gamma:nonlin', 'Nonlinear gain constraint function must not throw an exception of type ''%s'' with message ''%s''.', e.identifier, e.message);
					ex.addCause(e);
					throw(ex);
				end
			else
				if nargout(R_nonlin) == -1
					try
						[c_R, ceq_R, c_K, ceq_K, gradc_R, gradceq_R, gradc_K, gradceq_K] = callnonlcon2(R_nonlin, R_test, K_test);
						hasnonlingrad = true;
					catch e
						if ~strcmpi(e.identifier, 'MATLAB:maxlhs') && ~strcmpi(e.identifier, 'MATLAB:TooManyOutputs') && ~strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
							ex = MException('control:design:gamma:nonlin', 'Nonlinear gain constraint function must not throw an exception of type ''%s'' with message ''%s''.', e.identifier, e.message);
							ex.addCause(e);
							throw(ex);
						else
							try
								[c_R, ceq_R, c_K, ceq_K] = callnonlcon2(R_nonlin, R_test, K_test);
							catch e2
								ex = MException('control:design:gamma:nonlin', 'Nonlinear gain constraint function must not throw an exception of type ''%s'' with message ''%s''.', e2.identifier, e2.message);
								ex.addCause(e2);
								throw(ex);
							end
							gradc_R = NaN(number_controls, number_measurements, size(c_R, 1));
							gradceq_R = NaN(number_controls, number_measurements, size(ceq_R, 1));
							gradc_K = NaN(number_controls, number_measurements_xdot, size(c_K, 1));
							gradceq_K = NaN(number_controls, number_measurements_xdot, size(ceq_K, 1));
							hasnonlingrad = false;
						end
					end
				else
					try
						[c_R, ceq_R, c_K, ceq_K] = callnonlcon2(R_nonlin, R_test, K_test);
					catch e
						ex = MException('control:design:gamma:nonlin', 'Nonlinear gain constraint function must not throw an exception of type ''%s'' with message ''%s''.', e.identifier, e.message);
						ex.addCause(e);
						throw(ex);
					end
					gradc_R = NaN(number_controls, number_measurements, size(c_R, 1));
					gradceq_R = NaN(number_controls, number_measurements, size(ceq_R, 1));
					gradc_K = NaN(number_controls, number_measurements_xdot, size(c_K, 1));
					gradceq_K = NaN(number_controls, number_measurements_xdot, size(ceq_K, 1));
					hasnonlingrad = false;
				end
			end
			if isempty(c_R)
				number_c_R = 0;
				if ~isempty(gradc_R)
					error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return an empty gradient for empty proportional gain inequality constraints.');
				end
			else
				number_c_R = size(c_R, 1);
				if size(c_R, 2) ~= 1
					error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a column vector of proportional gain inequality constraints.');
				end
				if size(gradc_R, 3) ~= 1  && ndims(gradc_R) ~= 3
					error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a three dimensional gradient of proportional gain inequality constraints.');
				end
				if number_c_R ~= size(gradc_R, 3)
					error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a %dX%dX%d gradient of proportional gain inequality constraints with %d elements in the third dimension, not %d.', number_controls, number_measurements, number_c_R, number_c_R, size(c_R, 3));
				end
				if size(gradc_R, 1) ~= number_controls
					error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a %dX%dX%d gradient of proportional gain inequality constraints with %d elements in the first dimension.', number_controls, number_measurements, number_c_R, number_controls, size(c_R, 1));
				end
				if size(gradc_R, 2) ~= number_measurements
					error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a %dX%dX%d gradient of proportional gain inequality constraints with %d elements in the second dimension.', number_controls, number_measurements, number_c_R, number_measurements, size(c_R, 2));
				end
			end
			if isempty(ceq_R)
				if ~isempty(gradceq_R)
					error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return an empty gradient for empty proportional gain equality constraints.');
				end
			else
				number_ceq_R = size(ceq_R, 1);
				if size(ceq_R, 2) ~= 1
					error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a column vector of proportional gain equality constraints.');
				end
				if size(gradceq_R, 3) ~= 1  && ndims(gradceq_R) ~= 3
					error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a three dimensional gradient of proportional gain equality constraints.');
				end
				if number_ceq_R ~= size(gradceq_R, 3)
					error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a %dX%dX%d gradient of proportional gain equality constraints with %d elements in the third dimension, not %d.', number_controls, number_measurements, number_ceq_R, number_c_R, size(ceq_R, 3));
				end
				if size(gradceq_R, 1) ~= number_controls
					error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a %dX%dX%d gradient of proportional gain equality constraints with %d elements in the first dimension.', number_controls, number_measurements, number_ceq_R, number_controls, size(ceq_R, 1));
				end
				if size(gradceq_R, 2) ~= number_measurements
					error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a %dX%dX%d gradient of proportional gain equality constraints with %d elements in the second dimension.', number_controls, number_measurements, number_ceq_R, number_measurements, size(ceq_R, 2));
				end
			end
			if isempty(c_K)
				number_c_K = 0;
				if ~isempty(gradc_K)
					error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return an empty gradient for empty derivative gain inequality constraints.');
				end
			else
				number_c_K = size(c_K, 1);
				if size(c_K, 2) ~= 1
					error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a column vector of derivative gain inequality constraints.');
				end
				if size(gradc_K, 3) ~= 1  && ndims(gradc_K) ~= 3
					error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a three dimensional gradient of derivative gain inequality constraints.');
				end
				if number_c_K ~= size(gradc_K, 3)
					error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a %dX%dX%d gradient of derivative gain inequality constraints with %d elements in the third dimension, not %d.', number_controls, number_measurements_xdot, number_c_K, number_c_K, size(c_K, 3));
				end
				if size(gradc_K, 1) ~= number_controls
					error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a %dX%dX%d gradient of derivative gain inequality constraints with %d elements in the first dimension.', number_controls, number_measurements_xdot, number_c_K, number_controls, size(c_K, 1));
				end
				if size(gradc_K, 2) ~= number_measurements_xdot
					error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a %dX%dX%d gradient of derivative gain inequality constraints with %d elements in the second dimension.', number_controls, number_measurements_xdot, number_c_K, number_measurements_xdot, size(c_K, 2));
				end
			end
			if isempty(ceq_K)
				if ~isempty(gradceq_K)
					error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return an empty gradient for empty derivative gain equality constraints.');
				end
			else
				number_ceq_K = size(ceq_K, 1);
				if size(ceq_K, 2) ~= 1
					error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a column vector of derivative gain equality constraints.');
				end
				if size(gradceq_K, 3) ~= 1  && ndims(gradceq_K) ~= 3
					error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a three dimensional gradient of derivative gain equality constraints.');
				end
				if number_ceq_K ~= size(gradceq_K, 3)
					error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a %dX%dX%d gradient of derivative gain equality constraints with %d elements in the third dimension, not %d.', number_controls, number_measurements_xdot, number_ceq_K, number_c_K, size(ceq_K, 3));
				end
				if size(gradceq_K, 1) ~= number_controls
					error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a %dX%dX%d gradient of derivative gain equality constraints with %d elements in the first dimension.', number_controls, number_measurements_xdot, number_ceq_K, number_controls, size(ceq_K, 1));
				end
				if size(gradceq_K, 2) ~= number_measurements_xdot
					error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a %dX%dX%d gradient of derivative gain equality constraints with %d elements in the second dimension.', number_controls, number_measurements_xdot, number_ceq_K, number_measurements_xdot, size(ceq_K, 2));
				end
			end
			R_nonlin = convert3input(R_nonlin, hasnonlingrad);
		end
		if nargin(R_nonlin) ~= 3
			error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must accept 3 arguments for proportional and derivative gain and prefilter.');
		end
		if nargout(R_nonlin) == 12
			try
				[c_R, ceq_R, c_K, ceq_K, c_F, ceq_F, gradc_R, gradceq_R, gradc_K, gradceq_K, gradc_F, gradceq_F] = callnonlcon(R_nonlin, R_test, K_test, F_test);
				hasnonlingrad = true;
				hasnonlinfun = true;
			catch e
				ex = MException('control:design:gamma:nonlin', 'Nonlinear gain constraint function must not throw an exception of type ''%s'' with message ''%s''.', e.identifier, e.message);
				ex.addCause(e);
				throw(ex);
			end
		else
			if nargout(R_nonlin) == -1
				try
					[c_R, ceq_R, c_K, ceq_K, c_F, ceq_F, gradc_R, gradceq_R, gradc_K, gradceq_K, gradc_F, gradceq_F] = callnonlcon(R_nonlin, R_test, K_test, F_test);
					hasnonlingrad = true;
					hasnonlinfun = true;
				catch e
					if ~strcmpi(e.identifier, 'MATLAB:maxlhs') && ~strcmpi(e.identifier, 'MATLAB:TooManyOutputs') && ~strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
						ex = MException('control:design:gamma:nonlin', 'Nonlinear gain constraint function must not throw an exception of type ''%s'' with message ''%s''.', e.identifier, e.message);
						ex.addCause(e);
						throw(ex);
					else
						try
							[c_R, ceq_R, c_K, ceq_K, c_F, ceq_F] = callnonlcon(R_nonlin, R_test, K_test, F_test);
							hasnonlinfun = true;
						catch e2
							ex = MException('control:design:gamma:nonlin', 'Nonlinear gain constraint function must not throw an exception of type ''%s'' with message ''%s''.', e2.identifier, e2.message);
							ex.addCause(e2);
							throw(ex);
						end
						hasnonlingrad = false;
						gradc_R = NaN(number_controls, number_measurements, size(c_R, 1));
						gradceq_R = NaN(number_controls, number_measurements, size(ceq_R, 1));
						gradc_K = NaN(number_controls, number_measurements_xdot, size(c_K, 1));
						gradceq_K = NaN(number_controls, number_measurements_xdot, size(ceq_K, 1));
						gradc_F = NaN(number_controls, number_references, size(c_F, 1));
						gradceq_F = NaN(number_controls, number_references, size(ceq_F, 1));
					end
				end
			else
				try
					[c_R, ceq_R, c_K, ceq_K, c_F, ceq_F] = callnonlcon(R_nonlin, R_test, K_test, F_test);
					hasnonlinfun = true;
				catch e
					ex = MException('control:design:gamma:nonlin', 'Nonlinear gain constraint function must not throw an exception of type ''%s'' with message ''%s''.', e.identifier, e.message);
					ex.addCause(e);
					throw(ex);
				end
				hasnonlingrad = false;
				gradc_R = NaN(number_controls, number_measurements, size(c_R, 1));
				gradceq_R = NaN(number_controls, number_measurements, size(ceq_R, 1));
				gradc_K = NaN(number_controls, number_measurements_xdot, size(c_K, 1));
				gradceq_K = NaN(number_controls, number_measurements_xdot, size(ceq_K, 1));
				gradc_F = NaN(number_controls, number_references, size(c_F, 1));
				gradceq_F = NaN(number_controls, number_references, size(ceq_F, 1));
			end
		end
		if isempty(c_R)
			number_c_R = 0;
			if ~isempty(gradc_R)
				error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return an empty gradient for empty proportional gain inequality constraints.');
			end
		else
			number_c_R = size(c_R, 1);
			if size(c_R, 2) ~= 1
				error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a column vector of proportional gain inequality constraints.');
			end
			if size(gradc_R, 3) ~= 1  && ndims(gradc_R) ~= 3
				error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a three dimensional gradient of proportional gain inequality constraints.');
			end
			if number_c_R ~= size(gradc_R, 3)
				error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a %dX%dX%d gradient of proportional gain inequality constraints with %d elements in the third dimension, not %d.', number_controls, number_measurements, number_c_R, number_c_R, size(c_R, 3));
			end
			if size(gradc_R, 1) ~= number_controls
				error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a %dX%dX%d gradient of proportional gain inequality constraints with %d elements in the first dimension.', number_controls, number_measurements, number_c_R, number_controls, size(c_R, 1));
			end
			if size(gradc_R, 2) ~= number_measurements
				error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a %dX%dX%d gradient of proportional gain inequality constraints with %d elements in the second dimension.', number_controls, number_measurements, number_c_R, number_measurements, size(c_R, 2));
			end
		end
		if isempty(ceq_R)
			number_ceq_R = 0;
			if ~isempty(gradceq_R)
				error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return an empty gradient for empty proportional gain equality constraints.');
			end
		else
			number_ceq_R = size(ceq_R, 1);
			if size(ceq_R, 2) ~= 1
				error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a column vector of proportional gain equality constraints.');
			end
			if size(gradceq_R, 3) ~= 1  && ndims(gradceq_R) ~= 3
				error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a three dimensional gradient of proportional gain equality constraints.');
			end
			if number_ceq_R ~= size(gradceq_R, 3)
				error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a %dX%dX%d gradient of proportional gain equality constraints with %d elements in the third dimension, not %d.', number_controls, number_measurements, number_ceq_R, number_c_R, size(ceq_R, 3));
			end
			if size(gradceq_R, 1) ~= number_controls
				error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a %dX%dX%d gradient of proportional gain equality constraints with %d elements in the first dimension.', number_controls, number_measurements, number_ceq_R, number_controls, size(ceq_R, 1));
			end
			if size(gradceq_R, 2) ~= number_measurements
				error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a %dX%dX%d gradient of proportional gain equality constraints with %d elements in the second dimension.', number_controls, number_measurements, number_ceq_R, number_measurements, size(ceq_R, 2));
			end
		end
		if isempty(c_K)
			number_c_K = 0;
			if ~isempty(gradc_K)
				error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return an empty gradient for empty derivative gain inequality constraints.');
			end
		else
			number_c_K = size(c_K, 1);
			if size(c_K, 2) ~= 1
				error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a column vector of derivative gain inequality constraints.');
			end
			if size(gradc_K, 3) ~= 1  && ndims(gradc_K) ~= 3
				error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a three dimensional gradient of derivative gain inequality constraints.');
			end
			if number_c_K ~= size(gradc_K, 3)
				error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a %dX%dX%d gradient of derivative gain inequality constraints with %d elements in the third dimension, not %d.', number_controls, number_measurements_xdot, number_c_K, number_c_K, size(c_K, 3));
			end
			if size(gradc_K, 1) ~= number_controls
				error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a %dX%dX%d gradient of derivative gain inequality constraints with %d elements in the first dimension.', number_controls, number_measurements_xdot, number_c_K, number_controls, size(c_K, 1));
			end
			if size(gradc_K, 2) ~= number_measurements_xdot
				error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a %dX%dX%d gradient of derivative gain inequality constraints with %d elements in the second dimension.', number_controls, number_measurements_xdot, number_c_K, number_measurements_xdot, size(c_K, 2));
			end
		end
		if isempty(ceq_K)
			number_ceq_K = 0;
			if ~isempty(gradceq_K)
				error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return an empty gradient for empty derivative gain equality constraints.');
			end
		else
			number_ceq_K = size(ceq_K, 1);
			if size(ceq_K, 2) ~= 1
				error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a column vector of derivative gain equality constraints.');
			end
			if size(gradceq_K, 3) ~= 1  && ndims(gradceq_K) ~= 3
				error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a three dimensional gradient of derivative gain equality constraints.');
			end
			if number_ceq_K ~= size(gradceq_K, 3)
				error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a %dX%dX%d gradient of derivative gain equality constraints with %d elements in the third dimension, not %d.', number_controls, number_measurements_xdot, number_ceq_K, number_c_K, size(ceq_K, 3));
			end
			if size(gradceq_K, 1) ~= number_controls
				error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a %dX%dX%d gradient of derivative gain equality constraints with %d elements in the first dimension.', number_controls, number_measurements_xdot, number_ceq_K, number_controls, size(ceq_K, 1));
			end
			if size(gradceq_K, 2) ~= number_measurements_xdot
				error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a %dX%dX%d gradient of derivative gain equality constraints with %d elements in the second dimension.', number_controls, number_measurements_xdot, number_ceq_K, number_measurements_xdot, size(ceq_K, 2));
			end
		end
		if isempty(c_F)
			number_c_F = 0;
			if ~isempty(gradc_F)
				error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return an empty gradient for empty prefilter inequality constraints.');
			end
		else
			number_c_F = size(c_F, 1);
			if size(c_F, 2) ~= 1
				error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a column vector of prefilter inequality constraints.');
			end
			if size(gradc_F, 3) ~= 1  && ndims(gradc_F) ~= 3
				error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a three dimensional gradient of prefilter inequality constraints.');
			end
			if number_c_F ~= size(gradc_F, 3)
				error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a %dX%dX%d gradient of prefilter inequality constraints with %d elements in the third dimension, not %d.', number_controls, number_references, number_c_F, number_c_F, size(c_F, 3));
			end
			if size(gradc_F, 1) ~= number_controls
				error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a %dX%dX%d gradient of prefilter inequality constraints with %d elements in the first dimension.', number_controls, number_references, number_c_F, number_controls, size(c_F, 1));
			end
			if size(gradc_F, 2) ~= number_references
				error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a %dX%dX%d gradient of prefilter inequality constraints with %d elements in the second dimension.', number_controls, number_references, number_c_F, number_references, size(c_F, 2));
			end
		end
		if isempty(ceq_F)
			number_ceq_F = 0;
			if ~isempty(gradceq_F)
				error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return an empty gradient for empty prefilter equality constraints.');
			end
		else
			number_ceq_F = size(ceq_F, 1);
			if size(ceq_F, 2) ~= 1
				error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a column vector of prefilter equality constraints.');
			end
			if size(gradceq_F, 3) ~= 1  && ndims(gradceq_F) ~= 3
				error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a three dimensional gradient of prefilter equality constraints.');
			end
			if number_ceq_F ~= size(gradceq_F, 3)
				error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a %dX%dX%d gradient of prefilter equality constraints with %d elements in the third dimension, not %d.', number_controls, number_references, number_ceq_F, number_c_F, size(ceq_F, 3));
			end
			if size(gradceq_F, 1) ~= number_controls
				error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a %dX%dX%d gradient of prefilter equality constraints with %d elements in the first dimension.', number_controls, number_references, number_ceq_F, number_controls, size(ceq_F, 1));
			end
			if size(gradceq_F, 2) ~= number_references
				error('control:design:gamma:nonlin', 'Nonlinear gain constraint function must return a %dX%dX%d gradient of prefilter equality constraints with %d elements in the second dimension.', number_controls, number_references, number_ceq_F, number_references, size(ceq_F, 2));
			end
		end
		if T_nonlin.transform
			R_nonlin = transform_R_nonlin(R_nonlin, T_nonlin, number_controls, number_measurements, number_measurements_xdot);
		end
	end
end

function [R_nonlin_transformed] = transform_R_nonlin(R_nonlin, T_nonlin, number_controls, number_measurements, number_measurements_xdot)
	function [c_R, ceq_R, c_K, ceq_K, c_F, ceq_F, gradc_R, gradceq_R, gradc_K, gradceq_K, gradc_F, gradceq_F] = transformed_R_nonlin(R, K, F)
		r_hat = reshape(R, number_controls*number_measurements, 1);
		k_hat = reshape(K, number_controls*number_measurements_xdot, 1);
		f_hat = reshape(F, number_controls*T_nonlin.number_controls_original, 1);

		r = T_nonlin.K_A*r_hat + T_nonlin.K_b;
		k = T_nonlin.D_A*k_hat + T_nonlin.D_b;
		f = T_nonlin.F_A*f_hat + T_nonlin.F_b;

		% R, K, F in original dimensions
		R_original = reshape(r, T_nonlin.number_controls_original, T_nonlin.number_states_original);
		K_original = reshape(k, T_nonlin.number_controls_original, number_measurements_xdot);
		F_original = reshape(f, T_nonlin.number_controls_original, T_nonlin.number_controls_original);

		if nargout >= 7
			[c_R, ceq_R, c_K, ceq_K, c_F, ceq_F, gradc_R, gradceq_R, gradc_K, gradceq_K, gradc_F, gradceq_F] = R_nonlin(R_original, K_original, F_original); % TODO: hier gibt es eine Schleife, sodass zweimal verschachtelt transformed_K_nonlin aufgerufen wird.
			if ~isempty(gradc_R)
				[sz1, sz2, sz3] = size(grad_K);
				gradc_R = reshape(gradc_R, sz1*sz2, sz3);
				gradc_R = (gradc_R.'*T_nonlin.K_A).';
				gradc_R = reshape(gradc_R, number_controls, number_measurements, sz3); % grad_K_hat
			end
			if ~isempty(gradc_K)
				[sz1, sz2, sz3] = size(grad_D);
				gradc_K = reshape(gradc_K, sz1*sz2, sz3);
				gradc_K = (gradc_K.'*T_nonlin.D_A).';
				gradc_K = reshape(gradc_K, number_controls, number_measurements_xdot, sz3); % grad_D_hat
			end
			if ~isempty(gradc_F)
				[sz1, sz2, sz3] = size(grad_F);
				gradc_F = reshape(gradc_F, sz1*sz2, sz3);
				gradc_F = (gradc_F.'*T_nonlin.F_A).';
				gradc_F = reshape(gradc_F, number_controls, T_nonlin.number_controls_original, sz3); % grad_F_hat
			end
			if ~isempty(gradceq_R)
				[sz1, sz2, sz3] = size(gradceq_R);
				gradceq_R = reshape(gradceq_R, sz1*sz2, sz3);
				gradceq_R = (gradceq_R.'*T_nonlin.K_A).';
				gradceq_R = reshape(gradceq_R, number_controls, number_measurements, sz3); % gradceq_K_hat
			end
			if ~isempty(gradceq_K)
				[sz1, sz2, sz3] = size(gradceq_K);
				gradceq_K = reshape(gradceq_K, sz1*sz2, sz3);
				gradceq_K = (gradceq_K.'*T_nonlin.D_A).';
				gradceq_K = reshape(gradceq_K, number_controls, number_measurements_xdot, sz3); % gradceq_D_hat
			end
			if ~isempty(gradceq_F)
				[sz1, sz2, sz3] = size(gradceq_F);
				gradceq_F = reshape(gradceq_F, sz1*sz2, sz3);
				gradceq_F = (gradceq_F.'*T_nonlin.F_A).';
				gradceq_F = reshape(gradceq_F, number_controls, T_nonlin.number_controls_original, sz3); % gradceq_F_hat
			end
		else
			[c_R, ceq_R, c_K, ceq_K, c_F, ceq_F] = R_nonlin(R, K, F);
		end
	end
	R_nonlin_transformed = @transformed_R_nonlin;
end

function [T_nonlin] = check_T_nonlin_Ab(T_nonlin, field_A, field_b, dim_A1, dim_A2, default_A, default_b)
	% check A
	if isfield(T_nonlin, field_A)
		if ~isempty(T_nonlin.(field_A))
			if size(T_nonlin.(field_A), 1) ~= dim_A1 || size(T_nonlin.(field_A), 2) ~= dim_A2
				error('control:design:gamma:dimensions', 'T_nonlin.%s must be a %dX%d matrix.', field_A, dim_A1, dim_A2);
			end
			if size(T_nonlin.(field_A), 3) > 1
				error('control:design:gamma:dimensions', 'T_nonlin.%s must have size 1 in third dimension.', field_A);
			end
		else
			T_nonlin.(field_A) = default_A;
		end
	else
		T_nonlin.(field_A) = default_A;
	end

	% check b
	if isfield(T_nonlin, field_b)
		if ~isempty(T_nonlin.(field_b))
			if size(T_nonlin.(field_b), 1) ~= dim_A1 || size(T_nonlin.K_b, 2) ~= 1
				error('control:design:gamma:dimensions', 'T_nonlin.%s must be a %dX1 matrix.', field_b, number_states*number_controls);
			end
			if size(T_nonlin.(field_b), 3) > 1
				error('control:design:gamma:dimensions', 'T_nonlin.%s must have size 1 in third dimension.', field_b);
			end
		else
			T_nonlin.(field_b) = default_b;
		end
	else
		T_nonlin.(field_b) = default_b;
	end
end

function [c_R, ceq_R, c_K, ceq_K, c_F, ceq_F, gradc_R, gradceq_R, gradc_K, gradceq_K, gradc_F, gradceq_F] = callnonlcon(nonlcon, R, K, F)
	%CALLNONLCON wrapper function for calling nonlinear constraint function for calculating dimensions of constraints
	%	Input:
	%		nonlcon:	function to call
	%		R:			proportional gain matrix
	%		K:			derivative gain matrix
	%		F:			prefilter matrix
	%	Output:
	%		c_R:		inequality constraints on proportional gain matrix
	%		ceq_R:		equality constraints on proportional gain matrix
	%		c_K:		inequality constraints on derivative gain matrix
	%		ceq_K:		equality constraints on derivative gain matrix
	%		c_F:		inequality constraints on prefilter matrix
	%		ceq_F:		equality constraints on prefilter matrix
	%		gradc_R:	gradient of inequality constraints on proportional gain matrix
	%		gradceq_R:	gradient of equality constraints on proportional gain matrix
	%		gradc_K:	gradient of inequality constraints on derivative gain matrix
	%		gradceq_K:	gradient of equality constraints on derivative gain matrix
	%		gradc_F:	gradient of inequality constraints on prefilter matrix
	%		gradceq_F:	gradient of equality constraints on prefilter matrix
	if nargout >= 7
		[c_R, ceq_R, c_K, ceq_K, c_F, ceq_F, gradc_R, gradceq_R, gradc_K, gradceq_K, gradc_F, gradceq_F] = nonlcon(R, K, F);
	else
		[c_R, ceq_R, c_K, ceq_K, c_F, ceq_F] = nonlcon(R, K, F);
	end
end

function [c_R, ceq_R, c_K, ceq_K, gradc_R, gradceq_R, gradc_K, gradceq_K] = callnonlcon2(nonlcon, R, K)
	%CALLNONLCON2 wrapper function for calling nonlinear constraint function for calculating dimensions of constraints and two input arguments
	%	Input:
	%		nonlcon:	function to call
	%		R:			proportional gain matrix
	%		K:			derivative gain matrix
	%	Output:
	%		c_R:		inequality constraints on proportional gain matrix
	%		ceq_R:		equality constraints on proportional gain matrix
	%		c_K:		inequality constraints on derivative gain matrix
	%		ceq_K:		equality constraints on derivative gain matrix
	%		gradc_R:	gradient of inequality constraints on proportional gain matrix
	%		gradceq_R:	gradient of equality constraints on proportional gain matrix
	%		gradc_K:	gradient of inequality constraints on derivative gain matrix
	%		gradceq_K:	gradient of equality constraints on derivative gain matrix
	if nargout >= 5
		[c_R, ceq_R, c_K, ceq_K, gradc_R, gradceq_R, gradc_K, gradceq_K] = nonlcon(R, K);
	else
		[c_R, ceq_R, c_K, ceq_K] = nonlcon(R, K);
	end
end

function [R_nonlin] = convertsymfun(fun_c_R, fun_gradc_R, fun_ceq_R, fun_gradceq_R, fun_c_K, fun_gradc_K, fun_ceq_K, fun_gradceq_K, fun_c_F, fun_gradc_F, fun_ceq_F, fun_gradceq_F, isvarR, isvarK, isvarF)
	%CONVERTSYMFUN convert symbolic nonlinear gain constraint functions to a faunction handle to be accepted by gammasyn
	%	Input:
	%		fun_c_R:		nonlinear proportional gain matrix inequality constraint function
	%		fun_gradc_R:	gradient function of nonlinear proportional gain matrix inequality constraints
	%		fun_ceq_R:		nonlinear proportional gain matrix equality constraint function
	%		fun_gradceq_R:	gradient function of nonlinear proportional gain matrix equality constraints
	%		fun_c_K:		nonlinear differential gain matrix inequality constraint function
	%		fun_gradc_K:	gradient function of nonlinear differential gain matrix inequality constraints
	%		fun_ceq_K:		nonlinear differential gain matrix equality constraint function
	%		fun_gradceq_K:	gradient function of nonlinear differential gain matrix equality constraints
	%		fun_c_F:		nonlinear prefilter matrix inequality constraint function
	%		fun_gradc_F:	gradient function of nonlinear prefilter matrix inequality constraints
	%		fun_ceq_F:		nonlinear prefilter matrix equality constraint function
	%		fun_gradceq_F:	gradient function of nonlinear prefilter matrix equality constraints
	%		isvarR:			indicator matrix with variable proportional gain coefficients
	%		isvarK:			indicator matrix with variable derivative gain coefficients
	%		isvarF:			indicator matrix with variable prefilter coefficients
	%	Output:
	%		R_nonlin:		nonlinear gain constraint function accepted by gammasyn
	linidx_R = find(isvarR);
	[idx1_R, idx2_R] = ind2sub(size(isvarR), linidx_R);
	linidx_K = find(isvarK);
	[idx1_K, idx2_K] = ind2sub(size(isvarK), linidx_K);
	linidx_F = find(isvarF);
	[idx1_F, idx2_F] = ind2sub(size(isvarF), linidx_F);
	function [c_R, ceq_R, c_K, ceq_K, c_F, ceq_F, gradc_R, gradceq_R, gradc_K, gradceq_K, gradc_F, gradceq_F] = constraintfun(R, K, F)
		r = num2cell(R(isvarR));
		d = num2cell(K(isvarK));
		f = num2cell(F(isvarF));
		if ~isempty(fun_c_R)
			c_R = fun_c_R(r{:}, d{:}, f{:});
		else
			c_R = [];
		end
		if ~isempty(fun_ceq_R)
			ceq_R = fun_ceq_R(r{:}, d{:}, f{:});
		else
			ceq_R = [];
		end
		if ~isempty(fun_c_K)
			c_K = fun_c_K(r{:}, d{:}, f{:});
		else
			c_K = [];
		end
		if ~isempty(fun_ceq_K)
			ceq_K = fun_ceq_K(r{:}, d{:}, f{:});
		else
			ceq_K = [];
		end
		if ~isempty(fun_c_F)
			c_F = fun_c_F(r{:}, d{:}, f{:});
		else
			c_F = [];
		end
		if ~isempty(fun_ceq_F)
			ceq_F = fun_ceq_F(r{:}, d{:}, f{:});
		else
			ceq_F = [];
		end
		if nargout >= 5
			if ~isempty(fun_gradc_R)
				gradc = fun_gradc_R(r{:}, d{:}, f{:});
				gradc_R = zeros([size(isvarR), size(c_R)]);
				for hh = 1:size(c_R, 1) %#ok<FORPF> parfor is not possible because of two level indexing
					for ii = 1:length(linidx_R)
						gradc_R(idx1_R(ii), idx2_R(ii), hh) = gradc(hh, ii);
					end
				end
			else
				gradc_R = [];
			end
			if ~isempty(fun_gradceq_R)
				gradceq = fun_gradceq_R(r{:}, d{:}, f{:});
				gradceq_R = zeros([size(isvarR), size(ceq_R)]);
				for hh = 1:size(ceq_R, 1) %#ok<FORPF> parfor is not possible because of two level indexing
					for ii = 1:length(linidx_R)
						gradceq_R(idx1_R(ii), idx2_R(ii), hh) = gradceq(hh, ii);
					end
				end
			else
				gradceq_R = [];
			end
			if ~isempty(fun_gradc_K)
				gradc = fun_gradc_K(r{:}, d{:}, f{:});
				gradc_K = zeros([size(isvarK), size(c_K)]);
				for hh = 1:size(c_K, 1) %#ok<FORPF> parfor is not possible because of two level indexing
					for ii = 1:length(linidx_K)
						gradc_K(idx1_K(ii), idx2_K(ii), hh) = gradc(hh, ii);
					end
				end
			else
				gradc_K = [];
			end
			if ~isempty(fun_gradceq_K)
				gradceq = fun_gradceq_K(r{:}, d{:}, f{:});
				gradceq_K = zeros([size(isvarK), size(ceq_K)]);
				for hh = 1:size(ceq_K, 1) %#ok<FORPF> parfor is not possible because of two level indexing
					for ii = 1:length(linidx_K)
						gradceq_K(idx1_K(ii), idx2_K(ii), hh) = gradceq(hh, ii);
					end
				end
			else
				gradceq_K = [];
			end
			if ~isempty(fun_gradc_F)
				gradc = fun_gradc_F(r{:}, d{:}, f{:});
				gradc_F = zeros([size(isvarF), size(c_F)]);
				for hh = 1:size(c_F, 1) %#ok<FORPF> parfor is not possible because of two level indexing
					for ii = 1:length(linidx_F)
						gradc_F(idx1_F(ii), idx2_F(ii), hh) = gradc(hh, ii);
					end
				end
			else
				gradc_F = [];
			end
			if ~isempty(fun_gradceq_F)
				gradceq = fun_gradceq_F(r{:}, d{:}, f{:});
				gradceq_F = zeros([size(isvarF), size(ceq_F)]);
				for hh = 1:size(ceq_F, 1) %#ok<FORPF> parfor is not possible because of two level indexing
					for ii = 1:length(linidx_F)
						gradceq_F(idx1_F(ii), idx2_F(ii), hh) = gradceq(hh, ii);
					end
				end
			else
				gradceq_F = [];
			end
		end
	end
	R_nonlin = @constraintfun;
end

function [fun] = convert3input(R_nonlin, hasnonlingrad)
	%CONVERT3INPUT convert nonlinear constraint function with 2 inputs to function with 3 inputs
	%	Input:
	%		R_nonlin:	function handle to convert
	%	Output:
	%		fun:		converted function handle
	function [c_R, ceq_R, c_K, ceq_K, c_F, ceq_F, gradc_R, gradceq_R, gradc_K, gradceq_K, gradc_F, gradceq_F] = constraintfungrad(R, K, ~)
		if nargout >= 5
			[c_R, ceq_R, c_K, ceq_K, gradc_R, gradceq_R, gradc_K, gradceq_K] = R_nonlin(R, K);
			c_F = [];
			ceq_F = [];
			gradc_F = [];
			gradceq_F = [];
		else
			[c_R, ceq_R, c_K, ceq_K] = R_nonlin(R, K);
			c_F = [];
			ceq_F = [];
		end
	end
	function [c_R, ceq_R, c_K, ceq_K, c_F, ceq_F] = constraintfun(R, K, ~)
		[c_R, ceq_R, c_K, ceq_K] = R_nonlin(R, K);
		c_F = [];
		ceq_F = [];
	end
	if hasnonlingrad
		fun = @constraintfungrad;
	else
		fun = @constraintfun;
	end
end
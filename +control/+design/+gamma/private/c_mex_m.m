function [c, ceq, gradc, gradceq, hessc, hessceq] = c_mex_m(x, system, weight, areafun, dimensions, options)
	%C_MEX_M calculate constraint function and gradient for gamma pole placement
	%	Input:
	%		x:			current optimization value (gain reshaped as column vector)
	%		system:		structure with system matrices of systems to take into consideration
	%		weight:		weighting matrix with number of systems columns and number of pole area border functions rows
	%		areafun:	area border functions and gradients as cell array with number of system entries, each function returns a row vector with as much elements as area border functions are defined for the corresponding system
	%		dimensions:	structure with information about dimensions of the different variables and systems
	%		options:	structure with options for objective function
	%	Output:
	%		c:			inequality constraint function value for current optimization value
	%		ceq:		equality constraint function value for current optimization value
	%		gradc:		gradient of inequality constraint function value for current optimization value
	%		gradceq:	gradient of equality constraint function value for current optimization value
	%		hessc:		hessian of inequality constraint function value for current optimization value
	%		hessceq:	hessian of equality constraint function value for current optimization value
	if nargout >= 2 && ~dimensions.area_hasgrad
		error('control:design:gamma:gradient', 'Gradient for polearea functions was not supplied.');
	end
	if nargout >= 5 && ~dimensions.area_hashess
		error('control:design:gamma:hessian', 'Hessian for polearea functions was not supplied.');
	end
	ceq = [];
	if nargout >= 4
		gradceq = [];
		if nargout >= 6
			hessceq = [];
		end
	end
	number_models = dimensions.models;
	number_states = dimensions.states;
	number_areas_max = dimensions.areas_max;
	number_controls = dimensions.controls;
	number_measurements = dimensions.measurements;
	number_measurements_xdot = dimensions.measurements_xdot;
	number_references = dimensions.references;
	derivative_feedback = number_measurements_xdot > 0;
	R_fixed_has = dimensions.R_fixed_has;
	K_fixed_has = dimensions.K_fixed_has;
	F_fixed_has = dimensions.F_fixed_has;
	RKF_fixed_has = dimensions.RKF_fixed_has;
	numthreads = options.numthreads;
	eigenvaluederivativetype = options.eigenvaluederivative;
	[R, K, ~] = x2R(x, dimensions);
	if nargout >= 5
		if ~isempty(areafun)
			if derivative_feedback
				[eigenvalues, ~, ~, eigenvalue_derivative, eigenvalue_derivative_xdot, ~, ~, ~, ~, eigenvalue_2derivative, eigenvalue_2derivative_xdot, eigenvalue_2derivative_mixed, eigenvalue_2derivative_xdot_mixed] = calculate_eigenvalues_m(system, R, K, dimensions, eigenvaluederivativetype, numthreads);
				[eigenvalues, ~, ~, eigenvalue_derivative, eigenvalue_derivative_xdot, ~, ~, ~, ~, eigenvalue_2derivative, eigenvalue_2derivative_xdot, eigenvalue_2derivative_mixed, eigenvalue_2derivative_xdot_mixed] = calculate_eigenvalue_filter(options.eigenvaluefilter, eigenvalues, [], [], eigenvalue_derivative, eigenvalue_derivative_xdot, [], [], [], [], eigenvalue_2derivative, eigenvalue_2derivative_xdot, eigenvalue_2derivative_mixed, eigenvalue_2derivative_xdot_mixed);
				[areaval, areaval_derivative, areaval_2_derivative] = calculate_areas_fixed_m(areafun, weight, eigenvalues, dimensions, numthreads);
				gradtemp = calculate_constraint_gradient(weight, eigenvalue_derivative, eigenvalue_derivative_xdot, areaval_derivative, dimensions, numthreads);
				hesstemp = calculate_constraint_hessian(weight, eigenvalue_derivative, eigenvalue_derivative_xdot, areaval_derivative, dimensions, numthreads, areaval_2_derivative, eigenvalue_2derivative, eigenvalue_2derivative_xdot, eigenvalue_2derivative_mixed, eigenvalue_2derivative_xdot_mixed);
			else
				[eigenvalues, ~, ~, eigenvalue_derivative, ~, ~, ~, ~, ~, eigenvalue_2derivative] = calculate_eigenvalues_m(system, R, K, dimensions, eigenvaluederivativetype, numthreads);
				[eigenvalues, ~, ~, eigenvalue_derivative, ~, ~, ~, ~, ~, eigenvalue_2derivative] = calculate_eigenvalue_filter(options.eigenvaluefilter, eigenvalues, [], [], eigenvalue_derivative, [], [], [], [], [], eigenvalue_2derivative);
				[areaval, areaval_derivative, areaval_2_derivative] = calculate_areas_fixed_m(areafun, weight, eigenvalues, dimensions, numthreads);
				gradtemp = calculate_constraint_gradient(weight, eigenvalue_derivative, [], areaval_derivative, dimensions, numthreads);
				eigenvalue_derivative_xdot = NaN(number_states, number_controls, number_measurements_xdot, number_models) + 0i;
				hesstemp = calculate_constraint_hessian(weight, eigenvalue_derivative, eigenvalue_derivative_xdot, areaval_derivative, dimensions, numthreads, areaval_2_derivative, eigenvalue_2derivative);
			end
			c = reshape(areaval, number_models*number_areas_max*number_states, 1);
			removec = isnan(c);
			c(removec) = [];
			gradc = reshape(gradtemp, number_models*number_areas_max*number_states, number_controls*(number_measurements + number_measurements_xdot + number_references)).';
			gradc(:, removec) = [];
			hessc = reshape(hesstemp, number_models*number_areas_max*number_states, number_controls*(number_measurements + number_measurements_xdot + number_references), number_controls*(number_measurements + number_measurements_xdot + number_references));
			hessc(removec, :, :) = [];
		else
			c = zeros(0, 1);
			gradc = zeros(number_controls*(number_measurements + number_measurements_xdot + number_references), 0);
			hessc = zeros(0, number_controls*(number_measurements + number_measurements_xdot + number_references), number_controls*(number_measurements + number_measurements_xdot + number_references));
		end
		if RKF_fixed_has
			gradc = dimensions.RKF_fixed_T_inv'*gradc;% transformed here to avoid another loop in calculate_constraint_gradient
			gradc = gradc(dimensions.index_RKF_free, :);
			hessc = hessc(:, dimensions.index_RKF_free, dimensions.index_RKF_free);
		elseif R_fixed_has || K_fixed_has || F_fixed_has
			gradc = gradc(dimensions.index_all_free, :);
			hessc = hessc(:, dimensions.index_all_free, dimensions.index_all_free);
		end
		hessc = permute(hessc, [2, 3, 1]);% TODO: can this be combined with reshape?
	elseif nargout >= 3
		if ~isempty(areafun)
			if derivative_feedback
				[eigenvalues, ~, ~, eigenvalue_derivative, eigenvalue_derivative_xdot] = calculate_eigenvalues_m(system, R, K, dimensions, eigenvaluederivativetype, numthreads);
				[eigenvalues, ~, ~, eigenvalue_derivative, eigenvalue_derivative_xdot] = calculate_eigenvalue_filter(options.eigenvaluefilter, eigenvalues, [], [], eigenvalue_derivative, eigenvalue_derivative_xdot);
				[areaval, areaval_derivative] = calculate_areas_fixed_m(areafun, weight, eigenvalues, dimensions, numthreads);
				gradtemp = calculate_constraint_gradient(weight, eigenvalue_derivative, eigenvalue_derivative_xdot, areaval_derivative, dimensions, numthreads);
			else
				[eigenvalues, ~, ~, eigenvalue_derivative] = calculate_eigenvalues_m(system, R, K, dimensions, eigenvaluederivativetype, numthreads);
				[eigenvalues, ~, ~, eigenvalue_derivative] = calculate_eigenvalue_filter(options.eigenvaluefilter, eigenvalues, [], [], eigenvalue_derivative);
				[areaval, areaval_derivative] = calculate_areas_fixed_m(areafun, weight, eigenvalues, dimensions, numthreads);
				gradtemp = calculate_constraint_gradient(weight, eigenvalue_derivative, [], areaval_derivative, dimensions, numthreads);
			end
			c = reshape(areaval, number_models*number_areas_max*number_states, 1);
			removec = isnan(c);
			c(removec) = [];
			gradc = reshape(gradtemp, number_models*number_areas_max*number_states, number_controls*(number_measurements + number_measurements_xdot + number_references)).';
			gradc(:, removec) = [];
		else
			c = zeros(0, 1);
			gradc = zeros(number_controls*(number_measurements + number_measurements_xdot + number_references), 0);
		end
		if RKF_fixed_has
			gradc = dimensions.RKF_fixed_T_inv'*gradc;% transformed here to avoid another loop in calculate_constraint_gradient
			gradc = gradc(dimensions.index_RKF_free, :);
		elseif R_fixed_has || K_fixed_has || F_fixed_has
			gradc = gradc(dimensions.index_all_free, :);
		end
	else
		if ~isempty(areafun)
			eigenvalues = calculate_eigenvalues_m(system, R, K, dimensions, eigenvaluederivativetype, numthreads);
			eigenvalues = calculate_eigenvalue_filter(options.eigenvaluefilter, eigenvalues);
			areaval = calculate_areas_fixed_m(areafun, weight, eigenvalues, dimensions, numthreads);
			c = reshape(areaval, number_models*number_areas_max*number_states, 1);
			c(isnan(c)) = [];
		else
			c = zeros(0, 1);
		end
	end
end
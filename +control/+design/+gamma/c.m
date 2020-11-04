function [c, ceq, gradc, gradceq, hessc, hessceq] = c(x, system, weight, areafun, dimensions, options)
	%C calculate constraint function and gradient for gamma pole placement
	%This function is functionally equivalent to c_mex_* except, that it supports function handles in areafun, because it does not get compiled
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
	number_coefficients = number_controls*(number_measurements + number_measurements_xdot + number_references);
	derivative_feedback = number_measurements_xdot > 0;
	R_fixed_has = dimensions.R_fixed_has;
	K_fixed_has = dimensions.K_fixed_has;
	F_fixed_has = dimensions.F_fixed_has;
	RKF_fixed_has = dimensions.RKF_fixed_has;
	numthreads = options.numthreads;
	eigenvaluederivativetype = options.eigenvaluederivative;
	eigenvalueignoreinf = options.eigenvalueignoreinf;
	[R, K, F] = x2R(x, dimensions);
	needsdecouplingconditions = GammaDecouplingStrategy_needsdecouplingconditions(options.decouplingcontrol.decouplingstrategy);
	if nargout >= 5
		if needsdecouplingconditions
			error('control:design:gamma:hessian', 'No Hessians of decoupling conditions available.');
		end
		if ~isempty(areafun)
			if derivative_feedback
				[eigenvalues, ~, ~, eigenvalue_derivative, eigenvalue_derivative_xdot, ~, ~, ~, ~, eigenvalue_2derivative, eigenvalue_2derivative_xdot, eigenvalue_2derivative_mixed, eigenvalue_2derivative_xdot_mixed] = calculate_eigenvalues_m(system, R, K, dimensions, eigenvaluederivativetype, numthreads, options.eigenvaluefilter);
				[eigenvalues, ~, ~, eigenvalue_derivative, eigenvalue_derivative_xdot, ~, ~, ~, ~, eigenvalue_2derivative, eigenvalue_2derivative_xdot, eigenvalue_2derivative_mixed, eigenvalue_2derivative_xdot_mixed] = calculate_eigenvalue_filter(options.eigenvaluefilter, eigenvalues, [], [], eigenvalue_derivative, eigenvalue_derivative_xdot, [], [], [], [], eigenvalue_2derivative, eigenvalue_2derivative_xdot, eigenvalue_2derivative_mixed, eigenvalue_2derivative_xdot_mixed);
				[areaval, areaval_derivative, areaval_2_derivative] = calculate_areas(areafun, weight, eigenvalues, dimensions, options);
				gradtemp = calculate_constraint_gradient(weight, eigenvalue_derivative, eigenvalue_derivative_xdot, areaval_derivative, dimensions, numthreads);
				hesstemp = calculate_constraint_hessian(weight, eigenvalue_derivative, eigenvalue_derivative_xdot, areaval_derivative, dimensions, numthreads, areaval_2_derivative, eigenvalue_2derivative, eigenvalue_2derivative_xdot, eigenvalue_2derivative_mixed, eigenvalue_2derivative_xdot_mixed);
			else
				[eigenvalues, ~, ~, eigenvalue_derivative, ~, ~, ~, ~, ~, eigenvalue_2derivative] = calculate_eigenvalues_m(system, R, K, dimensions, eigenvaluederivativetype, numthreads, options.eigenvaluefilter);
				[eigenvalues, ~, ~, eigenvalue_derivative, ~, ~, ~, ~, ~, eigenvalue_2derivative] = calculate_eigenvalue_filter(options.eigenvaluefilter, eigenvalues, [], [], eigenvalue_derivative, [], [], [], [], [], eigenvalue_2derivative);
				[areaval, areaval_derivative, areaval_2_derivative] = calculate_areas(areafun, weight, eigenvalues, dimensions, options);
				gradtemp = calculate_constraint_gradient(weight, eigenvalue_derivative, [], areaval_derivative, dimensions, numthreads);
				eigenvalue_derivative_xdot = NaN(number_states, number_controls, number_measurements_xdot, number_models) + 0i;
				hesstemp = calculate_constraint_hessian(weight, eigenvalue_derivative, eigenvalue_derivative_xdot, areaval_derivative, dimensions, numthreads, areaval_2_derivative, eigenvalue_2derivative);
			end
			if eigenvalueignoreinf
				areaval(isinf(areaval)) = -1E30;
			end
			c = reshape(areaval, number_models*number_areas_max*number_states, 1);
			removec = isnan(c);
			c(removec) = [];
			gradc = reshape(gradtemp, number_models*number_areas_max*number_states, number_coefficients).';
			gradc(:, removec) = [];
			hessc = reshape(hesstemp, number_models*number_areas_max*number_states, number_coefficients, number_coefficients);
			hessc(removec, :, :) = [];
		else
			c = zeros(0, 1);
			gradc = zeros(number_coefficients, 0);
			hessc = zeros(0, number_coefficients, number_coefficients);
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
		if ~isempty(areafun) || needsdecouplingconditions
			if derivative_feedback
				[eigenvalues, ~, ~, eigenvalue_derivative, eigenvalue_derivative_xdot] = calculate_eigenvalues_m(system, R, K, dimensions, eigenvaluederivativetype, numthreads, options.eigenvaluefilter);
				[eigenvalues, ~, ~, eigenvalue_derivative, eigenvalue_derivative_xdot] = calculate_eigenvalue_filter(options.eigenvaluefilter, eigenvalues, [], [], eigenvalue_derivative, eigenvalue_derivative_xdot);
				if ~isempty(areafun)
					[areaval, areaval_derivative] = calculate_areas(areafun, weight, eigenvalues, dimensions, options);
					gradtemp = calculate_constraint_gradient(weight, eigenvalue_derivative, eigenvalue_derivative_xdot, areaval_derivative, dimensions, numthreads);
					if eigenvalueignoreinf
						areaval(isinf(areaval)) = -1E30;
					end
					c = reshape(areaval, number_models*number_areas_max*number_states, 1);
					removec = isnan(c);
					c(removec) = [];
					gradc = reshape(gradtemp, number_models*number_areas_max*number_states, number_coefficients).';
					gradc(:, removec) = [];
				else
					c = zeros(0, 1);
					gradc = zeros(number_coefficients, 0);
				end
				if needsdecouplingconditions
					error('control:design:gamma:derivative_feedback', 'Decoupling controller design not implemented for derivative feedback.');
				else
					c_decoupling = zeros(0, 1);
					ceq_decoupling = zeros(0, 1);
					gradc_decoupling = zeros(number_coefficients, 0);
					gradceq_decoupling = zeros(number_coefficients, 0);
				end
			else
				[eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative, ~, eigenvector_right_derivative, ~, eigenvector_left_derivative] = calculate_eigenvalues_m(system, R, K, dimensions, eigenvaluederivativetype, numthreads, options.eigenvaluefilter);
				[eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative, ~, eigenvector_right_derivative, ~, eigenvector_left_derivative] = calculate_eigenvalue_filter(options.eigenvaluefilter, eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative, [], eigenvector_right_derivative, [], eigenvector_left_derivative);
				if ~isempty(areafun)
					[areaval, areaval_derivative] = calculate_areas(areafun, weight, eigenvalues, dimensions, options);
					gradtemp = calculate_constraint_gradient(weight, eigenvalue_derivative, [], areaval_derivative, dimensions, numthreads);
					if eigenvalueignoreinf
						areaval(isinf(areaval)) = -1E30;
					end
					c = reshape(areaval, number_models*number_areas_max*number_states, 1);
					removec = isnan(c);
					c(removec) = [];
					gradc = reshape(gradtemp, number_models*number_areas_max*number_states, number_coefficients).';
					gradc(:, removec) = [];
				else
					c = zeros(0, 1);
					gradc = zeros(number_coefficients, 0);
				end
				if needsdecouplingconditions
					[c_decoupling, ceq_decoupling, gradc_decoupling, gradceq_decoupling] = calculate_decoupling_conditions(system, R, K, F, dimensions, options, eigenvalues, eigenvector_right, eigenvector_left, eigenvector_right_derivative, eigenvector_left_derivative);
				else
					c_decoupling = zeros(0, 1);
					ceq_decoupling = zeros(0, 1);
					gradc_decoupling = zeros(number_coefficients, 0);
					gradceq_decoupling = zeros(number_coefficients, 0);
				end
			end
		else
			c = zeros(0, 1);
			gradc = zeros(number_coefficients, 0);
			c_decoupling = zeros(0, 1);
			ceq_decoupling = zeros(0, 1);
			gradc_decoupling = zeros(number_coefficients, 0);
			gradceq_decoupling = zeros(number_coefficients, 0);
		end
		c = [
			c;
			c_decoupling
		];
		ceq = ceq_decoupling;
		gradc = [
			gradc,	gradc_decoupling
		];
		gradceq = gradceq_decoupling;
		if RKF_fixed_has
			gradc = dimensions.RKF_fixed_T_inv'*gradc;% transformed here to avoid another loop in calculate_constraint_gradient
			gradc = gradc(dimensions.index_RKF_free, :);
		elseif R_fixed_has || K_fixed_has || F_fixed_has
			gradc = gradc(dimensions.index_all_free, :);
			gradceq = gradceq(dimensions.index_all_free, :);
		end
	else
		if ~isempty(areafun) || needsdecouplingconditions
			[eigenvalues, eigenvector_right, eigenvector_left] = calculate_eigenvalues_m(system, R, K, dimensions, eigenvaluederivativetype, numthreads, options.eigenvaluefilter);
			[eigenvalues, eigenvector_right, eigenvector_left] = calculate_eigenvalue_filter(options.eigenvaluefilter, eigenvalues, eigenvector_right, eigenvector_left);
			if ~isempty(areafun)
				areaval = calculate_areas(areafun, weight, eigenvalues, dimensions, options);
				if eigenvalueignoreinf
					areaval(isinf(areaval)) = -1E30;
				end
				c = reshape(areaval, number_models*number_areas_max*number_states, 1);
				c(isnan(c)) = [];
			else
				c = zeros(0, 1);
			end
			if needsdecouplingconditions
				[c_decoupling, ceq_decoupling] = calculate_decoupling_conditions(system, R, K, F, dimensions, options, eigenvalues, eigenvector_right, eigenvector_left);
			else
				c_decoupling = zeros(0, 1);
				ceq_decoupling = zeros(0, 1);
			end
			c = [
				c;
				c_decoupling
			];
			ceq = ceq_decoupling;
		else
			c = zeros(0, 1);
			ceq = zeros(0, 1);
		end
	end
end
function [R, K, F] = x2R(x, dimensions)
	%X2R transform optimization variable to gain matrices
	%	Input:
	%		x:			current optimization variable
	%		dimensions:	structure with information about dimensions of the different variables and systems
	%	Output:
	%		R:			proportional gain matrix
	%		K:			derivative gain matrix
	%		F:			prefilter matrix
	number_controls = dimensions.controls;
	number_measurements = dimensions.measurements;
	number_measurements_xdot = dimensions.measurements_xdot;
	number_references = dimensions.references;
	derivative_feedback = number_measurements_xdot > 0;
	R_fixed_has = dimensions.R_fixed_has;
	K_fixed_has = dimensions.K_fixed_has;
	F_fixed_has = dimensions.F_fixed_has;
	RKF_fixed_has = dimensions.RKF_fixed_has;
	T_inv = dimensions.R_fixed_T_inv;
	T_inv_xdot = dimensions.K_fixed_T_inv;
	T_inv_prefilter = dimensions.F_fixed_T_inv;
	T_inv_RKF = dimensions.RKF_fixed_T_inv;
	if RKF_fixed_has
		RKF = reshape(T_inv_RKF*[
			dimensions.RKF_fixed_b;
			x
		], number_controls, number_measurements + number_measurements_xdot + number_references);
		R = RKF(:, 1:number_measurements);
		K = RKF(:, number_measurements + 1:number_measurements + number_measurements_xdot);
		F = RKF(:, number_measurements + number_measurements_xdot + 1:number_measurements + number_measurements_xdot + number_references);
	else
		if R_fixed_has
			R = reshape(T_inv*[
				dimensions.R_fixed_b;
				x(1:end - (number_controls*number_measurements_xdot - size(dimensions.K_fixed_b, 1) + number_controls*number_references - size(dimensions.F_fixed_b, 1)), 1)
			], number_controls, number_measurements);
		else
			if number_controls == 0 || number_measurements == 0
				R = zeros(number_controls, number_measurements);
			else
				R = reshape(x(1:end - (number_controls*number_measurements_xdot - size(dimensions.K_fixed_b, 1) + number_controls*number_references - size(dimensions.F_fixed_b, 1)), 1), number_controls, number_measurements);
			end
		end
		if derivative_feedback
			if K_fixed_has
				K = reshape(T_inv_xdot*[
					dimensions.K_fixed_b;
					x(end - (number_controls*number_measurements_xdot - size(dimensions.K_fixed_b, 1) + number_controls*number_references - size(dimensions.F_fixed_b, 1)) + 1:end - (number_controls*number_references - size(dimensions.F_fixed_b, 1)), 1)
				], number_controls, number_measurements_xdot);
			else
				K = reshape(x(end - (number_controls*number_measurements_xdot - size(dimensions.K_fixed_b, 1) + number_controls*number_references - size(dimensions.F_fixed_b, 1)) + 1:end - (number_controls*number_references - size(dimensions.F_fixed_b, 1)), 1), number_controls, number_measurements_xdot);
			end
		else
			K = zeros(number_controls, number_measurements_xdot);
		end
		if F_fixed_has
			F = reshape(T_inv_prefilter*[
				dimensions.F_fixed_b;
				x(end - (number_controls*number_references - size(dimensions.F_fixed_b, 1)) + 1:end, 1)
			], number_controls, number_references);
		else
			if number_controls == 0 || number_references == 0
				F = zeros(number_controls, number_references);
			else
				F = reshape(x(end - (number_controls*number_references - size(dimensions.F_fixed_b, 1)) + 1:end, 1), number_controls, number_references);
			end
		end
	end
end
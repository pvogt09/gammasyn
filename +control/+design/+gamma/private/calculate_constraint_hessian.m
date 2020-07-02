function [hessianc] = calculate_constraint_hessian(weight, eigenvalue_derivative, eigenvalue_derivative_xdot, areaval_derivative, dimensions, numthreads, areaval_2_derivative, eigenvalue_2derivative, eigenvalue_2derivative_xdot, eigenvalue_2derivative_mixed, eigenvalue_2derivative_xdot_mixed)
	%CALCULATE_CONSTRAINT_HESSIAN helper function for calculation of constraint hessian for gamma pole placement
	%	Input:
	%		weight:								weighting matrix with number of systems columns and number of pole area border functions rows
	%		eigenvalue_derivative:				derivative of eigenvalues with respect to proportional gain used for calculation of gradient of objective function
	%		eigenvalue_derivative_xdot:			derivative of eigenvalues with respect to derivative gain used for calculation of gradient of objective function
	%		areaval_derivative:					derivative of pole area border functions used for calculation of gradient of objective function
	%		dimensions:							structure with information about dimensions of the different variables and systems
	%		numthreads:							number of threads to run loops in
	%		areaval_2_derivative:				hessian of area border function for current optimization value
	%		eigenvalue_2derivative:				second derivative of eigenvalues with respect to proportional gain matrix for calculation of hessian matrix
	%		eigenvalue_2derivative_xdot:		second derivative of eigenvalues with respect to derivative gain matrix for calculation of hessian matrix
	%		eigenvalue_2derivative_mixed:		second derivative of eigenvalues with respect to proportional and derivative gain matrix for calculation of hessian matrix
	%		eigenvalue_2derivative_xdot_mixed:	second derivative of eigenvalues with respect to derivative and proportional gain matrix for calculation of hessian matrix
	%	Output:
	%		hessianc:							hessian of constraint function value for current optimization value
	numthreads = uint32(floor(max([0, numthreads])));
	number_models = dimensions.models;
	number_states = dimensions.states;
	number_controls = dimensions.controls;
	number_measurements = dimensions.measurements;
	number_measurements_xdot = dimensions.measurements_xdot;
	number_references = dimensions.references;
	derivative_feedback = number_measurements_xdot > 0;
	number_areas_max = dimensions.areas_max;
	T_inv = dimensions.R_fixed_T_inv;
	T_inv_xdot = dimensions.K_fixed_T_inv;
	number_R_coefficients = number_controls*number_measurements;% number of proportional parameters
	number_K_coefficients = number_controls*number_measurements_xdot; % number of derivative parameters
	number_F_coefficients = number_controls*number_references; % number of derivative parameters
	number_gain_coefficients = number_R_coefficients + number_K_coefficients; % number of gain parameters (relevant for calculation of hessian of constraints)
	number_coefficients = number_gain_coefficients + number_F_coefficients;% number of all coefficients
	hessianc = zeros(number_models, number_areas_max, number_states, number_coefficients, number_coefficients);
	if derivative_feedback && isempty(eigenvalue_derivative_xdot)
		error('control:design:gamma:gradient', 'Gradient for eigenvalues was not supplied.');
	end
	if derivative_feedback && nargin <= 8
		error('control:design:gamma:hessian', 'Hessian for eigenvalues was not supplied.');
	end
	if nargin <= 10
		eigenvalue_2derivative_xdot_mixed = zeros(number_states, number_controls*number_measurements, number_controls*number_measurements_xdot, number_models) + 0i;
		if nargin <= 9
			eigenvalue_2derivative_mixed = zeros(number_states, number_controls*number_measurements_xdot, number_controls*number_measurements, number_models) + 0i;
			if nargin <= 8
				eigenvalue_2derivative_xdot = zeros(number_states, number_controls*number_measurements_xdot, number_controls*number_measurements_xdot, number_models) + 0i;
			end
		end
	end
	if derivative_feedback && (isempty(eigenvalue_2derivative_xdot) || isempty(eigenvalue_2derivative_mixed) || isempty(eigenvalue_2derivative_xdot_mixed))
		error('control:design:gamma:hessian', 'Hessian for eigenvalues was not supplied.');
	end
	hessianc_gain = NaN(number_models, number_areas_max, number_states, number_gain_coefficients, number_gain_coefficients);
	parfor (ii = 1:number_models, numthreads)
		% copy slices of variables to temporary variables for parfor
		wii = weight(ii, :);
		eigen_derivative = eigenvalue_derivative(:, :, :, ii);
		eigen_derivative_xdot = eigenvalue_derivative_xdot(:, :, :, ii);
		eigen_2derivative = eigenvalue_2derivative(:, :, :, ii);
		eigen_2derivative_xdot = eigenvalue_2derivative_xdot(:, :, :, ii);
		eigen_2derivative_mixed = eigenvalue_2derivative_mixed(:, :, :, ii);
		eigen_2derivative_xdot_mixed = eigenvalue_2derivative_xdot_mixed(:, :, :, ii);
		area_derivative = areaval_derivative(:, :, :, ii);
		area_2_derivative = areaval_2_derivative(:, :, :, ii);
		for kk = 1:number_states
			for z = 1:number_gain_coefficients % first control parameter
				for q = 1:number_gain_coefficients % second control parameter
					% controller indices
					if z <= number_R_coefficients
						i = mod(z - 1, number_controls) + 1;
						j = idivide(z - 1, number_controls) + 1;
					else
						tmpZ = z - number_R_coefficients;
						i = mod(tmpZ - 1, number_controls) + 1;
						j = idivide(tmpZ - 1, number_controls) + 1;
					end

					if q <= number_R_coefficients
						s = mod(q - 1, number_controls) + 1;
						t = idivide(q - 1, number_controls) + 1;
					else
						tmpQ = q - number_R_coefficients;
						s = mod(tmpQ - 1, number_controls) + 1;
						t = idivide(tmpQ - 1, number_controls) + 1;
					end
					% read eigenvalue derivatives
					if ~derivative_feedback
						eigDerivative_ij = eigen_derivative(kk, i, j, 1);
						eigDerivative_st = eigen_derivative(kk, s, t, 1);
						eig2Derivative_re = real(eigen_2derivative(kk, q, z, 1));
						eig2Derivative_im = imag(eigen_2derivative(kk, q, z, 1));
					else
						if z <= number_R_coefficients || ~derivative_feedback
							if q <= number_R_coefficients || ~derivative_feedback
								eigDerivative_ij = eigen_derivative(kk, i, j, 1);
								eigDerivative_st = eigen_derivative(kk, s, t, 1);

								eig2Derivative_re = real(eigen_2derivative(kk, q, z, 1));
								eig2Derivative_im = imag(eigen_2derivative(kk, q, z, 1));
							else
								eigDerivative_ij = eigen_derivative(kk, i, j, 1);
								eigDerivative_st = eigen_derivative_xdot(kk, s, t, 1);

								eig2Derivative_re = real(eigen_2derivative_mixed(kk, q - number_R_coefficients, z, 1));
								eig2Derivative_im = imag(eigen_2derivative_mixed(kk, q - number_R_coefficients, z, 1));
							end
						else
							if q <= number_R_coefficients
								eigDerivative_ij = eigen_derivative_xdot(kk, i, j, 1);
								eigDerivative_st = eigen_derivative(kk, s, t, 1);

								eig2Derivative_re = real(eigen_2derivative_xdot_mixed(kk, q, z - number_R_coefficients, 1));
								eig2Derivative_im = imag(eigen_2derivative_xdot_mixed(kk, q, z - number_R_coefficients, 1));
							else
								eigDerivative_ij = eigen_derivative_xdot(kk, i, j, 1);
								eigDerivative_st = eigen_derivative_xdot(kk, s, t, 1);

								eig2Derivative_re = real(eigen_2derivative_xdot(kk, q - number_R_coefficients, z - number_R_coefficients, 1));
								eig2Derivative_im = imag(eigen_2derivative_xdot(kk, q - number_R_coefficients, z - number_R_coefficients, 1));
							end
						end
					end
					if isnan(eigDerivative_ij) || isnan(eigDerivative_st) || isnan(eig2Derivative_re) || isnan(eig2Derivative_im)
						continue;
					end
					% read areaval derivatives
					dfdre = wii.*squeeze(area_derivative(kk, :, 1, 1));
					dfdim = wii.*squeeze(area_derivative(kk, :, 2, 1));
					d2fdredre = wii.*area_2_derivative(kk, :, 1, 1);
					d2fdimdre = wii.*area_2_derivative(kk, :, 2, 1);
					d2fdredim = wii.*area_2_derivative(kk, :, 3, 1);
					d2fdimdim = wii.*area_2_derivative(kk, :, 4, 1);
					% calculate hessian for model ii, state kk and coefficient indices z q
					hessianc_gain(ii, :, kk, z, q) = ((d2fdredre.*real(eigDerivative_st) + d2fdimdre.*imag(eigDerivative_st)).*real(eigDerivative_ij) + dfdre.*eig2Derivative_re + (d2fdredim.*real(eigDerivative_st) + d2fdimdim.*imag(eigDerivative_st)).*imag(eigDerivative_ij) + dfdim.*eig2Derivative_im);
				end
			end
		end
	end
	% transform hessian from gain coefficients to optimization variables
	if dimensions.RKF_fixed_has
		blk_T_inv = dimensions.RKF_fixed_T_inv.';
		blk_T = dimensions.RKF_fixed_T;
		hessian_prefilter = zeros(number_F_coefficients, number_F_coefficients);
		% TODO: combine loop with above loop (may not be possible)
		parfor (ii = 1:number_models, numthreads)
			hessianc_gain_temp = hessianc_gain(ii, :, :, :, :);
			hessianc_temp = hessianc(ii, :, :, :, :);
			for kk = 1:number_states
				for ll = 1:number_areas_max
					hessianc_temp(1, ll, kk, :, :) = blk_T_inv*blkdiag(squeeze(hessianc_gain_temp(1, ll, kk, :, :)), hessian_prefilter)*blk_T;
				end
			end
			hessianc(ii, :, :, :, :) = hessianc_temp;
		end
	else
		blk_T_inv = blkdiag(T_inv, T_inv_xdot).';
		blk_T = blkdiag(T_inv, T_inv_xdot);
		% TODO: combine loop with above loop (may not be possible)
		parfor (ii = 1:number_models, numthreads)
			hessianc_gain_temp = hessianc_gain(ii, :, :, :, :);
			for kk = 1:number_states
				for ll = 1:number_areas_max
					hessianc_gain_temp(1, ll, kk, :, :) = blk_T_inv*squeeze(hessianc_gain_temp(1, ll, kk, :, :))*blk_T;
				end
			end
			hessianc_gain(ii, :, :, :, :) = hessianc_gain_temp;
		end
		hessianc(:, :, :, 1:number_gain_coefficients, 1:number_gain_coefficients) = hessianc_gain;
	end
end
function [gradc] = calculate_constraint_gradient(weight, eigenvalue_derivative, eigenvalue_derivative_xdot, areaval_derivative, dimensions, numthreads)
	%CALCULATE_CONSTRAINT_GRADIENT helper function for calculation of constraint gradient for gamma pole placement
	%	Input:
	%		weight:						weighting matrix with number of systems columns and number of pole area border functions rows
	%		eigenvalue_derivative:		derivative of eigenvalues with respect to proportional gain used for calculation of gradient of objective function
	%		eigenvalue_derivative_xdot:	derivative of eigenvalues with respect to derivative gain used for calculation of gradient of objective function
	%		areaval_derivative:			derivative of pole area border functions used for calculation of gradient of objective function
	%		dimensions:					structure with information about dimensions of the different variables and systems
	%		numthreads:					number of threads to run loops in
	%	Output:
	%		gradc:						gradient of constraint function value for current optimization value
	numthreads = uint32(floor(max([0, numthreads])));
	number_models = dimensions.models;
	number_states = dimensions.states;
	number_controls = dimensions.controls;
	number_measurements = dimensions.measurements;
	number_measurements_xdot = dimensions.measurements_xdot;
	number_references = dimensions.references;
	derivative_feedback = number_measurements_xdot > 0;
	number_areas_max = dimensions.areas_max;
	R_fixed_has = dimensions.R_fixed_has && ~dimensions.RKF_fixed_has;% checking RKF_fixed_has should not be neccessary, just to make sure
	K_fixed_has = dimensions.K_fixed_has && ~dimensions.RKF_fixed_has;% checking RKF_fixed_has should not be neccessary, just to make sure
	T_inv = dimensions.R_fixed_T_inv;
	T_inv_xdot = dimensions.K_fixed_T_inv;
	gradc_x = NaN(number_models, number_areas_max, number_states, number_controls*number_measurements);
	gradc_xdot = NaN(number_models, number_areas_max, number_states, number_controls*number_measurements_xdot);
	gradc_prefilter = zeros(number_models, number_areas_max, number_states, number_controls*number_references);
	if derivative_feedback && isempty(eigenvalue_derivative_xdot)
		error('control:design:gamma:gradient', 'Gradient for eigenvalues was not supplied.');
	end
	parfor (ii = 1:number_models, numthreads)
		wii = weight(ii, :);
		for kk = 1:number_states
			if all(all(isnan(eigenvalue_derivative(kk, :, :, ii))))
				continue;
			end
			re = real(squeeze(eigenvalue_derivative(kk, :, :, ii)));
			im = imag(squeeze(eigenvalue_derivative(kk, :, :, ii)));
			if derivative_feedback && ~isempty(eigenvalue_derivative_xdot)
				re_xdot = real(squeeze(eigenvalue_derivative_xdot(kk, :, :, ii)));
				im_xdot = imag(squeeze(eigenvalue_derivative_xdot(kk, :, :, ii)));
			else
				re_xdot = zeros(number_controls, number_measurements_xdot);
				im_xdot = zeros(number_controls, number_measurements_xdot);
			end
			for ll = 1:number_areas_max
				temp = squeeze(areaval_derivative(kk, ll, :, ii));
				if any(isnan(temp))
					error('control:design:gamma:gradient', 'NaN is not allowed in a valid eigenvalue derivative, there must be a programming error somewhere.');
				end
				if R_fixed_has
					gradc_x(ii, ll, kk, :) = wii(ll)*reshape(temp(1)*re + temp(2)*im, 1, number_controls*number_measurements)*T_inv;
				else
					gradc_x(ii, ll, kk, :) = wii(ll)*reshape(temp(1)*re + temp(2)*im, 1, number_controls*number_measurements);
				end
				if derivative_feedback
					if K_fixed_has
						gradc_xdot(ii, ll, kk, :) = wii(ll)*reshape(temp(1)*re_xdot + temp(2)*im_xdot, 1, number_controls*number_measurements_xdot)*T_inv_xdot;
					else
						gradc_xdot(ii, ll, kk, :) = wii(ll)*reshape(temp(1)*re_xdot + temp(2)*im_xdot, 1, number_controls*number_measurements_xdot);
					end
				end
			end
		end
	end
	% codegen crashes when concatenating empty variable size matrices
	if derivative_feedback
		if isempty(gradc_x)
			if isempty(gradc_prefilter)
				gradc = gradc_xdot;
			else
				gradc = cat(4, gradc_xdot, gradc_prefilter);
			end
		else
			if isempty(gradc_prefilter)
				gradc = cat(4, gradc_x, gradc_xdot);
			else
				gradc = cat(4, gradc_x, gradc_xdot, gradc_prefilter);
			end
		end
	else
		if isempty(gradc_prefilter)
			gradc = gradc_x;
		elseif isempty(gradc_x)
			gradc = gradc_prefilter;
		else
			gradc = cat(4, gradc_x, gradc_prefilter);
		end
	end
end
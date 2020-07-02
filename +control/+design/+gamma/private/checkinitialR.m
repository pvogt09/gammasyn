function [x_0, R_0, K_0, F_0] = checkinitialR(R_0, dimensions_strict)
	%CHECKINITIALR check initial value for gammasyn
	%	Input:
	%		R_0:				initial value for gain matrices
	%		dimensions_strict:	dimensions of problem variables and systems for strict problem formulation
	%	Output:
	%		x_0:				initial value to pass to optimization algorithm
	%		R_0:				initial value to pass to optimization algorithm that corresponds to proportional gain
	%		K_0:				initial value to pass to optimization algorithm that corresponds to derivative gain
	%		F_0:				initial value to pass to optimization algorithm that corresponds to prefilter gain
	if iscell(R_0)
		if numel(R_0) >= 3
			F_0 = R_0{3};
			K_0 = R_0{2};
			R_0 = R_0{1};
		elseif numel(R_0) >= 2
			K_0 = R_0{2};
			R_0 = R_0{1};
			F_0 = zeros(dimensions_strict.controls, dimensions_strict.references, size(R_0, 3));
		else
			R_0 = R_0{1};
			K_0 = zeros(dimensions_strict.controls, dimensions_strict.measurements_xdot, size(R_0, 3));
			F_0 = zeros(dimensions_strict.controls, dimensions_strict.references, size(R_0, 3));
		end
	else
		K_0 = zeros(dimensions_strict.controls, dimensions_strict.measurements_xdot, size(R_0, 3));
		F_0 = zeros(dimensions_strict.controls, dimensions_strict.references, size(R_0, 3));
	end
	if ~isnumeric(R_0)
		error('control:design:gamma:dimensions', 'Initial value for proportional gain must be numeric.');
	end
	if ~isnumeric(K_0)
		error('control:design:gamma:dimensions', 'Initial value for derivative gain must be numeric.');
	end
	if ~isnumeric(F_0)
		error('control:design:gamma:dimensions', 'Initial value for prefilter gain must be numeric.');
	end
	if ndims(R_0) > 3
		error('control:design:gamma:dimensions', 'Initial value for proportional gain must not have more than 3 dimensions.');
	end
	if ndims(K_0) > 3
		error('control:design:gamma:dimensions', 'Initial value for derivative gain must not have more than 3 dimensions.');
	end
	if ndims(F_0) > 3
		error('control:design:gamma:dimensions', 'Initial value for prefilter gain must not have more than 3 dimensions.');
	end
	if size(R_0, 3) ~= size(K_0, 3)
		error('control:design:gamma', 'Number of supplied initial proportional gain matrices must be equal to number of supplied initial derivative gain matrices.');
	end
	if size(R_0, 3) ~= size(F_0, 3)
		error('control:design:gamma', 'Number of supplied initial proportional gain matrices must be equal to number of supplied initial prefilter matrices.');
	end
	if isempty(R_0)
		R_0 = zeros(dimensions_strict.controls, dimensions_strict.measurements, size(R_0, 3));
	end
	if isempty(K_0)
		K_0 = zeros(dimensions_strict.controls, dimensions_strict.measurements_xdot, size(R_0, 3));
	end
	if isempty(F_0)
		F_0 = zeros(dimensions_strict.controls, dimensions_strict.references, size(R_0, 3));
	end
	if size(R_0, 1)*size(R_0, 2) ~= dimensions_strict.controls*dimensions_strict.measurements
		error('control:design:gamma', 'Initial proportional gain matrix must have %d elements.', dimensions_strict.controls*dimensions_strict.measurements);
	end
	R_0_temp = zeros(dimensions_strict.controls*dimensions_strict.measurements, size(R_0, 3));
	R_fixed_T = dimensions_strict.R_fixed_T;
	dimR = dimensions_strict.controls*dimensions_strict.measurements;
	if issparse(R_0)
		R_0_temp = full(R_fixed_T*reshape(R_0, dimR, 1));% must be converted to full to be compatible with generated code
	else
		parfor ii = 1:size(R_0, 3)
			R_0_temp(:, ii) = R_fixed_T*reshape(R_0(:, :, ii), dimR, 1);
		end
	end
	if size(K_0, 1)*size(K_0, 2) ~= dimensions_strict.controls*dimensions_strict.measurements_xdot
		error('control:design:gamma', 'Initial derivative gain matrix must have %d elements.', dimensions_strict.controls*dimensions_strict.measurements_xdot);
	end
	K_0_temp = zeros(dimensions_strict.controls*dimensions_strict.measurements_xdot, size(K_0, 3));
	K_fixed_T = dimensions_strict.K_fixed_T;
	dimK = dimensions_strict.controls*dimensions_strict.measurements_xdot;
	if issparse(K_0)
		K_0_temp = full(K_fixed_T*reshape(K_0, dimK, 1));% must be converted to full to be compatible with generated code
	else
		parfor ii = 1:size(K_0, 3)
			K_0_temp(:, ii) = K_fixed_T*reshape(K_0(:, :, ii), dimK, 1);
		end
	end
	if size(F_0, 1)*size(F_0, 2) ~= dimensions_strict.controls*dimensions_strict.references
		error('control:design:gamma', 'Initial prefilter matrix must have %d elements.', dimensions_strict.controls*dimensions_strict.references);
	end
	F_0_temp = zeros(dimensions_strict.controls*dimensions_strict.references, size(F_0, 3));
	F_fixed_T = dimensions_strict.F_fixed_T;
	dimF = dimensions_strict.controls*dimensions_strict.references;
	if issparse(F_0)
		F_0_temp = full(F_fixed_T*reshape(F_0, dimF, 1));% must be converted to full to be compatible with generated code
	else
		parfor ii = 1:size(F_0, 3)
			F_0_temp(:, ii) = F_fixed_T*reshape(F_0(:, :, ii), dimF, 1);
		end
	end
	if any(imag(R_0_temp(:)) ~= 0)
		error('control:design:gamma', 'Initial proportional matrix must not be complex.');
	end
	if any(imag(K_0_temp(:)) ~= 0)
		error('control:design:gamma', 'Initial derivative matrix must not be complex.');
	end
	if any(imag(F_0_temp(:)) ~= 0)
		error('control:design:gamma', 'Initial derivative matrix must not be complex.');
	end
	if dimensions_strict.RKF_fixed_has
		RKF_0_temp = dimensions_strict.RKF_fixed_T*[
			R_0_temp;
			K_0_temp;
			F_0_temp
		];
		x_0 = RKF_0_temp(size(dimensions_strict.RKF_fixed_b, 1) + 1:end, :);
	else
		R_0 = R_0_temp(size(dimensions_strict.R_fixed_b, 1) + 1:end, :);
		K_0 = K_0_temp(size(dimensions_strict.K_fixed_b, 1) + 1:end, :);
		F_0 = F_0_temp(size(dimensions_strict.F_fixed_b, 1) + 1:end, :);
		x_0 = [
			R_0;
			K_0;
			F_0
		];
	end
end
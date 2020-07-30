function [R_0, K_0, F_0] = checkinitialRKF(R_0, dimensions_strict, allownan)
	%CHECKINITIALRKF check initial values for proportional, derivative and prefilter gain for gammasyn
	%	Input:
	%		R_0:				initial value for gain matrices
	%		dimensions_strict:	dimensions of problem variables and systems for strict problem formulation
	%		allownan:			inicator if NaN and Inf are allowed as initial values
	%	Output:
	%		R_0:				initial value for proportional gain
	%		K_0:				initial value for derivative gain
	%		F_0:				initial value for prefilter gain
	if nargin <= 2
		allownan = false;
	end
	if ~isscalar(allownan)
		error('control:design:gamma:dimensions', 'Indicator for NaN values must be scalar.');
	end
	if ~islogical(allownan)
		error('control:design:gamma:dimensions', 'Indicator for NaN values must be of type ''logical''.');
	end
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
	% perform 'scalar' expansion
	number_initial_values = [
		size(R_0, 3),	size(K_0, 3),	size(F_0, 3)
	];
	if any(number_initial_values > 1) && any(number_initial_values == 1)
		number_initial_values_max = max(number_initial_values);
		if number_initial_values(1) == 1
			R_0 = repmat(R_0, [1, 1, number_initial_values_max]);
		end
		if number_initial_values(2) == 1
			K_0 = repmat(K_0, [1, 1, number_initial_values_max]);
		end
		if number_initial_values(3) == 1
			F_0 = repmat(F_0, [1, 1, number_initial_values_max]);
		end
	end
	% check dimensions
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
	if size(R_0, 1) ~= dimensions_strict.controls
		error('control:design:gamma', 'Initial proportional gain matrix must have %d rows.', dimensions_strict.controls);
	end
	if size(R_0, 2) ~= dimensions_strict.measurements
		error('control:design:gamma', 'Initial proportional gain matrix must have %d columns.', dimensions_strict.measurements);
	end
	if size(K_0, 1)*size(K_0, 2) ~= dimensions_strict.controls*dimensions_strict.measurements_xdot
		error('control:design:gamma', 'Initial derivative gain matrix must have %d elements.', dimensions_strict.controls*dimensions_strict.measurements_xdot);
	end
	if size(K_0, 1) ~= dimensions_strict.controls
		error('control:design:gamma', 'Initial derivative gain matrix must have %d rows.', dimensions_strict.controls);
	end
	if size(K_0, 2) ~= dimensions_strict.measurements_xdot
		error('control:design:gamma', 'Initial derivative gain matrix must have %d columns.', dimensions_strict.measurements_xdot);
	end
	if size(F_0, 1)*size(F_0, 2) ~= dimensions_strict.controls*dimensions_strict.references
		error('control:design:gamma', 'Initial prefilter matrix must have %d elements.', dimensions_strict.controls*dimensions_strict.references);
	end
	if size(F_0, 1) ~= dimensions_strict.controls
		error('control:design:gamma', 'Initial prefilter matrix must have %d rows.', dimensions_strict.controls);
	end
	if size(F_0, 2) ~= dimensions_strict.references
		error('control:design:gamma', 'Initial prefilter matrix must have %d columns.', dimensions_strict.references);
	end
	% check valid values
	if any(imag(R_0(:)) ~= 0)
		error('control:design:gamma', 'Initial proportional matrix must not be complex.');
	end
	if any(imag(K_0(:)) ~= 0)
		error('control:design:gamma', 'Initial derivative matrix must not be complex.');
	end
	if any(imag(F_0(:)) ~= 0)
		error('control:design:gamma', 'Initial derivative matrix must not be complex.');
	end
	if ~allownan
		if any(isnan(R_0(:)))
			error('control:design:gamma', 'Initial proportional matrix must not be NaN.');
		end
		if any(isnan(K_0(:)))
			error('control:design:gamma', 'Initial derivative matrix must not be NaN.');
		end
		if any(isnan(F_0(:)))
			error('control:design:gamma', 'Initial derivative matrix must not be NaN.');
		end
		if any(isinf(R_0(:)))
			error('control:design:gamma', 'Initial proportional matrix must not be infinite.');
		end
		if any(isinf(K_0(:)))
			error('control:design:gamma', 'Initial derivative matrix must not be infinite.');
		end
		if any(isinf(F_0(:)))
			error('control:design:gamma', 'Initial derivative matrix must not be infinite.');
		end
	end
end
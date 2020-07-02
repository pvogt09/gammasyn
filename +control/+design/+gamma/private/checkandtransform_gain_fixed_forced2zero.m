function [isforced2zero] = checkandtransform_gain_fixed_forced2zero(constraint_system_hadamard, constraint_border_hadamard, number_controls, number_measurements, gaintype, zerocoefficients)
	%CHECKANDTRANSFORM_GAIN_FIXED_FORCED2ZERO check if certain gain coefficients are forced to zero by constraints
	%	Input:
	%		constraint_system_hadamard:	fixed constraint system in hadamard form A vec(K) = b
	%		constraint_border_hadamard:	fixed constraint system in hadamard form A vec(K) = b
	%		number_controls:			number of controls
	%		number_measurements:		number of measurements (usual or derivative depending on constraint matrix)
	%		gaintype:					'proportional' for proportional gain, 'derivative' for derivative gain, 'prefilter' for prefilter gain and 'combined' for combined gain
	%		zerocoefficients:			indicator matrix for coefficient that should be forced to zero
	%	Output:
	%		isforced2zero:				true if the requested coefficients are forced to zero by the supplied constraint system else false
	if ~isnumeric(number_controls) || ~isscalar(number_controls)
		error('control:design:gamma:dimension', 'Number of controls must be a numeric scalar.');
	end
	if ~isnumeric(number_measurements) || ~isscalar(number_measurements)
		error('control:design:gamma:dimension', 'Number of measurements must be a numeric scalar.');
	end
	if ~any(strcmpi(gaintype, {'proportional', 'derivative', 'prefilter', 'combined'}))
		error('control:design:gamma:dimension', 'Gain type must be ''proportional'', ''derivative'', ''prefilter'' or ''combined''.');
	end
	if size(constraint_system_hadamard, 1) ~= number_controls || size(constraint_system_hadamard, 2) ~= number_measurements
		error('control:design:gamma:dimension', 'Fixed %s gain positions must be a %dX%d matrix.', gaintype, number_controls, number_measurements);
	end
	if size(constraint_system_hadamard, 3) ~= size(constraint_border_hadamard, 1) || size(constraint_border_hadamard, 2) ~= 1
		error('control:design:gamma:dimension', 'Fixed %s gain constraint system must be a %d vector of bounds.', gaintype, size(constraint_system_hadamard, 3));
	end
	if any(imag(constraint_system_hadamard(:)) ~= 0)
		error('control:design:gamma:dimension', 'Fixed %s gain constraint system must not be complex.', gaintype);
	end
	if any(imag(constraint_border_hadamard(:)) ~= 0)
		error('control:design:gamma:dimension', 'Fixed %s gain border must not be complex.', gaintype);
	end
	if nargin <= 5
		zerocoefficients = false(number_controls, number_measurements);
	end
	if ~islogical(zerocoefficients)
		error('control:design:gamma:dimension', 'Fixed %s gain zero coefficients must be of type ''logical''.', gaintype);
	end
	if size(zerocoefficients, 1) ~= number_controls || size(zerocoefficients, 2) ~= number_measurements
		error('control:design:gamma:dimension', 'Fixed %s gain zero coefficients must be a %dX%d matrix.', gaintype, number_controls, number_measurements);
	end
	if number_controls > 0 && number_measurements > 0
		rg = sum(zerocoefficients(:));
		if rg == 0
			isforced2zero = false;
		else
			constraint_system_zero = zeros(number_controls, number_measurements, rg);
			constraint_border_zero = zeros(rg, 1);
			[idxrow, idxcol] = find(zerocoefficients);
			idxlin = find(zerocoefficients);
			for ii = 1:size(idxrow, 1) %#ok<FORPF> no parfor because of dependent indexing
				constraint_system_zero(idxrow(ii), idxcol(ii), ii) = 1;
			end
			constraint_system_zero = cat(3, constraint_system_zero, constraint_system_hadamard);
			constraint_border_zero = [
				constraint_border_zero;
				constraint_border_hadamard
			];
			constraint_system_zero_vec = zeros(size(constraint_system_zero, 3), number_controls*number_measurements);
			temp = constraint_system_zero;
			parfor ii = 1:size(temp, 3)
				constraint_system_zero_vec(ii, :) = reshape(temp(:, :, ii), 1, number_controls*number_measurements);
			end
			% adding K(zerocoefficients) == 0 to constraint system is still feasible
			isforced2zero = rank(constraint_system_zero_vec) == rank([constraint_system_zero_vec, constraint_border_zero]);
			% kernel of constraint system is emtpy and solution is zero for relevant entries
			ker = null(constraint_system_zero_vec(rg + 1:end, :));
			zeroinker = isempty(ker) || all(all(ker(idxlin, :) == 0));
			warnstate = warning('off', 'MATLAB:singularMatrix');
			solution_inhomogenius = constraint_system_zero_vec(rg + 1:end, :)\constraint_border_hadamard;
			warning(warnstate);
			zero_inhomogenius = all(abs(solution_inhomogenius(idxlin, 1)) <= sqrt(eps)/(1 + norm(solution_inhomogenius(idxlin, 1))));
			isforced2zero = isforced2zero && zeroinker && zero_inhomogenius;
		end
	else
		isforced2zero = true;
	end
end
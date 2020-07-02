function [weight_strict, weight_loose] = checkandtransform_weight(weight, allownegativeweight, number_models, number_systems, expanded_models, areafun_strict, number_areas_max_strict, areafun_loose, number_areas_max_loose)
	%CHECKANDTRANSFORM_WEIGHT check user supplied weight coefficients and supply weighting matrices of correct matrix format
	%	Input:
	%		weight:						weighting matrix with number of systems columns and number of pole area border functions rows
	%		allownegativeweight:		indicator, if negative weights are allowed
	%		number_models:				number of systems after model expansion
	%		number_systems:				number of systems before model expansion
	%		expanded_models:			vector with number of expanded models
	%		areafun_strict:				area border functions and gradients as cell array with number of system entries, each function returns a row vector with as much elements as area border functions are defined for the corresponding system for strict optimization of areafunctions
	%		number_areas_max_strict:	maximum number of areafunction return values
	%		areafun_loose:				area border functions and gradients as cell array with number of system entries, each function returns a row vector with as much elements as area border functions are defined for the corresponding system for loose optimization of areafunctions
	%		number_areas_max_loose:		maximum number of areafunction return values
	%	Output:
	%		weight_strict:				weighting matrix with dimension number_models times number_areas_max_strict
	%		weight:loose:				weighting matrix with dimension number_models times number_areas_max_loose
	if ~isscalar(allownegativeweight) || ~islogical(allownegativeweight)
		error('control:design:gamma:dimension', 'Indicator for negative weights must be a logical scalar, not a ''%s''.', class(allownegativeweight));
	end
	if ~isnumeric(number_models) || ~isscalar(number_models)
		error('control:design:gamma:dimension', 'Number of models must be a numeric scalar.');
	end
	if ~isnumeric(number_systems) || ~isscalar(number_systems)
		error('control:design:gamma:dimension', 'Number of systems must be a numeric scalar.');
	end
	if ~isnumeric(expanded_models) || size(expanded_models, 2) ~= 1 || size(expanded_models, 1) ~= number_systems
		error('control:design:gamma:dimension', 'Expanded models must have %d elements.', number_systems);
	end
	if ~isnumeric(expanded_models) || size(expanded_models, 2) ~= 1 || size(expanded_models, 1) ~= number_systems
		error('control:design:gamma:dimension', 'Expanded models must have %d elements.', number_systems);
	end
	if ~isnumeric(number_areas_max_strict) || ~isscalar(number_areas_max_strict)
		error('control:design:gamma:dimension', 'Maximum number of areas must be a numeric scalar.');
	end
	if ~isnumeric(number_areas_max_loose) || ~isscalar(number_areas_max_loose)
		error('control:design:gamma:dimension', 'Maximum number of areas must be a numeric scalar.');
	end
	if iscell(weight)
		if length(weight) == 2
			weight_strict = weight{1};
			weight_loose = weight{2};
		else
			weight_strict = weight;
			weight_loose = 1;
		end
	else
		if size(weight, 3) == 2
			weight_strict = squeeze(weight(:, :, 1));
			weight_loose = squeeze(weight(:, :, 2));
		else
			if isscalar(weight)
				weight_strict = weight;
				weight_loose = weight;
			else
				weight_strict = weight;
				weight_loose = 1;
			end
		end
	end
	if ~isnumeric(weight_strict) || ~isnumeric(weight_loose)
		error('control:design:gamma:input', 'Weights must be numeric.');
	end
	if ~allownegativeweight
		if any(weight_strict(:) < 0)
			error('control:design:gamma:input', 'Weights must be positive unless allowed by ''allownegativeweight'' option.');
		end
		if any(weight_loose(:) < 0)
			error('control:design:gamma:input', 'Weights must be positive unless allowed by ''allownegativeweight'' option.');
		end
	end
	if isempty(areafun_strict)
		weight_strict = weight_strict(:, []);
	end
	if isempty(areafun_loose)
		weight_loose = weight_loose(:, []);
	end
	if isscalar(weight_strict)
		weight_strict = repmat(weight_strict, number_models, number_areas_max_strict);
	end
	if size(weight_strict, 2) < number_areas_max_strict
		warning('control:design:gamma:input', 'For %d area functions only %d weights are given, filling with zero.', number_areas_max_strict, size(weight_strict, 2));
		weight_strict = [
			weight_strict, zeros(size(weight_strict, 1), number_areas_max_strict - size(weight_strict, 2))
		];
	end
	if size(weight_strict, 2) > number_areas_max_strict
		error('control:design:gamma:input', 'Number of weights (%d) must match the number of polearea functions (%d).', size(weight_strict, 2), number_areas_max_strict);
	end
	if size(weight_strict, 1) == 1
		weight_strict = repmat(weight_strict, number_models, 1);
	end
	if size(weight_strict, 1) < number_models
		if size(weight_strict, 1) == number_systems
			weight_temp = cell(size(weight_strict, 1), 1);
			for ii = 1:size(weight_temp) %#ok<FORPF> parfor does not increase speed here
				weight_temp{ii, 1} = repmat(weight_strict(ii, :), expanded_models(ii, 1), 1);
			end
			weight_strict = cat(1, weight_temp{:});
		else
			if any(expanded_models ~= 1)
				error('control:design:gamma:input', 'Number of weights (%d) must match the number of systems (%d) for model expansion.', size(weight_strict, 1), number_systems);
			end
		end
	end
	if size(weight_strict, 1) ~= number_models
		error('control:design:gamma:input', 'Number of weights (%d) must match the number of systems (%d).', size(weight_strict, 1), number_models);
	end
	if any(isnan(weight_strict(:))) || any(isinf(weight_strict(:)))
		error('control:design:gamma:input', 'Weights must be finite.');
	end
	if all(imag(weight_strict(:)) == 0)
		weight_strict = real(weight_strict);
	end
	if ~isreal(weight_strict)
		error('control:design:gamma:input', 'Weights must not be complex.');
	end
	if isscalar(weight_loose)
		weight_loose = repmat(weight_loose, number_models, number_areas_max_loose);
	end
	if size(weight_loose, 2) < number_areas_max_loose
		warning('control:design:gamma:input', 'For %d area functions only %d weights are given, filling with zero.', number_areas_max_loose, size(weight_loose, 2));
		weight_loose = [
			weight_loose, zeros(size(weight_loose, 1), number_areas_max_loose - size(weight_loose, 2))
		];
	end
	if number_areas_max_loose == 0 && size(weight_loose, 2) > 0
		weight_loose = zeros(size(weight_loose, 1), number_areas_max_loose);
	end
	if size(weight_loose, 2) > number_areas_max_loose
		error('control:design:gamma:input', 'Number of weights (%d) must match the number of polearea functions (%d).', size(weight_loose, 2), number_areas_max_loose);
	end
	if size(weight_loose, 1) == 1
		weight_loose = repmat(weight_loose, number_models, 1);
	end
	if size(weight_loose, 1) < number_models
		if size(weight_loose, 1) == number_systems
			weight_temp = cell(size(weight_loose, 1), 1);
			for ii = 1:size(weight_temp) %#ok<FORPF> parfor does not increase speed here
				weight_temp{ii, 1} = repmat(weight_loose(ii, :), expanded_models(ii, 1), 1);
			end
			weight_loose = cat(1, weight_temp{:});
		else
			if any(expanded_models ~= 1)
				error('control:design:gamma:input', 'Number of weights (%d) must match the number of systems (%d) for model expansion.', size(weight_loose, 1), number_systems);
			end
		end
	end
	if size(weight_loose, 1) ~= number_models
		error('control:design:gamma:input', 'Number of weights (%d) must match the number of systems (%d).', size(weight_loose, 1), number_models);
	end
	if size(weight_loose, 2) == 0 && size(weight_strict, 2) == 0
		error('control:design:gamma:input', 'At least one weight must be given.');
	end
	if any(isnan(weight_loose(:))) || any(isinf(weight_loose(:)))
		error('control:design:gamma:input', 'Weights must be finite.');
	end
	if all(imag(weight_loose(:)) == 0)
		weight_loose = real(weight_loose);
	end
	if ~isreal(weight_loose)
		error('control:design:gamma:input', 'Weights must not be complex.');
	end
end
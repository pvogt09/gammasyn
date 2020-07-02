function [eigenvalues, weight] = convert_areas_to_eigenvalues(areafun, weights, number_states)
	%CONVERT_AREAS_TO_EIGENVALUES convert pole area to single set of eigenvalues
	%	Input:
	%		areafun:		area border functions and gradients as cell array with number of system entries, each function returns a row vector with as much elements as area border functions are defined for the corresponding system for strict optimization of areafunctions
	%		weights:		weights for different pole area functions
	%		number_states:	number of states
	%	Output:
	%		eigenvalues:	single set of eigenvalues inside the pole areas
	%		weight:			weight for eigenvalues to use in objective funtion
	% TODO: Implementation
	if ~isnumeric(weights)
		error('control:design:gamma:areas2eigenvalues', 'Weights must be numeric.');
	end
	if ~isnumeric(number_states) || ~isscalar(number_states)
		error('control:design:gamma:areas2eigenvalues', 'Number of states must be a numeric scalar.');
	end
	eigenvalues = [
		0.9;
		0.25;
		0.65 + 0.45i;
		0.65 - 0.45i
		zeros(max([number_states - 4, 0]), 1)
	];
	if nargout >= 2
		weight = [
			%1;
			%0.5;
			%1;
			%1
			%0.1*ones(max([number_states - 4, 0]), 1)
		];
	end
end
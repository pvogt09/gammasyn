function [R, K, F] = input_initial_value2RKF(initial)
	%INPUT_INITIAL_VALUE2RKF convert initial value to initial value for different gain matrices
	%	Input:
	%		initial:	initial value for gain matrices
	%	Output:
	%		R:			part of the initial value belonging to proportional gain
	%		K:			part of the initial value belonging to derivative gain
	%		F:			part of the initial value belonging to prefilter gain
	if nargin <= 0
		initial = [];
	end
	if iscell(initial)
		if numel(initial) >= 3
			F = initial{3};
			K = initial{2};
			R = initial{1};
		elseif numel(initial) >= 2
			K = initial{2};
			R = initial{1};
			F = [];
		else
			R = initial{1};
			K = [];
			F = [];
		end
	else
		R = initial;
		K = [];
		F = [];
	end
end
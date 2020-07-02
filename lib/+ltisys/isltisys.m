function [is] = isltisys(sys)
	%ISLTISYS return if a variable is a ltisys
	%	Input:
	%		sys:	system to check
	%	Output:
	%		is:		true, if the variable is a ltisys, else false
	is = false;
	if ~isnumeric(sys)
		return;
	end
	if ndims(sys) ~= 2
		return;
	end
	if isempty(sys)
		return;
	end
	if size(sys, 1) <= 2
		return;
	end
	if size(sys, 2) <= 2
		return;
	end
	if ~isinf(sys(end, end))
		return;
	end
	if sys(end, end) >= 0
		return;
	end
	if any(sys(end, 1:end - 1) ~= 0)
		return;
	end
	if any(sys(2:end - 1, end) ~= 0)
		return;
	end
	n_states = real(sys(1, end));
	if floor(n_states) ~= ceil(n_states)
		return;
	end
	if size(sys, 1) < n_states + 2
		return;
	end
	if size(sys, 2) < n_states + 2
		return;
	end
	if any(imag(sys(end, :)) ~= 0)
		return;
	end
	if any(imag(sys(:, end)) ~= 0)
		return;
	end
	is = true;
end
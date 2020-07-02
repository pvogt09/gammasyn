function [is] = ispsys(sys)
	%ISPSYS check if a matrix is a valid psys (parameter dependent system)
	%	Input:
	%		sys:	system to check
	%	Output:
	%		is:		true, if the matrix is a psys, else false
	is = false;
	if isempty(sys)
		return;
	end
	if ~isnumeric(sys)
		return;
	end
	if size(sys, 1) < 7
		return;
	end
	if ~isinf(sys(1, 1))
		return;
	end
	if sys(1, 1) >= 0
		return;
	end
	if any(imag(sys(:, 1)) ~= 0)
		return;
	end
	if any(sys(:, 2) ~= 0)
		return;
	end
	if floor(sys(2, 1)) ~= ceil(sys(2, 1))
		return;
	end
	pstype = real(sys(2, 1));
	if pstype ~= 1 && pstype ~= 2
		return;
	end
	if floor(sys(3, 1)) ~= ceil(sys(3, 1))
		return;
	end
	n_vertices = real(sys(3, 1));
	if floor(sys(4, 1)) ~= ceil(sys(4, 1))
		return;
	end
	n_states = real(sys(4, 1));
	if floor(sys(5, 1)) ~= ceil(sys(5, 1))
		return;
	end
	n_controls = real(sys(5, 1));
	if floor(sys(6, 1)) ~= ceil(sys(6, 1))
		return;
	end
	n_measurements = real(sys(6, 1));
	if size(sys, 2) < 1 + n_vertices*(n_states + n_controls + 2)
		return;
	end
	if size(sys, 1) < n_states + n_measurements + 1
		return;
	end
	systems = sys(:, 1:1 + n_vertices*(n_states + n_controls + 2));
	for ii = 1:n_vertices
		b = 2 + (ii - 1)*(n_states + n_controls + 2);
		is = ltisys.isltisys(systems(1:n_states + n_measurements + 1, b + 1:b + n_states + n_controls + 1));
		if ~is
			return;
		end
	end
	is = false;
	if size(sys, 2) < 2 + n_vertices*(n_states + n_controls + 2)
		return;
	end
	if floor(sys(2, 2 + n_vertices*(n_states + n_controls + 2))) ~= ceil(sys(2, 2 + n_vertices*(n_states + n_controls + 2)))
		return;
	end
	vertices = sys(1:max(2, sys(2, 2 + n_vertices*(n_states + n_controls + 2))), 2 + n_vertices*(n_states + n_controls + 2):size(sys, 2));
	is = ltisys.ispvec(vertices);
end
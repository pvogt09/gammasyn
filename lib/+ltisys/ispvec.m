function [is] = ispvec(vec)
	%ISPVEC return if a matrix is a pvec parameter vector of vertices
	%	Input:
	%		vec:	matrix to check
	%	Output:
	%		is:		indicator if the matrix is a vector of vertices
	is = false;
	if isempty(vec)
		return;
	end
	if ndims(vec) ~= 2 %#ok<ISMAT> compatibility with Octave
		return;
	end
	if size(vec, 1) < 2
		return;
	end
	type = vec(1, 1);
	if floor(type) ~= ceil(type)
		return;
	end
	if isnan(type) || isinf(type)
		return;
	end
	n_vertices = vec(2, 1);
	if floor(n_vertices) ~= ceil(n_vertices)
		return;
	end
	if isnan(n_vertices) || isinf(n_vertices)
		return;
	end
	if size(vec, 1) < max([2, n_vertices])
		return;
	end
	if size(vec, 2) < 5
		return;
	end
	is = true;
end
function [sys] = convert_vectorized2hadamard(A, b, sz)
	%CONVERT_VECTORIZED2HADAMARD convert vectorized equation system to equation system in Hadamard form
	%	Input:
	%		A:		left hand side matrix of equation system A*x =/<= b
	%		b:		right hand side matrix of equation system A*x =/<= b
	%		sz:		size of matrix form of x
	%	Output:
	%		sys:	system in Hadamard form to convert
	if ~isnumeric(A)
		error('control:design:gamma:dimension', 'Left hand side of equation system must be numeric.');
	end
	if ndims(A) >= 3 %#ok<ISMAT> compatibility with Octave
		error('control:design:gamma:dimension', 'Left hand side of equation system must be a matrix.');
	end
	if ~isnumeric(b)
		error('control:design:gamma:dimension', 'Right hand side of equation system must be numeric.');
	end
	if size(b, 2) ~= 1
		error('control:design:gamma:dimension', 'Right hand side of equation system must be a row vector');
	end
	if size(b, 1) ~= size(A, 1)
		error('control:design:gamma:dimension', 'Number of right hand sides (%d) must match number of left hand sides (%d)', size(b, 1), size(A, 1));
	end
	if ~isnumeric(sz)
		error('control:design:gamma:dimension', 'Size must be numeric.');
	end
	if numel(sz) ~= 2
		error('control:design:gamma:dimension', 'Size must have 2 elements.');
	end
	if any(isnan(sz(:)))
		error('control:design:gamma:dimension', 'Size must not be NaN.');
	end
	if any(isinf(sz(:)))
		error('control:design:gamma:dimension', 'Size must be finite.');
	end
	if any(sz(:) < 0)
		error('control:design:gamma:dimension', 'Size must be nonnegative.');
	end
	if prod(sz)*size(A, 1) ~= numel(A)
		error('control:design:gamma:dimension', 'Left hand side of equation system can not be converted to the desired size.');
	end
	sys = {reshape(A', [sz, size(A, 1)]), b};
end
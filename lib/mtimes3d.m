function [prod] = mtimes3d(A, B, numthreads)
	%MTIMES3D product of matrices A and B element wise along the third dimension
	%	Input:
	%		A:				matrix to multiply from the left
	%		B:				matrix to multiply from the right
	%		numthreads:		number of threads to use
	%	Output:
	%		prod:			product of matrices A and B element wise along the third dimension
	persistent iscompiled;
	if isempty(iscompiled)
		iscompiled = any(exist('mtimes3d_mex', 'file') == [2, 3, 5, 6]); %#ok<EXIST> exist('mtimes3d_mex', 'function')
	end
	persistent numwork;
	if isempty(numwork)
		numwork = uint32(feature('numcores'));
	end
	if nargin <= 3
		numthreads = numwork;
	end
	numthreads = uint32(numthreads);
	if iscompiled && isnumeric(A) && isnumeric(B)
		prod = mtimes3d_mex(A, B, numthreads);
	else
		prod = mtimes3d_m(A, B, numthreads);
	end
end
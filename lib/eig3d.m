function [V, D] = eig3d(A, B, eigvalOption, numthreads)
	%EIG3D calculate eigenvalues of A - lambda B element wise along the third dimension
	%	Input:
	%		A:					matrix to calculate eigenvalues of
	%		B:					generalized eigenproblem matrix
	%		eigenvalueOption:	return eigenvalues as matrix or vector
	%		numthreads:			number of threads to use
	%	Output:
	%		V:					eigenvectors of eigenvalue problem element wise along the third dimension
	%		D:					eigenvalues of eigenvalue problem element wise along the third dimension
	persistent iscompiled;
	if isempty(iscompiled)
		iscompiled = any(exist('eig3d_mex', 'file') == [2, 3, 5, 6]); %#ok<EXIST> exist('eig3d_mex', 'function')
	end
	persistent numwork;
	if isempty(numwork)
		numwork = uint32(feature('numcores'));
	end
	if nargin <= 3
		numthreads = numwork;
	end
	numthreads = uint32(numthreads);
	generalized = false;
	if nargin <= 1
		B = [];
		eigvalOption = 'matrix';
	elseif nargin <= 2
		if ischar(B)
			eigvalOption = B;
			B = [];
		else
			generalized = true;
		end
	elseif nargout <= 3
		generalized = true;
		eigvalOption = 'matrix';
	end
	if iscompiled && isnumeric(A) && isnumeric(B)
		if generalized
			if nargout >= 2
				[V, D] = eig3d_mex(A, B, eigvalOption, numthreads);
			else
				[~, V] = eig3d_mex(A, B, eigvalOption, numthreads);
			end
		else
			if nargout >= 2
				[V, D] = eig3d_mex(A, [], eigvalOption, numthreads);
			else
				[~, V] = eig3d_mex(A, [], eigvalOption, numthreads);
			end
		end
	else
		if generalized
			if nargout >= 2
				[V, D] = eig3d_m(A, B, eigvalOption, numthreads);
			else
				V = eig3d_m(A, B, eigvalOption, numthreads);
			end
		else
			if nargout >= 2
				[V, D] = eig3d_m(A, [], eigvalOption, numthreads);
			else
				V = eig3d_m(A, [], eigvalOption, numthreads);
			end
		end
	end
end
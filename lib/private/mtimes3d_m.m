function [prod] = mtimes3d_m(A, B, numthreads)
	%MTIMES3D_M wrapper function for compilation of mtimes3d
	%	Input:
	%		A:		matrix to multiply from the left
	%		B:		matrix to multiply from the right
	%	Output:
	%		prod:	product of matrices A and B element wise along the third dimension
	isA3d = size(A, 3) > 1;
	isB3d = size(B, 3) > 1;
	if size(A, 2) ~= size(B, 1)
		error('MATLAB:innerdim', 'Inner matrix dimensions must agree.');
	end
	if isA3d && isB3d
		if size(A, 3) ~= size(B, 3)
			error('MATLAB:innerdim', 'Inner matrix dimensions must agree.');
		end
		prod = zeros(size(A, 1), size(B, 2), size(A, 3));
		parfor (ii = 1:size(A, 3), numthreads)
			prod(:, :, ii) = A(:, :, ii)*B(:, :, ii);
		end
	elseif isA3d
		prod = zeros(size(A, 1), size(B, 2), size(A, 3));
		parfor (ii = 1:size(A, 3), numthreads)
			prod(:, :, ii) = A(:, :, ii)*B;
		end
	elseif isB3d
		prod = zeros(size(A, 1), size(B, 2), size(B, 3));
		parfor (ii = 1:size(B, 3), numthreads)
			prod(:, :, ii) = A*B(:, :, ii);
		end
	else
		prod = A*B;
	end
end
function [norms] = coupling_vectorTwoNorm(arr, dim)
	%COUPLING_VECTORTWONORM calcualte vector-2 norm for dimension of a matrix
	%	Input:
	%		arr:	matrix to calulate norm for
	%		dim:	dimension to operate along
	%	Output:
	%		norms:	norm of matrix
	if ndims(arr) > 2 %#ok<ISMAT> compatibility with octave
		error('control:design:gamma:dimensions', 'Matrix must be a matrix.');
	end
	if dim == 2
		norms = zeros(size(arr, 1), 1);
		for ii = 1:size(arr, 1)
			norms(ii, 1) = norm(arr(ii, :), 2);
		end
	elseif dim == 1
		norms = zeros(1, size(arr, 2));
		for ii = 1:size(arr, 2)
			norms(1, ii) = norm(arr(:, ii), 2);
		end
	else
		error('control:design:gamma:dimensions', 'Dimension to operate along must be 1 or 2.');
	end
end
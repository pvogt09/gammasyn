function [A, b] = convert_hadamard2vectorized(sys)
	%CONVERT_HADAMARD2VECTORIZED convert equation system in Hadamard form to vectorized equation system
	%	Input:
	%		sys:	system in Hadamard form to convert
	%	Output:
	%		A:		left hand side matrix of equation system A*x =/<= b
	%		b:		right hand side matrix of equation system A*x =/<= b
	if ~iscell(sys)
		error('control:design:gamma:dimension', 'Equation system in Hadamard form must be a cell array.');
	end
	if numel(sys) ~= 2
		error('control:design:gamma:dimension', 'Equation system must have left and right hand side.');
	end
	if ~isnumeric(sys{1}) || ~isnumeric(sys{2})
		error('control:design:gamma:dimension', 'Equation system must be numeric.');
	end
	if size(sys{2}, 2) ~= 1
		error('control:design:gamma:dimension', 'Right hand side must be a column vector.');
	end
	if size(sys{1}, 3) ~= size(sys{2}, 1)
		error('control:design:gamma:dimension', 'Number of left hand sides (%d) does not match number of right hand sides (%d).', size(sys{1}, 3), size(sys{2}, 1));
	end
	b = sys{2};
	A = NaN(size(sys{1}, 3), size(sys{1}, 1)*size(sys{1}, 2));
	temp = sys{1};
	num = size(A, 2);
	parfor ii = 1:size(temp, 3)
		A(ii, :) = reshape(temp(:, :, ii), 1, num);
	end
end
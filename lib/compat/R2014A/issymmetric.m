function [is] = issymmetric(A, skewOption)
	%ISSYMMETRIC check wheter a matrix is symmetric
	%	Input:
	%		A:			matrix to check
	%		skewOption:	'nonskew' to check for a symmetric matrix, 'skew' to check for a skewsymmetric matrix
	%	Output:
	%		is:			true if the matrix is symmetric (or skewsymmetric if requested)
	if nargin <= 1
		skew = false;
	else
		if strcmpi(skewOption, 'skew')
			skew = true;
		elseif strcmpi(skewOption, 'nonskew')
			skew = false;
		else
			error('MATLAB:issymmetric:inputFlag', 'Second input must be ''skew'' or ''nonskew''.');
		end
	end
	if skew
		sym = A + A.';
	else
		sym = A - A.';
	end
	is = all(sym(:) == 0);
end


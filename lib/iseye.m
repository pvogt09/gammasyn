function [is] = iseye(mat)
	%ISEYE return if a matrix is an identity matrix
	%	Input:
	%		mat:	matrix to check
	%	Output:
	%		is:		true, if the matrix is an identity matrix, else false
	is = isdiag(mat) && all(diag(mat) == 1);
end
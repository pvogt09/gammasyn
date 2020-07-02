function [B] = adjugate(A)
	%ADJUGATE return the adjoint matrix of A for numeric matrices
	%	Input:
	%		A:	matrix the adjoint is calculated for
	%	Output:
	%		B:	adjoint matrix
	% This finds the adjugate (adjoint) of square matrix A,
	% and is valid even if A is singular or complex-valued.
	% With u, s, and v obtained from [u,s,v] = svd(A), it
	% makes use of the identity adj(A) = det(u*v')*v*adj(s)*u',
	% which holds even if A and s are singular.  The expression,
	% diag(prod(reshape(s0(ix),n-1,n),1)), accomplishes the
	% evaluation of adj(s), each of whose diagonal elements
	% is the product of all but one of the diagonal elements
	% of s.  This requires that A be n x n where n >= 2.
	% Roger Stafford - 10/18/06
	%https://de.mathworks.com/matlabcentral/fileexchange/12692-adjugate--adjoint--of-a-square-matrix
	if isa(A, 'sym')
		B = adjoint(A);
		return;
	end
	[m, n] = size(A);
	if m ~= n || n < 2
		error('math:adjugate', 'Matrix A should be size n x n with n >= 2.')
	end
	[u, s, v] = svd(A);
	s0 = diag(s);
	ix = toeplitz(ones(n - 1, 1), [1, zeros(1, n - 1)]) + repmat((1:n - 1)', 1, n);
	B = det(u*v')*v*diag(prod(reshape(s0(ix), n - 1, n), 1))*u';
end
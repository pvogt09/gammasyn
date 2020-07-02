function [D_dervderv] = hessian_secondDerivativeDifferentParams(A_diff, B_diff, Lambda_diff, V_diff, W)
	%HESSIAN_SECONDDERIVATIVEDIFFERENTPARAMS calculates the second order derivative of the generalized eigenvalues to the generalized eigenvalue problem specified by A*V = B*V*Lambda with respect to two DIFFERENT parameters
	%		A,B:			Matrices of the generalized eigenvalue Problem
	%	Input:
	%		A_diff:			TODO: derivative of matrix A with respect to ?
	%		B_diff:			TODO: derivative of matrix B with respect to ?
	%		Lambda_diff:	TODO: derivative of eigenvalues with respect to ?
	%		V_diff:			TODO: derivative of right eigenvectormatrix with respect to ?
	%		W_H:			matrix of left eigenvectors (W = inv(V)')
	%	Output:
	%		D_dervderv:		diagonal matrix of second order derivatives with respect to two different parameters
	A_ij = A_diff(:, :, 1);
	A_st = A_diff(:, :, 2);
	B = B_diff(:, :, 1);
	B_ij = B_diff(:, :, 2);
	B_st = B_diff(:, :, 3);
	Lambda = Lambda_diff(:, :, 1);
	Lambda_ij = Lambda_diff(:, :, 2);
	Lambda_st = Lambda_diff(:, :, 3);
	V = V_diff(:, :, 1);
	V_ij = V_diff(:, :, 2);
	V_st = V_diff(:, :, 3);
	n = size(A_ij, 1);
	D_dervderv = zeros(n, 1) + 0i;
	for kk = 1:n
		D_dervderv(kk, 1) = W(:, kk)'*(A_ij - B_ij*Lambda(kk, kk) - B*Lambda_ij(kk, kk))*V_st(:, kk) + W(:, kk)'*(A_st - B_st*Lambda(kk, kk) - B*Lambda_st(kk, kk))*V_ij(:, kk) + W(:, kk)'*(-B_ij*Lambda_st(kk, kk) - B_st*Lambda_ij(kk, kk))*V(:, kk);
	end
end
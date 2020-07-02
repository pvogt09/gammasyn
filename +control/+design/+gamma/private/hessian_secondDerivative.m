function [D_dervderv] = hessian_secondDerivative(A, B, V_tilde, lambda, W_tilde)
	%HESSIAN_SECONDDERIVATIVE calculates the second order derivative of the generalized eigenvalues to the generalized eigenvalue problem specified by A*V = B*V*Lambda with respect to a parameter
	%	A, B:			Matrices of the generalized eigenvalue problem
	%	Input:
	%		A:			matrix of the generalized eigenvalue problem
	%		B:			matrix of the generalized eigenvalue problem
	%	Output:
	%		D_dervderv:	vector of second order derivatives
	if nargin >= 5
		[V_derv, D_derv, ~, V, W, D] = hessian_firstDerivative(A, B, V_tilde, lambda, W_tilde);
	else
		[V_derv, D_derv, ~, V, W, D] = hessian_firstDerivative(A, B, V_tilde, lambda);
	end
	n = size(A, 1);
	D_dervderv = zeros(n);
	for ii = 1:n
		D_dervderv(ii, ii) = W(:, ii)'*(A(:, :, 3) - D(ii, 1)*B(:, :, 3) - 2*D_derv(ii, 1)*B(:, :, 2))*V(:, ii) - 2*W(:, ii)'*(D(ii, 1)*B(:, :, 2) + D_derv(ii, 1)*B(:, :, 1) - A(:, :, 2))*V_derv(:, ii);
	end
end
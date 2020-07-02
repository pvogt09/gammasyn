function [V_derv, D_derv, W_derv, V, W, lambda] = hessian_firstDerivative(A, B, V_tilde, lambda, W_tilde)
	%FIRSTDERIVATIVE calculates the first order derivative of the eigenvalues and eigenvectors of the generalized eigenvalue problem (distinct eigenvalues only!) with respect to a parameter
	%	Input:
	%		A:			system matrix and contains as well the first order derivative on the second plane
	%		B:			Matrix B of generalized eigenvalue problem and contains as well the first order derivative on the second plane
	%		V_ilde:		matrix of right eigenvectors
	%		lambda:		vector of eigenvalues
	%		W_tilde:	matrix of left eigenvectors
	%	Output:
	%		V_derv:		first order derivative of right eigenvector matrix
	%		D_derv:		first order derivative of eigenvalue matrix
	%		W_derv:		first order derivative of left eigenvector matrix
	%		V:			right eigenvector matrix
	%		W_H:		left eigenvector matrix
	%		D:			eigenvalue matrix

	% size of the system
	n = size(A, 1);
	% initialize
	V_derv = zeros(n, n) + 0i;
	D_derv = zeros(n, 1) + 0i;
	% TODO: add precision
	if length(unique(lambda)) ~= n
		error('control:design:gamma:eigenvalues:vanderaa', 'Eigenvalue problem has repeated eigenvalues');
	end
	V = V_tilde;
	% conjugate-complex transpose left eigenvector matrix (chosen so that W_H*B*V = I)
	% TODO: what happens, if not invertible, also suppress warning about badly scaled and singular close to working precision
	if nargin <= 4
		W_H = inv(B(:, :, 1)*V_tilde);
	else
		W_H = W_tilde';
	end
	% eigenvalue derivatives
	for ii = 1:n
		D_derv(ii, 1) = W_H(ii, :)*(A(:, :, 2) - lambda(ii, 1)*B(:, :, 2))*V(:, ii);
	end
	% normalization of matrix V
	[Gamma, invGamma, M] = vanDerAa_normalizeEigenvectorMatrix(V, W_H);
	V = V*Gamma;
	W_H = invGamma*W_H;
	% initialization
	C = zeros(n, n) + 0i;
	% calculation of the entries of matrix C
	for ii = 1:n
		for jj = 1:n
			if ii ~= jj
				difference = lambda(jj, 1) - lambda(ii, 1);
				C(ii, jj) = W_H(ii, :)*(A(:, :, 2) - lambda(jj, 1)*B(:, :, 2))*V(:, jj)/difference;
			end
		end
	end
	for kk = 1:n
		sum = 0 + 0i;
		m = M(kk);
		for ii = 1:n
			if ii ~= kk
				sum = sum + V(m, ii)*C(ii, kk);
			end
		end
		C(kk, kk) = -sum;
	end
	V_derv = V*C;
	W = W_H';
	if nargout >= 3
		W_derv = -W*V_derv'*W;
	end
end
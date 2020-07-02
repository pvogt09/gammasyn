function [Gamma, invGamma, M] = vanDerAa_normalizeEigenvectorMatrix(X, Y)
	%VANDERAA_NORMALIZEEIGENVECTORMATRIX Calculates diagonal matrix Gamma which normalizes X. Determines the index m for every eigenvector
	%	Input:
	%		X:			right eigenvector matrix
	%		Y:			left eigenvector matrix hermitian transposed
	%	Output:
	%		Gamma:		diagonal matrix to normalize X
	%		invGamma:	inverse matrix of Gamma
	%		M:			stores the index m for every eigenvector
	N = size(X, 1);
	M = zeros(N, 1);
	for ii = 1:N
		temp = -1;
		m = 1;
		for jj = 1:N
			%if abs(X(i, j))*abs(Y(j, i)) > temp
			if abs(X(jj, ii))*abs(Y(ii, jj)) > temp
				temp = abs(X(jj, ii))*abs(Y(ii, jj));
				m = jj;
			end
		end
		M(ii, 1) = m;
	end
	gamma = X(sub2ind(size(X), M, (1:size(M, 1))'));% get element from M for every column (gamma(ii, 1) = X(M(ii, 1), ii))
	invGamma = diag(gamma);
	Gamma = diag(1./gamma);
end
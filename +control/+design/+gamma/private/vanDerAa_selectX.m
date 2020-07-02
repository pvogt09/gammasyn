function [Xq] = vanDerAa_selectX(q, X, part)
	%VANDERAA_SELECTX selects the q-th block of the current partitioning of X
	%		q:		number of block
	%		X:		right eigenvector matrix
	%		part:	partitioning
	%	Output:
	%		Xq:		q-th block of right eigenvector matrix
	k = sum(part(1:(q - 1)));
	Xq = X(:, k + 1:k + part(q));
end
function [Yq] = vanDerAa_selectY(q, Y, part)
	%VANDERAA_SELECTY selects the q-th block of the current partitioning of Y
	%	Input:
	%		q:		number of block
	%		Y:		left eigenvector matrix
	%		part:	partitioning
	%	Output:
	%		Yq:		q-th block of left igenvector matrix
	k = sum(part(1:(q - 1)));
	Yq = Y(k + 1:k + part(q), :);
end
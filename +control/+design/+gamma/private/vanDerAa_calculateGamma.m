function [gamma] = vanDerAa_calculateGamma(n, k, l)
	%VANDERAA_CALCULATEGAMMA calculates the factor for L
	%	Input:
	%		n:		input parameters for calculation of gamma
	%		k:		input parameters for calculation of gamma
	%		l:		input parameters for calculation of gamma
	%	Output:
	%		gamma:	factor gamma needed in calculation of L
	gamma = 0;
	if k >= n - k + l - 1
		gamma = nchoosek(n, k);
	else
		gamma = nchoosek(n, n - k + l - 1);
	end
end
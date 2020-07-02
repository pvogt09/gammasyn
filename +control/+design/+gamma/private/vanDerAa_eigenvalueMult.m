function [M, lambda] = vanDerAa_eigenvalueMult(Lambda)
	%VANDERAA_EIGENVALUEMULT determines eigenvalue with highest multiplicity in Lambda and returns it together with its multiplicity
	%	Input:
	%		Lambda:	eigenvalue matrix (diagonal matrix where eigenvalue are sorted after multiplicity)
	%	Output:
	%		M:		highest multiplicity
	%		lambda:	eigenvalue with highest multiplicity 
	% TODO: allow for numerical inaccuracies
	n = size(Lambda, 1);
	% vector to stores multiplicity of eigenvalues
	mult = zeros(1, n, 'int32');
	current = Lambda(1, 1);
	% vector to store distinct eigenvalues
	diffEigs = zeros(1, n) + 0i;
	diffEigs(1) = current;
	tmp = int32(1);
	currDiff = int32(1);
	for index = 2:n
		if (Lambda(index, index) == current)
			tmp = tmp + 1;
		else
			mult(currDiff) = tmp;
			current = Lambda(index, index);
			diffEigs(currDiff) = current;
			tmp = int32(1);
			currDiff = currDiff + 1;
		end
	end
	mult(currDiff) = tmp;
	M = max(mult);
	lambda = Lambda(n, n);
end
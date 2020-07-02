function [multiplicity, multiplicity_map] = eigenvalue_multiplicity(eigenvalues, tolerance)
	%EIGENVALUE_MULTIPLICITY caluclate eigenvalue multiplicity with specified tolerance
	%	Input:
	%		eigenvalues:		vector of eigenvalues of a matrix
	%		tolerance:			tolerance of eigenvalue equality
	%	Output:
	%		multiplicity:		multiplicity of the eigenvalues
	%		multiplicity_map:	corresponding multiple eigenvalues
	multiplicity_map = false(size(eigenvalues, 1), size(eigenvalues, 1));
	parfor ii = 1:size(eigenvalues, 1)
		m = multiplicity_map(ii, :);
		m(1, ii) = true;
		for jj = ii + 1:size(eigenvalues, 1)
			if abs(eigenvalues(ii, 1) - eigenvalues(jj, 1)) < tolerance
				m(1, jj) = true;
			end
		end
		multiplicity_map(ii, :) = m;
	end
	multiplicity_map = logical(multiplicity_map + multiplicity_map');
	multiplicity = sum(uint32(multiplicity_map), 2, 'native');
end
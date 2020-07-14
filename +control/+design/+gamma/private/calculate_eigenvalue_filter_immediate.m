function [eigenvalues] = calculate_eigenvalue_filter_immediate(filtertype, eigenvalues)
	%CALCULATE_EIGENVALUE_FILTER_IMMEDIATE function for filtering eigenvalues immediately after solving eigenvalue problem
	%	Input:
	%		filtertype:		type of eigenvalue filtering
	%		eigenvalues:	eigenvalues of all systems (NaN + 1i*NaN for systems with fewer states than maximum number of states)
	%	Output:
	%		eigenvalues:	eigenvalues of all systems (NaN + 1i*NaN for systems with fewer states than maximum number of states)
	if isempty(filtertype)
		return;
	end
	earlyexit = filtertype == GammaEigenvalueFilterType.NONE | filtertype == GammaEigenvalueFilterType.NEGATIVEIMAG | filtertype == GammaEigenvalueFilterType.POSITIVEIMAG;
	if all(earlyexit)
		return;
	end
	notNaNeigenvalues = ~isnan(eigenvalues);
	for ii = 1:size(filtertype, 1) %#ok<FORPF> no parfor because of change in eigenvalues
		remove = false(size(eigenvalues));
		replacevalue = NaN;
		switch filtertype(ii, 1)
			case GammaEigenvalueFilterType.NONE
			case GammaEigenvalueFilterType.NEGATIVEIMAG
			case GammaEigenvalueFilterType.POSITIVEIMAG
			case GammaEigenvalueFilterType.INFTOMINUSINF
				remove = isinf(eigenvalues) & real(eigenvalues) > 0;
				replacevalue = -Inf;
			case GammaEigenvalueFilterType.MINUSINFTOINF
				remove = isinf(eigenvalues) & real(eigenvalues) < 0;
				replacevalue = Inf;
			case GammaEigenvalueFilterType.INFTOZERO
				remove = isinf(eigenvalues) & real(eigenvalues) > 0;
				replacevalue = 0;
			case GammaEigenvalueFilterType.MINUSINFTOZERO
				remove = isinf(eigenvalues) & real(eigenvalues) < 0;
				replacevalue = 0;
			otherwise
				error('control:design:gamma:eigenvalues:filter', 'Undefined eigenvalue filter type.');
		end
		remove = remove & notNaNeigenvalues;
		if ~any(remove(:))
			continue;
		end
		eigenvalues(remove) = replacevalue;
	end
end
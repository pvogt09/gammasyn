function [sorted, idx] = mindistsort(sortval, reference)
	%MINDISTSORT sort a list of values according to a list of reference values
	%	Input:
	%		sortval:	list of values to be sorted, which contains the values in the columns
	%		reference:	list of values to take as reference
	%	Output:
	%		sorted:		values sorted such that the distance to the reference is minimized
	%		idx:		indices such that sorted = sortval(:, idx)
	if ndims(sortval) ~= ndims(reference) || ndims(sortval) > 2
		error('mindistsort:input', 'Dimension of values to sort and reference must be equal.');
	end
	if size(sortval, 1) ~= size(reference, 1) || size(sortval, 2) ~= size(reference, 2)
		error('mindistsort:input', 'Dimension of values to sort and reference must be equal.');
	end
	idxmat = perms(1:size(sortval, 2));
	distancemat = Inf(size(idxmat, 1), 1);
	parfor ii = 1:size(idxmat, 1)
		distance = Inf(size(sortval, 2), 1);
		for jj = 1:size(sortval, 2)
			distance(jj, 1) = norm(sortval(:, idxmat(ii, jj)) - reference(:, jj), 2);
		end
		distancemat(ii, 1) = sum(distance);
	end
	[~, idx] = min(distancemat);
	if ~isempty(idx)
		idx = idxmat(idx, :);
		sorted = sortval(:, idx);
	else
		idx = 1:size(sortval, 2);
		sorted = sortval;
	end
end
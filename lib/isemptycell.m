function [is] = isemptycell(cellarray)
	%ISEMPTYCELL return true for empty elements of a cell array
	%	Input:
	%		cellarray:	cell array to check for empty elements
	%	Output:
	%		is:			true, where elements of cell array are empty, else false
	if ~iscell(cellarray)
		error('isemptycell:input', 'undefined function or variable ''iscellarray'' for arguments of type ''%s''.', class(cellarray));
	end
	is = cellfun(@isempty, cellarray, 'UniformOutput', true);
end
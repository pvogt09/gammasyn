function [ordered] = sortstruct(structure)
	%SORTSTRUCT sort structure fieldnames alphabetically
	%	Input:
	%		structure:	structure to sort
	%	Output:
	%		ordered:	structure with ordered fieldnames
	%	See:
	%		https://de.mathworks.com/matlabcentral/newsreader/view_thread/17094
	Dimensions = size(structure);
	structure = structure(:);
	[f, ix] = sort(fieldnames(structure));
	v = struct2cell(structure);
	ordered = cell2struct(v(ix, :), f, 1);
	ordered = reshape(ordered,Dimensions);
end
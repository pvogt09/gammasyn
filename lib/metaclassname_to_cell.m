function [cell] = metaclassname_to_cell(metaclass)
	%METACLASSNAME_TO_CELL convert names of a metaclass to a cell-array of names
	%	Input:
	%		metaclass:	object or array of meta.class type
	%	Output:
	%		cell:		names of the metaclasses as cellstring
	if ~isa(metaclass, 'meta.class')
		error('meta:class:tocell', 'class must be of type ''meta.class''.');
	end
	cell = {metaclass.Name}';
end
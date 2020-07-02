function [escaped] = escape_printf(input)
	%ESCAPE_PRINTF escape string for usage in printf
	%	Input:
	%		input:		string to escape
	%	Output:
	%		escaped:	escaped string
	escaped = strrep(input, '\', '\\');
end
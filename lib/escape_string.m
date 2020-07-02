function [string] = escape_string(s)
	%ESCAPE_STRING escape single quotes in string
	%	Input:
	%		s:		string to escape
	%	Output:
	%		string:	escaped string
	string = strrep(s, '''', '''''');
end
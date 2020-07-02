function [is] = isparameterstruct(param)
	%ISPARAMETERSTRUCT zurückgeben, ob eine Parameterstruktur vorliegt
	%	Input:
	%		param:	zu überprüfende Variable
	%	Output:
	%		is:		true, wenn eine Parameterstruktur vorliegt, sonst false
	%	TODO: Felder abfragen
	is = isstruct(param);
end
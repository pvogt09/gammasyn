function [is] = isparameterstruct(param)
	%ISPARAMETERSTRUCT zur�ckgeben, ob eine Parameterstruktur vorliegt
	%	Input:
	%		param:	zu �berpr�fende Variable
	%	Output:
	%		is:		true, wenn eine Parameterstruktur vorliegt, sonst false
	%	TODO: Felder abfragen
	is = isstruct(param);
end
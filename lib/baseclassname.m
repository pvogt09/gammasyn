function [baseclass] = baseclassname(object)
	%BASECLASSNAME Klassennamen einer Klasse mit Paketnamen ermitteln
	%	Input:
	%		object:		Objekt vom Typ der zu ermittelnden Klasse
	%	Output:
	%		baseclass:	Klassenname der Klasse
	cls = strsplit(class(object), '.');
	baseclass = cls{end};
end
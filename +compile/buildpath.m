function [path] = buildpath()
	%BUILDPATH Pfad, in dem die Codegenerierung stattfinden soll, zur�ckgeben
	%	Output:
	%		path:	Pfad der Codegenerierung
	path = realpath(fullfile(mfilename('fullpath'), '..', '..', 'codegen'));
end
function [path] = destpath()
	%DESTPATH Pfad, in den die generierten Dateien verschoben werden sollen, zurückgeben
	%	Output:
	%		path:	Zielpfad der Codegenerierung
	path = realpath(fullfile(mfilename('fullpath'), '..', '..'));
end
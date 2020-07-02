function [path] = destpath()
	%DESTPATH Pfad, in den die generierten Dateien verschoben werden sollen, zur�ckgeben
	%	Output:
	%		path:	Zielpfad der Codegenerierung
	path = realpath(fullfile(mfilename('fullpath'), '..', '..'));
end
function [path] = destpath()
	%DESTPATH Zielpfad f�r Ergebnisse
	%	Output:
	%		path:	Zielpfad f�r Ergebnisse
	path = realpath(fullfile(mfilename('fullpath'), '..', '..', 'Ergebnisse'));
end
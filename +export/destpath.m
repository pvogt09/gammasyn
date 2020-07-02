function [path] = destpath()
	%DESTPATH Zielpfad für Ergebnisse
	%	Output:
	%		path:	Zielpfad für Ergebnisse
	path = realpath(fullfile(mfilename('fullpath'), '..', '..', 'Ergebnisse'));
end
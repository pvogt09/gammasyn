function [s] = getstruct(s, path)
	%GETSTRUCT zurückgeben eines Elementes einer Struktur
	%	Input:
	%		s:		Struktur, deren Wert zurückgegeben werden sollen
	%		path:	Pfad des zu lesenden Elementes
	%	Output:
	%		value:	Wert aus der Struktur
	narginchk(2, 2);
	if ~isstruct(s)
		error('getstruct:input', 'function can only be used with structures.');
	end
	if ~ischarstring(path)
		error('getstruct:input', 'path must be a string.');
	end
	if ischar(path)
		path = {path};
	end
	if size(path, 1) > size(path, 2)
		path = path';
	end
	path(2, :) = repmat({'.'}, 1, size(path, 2));
	path = reshape(path, 1, 2*size(path, 2));
	path(end) = [];
	path = [{'.'}, path];
	s = subsref(s, substruct(path{:}));
end
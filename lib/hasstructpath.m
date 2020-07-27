function [has] = hasstructpath(struct, path)
	%HASSTRUCTPATH ermitteln, ob ein Pfad in einer Struktur existiert
	%	Input:
	%		struct:		zu überprüfende Struktur
	%		path:		zu überprüfender Pfad
	%	Output:
	%		has:		true, wenn der Pfad existiert, sonst false
	narginchk(2, 2);
	if ~isstruct(struct)
		error('hasstructpath:input', 'function can only be used with structures.');
	end
	if ~ischarstring(path)
		error('hasstructpath:input', 'path must be a string.');
	end
	if ischar(path)
		path = {path};
	end
	tempstruct = struct;
	has = true;
	for i = 1:length(path)
		if isstruct(tempstruct) && isfield(tempstruct, path{i})
			tempstruct = tempstruct.(path{i});
		else
			has = false;
			return;
		end
	end
end
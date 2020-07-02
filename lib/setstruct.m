function [s] = setstruct(s, path, value)
	%SETSTRUCT setzten eines Elementes einer Struktur
	%	Input:
	%		s:		Struktur, in der der Wert gesetzt werden sollen
	%		path:	Pfad des zu setzenden Elementes
	%		value:	Wert des Elementes
	%	Output:
	%		s:		Struktur mit gesetzten Werten
	narginchk(3, 3);
	if ~isstruct(s)
		error('setstruct:input', 'function can only be used with structures.');
	end
	if ~ischarstring(path)
		error('setstruct:input', 'path must be a string.');
	end
	if ischar(path)
		path = {path};
	end
	if size(path, 1) > size(path, 2)
		path = path';
	end
% 	if isa(value, 'sym')
% 		tempstruct = s;
% 		hierarchy = cell(length(path), 1);
% 		wasadded = false;
% 		for i = 1:length(path)
% 			if isstruct(tempstruct) && isfield(tempstruct, path{i})
% 				hierarchy{i} = tempstruct;
% 				tempstruct = tempstruct.(path{i});
% % 				if i == length(path)
% % 					default = tempstruct;
% % 				end
% 			else
% 				if isstruct(tempstruct) && ~wasadded
% 					hierarchy{i} = tempstruct;
% 					wasadded = true;
% 					tempstruct = struct();
% 				elseif i ~= length(path)
% 					hierarchy{i} = struct();
% 					tempstruct = struct();
% 				end
% 			end
% 		end
% 		tempstruct = value;
% 		for i = size(hierarchy, 1):-1:1
% 			hierarchy{i}.(path{i}) = tempstruct;
% 			tempstruct = hierarchy{i};
% 		end
% 		s = tempstruct;
% 	else
	path(2, :) = repmat({'.'}, 1, size(path, 2));
	path = reshape(path, 1, 2*size(path, 2));
	path(end) = [];
	path = [{'.'}, path];
	if isnumeric(value)
		s = subsasgn(s, substruct(path{:}), value);
	else
		if isa(value, 'sym') || isa(value, 'sdpvar')
			s = builtin('subsasgn', s, substruct(path{:}), value);
		else
			s = subsasgn(s, substruct(path{:}), value);
		end
	end
end
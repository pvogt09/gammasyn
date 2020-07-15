function [parameter] = isset(parameter, path, default, set)
	%ISSET setzten der Elemente einer Struktur oder Verwendung des bereits gesetzten Wertes
	%	Input:
	%		parameter:	Struktur, in der die Werte gesetzt werden sollen
	%		path:		Pfad des zu setzenden Elementes
	%		default:	Wert des Elementes, wenn es noch nicht gesetzt ist
	%		set:		Indikator, ob der Standardwert gesetzt werden soll, falls das vorletzte Element im Pfad ein Objekt ist oder Funktionszeiger auf eine Funktion, mit der ein vorhandener Wert konvertiert werden soll (diese Funktion muss idempotent sein)
	%	Output:
	%		parameter:	Struktur mit gesetzten oder übernommenen Werten
	narginchk(3, 4);
	setfunction = [];
	if nargin <= 3
		set = true;
	else
		if ~islogical(set)
			if isfunctionhandle(set)
				setfunction = set;
				set = true;
			end
		end
	end
	if ~isstruct(parameter)
		error('isset:input', 'function can only be used with structures.');
	end
	if ~ischarstring(path)
		error('isset:input', 'path must be a string.');
	end
	if ~islogical(set)
		error('isset:input', 'set must be a logical.');
	end
	if ischar(path)
		path = {path};
	end
	if ~set
		return;
	end
	if isa(default, 'identifier.parameter.Parameter')
		default.setPath(path);
	end
	tempstruct = parameter;
	hierarchy = cell(length(path), 1);
	wasadded = false;
	for i = 1:length(path)
		if isstruct(tempstruct) && isfield(tempstruct, path{i})
			hierarchy{i} = tempstruct;
			tempstruct = tempstruct.(path{i});
			if i == length(path)
				default = tempstruct;
			end
		elseif isobject(tempstruct) && isprop(tempstruct, path{i})
			hierarchy{i} = tempstruct;
		else
			if isstruct(tempstruct) && ~wasadded
				hierarchy{i} = tempstruct;
				wasadded = true;
				tempstruct = struct();
			elseif isobject(tempstruct) && ~wasadded
				hierarchy{i} = tempstruct;
				wasadded = true;
			elseif i ~= length(path)
				hierarchy{i} = struct();
				tempstruct = struct();
			end
		end
	end
	if ~isempty(setfunction)
		tempstruct = setfunction(default);
	else
		tempstruct = default;
	end
	for i = size(hierarchy, 1):-1:1
		hierarchy{i}.(path{i}) = tempstruct;
		tempstruct = hierarchy{i};
	end
	parameter = tempstruct;
end
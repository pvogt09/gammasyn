function [pathabs] = realpath(pathrel)
	%REALPATH relativen Pfad in absoluten Pfad umwandeln
	%	Input:
	%		pathrel:	relativer Pfad
	%	Output:
	%		pathabs:	absoluter Pfad
	pathrel = strrep(strrep(pathrel, '/', filesep), '\\', filesep);
	parts = strsplit(pathrel, filesep);
	notempty = ~cellfun(@isempty, parts, 'UniformOutput', true);
	idx = find(notempty, 1, 'first');
	if ~isempty(idx) && idx > 0
		notempty(1:idx) = true(idx, 1);
	end
	parts = parts(notempty);
	abs = cell(0, 1);
	for i = 1:length(parts)
		if isempty(parts{i})
			abs = [
				abs;
				parts(i)
			];
			continue;
		end
		if strcmp(parts{i}, '.')
			continue;
		end
		if strcmp(parts{i}, '..')
			if ~isempty(abs)
				abs = abs(1:end - 1, :);
			end
			continue;
		end
		abs = [
			abs;
			parts(i)
		];
	end
	pathabs = strjoin(abs, filesep);
end
function [tests] = findTests(pfad, exclude, pattern)
	%FINDTESTS Testdateien rekursiv suchen
	%	Input:
	%		pfad:		Pfad, in dem die Tesdateien gesucht werden sollen
	%		exclude:	Suchmuster, die ignoriert werden sollen
	%		pattern:	Suchmuster für Testdateien
	%	Output:
	%		tests:		cell-Array mit Testdateien
	if nargin <= 1
		exclude = {
			'private'
		};
	end
	if nargin <= 2 || isempty(pattern)
		pattern = '*Test.m';
	end
	if ~ischar(pfad)
		error('test:find', 'Der Suchpfad muss ein String sein.');
	end
	if ischar(exclude)
		exclude = {exclude};
	end
	if ~iscell(exclude)
		error('test:find', 'Das Ignoriermuster muss ein String sein.');
	end
	if ~ischar(pattern)
		error('test:find', 'Das Suchmuster muss ein String sein.');
	end
	%% Tests in diesem Ordner finden
	pfad = realpath(fullfile(pfad));
	tests = findTest(pfad, pattern);

	%% Unterferzeichnisse durchsuchen
	Eintraege = dir(pfad);
	dirs = Eintraege([Eintraege.isdir]);
	parfor i = 1:length(dirs)
		if ~strcmpi(dirs(i).name, '.') && ~strcmpi(dirs(i).name, '..') && ~ismember(dirs(i).name, exclude)
			tests = [
				tests;
				findTests(fullfile(pfad, dirs(i).name), exclude, pattern);
			];
		end
	end
end


function [tests] = findTest(path, pattern)
	%FINDTEST Testdateien in einem Ordner finden
	%	Input:
	%		path:		Pfad, in dem nach Testdateien gesucht wird
	%		pattern:	Suchmuster für Testdateien
	%	Output:
	%		tests:		Testdateien
	l = dir(fullfile(path, pattern));
	idx = strfind(path, '+');
	if any(idx)
		packagename = strrep(path, path(1:idx(1)), '');
		packagename = [strrep(packagename, [filesep, '+'], '.'), '.'];
	else
		packagename = '';
	end
	l = l(~[l.isdir]);
	if ~isempty(l)
		% dir(path, pattern) ignoriert Klein- und Großschreibung (unter Windows) und findet deshalb mehr, als gefunden werden soll
		pattern = strrep(pattern, '.', '\.');
		if pattern(1) == '*'
			pattern = ['.', pattern];
		end
		if pattern(end) == '*'
			pattern = [pattern(1:end - 1), '.', pattern(end)];
		end
		matches = arrayfun(@(x) ~isempty(regexp(x.name, pattern, 'once')), l, 'UniformOutput', true);
		l = l(matches);
		l = {l.name}';
		l = strrep(l, '.m', '');
		l = cellfun(@(x) [packagename, x], l, 'UniformOutput', false);
		tests = l;
	else
		tests = [];
	end
end
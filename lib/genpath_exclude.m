function [pathstr] = genpath_exclude(directory, excludeDirs)
	%GENPATH_EXCLUDE generate path string excluding several directories
	%	Input:
	%		directory:		directory to generate path for
	%		excludeDirs:	pattern for directories to exclude
	%	Output:
	%		pathstr:		string to use in path
	if ischar(excludeDirs)
		excludeStr = excludeDirs;
	else
		excludeStr = '';
		if ~iscellstr(excludeDirs)
			error('genpath:input', 'excludeDirs input must be a cell-array of strings');
		end
		for ii = 1:length(excludeDirs) %#ok<FORPF> no parfor because of growing of excludeStr
			excludeStr = [excludeStr '|^' excludeDirs{ii} '$'];
		end
	end

	files = dir(directory);
	if isempty(files)
		pathstr = directory;
		return;
	end

	pathstr = [directory, pathsep];

	isdir = logical(cat(1, files.isdir));
	dirs = files(isdir);

	for ii = 1:length(dirs) %#ok<FORPF> no parfor because of growing of pathstr
		dirname = dirs(ii).name;
		if strcmpi(dirname, '.') || strcmpi(dirname, '..') || strcmpi(dirname, 'private')
			continue;
		end
		if ~any(regexp(dirname, ['^\@.*|^\+.*|', excludeStr ], 'start'))
			pathstr = [pathstr, genpath_exclude(fullfile(directory, dirname), excludeStr)];
		end
	end
end
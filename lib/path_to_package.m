function [package] = path_to_package(path)
	%PACKAGE_TO_PATH convert pathname to packagename
	%	Input:
	%		path:		pathname
	%	Output:
	%		package:	packagename
	if ~ischar(path)
		error('path2package2:input', 'Filename must be of type ''char''.');
	end
	if ispackage(path)
		find = strfind(path, filesep);
		if isempty(find)
			error('path2package2:input', 'Invalid pathname supplied.');
		end
		if strcmpi(path(1), '+')
			inpackage = true;
			package = {path(2:find(1) - 1)};
		else
			inpackage = false;
			package = cell(0, 1);
		end
		for i = 1:length(find) - 1
			if find(i) + 1 < length(path)
				if strcmpi(path(find(i) + 1), '+')
					inpackage = true;
					if i == length(find)
						package(size(package, 1) + 1, 1) = {path(find(i) + 2:end)};
					else
						package(size(package, 1) + 1, 1) = {path(find(i) + 2:find(i + 1) - 1)};
					end
				else
					if inpackage
						package= cell(0, 1);
					end
				end
			end
		end
		package = strjoin(package, '.');
		if inpackage
			if find(end) ~= length(path)
				if strcmpi(path(find(end) + 1), '+')
					name = path(find(end) + 1:end);
				else
					[~, name, ~] = fileparts(path(find(end) + 1:end));
				end
				package = [package, '.', name];
			else
				package = strrep(strrep(package, '+', ''), filesep, '');
			end
		end
	else
		[~, package, ~] = fileparts(path);
	end
	if isempty(package)
		package = '';
	end
end
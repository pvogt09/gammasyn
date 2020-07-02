function [is] = ispackage(path)
	%ISPACKAGE return if path is a package path
	%	Input:
	%		path:	path to proceed
	%	Output:
	%		is:		true, if path is a package path, else false
	if ischar(path)
		is = checkpackage(path);
		return;
	end
	if ~iscellstr(path)
		error('ispackage2:input', 'Input argument must be of type ''char''.');
	end
	is = cellfun(@checkpackage, path, 'UniformOutput', true);
end

function [is] = checkpackage(path)
	%CHECKPACKAGE check if a path is a package path
	%	Input:
	%		path:	path to proceed
	%	Output:
	%		is:		true, if path is a package, else false
	is = false;
	if isempty(path)
		return;
	end
	find = strfind(path, filesep);
	if isempty(find)
		%is = strcmpi(path(1), '+');
		return;
	end
	if find(end) < length(path) && strcmpi(path(find(end) + 1), '+')
		return;
	end
	inpackage = strcmpi(path(1), '+');
	for i = 1:length(find) - 1
		if find(i) + 1 < length(path)
			if strcmpi(path(find(i) + 1), '+')
				inpackage = true;
			else
				if inpackage
					return;
				end
			end
		end
	end
	is = inpackage;
end
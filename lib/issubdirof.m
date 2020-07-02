function [is] = issubdirof(subdir, parent)
	%ISSUBDIROF return if a directory is a subdirectory of another one
	%	Input:
	%		subdir:	directory that is supposed to be placed in parent directory
	%		parent:	directory to search for subdirectory
	%	Output:
	%		is:		true, if subdirectory is inside parent directory
	if nargin <= 1
		parent = pwd();
	end
	if ~ischar(subdir)
		error('issubdirof', 'Undefined function or variable issubdir for input arguments of type ''%s''.', class(subdir));
	end
	if ~ischar(parent)
		error('issubdirof', 'Undefined function or variable issubdir for input arguments of type ''%s''.', class(parent));
	end
	subpath = strsplit(strrep(strrep(subdir, '\', '/'), filesep, '/'), '/');
	parentpath = strsplit(strrep(strrep(parent, '\', '/'), filesep, '/'), '/');
	is = false;
	if size(parentpath, 2) <= size(subpath, 2)
		for ii = 1:size(parentpath, 2)
			if size(subpath, 2) >= ii
				if strcmp(parentpath{ii}, subpath{ii})
					if size(parentpath, 2) == ii
						is = true;
					end
				else
					break;
				end
			else
				break;
			end
		end
	end
end
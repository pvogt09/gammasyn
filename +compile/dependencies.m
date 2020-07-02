function [files] = dependencies(file, level)
	%DEPENDENCIES return dependecies of a function
	%	Input:
	%		file:	function the dependencies should be returned for
	%		level:	recursion depth for dependency analysis
	%	Output:
	%		files:	functions that get called by the function to inspect
	if nargin <= 2
		level = 2;
	end
	if ~ischar(file)
		error('dependency', 'The function must be of type ''char''.');
	end
	if ~exist(file, 'file')
		error('dependency', 'The function ''%s'' does not exist.', file);
	end
	files = unique(dependency(file, level));
end

function [files] = dependency(file, level)
	%DEPENDENCY helper function for returning dependencies of a function
	%	Input:
	%		file:	function the dependencies should be returned for
	%		level:	recursion depth for dependency analysis
	%	Output:
	%		files:	functions that get called by the function to inspect
	if level > 0
		if matlab.Version.CURRENT >= matlab.Version.R2015B
			files = matlab.codetools.requiredFilesAndProducts(file, 'toponly')';
		else
			files = depfun(file, '-toponly', '-quiet'); %#ok<DEPFUN> depfun has been removed in R2015B
		end
		noroot = cellfun(@(x) strfind(x, matlabroot), files, 'UniformOutput', false);
		noroot = cellfun(@(x) isempty(x) || x(1) ~= 1, noroot, 'UniformOutput', true);
		files = files(noroot);
		dependentfiles = cell(size(files, 1), 1);
		parfor i = 1:size(files, 1)
			if ~strcmpi(files{i}, file)
				dependentfiles{i} = dependency(files{i}, level - 1);
			end
		end
		files = [
			files;
			vertcat(dependentfiles{:, 1})
		];
	else
		files = cell(0, 1);
	end
end
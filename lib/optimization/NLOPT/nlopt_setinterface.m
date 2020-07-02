function [] = nlopt_setinterface(version)
	%NLOPT_INTERFACE switch between different interfaces of NLOPT
	%	Input:
	%		version:	interface to use
	if nargin <= 0
		version = 'OPTI';
	end
	if ~ischar(version)
		error('optimization:solver:snopt:interface', 'Input argument must be of type ''char''.');
	end
	paths = strsplit(path, pathsep);
	interfacepath = realpath(fullfile(mfilename('fullpath'), '..', 'interface'));
	if exist(realpath(fullfile(interfacepath, 'NLOPT', ['nlopt_optimize.', mexext])), 'file')
		hasnloptinterface = true;
	else
		if exist('nlopt_optimize', 'file') == 3
			hasnloptinterface = true;
		else
			hasnloptinterface = false;
		end
	end
	if exist(realpath(fullfile(interfacepath, 'OPTI', ['nlopt.', mexext])), 'file')
		hasoptiinterface = true;
	else
		if exist('nlopt', 'file') == 3
			hasoptiinterface = true;
		else
			hasoptiinterface = false;
		end
	end
	if strcmpi(version, 'OPTI')
		if ~hasoptiinterface
			warning('optimization:solver:nlopt:interface', 'NLOPT OPTI-Toolbox interface could not be found.')
		end
		if any(strcmp(realpath(fullfile(interfacepath, 'NLOPT')), paths))
			rmpath(realpath(fullfile(interfacepath, 'NLOPT')));
		end
		addpath(realpath(fullfile(interfacepath, 'OPTI')));
		nloptdir = fileparts(which('nlopt'));
		if ~issubdirof(nloptdir, fullfile(interfacepath, 'OPTI'))
			error('optimization:solver:nlopt:interface', 'Changing interface did not succeed, remove %s from the path manually.', fullfile(interfacepath, 'NLOPT'));
		end
	else
		if ~hasnloptinterface
			warning('optimization:solver:nlopt:interface', 'NLOPT original interface could not be found.')
		end
		if any(strcmp(fullfile(interfacepath, 'OPTI'), paths))
			rmpath(realpath(fullfile(interfacepath, 'OPTI')));
		end
		addpath(realpath(fullfile(interfacepath, 'NLOPT')));
		nloptdir = fileparts(which('nlopt_optimize'));
		if ~issubdirof(nloptdir, realpath(fullfile(interfacepath, 'NLOPT')))
			error('optimization:solver:snopt:interface', 'Changing interface did not succeed, remove %s from the path manually.', realpath(fullfile(interfacepath, 'OPTI')));
		end
	end
end
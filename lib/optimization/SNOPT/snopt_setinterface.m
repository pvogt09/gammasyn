function [] = snopt_setinterface(version)
	%SNOPT_INTERFACE switch between different interfaces of SNOPT
	%	Input:
	%		version:	matlab version the interface was made for
	if nargin <= 0
		version = matlab.Version.CURRENT;
	end
	if ~isa(version, 'matlab.Version')
		error('optimization:solver:snopt:interface', 'Input argument must be of type ''matlab.Version''.');
	end
	paths = strsplit(path, pathsep);
	interfacepath = realpath(fullfile(mfilename('fullpath'), '..', 'interface'));
	if version >= matlab.Version.R2015B
		SNOPT_LICENSE = getenv('SNOPT_LICENSE');
		if isempty(SNOPT_LICENSE) || ~exist(SNOPT_LICENSE, 'file')
			warning('optimization:solver:snopt:interface', 'SNOPT License file does not exist, old interface has to be used.')
		end
		if any(strcmp(realpath(fullfile(interfacepath, 'R2015A')), paths))
			rmpath(realpath(fullfile(interfacepath, 'R2015A')));
		end
		addpath(realpath(fullfile(interfacepath, 'R2015B')));
		snoptdir = fileparts(which('snoptmex'));
		if ~issubdirof(snoptdir, fullfile(interfacepath, 'R2015B'))
			error('optimization:solver:snopt:interface', 'Changing interface did not succeed, remove %s from the path manually.', fullfile(interfacepath, 'R2015A'));
		end
	else
		if any(strcmp(fullfile(interfacepath, 'R2015B'), paths))
			rmpath(realpath(fullfile(interfacepath, 'R2015B')));
		end
		addpath(realpath(fullfile(interfacepath, 'R2015A')));
		snoptdir = fileparts(which('snoptmex'));
		if ~issubdirof(snoptdir, realpath(fullfile(interfacepath, 'R2015A')))
			error('optimization:solver:snopt:interface', 'Changing interface did not succeed, remove %s from the path manually.', realpath(fullfile(interfacepath, 'R2015B')));
		end
	end
end
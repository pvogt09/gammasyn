warning('on', 'verbose');
[path, ~, ~] = fileparts(mfilename('fullpath'));
if ispc
	startupexist = strcmpi(path, fullfile(matlabroot, 'bin')) == 0;
else
	startupexist = strcmp(path, fullfile(matlabroot, 'bin')) == 0;
end
if startupexist
	if exist(fullfile(matlabroot, 'bin', 'startup.m'), 'file')
		run(fullfile(matlabroot, 'bin', 'startup.m'));
	end
end
%% set path
addpath(fullfile(path));
addpath(fullfile(path, 'lib'));
addpath(realpath(fullfile(path, 'lib', 'optimization')));
addpath(realpath(fullfile(path, 'lib', 'optimization', 'SNOPT')));
SNOPT_LICENSE = getenv('SNOPT_LICENSE');
if isempty(SNOPT_LICENSE) || ~exist(SNOPT_LICENSE, 'file')
	SNOPT_oldinterface = true;
else
	SNOPT_oldinterface = false;
end
setenv('SNOPT_LICENSE', realpath(fullfile(path, 'lib', 'optimization', 'SNOPT', 'snopt7.lic')));
if ~SNOPT_oldinterface && matlab.Version.CURRENT >= matlab.Version.R2015B
	addpath(realpath(fullfile(path, 'lib', 'optimization', 'SNOPT', 'interface', 'R2015B')));
else
	addpath(realpath(fullfile(path, 'lib', 'optimization', 'SNOPT', 'interface', 'R2015A')));
end
snopt_setinterface(matlab.Version.R2015A);
addpath(realpath(fullfile(path, 'lib', 'optimization', 'NLOPT')));
nlopt_setinterface('OPTI');
addpath(realpath(fullfile(path, 'lib', 'optimization', 'IPOPT')));
addpath(realpath(fullfile(path, 'lib', 'optimization', 'PPPBox')));
addpath(realpath(fullfile(path, 'lib', 'optimization', 'SLQP-GS')));
addpath(realpath(fullfile(path, 'lib', 'optimization', 'SC-BFGS')));
addpath(realpath(fullfile(path, 'lib', 'optimization', 'KSOPT')));
GUROBI_HOME = getenv('GUROBI_HOME');
if ~isempty(GUROBI_HOME) && exist(GUROBI_HOME, 'dir')
	addpath(realpath(fullfile(GUROBI_HOME, 'matlab')));
end
addpath(genpath(realpath(fullfile(path, 'mex'))));
addpath(genpath(realpath(fullfile(path, 'lib', 'mex', 'lib'))));
addpath(realpath(fullfile(path, 'lib', 'mex', 'MinGW')));
addpath(genpath_exclude(realpath(fullfile(path, 'lib', 'YALMIP')), {'.git', 'demos'}));
addpath(genpath_exclude(realpath(fullfile(path, 'lib', 'ROLMIP')), {'.git', 'manual_examples'}));
if exist(realpath(fullfile(path, 'lib', 'oplace')), 'dir')
	addpath(realpath(fullfile(path, 'lib', 'oplace')));
end

%% compatibility settings
% if new functions are added, the function files have to be placed in the folder of the release before the function was included (e. g. newline is available from Matlab 2016B and later and is therefore placed in the folder R2016A)
if verLessThan('matlab', '7.14')
	addpath(genpath(realpath(fullfile(path, 'lib', 'compat', 'R2011B'))));
end
% if verLessThan('matlab', '8.0')
% 	addpath(genpath(realpath(fullfile(path, 'lib', 'compat', 'R2012A'))));
% end
% if verLessThan('matlab', '8.1')
% 	addpath(genpath(realpath(fullfile(path, 'lib', 'compat', 'R2012B'))));
% end
% if verLessThan('matlab', '8.2')
% 	addpath(genpath(realpath(fullfile(path, 'lib', 'compat', 'R2013A'))));
% end
if verLessThan('matlab', '8.3')
	addpath(genpath(realpath(fullfile(path, 'lib', 'compat', 'R2013B'))));
end
if verLessThan('matlab', '8.4')
	addpath(genpath(realpath(fullfile(path, 'lib', 'compat', 'R2014A'))));
end
if verLessThan('matlab', '8.5')
	addpath(genpath(realpath(fullfile(path, 'lib', 'compat', 'R2014B'))));
end
%if verLessThan('matlab', '8.6')
%	addpath(genpath(realpath(fullfile(path, 'lib', 'compat', 'R2015A'))));
%end
% if verLessThan('matlab', '9.0')
% 	addpath(genpath(realpath(fullfile(path, 'lib', 'compat', 'R2015B'))));
% end
if verLessThan('matlab', '9.1')
	addpath(genpath(realpath(fullfile(path, 'lib', 'compat', 'R2016A'))));
end
%if verLessThan('matlab', '9.2')
%	addpath(genpath(realpath(fullfile(path, 'lib', 'compat', 'R2016B'))));
%end
% if verLessThan('matlab', '9.3')
% 	addpath(genpath(realpath(fullfile(path, 'lib', 'compat', 'R2017A'))));
% end
%%
clear encodings i ans e startupexist path SNOPT_LICENSE SNOPT_oldinterface GUROBI_HOME;
dbstop if error;
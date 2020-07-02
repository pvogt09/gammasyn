%MINGW_ENABLEOPENMP enable OpenMP support for MinGW and set compiler to MinGW
MinGWdir = fullfile('D:', 'Programme', 'Msys2', 'MinGW64');
compilerC = mex.getCompilerConfigurations('C', 'Selected');
compilerCPP = mex.getCompilerConfigurations('C++', 'Selected');
compilerFortran = mex.getCompilerConfigurations('Fortran', 'Selected');
if isempty(compilerC)
	C = struct(...
		'Name',			'',...
		'ShortName',	'',...
		'MexOpt',		''...
	);
else
	C = struct(...
		'Name',			compilerC.Name,...
		'ShortName',	compilerC.ShortName,...
		'MexOpt',		compilerC.MexOpt...
	);
end
if isempty(compilerCPP)
	CPP = struct(...
		'Name',			'',...
		'ShortName',	'',...
		'MexOpt',		''...
	);
else
	CPP = struct(...
		'Name',			compilerCPP.Name,...
		'ShortName',	compilerCPP.ShortName,...
		'MexOpt',		compilerCPP.MexOpt...
	);
end
if isempty(compilerFortran)
	Fortran = struct(...
		'Name',			'',...
		'ShortName',	'',...
		'MexOpt',		''...
	);
else
	Fortran = struct(...
		'Name',			compilerFortran.Name,...
		'ShortName',	compilerFortran.ShortName,...
		'MexOpt',		compilerFortran.MexOpt...
	);
end
compilerinformation = struct(...
	'C',				C,...
	'CPP',				CPP,...
	'Fortran',			Fortran,...
	'MW_MINGW64_LOC',	getenv('MW_MINGW64_LOC'),...
	'PATH',				getenv('PATH')...
);
save(fullfile(mfilename('fullpath'), '..', 'oldcompiler.mat'), 'compilerinformation', '-mat');
addpath(realpath(fullfile(mfilename('fullpath'), '..', 'enableopenmp')));
setenv('PATH', [getenv('PATH'), pathsep, MinGWdir, pathsep, fullfile(MinGWdir, 'bin')]);
% https://github.com/firstborg/matlab-2017a/tree/master/toolbox/coder/coder/%2Bcoderprivate
setenv('MW_MINGW64_LOC', MinGWdir);
filename = realpath(fullfile(mfilename('fullpath'), '..', 'mex_C++_mingwopenmp_win64.xml'));
eval(['mex -setup:''', filename, ''' CPP']);
filename = realpath(fullfile(mfilename('fullpath'), '..', 'mex_C_mingwopenmp_win64.xml'));
eval(['mex -setup:''', filename, ''' C']);
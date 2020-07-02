%MINGW_DISABLEOPENMP disable OpenMP support for MinGW and set compiler to old compiler
rmpath(realpath(fullfile(mfilename('fullpath'), '..', 'enableopenmp')));
if exist(fullfile(mfilename('fullpath'), '..', 'oldcompiler.mat'), 'file')
	compilerinformation = load(fullfile(mfilename('fullpath'), '..', 'oldcompiler.mat'));
	compilerinformation = compilerinformation.compilerinformation;
else
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
	save(realpath(fullfile(mfilename('fullpath'), '..', 'oldcompiler.mat')), 'compilerinformation', '-mat');
end
if ~strcmpi(getenv('MW_MINGW64_LOC'), compilerinformation.MW_MINGW64_LOC)
	setenv('MW_MINGW64_LOC', compilerinformation.MW_MINGW64_LOC);
end
if ~strcmpi(getenv('PATH'), compilerinformation.PATH)
	setenv('PATH', compilerinformation.PATH);
end
compilerC = mex.getCompilerConfigurations('C', 'Selected');
compilerCinstalled = mex.getCompilerConfigurations('C', 'Installed');
compilerCPP = mex.getCompilerConfigurations('C++', 'Selected');
compilerCPPinstalled = mex.getCompilerConfigurations('C++', 'Installed');
compilerFortran = mex.getCompilerConfigurations('Fortran', 'Selected');
compilerFortraninstalled = mex.getCompilerConfigurations('Fortran', 'Installed');
if ~isempty(compilerC) && ~strcmpi(compilerC.Name, compilerinformation.C.Name)
	found = false;
	compilerName = '';
	for ii = 1:size(compilerCinstalled, 1) %#ok<FORPF> no parfor for mex setup
		if strcmpi(compilerCinstalled(ii).Name, compilerinformation.C.Name)
			found = true;
			compilerName = compilerCinstalled(ii).Name;
			[folder, file, ~] = fileparts(compilerinformation.C.MexOpt);
			if issubdirof(folder, prefdir())
				if any(strcmpi(file, {
					'mex_C_win32';
					'mex_C_win64'
				}))
					compilerinformation.C.MexOpt = compilerCinstalled(ii).MexOpt;
				end
			end
			eval(['mex -setup:''', compilerinformation.C.MexOpt, ''' C']);
		elseif isempty(compilerinformation.C.ShortName)
			found = true;
		end
	end
	if ~found
		error('mex:mingw:openmp', 'Can not reset C compiler to %s, because it is not installed.', compilerName);
	end
end
if ~isempty(compilerCPP) && ~strcmpi(compilerCPP.Name, compilerinformation.CPP.Name)
	found = false;
	compilerName = '';
	for ii = 1:size(compilerCPPinstalled, 1) %#ok<FORPF> no parfor for mex setup
		if strcmpi(compilerCPPinstalled(ii).Name, compilerinformation.CPP.Name)
			found = true;
			compilerName = compilerCPPinstalled(ii).Name;
			[folder, file, ~] = fileparts(compilerinformation.CPP.MexOpt);
			if issubdirof(folder, prefdir())
				if any(strcmpi(file, {
					'mex_C++_win32';
					'mex_C++_win64'
				}))
					compilerinformation.CPP.MexOpt = compilerCPPinstalled(ii).MexOpt;
				end
			end
			eval(['mex -setup:''', compilerinformation.CPP.MexOpt, ''' CPP']);
		elseif isempty(compilerinformation.CPP.ShortName)
			found = true;
		end
	end
	if ~found
		error('mex:mingw:openmp', 'Can not reset C compiler to %s, because it is not installed.', compilerName);
	end
end
if ~isempty(compilerFortran) && ~strcmpi(compilerFortran.Name, compilerinformation.Fortran.Name)
	found = false;
	compilerName = '';
	for ii = 1:size(compilerFortraninstalled, 1) %#ok<FORPF> no parfor for mex setup
		if strcmpi(compilerFortraninstalled(ii).Name, compilerinformation.Fortran.Name)
			found = true;
			compilerName = compilerFortraninstalled(ii).Name;
			[folder, file, ~] = fileparts(compilerinformation(ii).Fortran.MexOpt);
			if issubdirof(folder, prefdir())
				if any(strcmpi(file, {
					'mex_Fortran_win32';
					'mex_Fortran_win64'
				}))
					compilerinformation.Fortran.MexOpt = compilerFortraninstalled.MexOpt;
				end
			end
			eval(['mex -setup:''', compilerinformation.Fortran.MexOpt, ''' Fortran']);
		elseif isempty(compilerinformation.Fortran.ShortName)
			found = true;
		end
	end
	if ~found
		error('mex:mingw:openmp', 'Can not reset C compiler to %s, because it is not installed.', compilerName);
	end
end

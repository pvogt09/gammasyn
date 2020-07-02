function [success, fileinfo] = ksopt(overwrite, nobuild)
	%KSOPT compile MEX-Function file for KSOPT
	%	Input:
	%		overwrite:	indicator, if mex file should be overwritten
	%		nobuild:	indicator, if compilation should not be done and only the file to compile should be returned
	%	Output:
	%		success:	true, if compilation was successful, else false
	%		fileinfo:	information about the compiled function
	if nargin < 1
		overwrite = true;
	end
	if nargin <= 1
		nobuild = false;
	end
	%cd D:\Programme\f2c\libf2c>
	%D:\Programme\f2c\libf2c>"C:\Program Files (x86)\Microsoft Visual Studio 11.0\VC\bin\amd64\vcvars64.bat"
	%D:\Programme\f2c\libf2c>"C:\Program Files (x86)\Microsoft Visual Studio 11.0\VC\bin\nmake" -f makefile.vc
	%mex -ID:\Programme\f2c\libf2c -I. ksopt_optimize.c ksmain.c ksopt_checkhandle.c ksinit.c ksinfo.c ksopt.c kscomp.c ksprnt.c ksxlim.c ksscal.c ksando.c ksdfp.c ksfun.c ksgrad.c kscomg.c ksoned.c kshess.c kshmul.c ksvprd.c ks.c ksunsc.c ksside.c ksqmin.c ksquad.c kscube.c ksdfun.c ksd.c ksales.c -LD:\Programme\f2c\libf2c -lvcf2c.lib
	filename = 'ksopt_optimize';
	file		= realpath(fullfile(compile.destpath(), 'lib', 'optimization', 'KSOPT', 'pyKSOPT', filename));
	makePfad	= realpath(fullfile(compile.buildpath(), filename));
	if nargout >= 2
		fileinfo = struct(...
			'm',		'',...
			'c',		[file, '.c'],...
			'mex',		[file, '.', mexext],...
			'build',	[mfilename('fullpath'), '.m']...
		);
	end
	if nobuild
		success = true;
		return;
	end
	
	f2clib_dir = realpath(fullfile(compile.destpath(), 'lib', 'optimization', 'KSOPT', 'pyKSOPT', 'libf2c'));

    config = compile.constant();
	if strcmpi(config.TargetLang, 'C++')
		makeC = true;
	else
		makeC = true;
	end
	compiler = mex.getCompilerConfigurations(config.TargetLang, 'Selected');
	if isprop(compiler, 'Version')
		compiler_version = compiler.Version;
	else
		compiler_version = '0';
	end
	if isprop(compiler, 'Location')
		compiler_path = realpath(compiler.Location);
	else
		compiler_path = '';
	end
	oldpwd = pwd;
	files = cell(0, 1);
	filenames = cell(0, 1);
	isCfile = false(0, 1);
	directory = dir(realpath(fullfile([file, '.c'], '..')));
	for ii = 1:length(directory)
		[~, n, e] = fileparts(directory(ii).name);
		if makeC
			temp = strcmpi(e, '.c');
		else
			temp = strcmpi(e, '.c') || strcmpi(e, '.cpp');
		end
		if (temp || strcmpi(e, '.h')) && ~strcmpi(e, '.bat')
			if ~strcmpi(n, filename) && ~strcmpi(n, 'closeunit') && ~strcmpi(n, 'openunit')
				if ~makeC && (~strcmpi(n, 'ksopt_optimize') && ~strcmpi(n, 'ksopt_checkhandle') && ~(strcmpi(n, 'ksopt') && strcmpi(e, '.h')) && ~(strcmpi(n, 'ksopt_mex') && strcmpi(e, '.h')))
					continue;
				end
				isCfile(end + 1, 1) = temp;
				files{end + 1, 1} = realpath(fullfile(file, '..', [n, e]));
				filenames{end + 1, 1} = [n, e];
			end
		end
	end
	try
		if overwrite
			if ~exist(makePfad, 'dir')
				mkdir(makePfad);
			end
			cd(makePfad);
			directory = dir(realpath(makePfad));
			for ii = 1:length(directory)
				if ~directory(ii).isdir
					delete(realpath(fullfile(makePfad, directory(ii).name)));
				end
			end
			if makeC
				[status, message, messageid] = copyfile([file, '.c'], fullfile(makePfad, [filename, '.c']), 'f');
			else
				[status, message, messageid] = copyfile([file, '.c'], fullfile(makePfad, [filename, 'cpp.cpp']), 'f');
			end
			if ~status
				error(messageid, message);
			end
			for ii = 1:size(files, 1)
				[status, message, messageid] = copyfile(files{ii, 1}, fullfile(makePfad, filenames{ii, 1}), 'f');
				if ~status
					error(messageid, message);
				end
				if ~makeC
					[status, message, messageid] = copyfile(files{ii, 1}, fullfile(realpath(fullfile([file, '.c'], '..', 'build', filenames{ii, 1}))), 'f');
					if ~status
						error(messageid, message);
					end
				end
			end
			if ~makeC
				directory = dir(realpath(fullfile(f2clib_dir)));
				fileslibf2c = cell(0, 1);
				filenameslibf2c = cell(0, 1);
				for ii = 1:length(directory)
					[~, n, e] = fileparts(directory(ii).name);
					if ~directory(ii).isdir
						if ~strcmpi(e, '.vc') && ~strcmpi(n, 'makefile')
							fileslibf2c{end + 1, 1} = realpath(fullfile(f2clib_dir, [n, e]));
							filenameslibf2c{end + 1, 1} = [n, e];
						else
							fileslibf2c{end + 1, 1} = realpath(fullfile(f2clib_dir, [n, 'f2c', e]));
							filenameslibf2c{end + 1, 1} = [n, e];
						end
					end
				end
				for ii = 1:size(fileslibf2c, 1)
					[status, message, messageid] = copyfile(fileslibf2c{ii, 1}, fullfile(realpath(fullfile([file, '.c'], '..', 'build', filenameslibf2c{ii, 1}))), 'f');
					if ~status
						error(messageid, message);
					end
				end
				cd(realpath(fullfile([file, '.c'], '..', 'build')));
				[status, cmdout] = system(['make.bat -vspath "', compiler_path, '" -vsver "', compiler_version, '"'], '-echo');
				if status ~= 0
					error('compile:ksopt', escape_printf(cmdout));
				end
				cd(makePfad);
				[status, message, messageid] = copyfile(realpath(fullfile([file, '.c'], '..', 'build', 'ksopt.lib')), fullfile(makePfad, ['libksopt.lib']), 'f');
				if ~status
					error(messageid, message);
				end
				[status, message, messageid] = copyfile(realpath(fullfile([file, '.c'], '..', 'build', 'vcf2c.lib')), fullfile(f2clib_dir, ['vcf2c_', computer('arch'), '.lib']), 'f');
				if ~status
					error(messageid, message);
				end
			end
			if makeC && ~exist(realpath(fullfile(f2clib_dir, ['vcf2c_', computer('arch'), '.lib'])), 'file')
				directory = dir(realpath(fullfile(f2clib_dir)));
				fileslibf2c = cell(0, 1);
				filenameslibf2c = cell(0, 1);
				for ii = 1:length(directory)
					[~, n, e] = fileparts(directory(ii).name);
					if ~directory(ii).isdir
						fileslibf2c{end + 1, 1} = realpath(fullfile(f2clib_dir, [n, e]));
						filenameslibf2c{end + 1, 1} = [n, e];
					end
				end
				for ii = 1:size(fileslibf2c, 1)
					[status, message, messageid] = copyfile(fileslibf2c{ii, 1}, fullfile(realpath(fullfile([file, '.c'], '..', 'build', filenameslibf2c{ii, 1}))), 'f');
					if ~status
						error(messageid, message);
					end
				end
				cd(realpath(fullfile([file, '.c'], '..', 'build')));
				[status, cmdout] = system(['makef2c.bat -', computer('arch'), ' -vspath "', compiler_path, '" -vsver "', compiler_version, '"'], '-echo');
				if status ~= 0
					error('compile:ksopt', escape_printf(cmdout));
				end
				cd(makePfad);
				[status, message, messageid] = copyfile(realpath(fullfile([file, '.c'], '..', 'build', 'vcf2c.lib')), fullfile(f2clib_dir, ['vcf2c_', computer('arch'), '.lib']), 'f');
				if ~status
					error(messageid, message);
				end
			end
			Args = {
				'-I.';
				['-I', f2clib_dir, ''];
				['-L', f2clib_dir];
				['-lvcf2c_', computer('arch'), '.lib'];
				%['COMPFLAGS=$COMPFLAGS /MDd']
				%['-D__cplusplus']
				['LINKFLAGS=$LINKFLAGS /NODEFAULTLIB:LIBCMT'];
				['CXXFLAGS=$CXXFLAGS /W3'];
				['CFLAGS=$CFLAGS /W3'];
				['COMPFLAGS=$COMPFLAGS /W3'];
				['-llibut']
			};
			if ~makeC
				Args = [
					Args;
					{
						['-L.'];
						['-llibksopt.lib']
					};
				];
			end
			if config.EnableDebugging
				Args{end + 1} = '-g';
			end
			if config.GenerateReport
				Args{end + 1} = '-v';
			end
			if ~config.GenCodeOnly
				if makeC
					temp = filenames(isCfile, 1);
					fname = [filename, '.c'];
				else
					temp = {'ksopt_checkhandle.c'};
					fname = [filename, 'cpp.cpp'];
				end
				exitcode = mex(Args{:}, '-largeArrayDims', fname, temp{:});
				if exitcode == 0
					[status, message, messageid] = copyfile([filename, '.', mexext], [file, '.', mexext], 'f');
					if ~status
						error(messageid, message);
					end
				else
					error('compile:mex', 'Could not compile file ''%s''.', [file, '.c']);
				end
				%delete(fullfile(makePfad, [filename, '.c']));
			end
		end
		success = true;
	catch e
		if strcmpi('', e.message)
			warning(e.identifier, 'Error in code generation.');
		else
			warning(e.identifier, escape_printf(e.message));
		end
		success = false;
	end
	cd(oldpwd);
	if nargout < 1
		clear success;
	end
end
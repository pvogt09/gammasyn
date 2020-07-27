function [success, fileinfo] = findhash(overwrite, nobuild)
	%FINDHASH compile findhash.c file for YALMIP
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
	filename = 'findhash';
	file		= realpath(fullfile(compile.destpath(), 'lib', 'YALMIP', 'extras', filename));
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

	config = compile.constant();

	oldpwd = pwd;
	try
		if overwrite
			if ~exist(makePfad, 'dir')
				mkdir(makePfad);
			end
			cd(makePfad);
			[status, message, messageid] = copyfile(fullfile([file, '.c']), fullfile(makePfad, [filename, '.c']), 'f');
			if ~status
				error(messageid, message);
			end
			Args = {};
			if config.EnableDebugging
				Args{end + 1} = '-g';
			end
			if config.GenerateReport
				Args{end + 1} = '-v';
			end
			if ~config.GenCodeOnly
				exitcode = mex(Args{:}, '-largeArrayDims', [filename, '.c']);
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
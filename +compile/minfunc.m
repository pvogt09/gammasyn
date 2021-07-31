function [success, fileinfo] = minfunc(overwrite, nobuild)
	%MTIMES3D compile minFunc
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
	file		= realpath(fullfile(compile.destpath(), 'lib', 'optimization', 'minFunc', 'minFunc', 'mex', 'lbfgsC'));
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
	currdir = pwd();
	cd(fullfile(compile.destpath(), 'lib', 'optimization', 'minFunc'));
	try
		if overwrite
			mexAll;
		end
		success = true;
	catch e
		if ~configuration.matlab.hascodertoolbox()
			warning(e.identifier, 'No license for Matlab Coder Toolbox available.');
		else
			if strcmpi('', e.message)
				warning(e.identifier, 'Error in code generation.');
			else
				warning(e.identifier, e.message);
			end
		end
		success = false;
	end
	cd(currdir);
	if nargout < 1
		clear success;
	end
end
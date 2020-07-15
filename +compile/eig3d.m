function [success, fileinfo] = eig3d(overwrite, nobuild)
	%EIG3D compile eig3d
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
	file		= realpath(fullfile(compile.destpath(), 'lib', 'private', 'eig3d_m.m'));
	makePfad	= realpath(fullfile(compile.buildpath(), 'eig3d'));
	outfile		= realpath(fullfile(compile.destpath(), 'lib', 'private', 'eig3d_mex'));
	if nargout >= 2
		fileinfo = struct(...
			'm',		file,...
			'mex',		[outfile, '.', mexext],...
			'build',	[mfilename('fullpath'), '.m']...
		);
	end
	if nobuild
		success = true;
		return;
	end
	%#ok<*NASGU> unused variables are function arguments
	TA = coder.typeof(1, [Inf, Inf, Inf], [1, 1, 1]);
	TB = coder.typeof(1, [Inf, Inf, Inf], [1, 1, 1]);
	TeigenvalueOption = coder.typeof('a', [1, Inf], [0, 1]);
	Tnumthreads = coder.typeof(uint32(1));

	config = compile.constant();

	Args = '{TA TB TeigenvalueOption Tnumthreads}';
	try
		if overwrite
			codegen(file, '-args', Args, '-d', makePfad, '-config', 'config', '-o', outfile);
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
	if nargout < 1
		clear success;
	end
end
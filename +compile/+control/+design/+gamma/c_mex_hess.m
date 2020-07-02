function [success, fileinfo] = c_mex_hess(overwrite, nobuild)
	%C_MEX_HESS compile constraint function for gamma pole placement with 6 output arguments
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
	file		= realpath(fullfile(compile.destpath(), '+control', '+design', '+gamma', 'private', 'c_mex_hess'));
	makePfad	= realpath(fullfile(compile.buildpath(), 'c_mex_hess'));
	buildfile	= mfilename('fullpath');
	if nargout >= 2
		[success, fileinfo] = c_mex_common(file, makePfad, buildfile, overwrite, nobuild);
	elseif nargout >= 1
		success = c_mex_common(file, makePfad, buildfile, overwrite, nobuild);
	else
		c_mex_common(file, makePfad, buildfile, overwrite, nobuild);
	end
end
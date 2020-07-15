function [success, fileinfo] = J_mex_grad(overwrite, nobuild)
	%J_MEX_GRAD compile objective function for gamma pole placement with 2 output arguments
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
	file		= realpath(fullfile(compile.destpath(), '+control', '+design', '+gamma', 'private', 'J_mex_grad'));
	makePfad	= realpath(fullfile(compile.buildpath(), 'J_mex_grad'));
	buildfile	= mfilename('fullpath');
	if nargout >= 2
		[success, fileinfo] = J_mex_common(file, makePfad, buildfile, overwrite, nobuild);
	elseif nargout >= 1
		success = J_mex_common(file, makePfad, buildfile, overwrite, nobuild);
	else
		J_mex_common(file, makePfad, buildfile, overwrite, nobuild);
	end
end
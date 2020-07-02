function [success, filelist] = J_mex(overwrite, nobuild)
	%J_MEX compile objective functions with different number of output arguments for gamma pole placement
	%	Input:
	%		overwrite:	indicator, if mex file should be overwritten
	%		nobuild:	indicator, if compilation should not be done and only the file to compile should be returned
	%	Output:
	%		success:	true, if compilation was successful, else false
	%		filelist:	list of functions to compile
	if nargin < 1
		overwrite = true;
	end
	if nargin <= 1
		nobuild = false;
	end
	buildfiles = {
		@compile.control.design.gamma.J_mex_fun,				[];
		@compile.control.design.gamma.J_mex_grad,				[];
		@compile.control.design.gamma.J_mex_hess,				[]
	};
	if nargout >= 2
		filelist = buildfiles;
	end
	if nobuild
		success = true;
		return;
	end
	success = false(size(buildfiles, 1), 1);
	wait = Progress(size(buildfiles, 1), 'Compiling Gamma Area Control objective functions');
	for ii = 1:size(buildfiles, 1)
		if wait.iscancelled()
			break;
		end
		build = buildfiles{ii, 1};
		success(ii) = build(overwrite);
		wait.step();
	end
	success = all(success);
	clear wait;
	if nargout < 1
		clear success;
	end
end
function [success, filelist] = all(overwrite, nobuild)
	%ALL compile all YALMIP functions
	%	Input:
	%		overwrite:	overwrite existing functions
	%		nobuild:	indicator, if compilation should be done or only file to compile should be returned
	%	Output:
	%		success:	indicator if compilation was successful
	%		filelist:	list of functions to compile
	if nargin < 1
		overwrite = true;
	end
	if nargin <= 1
		nobuild = false;
	end
	buildfiles = {
		@compile.yalmip.findhash,			[];
		@compile.yalmip.findhashsorted,		[]
	};
	if nargout >= 2
		filelist = buildfiles;
	end
	if nobuild
		success = true;
		return;
	end
	success = false(size(buildfiles, 1), 1);
	wait = Progress(size(buildfiles, 1), 'Compiling YALMIP functions');
	for ii = 1:size(buildfiles, 1)
		if wait.iscancelled()
			break;
		end
		build = buildfiles{ii, 1};
		success(ii) = build(overwrite);
		wait.step();
	end
	clear wait;
	if nargout < 1
		clear success;
	end
end
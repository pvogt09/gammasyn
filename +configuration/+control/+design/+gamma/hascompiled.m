function [has] = hascompiled()
	%HASCOMPILED return if the compiled gammasyn functions are available
	%	Output:
	%		has:		true, if compiled gammasyn functions are available, else false
	persistent hascompiledgammasyn;
	if isempty(hascompiledgammasyn)
		files = {
			realpath(fullfile(mfilename('fullpath'), '..', '..', '..', '..', '..', '+control', '+design', '+gamma', 'private', ['c_mex_fun_mex.', mexext]));
			realpath(fullfile(mfilename('fullpath'), '..', '..', '..', '..', '..', '+control', '+design', '+gamma', 'private', ['c_mex_grad_mex.', mexext]));
			realpath(fullfile(mfilename('fullpath'), '..', '..', '..', '..', '..', '+control', '+design', '+gamma', 'private', ['c_mex_hess_mex.', mexext]));
			realpath(fullfile(mfilename('fullpath'), '..', '..', '..', '..', '..', '+control', '+design', '+gamma', 'private', ['calculate_areas_fixed_mex.', mexext]));
			realpath(fullfile(mfilename('fullpath'), '..', '..', '..', '..', '..', '+control', '+design', '+gamma', 'private', ['calculate_eigenvalues_mex.', mexext]));
			realpath(fullfile(mfilename('fullpath'), '..', '..', '..', '..', '..', '+control', '+design', '+gamma', 'private', ['J_mex_fun_mex.', mexext]));
			realpath(fullfile(mfilename('fullpath'), '..', '..', '..', '..', '..', '+control', '+design', '+gamma', 'private', ['J_mex_grad_mex.', mexext]));
			realpath(fullfile(mfilename('fullpath'), '..', '..', '..', '..', '..', '+control', '+design', '+gamma', 'private', ['J_mex_hess_mex.', mexext]))
		};
		mexexist = false(size(files, 1), 1);
		parfor ii = 1:size(files, 1)
			mexexist(ii, 1) = exist(files{ii, 1}, 'file') == 3;
		end
		hascompiledgammasyn = all(mexexist);
	end
	has = hascompiledgammasyn;
end
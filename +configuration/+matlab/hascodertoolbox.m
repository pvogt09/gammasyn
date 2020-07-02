function [has] = hascodertoolbox()
	%HASCODERTOOLBOX return if the Matlab coder toolbox is available
	%	Output:
	%		has:		true, if control toolbox is installed, else false
	persistent hascoder;
	if isempty(hascoder)
		v = ver;
		[installedToolboxes{1:length(v)}] = deal(v.Name);
		hascoder = ismember('MATLAB Coder', installedToolboxes);
		hascoder = hascoder && logical(license('test', 'Matlab_Coder'));
	end
	has = hascoder;
end
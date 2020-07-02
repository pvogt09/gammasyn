function [has] = hasoptimization()
	%HASOPTIMIZATION return if the optimization toolbox is available
	%	Output:
	%		has:		true, if optimization toolbox is installed, else false
	persistent hasoptimization;
	if isempty(hasoptimization)
		v = ver;
		[installedToolboxes{1:length(v)}] = deal(v.Name);
		hasoptimization = ismember('Optimization Toolbox', installedToolboxes);
		hasoptimization = hasoptimization && logical(license('test', 'optimization_toolbox'));
	end
	has = hasoptimization;
end
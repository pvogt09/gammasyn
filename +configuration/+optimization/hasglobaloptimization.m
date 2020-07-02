function [has] = hasglobaloptimization()
	%HASGLOBALOPTIMIZATION return if the global optimization toolbox is available
	%	Output:
	%		has:		true, if global optimization toolbox is installed, else false
	persistent hasglobaloptimization;
	if isempty(hasglobaloptimization)
		v = ver;
		[installedToolboxes{1:length(v)}] = deal(v.Name);
		hasglobaloptimization = ismember('Global Optimization Toolbox', installedToolboxes);
		hasglobaloptimization = hasglobaloptimization && logical(license('test', 'GADS_Toolbox'));
	end
	has = hasglobaloptimization;
end
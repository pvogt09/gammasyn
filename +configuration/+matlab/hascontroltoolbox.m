function [has] = hascontroltoolbox()
	%HASCONTROLTOOLBOX return if the control toolbox toolbox is available
	%	Output:
	%		has:		true, if control toolbox is installed, else false
	persistent hascontrol;
	if isempty(hascontrol)
		v = ver;
		[installedToolboxes{1:length(v)}] = deal(v.Name);
		hascontrol = ismember('Control System Toolbox', installedToolboxes);
		hascontrol = hascontrol && logical(license('test', 'Control_Toolbox'));
	end
	has = hascontrol;
end
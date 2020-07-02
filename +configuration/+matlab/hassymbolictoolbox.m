function [has] = hassymbolictoolbox()
	%HASSYMBOLICTOOLBOX return if the symbolic toolbox toolbox is available
	%	Output:
	%		has:		true, if symbolic toolbox is installed, else false
	persistent hassymbolic;
	if isempty(hassymbolic)
		v = ver;
		[installedToolboxes{1:length(v)}] = deal(v.Name);
		hassymbolic = ismember('Symbolic Math Toolbox', installedToolboxes);
		hassymbolic = hassymbolic && logical(license('test', 'Symbolic_Toolbox'));
	end
	has = hassymbolic;
end
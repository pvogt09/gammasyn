function [has] = hasGeometricApproachToolbox()
	%HASGEOMETRICAPPROACHTOOLBOX return if Geometric Approach Toolbox is available
	%	Output:
	%		has:		true, if Geometric Approach toolbox is installed, else false
	persistent hasGEOM;
	if isempty(hasGEOM)
		maincofun = exist('mainco', 'file');
		ortcofun = exist('ortco', 'file');
		miincofun = exist('miinco', 'file');
		hasGEOM = maincofun && ortcofun && miincofun;
	end
	has = hasGEOM;
end
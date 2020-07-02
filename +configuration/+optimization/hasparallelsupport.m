function [has] = hasparallelsupport(asstring)
	%HASPARALLELSUPPORT zur�ckgeben, ob parallele Verarbeitung von MATLAB unterst�tzt wird
	%	Input:
	%		asstring:	true, wenn die Unterst�tzung als in fmincon verwendbarar String zur�ckgegeben werden soll, sonst false
	%	Output:
	%		has:		true, wenn parallele Berechnungen m�glich sind, sonst false
	persistent hasparallel;
	if isempty(hasparallel)
		v = ver;
		[installedToolboxes{1:length(v)}] = deal(v.Name);
		hasparallel = ismember('MATLAB Distributed Computing Server', installedToolboxes) || ismember('Parallel Computing Toolbox', installedToolboxes);
		hasparallel = hasparallel && (logical(license('test', 'MATLAB_Distrib_Comp_Engine')) || logical(license('test', 'Distrib_Computing_Toolbox')));
	end
	if nargin <= 0
		asstring = false;
	end
	if asstring
		if hasparallel
			has = 'always';
		else
			has = 'never';
		end
	else
		has = hasparallel;
	end
end
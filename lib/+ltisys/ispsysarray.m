function [is] = ispsysarray(sys)
	%ISPSYSARRAY check if a variable is an array of psys
	%	Input:
	%		sys:	system to check
	%	Output:
	%		is:		true, if the variable is a psys, else false
	if ndims(sys) == 1
		is = false;
	elseif ndims(sys) == 2 %#ok<ISMAT> compatibility with Octave
		is = ltisys.ispsys(sys);
	else
		dim = size(sys);
		systest = reshape(sys, [dim(1), dim(2), prod(dim(3:end))]);
		is = false(size(systest, 3), 1);
		for ii = 1:size(systest, 3)
			is(ii, 1) = ltisys.ispsys(squeeze(systest(:, :, ii)));
		end
		is = reshape(is, [1, 1, prod(dim(3:end))]);
	end
end
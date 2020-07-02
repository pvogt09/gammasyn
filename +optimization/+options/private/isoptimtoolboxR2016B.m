function [is] = isoptimtoolboxR2016B()
	%ISOPTIMTOOLBOXR2016B return if the optimization toolbox has a version above of 2016B
	%	Output:
	%		is:	true, if the matlab version is newer than 2016B, else false
	persistent is2016B;
	if isempty(is2016B)
		is2016B = matlab.Version.CURRENT >= matlab.Version.R2016B;
	end
	is = is2016B;
end
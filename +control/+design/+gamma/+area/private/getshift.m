function [shift] = getshift(varargin)
	%GETSHIFT superfluous function to allow for optional argument 'shift' in constructors of GammaArea derived classes as described in http://de.mathworks.com/help/coder/ug/how-working-with-matlab-classes-is-different-for-code-generation.html#btsyast-1, because if stetements are not allowed before constructor calls
	%	Input:
	%		varargin:	value for shift or nothing
	%	Output:
	%		shift:		value for shift
	if nargin > 0
		shift = varargin{1};
	else
		shift = 0;
	end
end
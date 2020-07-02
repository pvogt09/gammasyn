function [f, dfdre, dfdim, d2fdredre, d2fdimdre, d2fdredim, d2fdimdim] = Custom_border(re, im, parameter)
	%CUSTOM_BORDER return border of polearea with f < 0 for points left of the border, f = 0 for points on the border and f > 0 for points right of the border
	%	Input:
	%		re:			real part of point
	%		im:			imaginary part of the point
	%		parameter:	structure with parameters
	%	Output:
	%		f:			border function value at point [re, im]
	%		dfdre:		gradient of border function value at point [re, im] for coordinate re
	%		dfdim:		gradient of border function value at point [re, im] for coordinate im
	%		d2fdredre:	partial derivative of the gradient dfdre of border function value at [re, im] for coordinate re
	%		d2fdimdre:	partial derivative of the gradient dfdre of border function value at [re, im] for coordinate im
	%		d2fdredim:	partial derivative of the gradient dfdim of border function value at [re, im] for coordinate re
	%		d2fdimdim:	partial derivative of the gradient dfdim of border function value at [re, im] for coordinate im
	if isempty(parameter.custom_fun) || (isnumeric(parameter.custom_fun) && isnan(parameter.custom_fun)) || ~isfunctionhandle(parameter.custom_fun)
		error('control:design:gamma:area:border', 'Invalid parameter structure supplied');
	end
	if nargout >= 4 && ~parameter.custom_hashess
		error('control:design:gamma:area:border', 'Custom area border function does not supply hessian information');
	end
	if nargout >= 2 && ~parameter.custom_hasgrad
		error('control:design:gamma:area:border', 'Custom area border function does not supply gradient information');
	end
	func = parameter.custom_fun;
	if nargout >= 4
		[f, dfdre, dfdim, d2fdredre, d2fdimdre, d2fdredim, d2fdimdim] = func(re - parameter.reshift, im - parameter.imshift);
	elseif nargout >= 2
		[f, dfdre, dfdim] = func(re - parameter.reshift, im - parameter.imshift);
	else
		f = func(re - parameter.reshift, im - parameter.imshift);
	end
end
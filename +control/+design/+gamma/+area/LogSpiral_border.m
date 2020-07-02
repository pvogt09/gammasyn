function [f, dfdre, dfdim, d2fdredre, d2fdimdre, d2fdredim, d2fdimdim] = LogSpiral_border(re, im, parameter)
	%LOGSPIRAL_BORDER return border of polearea with f < 0 for points left of the border, f = 0 for points on the border and f > 0 for points right of the border
	%	Input:
	%		re:			real part of point
	%		im:			imaginary part of the point
	%		parameter:	structure with parameters
	%	Output:
	%		f:			border function value at point [re, im]
	%		dfdre:		gradient of border function value at point [re, im] for coordinate re
	%		dfdim:		gradient of border function value at point [re, im] for coordinate im
	%		d2fdredre:	partial derivative of the gradient dfdre of border function value at point [re, im] for coordinate re
	%		d2fdimdre:	partial derivative of the gradient dfdre of border function value at point [re, im] for coordinate im
	%		d2fdredim:	partial derivative of the gradient dfdim of border function value at point [re, im] for coordinate re
	%		d2fdimdre:	partial derivative of the gradient dfdim of border function value at point [re, im] for coordinate im
	if isnan(parameter.logspiral_R) || isnan(parameter.logspiral_k)
		error('control:design:gamma:area:border', 'Invalid parameter structure supplied.');
	end
	expvalue = exp(parameter.logspiral_k*2*atan2(-abs(im - parameter.imshift), re - parameter.reshift));
	f = (re - parameter.reshift)^2 + (im - parameter.imshift)^2 - parameter.logspiral_R^2*expvalue;
	if nargout >= 2
		absval = ((re - parameter.reshift)^2 + (im - parameter.imshift)^2);
		absval(absval == 0) = realmin('double');
		dfdre = 2*(re - parameter.reshift) - parameter.logspiral_R^2*expvalue*parameter.logspiral_k*2*abs(im - parameter.imshift)/absval;
		if nargout >= 3
			dfdim = 2*(im - parameter.imshift) + parameter.logspiral_R^2*expvalue*parameter.logspiral_k*2*sign(im - parameter.imshift)*(re - parameter.reshift)/absval;
			if nargout >= 4
				absval_square = absval^2;
				absval_square(absval_square == 0) = realmin('double');
				d2fdredre = 2 - parameter.logspiral_R^2*2*parameter.logspiral_k*(2*parameter.logspiral_k*expvalue*(abs(im - parameter.imshift)/absval)^2 + expvalue*abs(im - parameter.imshift)*2*(re - parameter.reshift)/absval_square);
				if nargout >= 5
					d2fdimdre = - parameter.logspiral_R^2*2*parameter.logspiral_k*(expvalue*parameter.logspiral_k*2*sign(im - parameter.imshift)*(-re + parameter.reshift)/(absval)*abs(im - parameter.imshift)/(absval) + expvalue*(sign(im - parameter.imshift)*(absval) - abs(im - parameter.imshift)*2*(im - parameter.imshift))/absval_square);
					if nargout >= 6
						d2fdredim = - parameter.logspiral_R^2*2*parameter.logspiral_k*(expvalue*parameter.logspiral_k*2*sign(im - parameter.imshift)*(-re + parameter.reshift)/(absval)*abs(im - parameter.imshift)/(absval) + expvalue*(-sign(im - parameter.imshift)*(absval) + 2*sign(im - parameter.imshift)*(re - parameter.reshift)^2)/absval_square);
						if nargout >= 7
							% TODO: the case im = imshift is NOT HANDLED, because the sign-Distribution is not continuously differentiable for x = 0
							d2fdimdim = 2 - parameter.logspiral_R^2*2*parameter.logspiral_k*(2*parameter.logspiral_k*expvalue*(sign(im - parameter.imshift)*(re - parameter.reshift)/(absval))^2 + expvalue*(sign(im - parameter.imshift)*(re - parameter.reshift)*2*(im - parameter.imshift))/(absval_square));
						end
					end
				end
			end
		end
	end
end
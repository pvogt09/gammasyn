function [f, dfdre, dfdim, d2fdredre, d2fdimdre, d2fdredim, d2fdimdim] = Circle_border(re, im, parameter)
	%CIRCLE_BORDER return border of polearea with f < 0 for points left of the border, f = 0 for points on the border and f > 0 for points right of the border
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
	if isnan(parameter.circle_R)
		error('control:design:gamma:area:border', 'Invalid parameter structure supplied');
	end
	f = realsqrt((re - parameter.reshift)^2 + (im - parameter.imshift)^2) - parameter.circle_R;
	if nargout >= 2
		sr = realsqrt((re - parameter.reshift)^2 + (im - parameter.imshift)^2);
		sr(sr == 0) = realmin('double');
		dfdre = (re - parameter.reshift)/sr;
		if nargout >= 3
			dfdim = (im - parameter.imshift)/sr;
			if nargout >= 4
				denom = (re - parameter.reshift)^2 + (im - parameter.imshift)^2;
				denom(denom == 0) = realmin('double');
				d2fdredre = (sr - (re - parameter.reshift)^2/sr)/denom;
				if nargout >= 5
					d2fdimdre = -((re - parameter.reshift)*(im - parameter.imshift)/sr)/denom;
					if nargout >= 6
						d2fdredim = -((re - parameter.reshift)*(im - parameter.imshift)/sr)/denom;
						if nargout >= 7
							d2fdimdim = (sr - (im - parameter.imshift)^2/sr)/denom;
						end
					end
				end
			end
		end
	end
end
function [f, dfdre, dfdim, d2fdredre, d2fdimdre, d2fdredim, d2fdimdim] = Ellipse_border(re, im, parameter)
	%ELLIPSE_BORDER return border of polearea with f < 0 for points left of the border, f = 0 for points on the border and f > 0 for points right of the border
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
	if isnan(parameter.ellipse_a) || isnan(parameter.ellipse_b)
		error('control:design:gamma:area:border', 'Invalid parameter structure supplied');
	end
	f = realsqrt((re - parameter.reshift)^2/parameter.ellipse_a^2 + (im - parameter.imshift)^2/parameter.ellipse_b^2) - 1;
	if nargout >= 2
		k = 1/parameter.ellipse_a^2;
		m = 1/parameter.ellipse_b^2;
		sr = realsqrt(k*(re - parameter.reshift)^2 + m*(im - parameter.imshift)^2);
		sr(sr == 0) = realmin('double');
		dfdre = (re - parameter.reshift)/parameter.ellipse_a^2/sr;
		if nargout >= 3
			dfdim = (im - parameter.imshift)/parameter.ellipse_b^2/sr;
			if nargout >= 4
				denom = k*(re - parameter.reshift)^2 + m*(im - parameter.imshift)^2;
				denom(denom == 0) = realmin('double');
				d2fdredre = (k*sr - k^2*(re - parameter.reshift)^2/sr)/denom;
				if nargout >= 5
					d2fdimdre = -(k*m*(re - parameter.reshift)*(im - parameter.imshift)/sr)/denom;
					if nargout >= 6
						d2fdredim = -(k*m*(re - parameter.reshift)*(im - parameter.imshift)/sr)/denom;
						if nargout >= 7
							d2fdimdim = (m*sr - m^2*(im - parameter.imshift)^2/sr)/denom;
						end
					end
				end
			end
		end
	end
end
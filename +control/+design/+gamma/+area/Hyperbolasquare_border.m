function [f, dfdre, dfdim, d2fdredre, d2fdimdre, d2fdredim, d2fdimdim] = Hyperbolasquare_border(re, im, parameter)
	%HYPERBOLASQUARE_BORDER return border of polearea with f < 0 for points left of the border, f = 0 for points on the border and f > 0 for points right of the border
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
	if isnan(parameter.hyperbola_a) || isnan(parameter.hyperbola_b)
		error('control:design:gamma:area:border', 'Invalid parameter structure supplied');
	end
	f = -(re - parameter.reshift)^2/parameter.hyperbola_a^2 + (im - parameter.imshift)^2/parameter.hyperbola_b^2 + 1;
	if nargout >= 2
		dfdre = -2*(re - parameter.reshift)/parameter.hyperbola_a^2;
		if nargout >= 3
			dfdim = 2*(im - parameter.imshift)/parameter.hyperbola_b^2;
			if nargout >= 4
				d2fdredre = -2/parameter.hyperbola_a^2;
				if nargout >= 5
					d2fdimdre = 0;
					if nargout >= 6
						d2fdredim = 0;
						if nargout >= 7
							d2fdimdim = 2/parameter.hyperbola_b^2;
						end
					end
				end
			end
		end
	end
end
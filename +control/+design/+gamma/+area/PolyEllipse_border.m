function [f, dfdre, dfdim, d2fdredre, d2fdimdre, d2fdredim, d2fdimdim] = PolyEllipse_border(re, im, parameter)
	%POLYELLIPSE_BORDER return border of polearea with f < 0 for points left of the border, f = 0 for points on the border and f > 0 for points right of the border
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
	if any(isnan(parameter.polyellipse_f)) || any(isnan(parameter.polyellipse_w)) || size(parameter.polyellipse_f, 2) ~= size(parameter.polyellipse_w, 2)
		error('control:design:gamma:area:border', 'Invalid parameter structure supplied');
	end
	sqrtvalue = realsqrt((re - parameter.reshift - real(parameter.polyellipse_f)).^2 + (im - parameter.imshift - imag(parameter.polyellipse_f)).^2);
	sqrtvalue(sqrtvalue == 0) = realmin('double');
	sumvalue = sum(sqrtvalue.*parameter.polyellipse_w);
	f = sumvalue - 1;
	if nargout >= 2
		dfdre = sum(((re - parameter.reshift - real(parameter.polyellipse_f)).*parameter.polyellipse_w)./sqrtvalue);
		if nargout >= 3
			dfdim = sum(((im - parameter.imshift - imag(parameter.polyellipse_f)).*parameter.polyellipse_w)./sqrtvalue);
			if nargout >= 4
				d2fdredre = sum((((im - parameter.imshift - imag(parameter.polyellipse_f)).^2).*parameter.polyellipse_w)./(sqrtvalue).*((re - parameter.reshift - real(parameter.polyellipse_f)).^2 + (im - parameter.imshift - imag(parameter.polyellipse_f)).^2));
				if nargout >= 5
					d2fdimdre = sum((((re - parameter.reshift - real(parameter.polyellipse_f)).^2).*parameter.polyellipse_w)./(sqrtvalue).*((re - parameter.reshift - real(parameter.polyellipse_f)).^2 + (im - parameter.imshift - imag(parameter.polyellipse_f)).^2));
					if nargout >= 6
						d2fdredim = - sum((((im - parameter.imshift - imag(parameter.polyellipse_f)).*(re - parameter.reshift - real(parameter.polyellipse_f))).*parameter.polyellipse_w)./(sqrtvalue).*((re - parameter.reshift - real(parameter.polyellipse_f)).^2 + (im - parameter.imshift - imag(parameter.polyellipse_f)).^2));
						if nargout >= 7
							d2fdimdim = - sum((((im - parameter.imshift - imag(parameter.polyellipse_f)).*(re - parameter.reshift - real(parameter.polyellipse_f))).*parameter.polyellipse_w)./(sqrtvalue).*((re - parameter.reshift - real(parameter.polyellipse_f)).^2 + (im - parameter.imshift - imag(parameter.polyellipse_f)).^2));
						end
					end
				end
			end
		end
	end
end
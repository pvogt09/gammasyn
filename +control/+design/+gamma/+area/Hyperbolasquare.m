classdef Hyperbolasquare < control.design.gamma.area.GammaArea
	%HYPERBOLASQUARE class for representation of a hyperbolic pole area in square form re^2/a^2 - im^2/b^2 - 1 = 0

	properties
		% semi major axis of hyperbola
		a;
		% semi minor axis of hyperbola
		b
	end

	methods(Static=true)
		function [f, dfdre, dfdim, d2fdredre, d2fdimdre, d2fdredim, d2fdimdim] = border(re, im, parameter)
			%BORDER return border of polearea with f < 0 for points left of the border, f = 0 for points on the border and f > 0 for points right of the border
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
			if nargout >= 7
				[f, dfdre, dfdim, d2fdredre, d2fdimdre, d2fdredim, d2fdimdim] = control.design.gamma.area.Hyperbolasquare_border(re, im, parameter);
			elseif nargout >= 6
				[f, dfdre, dfdim, d2fdredre, d2fdimdre, d2fdredim] = control.design.gamma.area.Hyperbolasquare_border(re, im, parameter);
			elseif nargout >= 5
				[f, dfdre, dfdim, d2fdredre, d2fdimdre] = control.design.gamma.area.Hyperbolasquare_border(re, im, parameter);
			elseif nargout >= 4
				[f, dfdre, dfdim, d2fdredre] = control.design.gamma.area.Hyperbolasquare_border(re, im, parameter);
			elseif nargout >= 3
				[f, dfdre, dfdim] = control.design.gamma.area.Hyperbolasquare_border(re, im, parameter);
			elseif nargout >= 2
				[f, dfdre] = control.design.gamma.area.Hyperbolasquare_border(re, im, parameter);
			else
				f = control.design.gamma.area.Hyperbolasquare_border(re, im, parameter);
			end
		end

		function [dfdre, dfdim] = gradborder(re, im, parameter)
			%GRADBORDER return gradient of border of polearea with f < 0 for points left of the border, f = 0 for points on the border and f > 0 for points right of the border
			%	Input:
			%		re:			real part of point
			%		im:			imaginary part of the point
			%		parameter:	structure with parameters
			%	Output:
			%		dfdre:		gradient of border function value at point [re, im] for coordinate re
			%		dfdim:		gradient of border function value at point [re, im] for coordinate im
			[~, dfdre, dfdim] = control.design.gamma.area.Hyperbolasquare.border(re, im, parameter);
		end

		function [d2fdredre, d2fdimdre, d2fdredim, d2fdimdim] = hessborder(re, im, parameter)
			%HESSBORDER return hessian of border of polearea with f < 0 for points left of the border, f = 0 for points on the border and f > 0 for points right of the border
			%	Input:
			%		re:			real part of point
			%		im:			imaginary part of the point
			%		parameter:	structure with parameters
			%	Output:
			%		d2fdredre:	partial derivative of the gradient dfdre of border function value at [re, im] for coordinate re
			%		d2fdimdre:	partial derivative of the gradient dfdre of border function value at [re, im] for coordinate im
			%		d2fdredim:	partial derivative of the gradient dfdim of border function value at [re, im] for coordinate re
			%		d2fdimdim:	partial derivative of the gradient dfdim of border function value at [re, im] for coordinate im
			[~, ~, ~, d2fdredre, d2fdimdre, d2fdredim, d2fdimdim] = control.design.gamma.area.Hyperbolasquare.border(re, im, parameter);
		end
	end

	methods(Access=protected)
		function [parameters] = getparameters(this)
			%GETPARAMETERS return structure with parameters of current object unknown to the superclass
			%	Input:
			%		this:		instance
			%	Output:
			%		parameter:	structure with parameters
			parameters = struct(...
				'a',	this.a,...
				'b',	this.b...
			);
		end
	end

	methods
		function [this] = Hyperbolasquare(a, b, varargin)
			%HYPERBOLASQUARE return new hyperboloidal pole area with specified semi axes and shift in square form
			%	Input:
			%		a:		semi major axis of hyperbola
			%		b:		semi minor axis of hyperbola
			%		shift:	shift of hyperbola
			%	Output:
			%		this:	instance
			shift = getshift(varargin{:});
			this@control.design.gamma.area.GammaArea(GammaArea.HYPERBOLASQUARE, shift);
			this.a = a;
			this.b = b;
		end

		function [this] = set.a(this, a)
			%A setter for semi major axis of hyperbola
			%	Input:
			%		this:	instance
			%		a:		semi major axis of hyperbola
			%	Output:
			%		this:	instance
			if ~isscalar(a) || ~isnumeric(a) || a < 0 || ~isreal(a)
				error('control:design:gamma:area', 'Semi major axis must be a real positive scalar.');
			end
			this.a = a;
		end

		function [this] = set.b(this, b)
			%B setter for semi minor axis of hyperbola
			%	Input:
			%		this:	instance
			%		b:		semi minor axis of hyperbola
			%	Output:
			%		this:	instance
			if ~isscalar(b) || ~isnumeric(b) || b < 0 || ~isreal(b)
				error('control:design:gamma:area', 'Semi minor axis must be a real positive scalar.');
			end
			this.b = b;
		end

		function [border] = plotinstanceborder(this, limit, numpoints)
			%PLOTINSTANCEBORDER plot border of area function
			%	Input:
			%		this:		instance
			%		limit:		maximum and minimum values for plotting
			%		numpoints:	number of points to plot
			%	Output:
			%		border:		border points
			if nargin <= 2
				numpoints = this.NUMBEROFPOINTS;
			end
			if nargin <= 1
				limit = 1 + 1i;
			end
			xmax = max(real(limit(:)));
			xmin = min(real(limit(:)));
			ymax = max(imag(limit(:)));
			ymin = min(imag(limit(:)));
			R = max([
				abs(xmax);
				abs(xmin);
				abs(ymax);
				abs(ymin);
				sqrt(xmax^2 + ymax^2);
				sqrt(xmin^2 + ymin^2);
				sqrt(xmax^2 + ymin^2);
				sqrt(xmin^2 + ymax^2);
			]);
			phi = atan(this.b/this.a);
			p = linspace(-sin(phi)*R, sin(phi)*R, numpoints);
			border = [
				this.a/this.b*sqrt(p.^2 + this.b^2) + this.reshift + 1i*p + 1i*this.imshift;
				-this.a/this.b*sqrt(p.^2 + this.b^2) + this.reshift + 1i*p + 1i*this.imshift
			];
			if nargout <= 0
				plot(real(border).', imag(border).');
			end
		end
	end

	methods(Access=protected)
		function [f] = getinstanceborder(this, re, im)
			%GETINSTANCEBORDER return border of polearea with f < 0 for points left of the border, f = 0 for points on the border and f > 0 for points right of the border
			%	Input:
			%		this:		instance
			%		re:			real part of point
			%		im:			imaginary part of the point
			%	Output:
			%		f:			border function value at point [re, im]
			parameter = this.PARAMETERPROTOTYPE;
			parameter.reshift = this.reshift;
			parameter.imshift = this.imshift;
			parameter.hyperbola_a = this.a;
			parameter.hyperbola_b = this.b;
			f = this.border(re, im, parameter);
		end

		function [dfdre, dfdim] = getinstancegradborder(this, re, im)
			%GETINSTANCEGRADBORDER return gradient of border of polearea with f < 0 for points left of the border, f = 0 for points on the border and f > 0 for points right of the border
			%	Input:
			%		this:		instance
			%		re:			real part of point
			%		im:			imaginary part of the point
			%	Output:
			%		dfdre:		gradient of border function value at point [re, im] for coordinate re
			%		dfdim:		gradient of border function value at point [re, im] for coordinate im
			parameter = this.PARAMETERPROTOTYPE;
			parameter.reshift = this.reshift;
			parameter.imshift = this.imshift;
			parameter.hyperbola_a = this.a;
			parameter.hyperbola_b = this.b;
			[dfdre, dfdim] = this.gradborder(re, im, parameter);
		end

		function [d2fdredre, d2fdimdre, d2fdredim, d2fdimdim] = getinstancehessborder(this, re, im)
			%GETINSTANCEHESSBORDER return hessian of border of polearea with f < 0 for points left of the border, f = 0 for points on the border and f > 0 for points right of the border
			%	Input:
			%		this:		instance
			%		re:			real part of point
			%		im:			imaginary part of the point
			%	Output:
			%		d2fdredre:	partial derivative of the gradient dfdre of border function value at [re, im] for coordinate re
			%		d2fdimdre:	partial derivative of the gradient dfdre of border function value at [re, im] for coordinate im
			%		d2fdredim:	partial derivative of the gradient dfdim of border function value at [re, im] for coordinate re
			%		d2fdimdim:	partial derivative of the gradient dfdim of border function value at [re, im] for coordinate im
			parameter = this.PARAMETERPROTOTYPE;
			parameter.reshift = this.reshift;
			parameter.imshift = this.imshift;
			parameter.hyperbola_a = this.a;
			parameter.hyperbola_b = this.b;
			[d2fdredre, d2fdimdre, d2fdredim, d2fdimdim] = this.hessborder(re, im, parameter);
		end

		function [str] = getinstancestring(this, format)
			%GETINSTANCESTRING return string representation of object
			%	Input:
			%		this:		instance
			%		format:		format to use for printing numbers
			%	Output:
			%		str:		string representation of the instance
			if nargin <= 1
				format = this.NUMBERFORMAT;
			end
			str = ['Hyperbolasquare(', sprintf(format, this.a), ', ', sprintf(format, this.b), this.printshift(this.reshift, this.imshift, format, true), ')'];
		end

		function [L, M, success] = toinstanceLMIregion(this)
			%TOINSTANCELMIREGION convert area to LMI region that can be used by Matlab functions for object (only left part of hyperbola is used, the right part can be obtained by using -L and -M)
			%	Input:
			%		this:		instance
			%	Output:
			%		L:			L = L^T real matrix of constant coefficients in the LMI, where the L(1, 1) element contains a imaginary part indicating the dimension of the LMI variable
			%		M:			real matrix of linear coefficients in the LMI
			%		success:	indicator, if conversion to LMI was successful
			if this.imshift ~= 0
				L = [];
				M = [];
				success = false;
			else
				L = -[
					this.reshift + this.a + 2i,	0;
					0								this.reshift - this.a
				];
				M = -[
					-1/2,					-this.a/this.b/2;
					this.a/this.b/2,		-1/2
				];
				success = true;
			end
		end
	end
end
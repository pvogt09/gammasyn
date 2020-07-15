classdef Circlesquare < control.design.gamma.area.GammaArea
	%CIRCLESQUARE class for representation of a circular pole area re^2 + im^2 - R^2 = 0

	properties
		% radius of circle
		R
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
				[f, dfdre, dfdim, d2fdredre, d2fdimdre, d2fdredim, d2fdimdim] = control.design.gamma.area.Circlesquare_border(re, im, parameter);
			elseif nargout >= 6
				[f, dfdre, dfdim, d2fdredre, d2fdimdre, d2fdredim] = control.design.gamma.area.Circlesquare_border(re, im, parameter);
			elseif nargout >= 5
				[f, dfdre, dfdim, d2fdredre, d2fdimdre] = control.design.gamma.area.Circlesquare_border(re, im, parameter);
			elseif nargout >= 4
				[f, dfdre, dfdim, d2fdredre] = control.design.gamma.area.Circlesquare_border(re, im, parameter);
			elseif nargout >= 3
				[f, dfdre, dfdim] = control.design.gamma.area.Circlesquare_border(re, im, parameter);
			elseif nargout >= 2
				[f, dfdre] = control.design.gamma.area.Circlesquare_border(re, im, parameter);
			else
				f = control.design.gamma.area.Circlesquare_border(re, im, parameter);
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
			[~, dfdre, dfdim] = control.design.gamma.area.Circlesquare.border(re, im, parameter);
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
			[~, ~, ~, d2fdredre, d2fdimdre, d2fdredim, d2fdimdim] = control.design.gamma.area.Circlesquare.border(re, im, parameter);
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
				'R',	this.R...
			);
		end
	end

	methods
		function [this] = Circlesquare(R, varargin)
			%CIRCLESQUARE return new circle pole area with specified radius and shift
			%	Input:
			%		R:		radius of circle
			%		shift:	shift of circle center
			%	Output:
			%		this:	instance
			shift = getshift(varargin{:});
			this@control.design.gamma.area.GammaArea(GammaArea.CIRCLESQUARE, shift);
			this.R = R;
		end

		function [this] = set.R(this, R)
			%R setter for radius
			%	Input:
			%		this:	instance
			%		R:		radius to set
			%	Output:
			%		this:	instance
			if ~isscalar(R) || ~isnumeric(R) || R < 0 || ~isreal(R)
				error('control:design:gamma:area', 'Radius must be a real positive scalar.');
			end
			this.R = R;
		end

		function [border] = plotinstanceborder(this, ~, numpoints)
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
			p = linspace(0, 2*pi, numpoints);
			border = this.R*(cos(p) + 1i*sin(p)) + this.reshift + 1i*this.imshift;
			if nargout <= 0
				plot(real(border), imag(border));
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
			parameter.circle_R = this.R;
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
			parameter.circle_R = this.R;
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
			parameter.circle_R = this.R;
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
			str = ['Circlesquare(', sprintf(format, this.R), this.printshift(this.reshift, this.imshift, format, true), ')'];
		end

		function [L, M, success] = toinstanceLMIregion(this)
			%TOINSTANCELMIREGION convert area to LMI region that can be used by Matlab functions for object
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
				L = [
					-this.R + 2i,		-this.reshift;
					-this.reshift,		-this.R
				];
				M = [
					0,	1;
					0,	0
				];
				success = true;
			end
		end
	end
end
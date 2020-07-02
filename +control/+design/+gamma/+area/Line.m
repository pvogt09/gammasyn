classdef Line < control.design.gamma.area.GammaArea
	%LINE class for representation of a half plane pole area a*re + b - im = 0
	
	properties
		% slope of line
		a;
		% shift of line
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
				[f, dfdre, dfdim, d2fdredre, d2fdimdre, d2fdredim, d2fdimdim] = control.design.gamma.area.Line_border(re, im, parameter);
			elseif nargout >= 6
				[f, dfdre, dfdim, d2fdredre, d2fdimdre, d2fdredim] = control.design.gamma.area.Line_border(re, im, parameter);
			elseif nargout >= 5
				[f, dfdre, dfdim, d2fdredre, d2fdimdre] = control.design.gamma.area.Line_border(re, im, parameter);
			elseif nargout >= 4
				[f, dfdre, dfdim, d2fdredre] = control.design.gamma.area.Line_border(re, im, parameter);
			elseif nargout >= 3
				[f, dfdre, dfdim] = control.design.gamma.area.Line_border(re, im, parameter);
			elseif nargout >= 2
				[f, dfdre] = control.design.gamma.area.Line_border(re, im, parameter);
			else
				f = control.design.gamma.area.Line_border(re, im, parameter);
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
			[~, dfdre, dfdim] = control.design.gamma.area.Line.border(re, im, parameter);
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
			[~, ~, ~, d2fdredre, d2fdimdre, d2fdredim, d2fdimdim] = control.design.gamma.area.Line.border(re, im, parameter);
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
		function [this] = Line(a, b, varargin)
			%LINE return new half plane pole area with specified slope and shift
			%	Input:
			%		a:		slope of line
			%		b:		shift of line
			%		shift:	shift of line points
			%	Output:
			%		this:	instance
			shift = getshift(varargin{:});
			this@control.design.gamma.area.GammaArea(GammaArea.LINE, shift);
			this.a = a;
			this.b = b;
		end
		
		function [this] = set.a(this, a)
			%A setter for slope of line
			%	Input:
			%		this:	instance
			%		a:		slope of line
			%	Output:
			%		this:	instance
			if ~isscalar(a) || ~isnumeric(a) || ~isreal(a)
				error('control:design:gamma:area', 'Slope must be a real scalar.');
			end
			this.a = a;
		end
		
		function [this] = set.b(this, b)
			%B setter for shift of line
			%	Input:
			%		this:	instance
			%		b:		shift of line
			%	Output:
			%		this:	instance
			if ~isscalar(b) || ~isnumeric(b) || ~isreal(b)
				error('control:design:gamma:area', 'Shift must be a real scalar.');
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
				limit = [
					1 + 1i;
					-1 - 1i
				];
			end
			xmax = max(real(limit(:)));
			xmin = min(real(limit(:)));
			p = linspace(xmin, xmax, numpoints);
			if this.a == 0
				border = p + 1i*this.b*ones(1, numpoints);
			else
				border = p + 1i*(this.a*(p - this.reshift) + this.b + this.imshift);
			end
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
			parameter.line_a = this.a;
			parameter.line_b = this.b;
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
			parameter.line_a = this.a;
			parameter.line_b = this.b;
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
			parameter.line_a = this.a;
			parameter.line_b = this.b;
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
			str = ['Line(', sprintf(format, this.a), ', ', sprintf(format, this.b), this.printshift(this.reshift, this.imshift, format, true), ')'];
		end
		
		function [L, M, success] = toinstanceLMIregion(this)
			%TOINSTANCELMIREGION convert area to LMI region that can be used by Matlab functions for object (only left part of sector is used and symmetry to real axis enforced)
			%	the right part of the sector from the zero up to infinity can be obtained by using -L and -M
			%	Input:
			%		this:		instance
			%	Output:
			%		L:			L = L^T real matrix of constant coefficients in the LMI, where the L(1, 1) element contains a imaginary part indicating the dimension of the LMI variable
			%		M:			real matrix of linear coefficients in the LMI
			%		success:	indicator, if conversion to LMI was successful
			L = [
				this.b - this.a*this.reshift + this.imshift + 2i,	0;
				0,													this.b - this.a*this.reshift + this.imshift
			];
			M = [
				this.a/2,	-1/2;
				1/2,		this.a/2
			];
			success = true;
		end
	end
end
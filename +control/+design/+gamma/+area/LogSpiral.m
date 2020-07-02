classdef LogSpiral < control.design.gamma.area.GammaArea
	%LOGSPIRAL class for representation of a logarithmic spiral pole area re^2 + im^2 - R^2exp(k*arctan(y/x)) = 0
	
	properties
		% radius of logarithmic spiral
		R;
		% slope of logarithmic spiral
		k
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
				[f, dfdre, dfdim, d2fdredre, d2fdimdre, d2fdredim, d2fdimdim] = control.design.gamma.area.LogSpiral_border(re, im, parameter);
			elseif nargout >= 6
				[f, dfdre, dfdim, d2fdredre, d2fdimdre, d2fdredim] = control.design.gamma.area.LogSpiral_border(re, im, parameter);
			elseif nargout >= 5
				[f, dfdre, dfdim, d2fdredre, d2fdimdre] = control.design.gamma.area.LogSpiral_border(re, im, parameter);
			elseif nargout >= 4
				[f, dfdre, dfdim, d2fdredre] = control.design.gamma.area.LogSpiral_border(re, im, parameter);
			elseif nargout >= 3
				[f, dfdre, dfdim] = control.design.gamma.area.LogSpiral_border(re, im, parameter);
			elseif nargout >= 2
				[f, dfdre] = control.design.gamma.area.LogSpiral_border(re, im, parameter);
			else
				f = control.design.gamma.area.LogSpiral_border(re, im, parameter);
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
			[~, dfdre, dfdim] = control.design.gamma.area.LogSpiral.border(re, im, parameter);
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
			[~, ~, ~, d2fdredre, d2fdimdre, d2fdredim, d2fdimdim] = control.design.gamma.area.LogSpiral.border(re, im, parameter);
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
				'R',	this.R,...
				'k',	this.k...
			);
		end
	end
	
	methods
		function [this] = LogSpiral(R, k, varargin)
			%LOGSPIRAL return new logarithmic spiral pole area with specified radius, slope and shift
			%	Input:
			%		R:		radius of logarithmic spiral
			%		k:		slope of logarithmic spiral
			%		shift:	shift of circle center
			%	Output:
			%		this:	instance
			shift = getshift(varargin{:});
			this@control.design.gamma.area.GammaArea(GammaArea.LOGSPIRAL, shift);
			this.R = R;
			this.k = k;
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
		
		function [this] = set.k(this, k)
			%K setter for slope
			%	Input:
			%		this:	instance
			%		k:		slope to set
			%	Output:
			%		this:	instance
			if ~isscalar(k) || ~isnumeric(k) || k < 0 || ~isreal(k)
				error('control:design:gamma:area', 'Slope must be a real positive scalar.');
			end
			this.k = k;
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
			phi = linspace(0, -pi, numpoints);
			border = [
				this.R*exp(this.k.*phi).*cos(phi) + 1i*this.R*exp(this.k.*phi).*sin(phi);
				this.R*exp(this.k.*phi).*cos(phi) - 1i*this.R*exp(this.k.*phi).*sin(phi)
			] + this.reshift + 1i*this.imshift;
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
			parameter.logspiral_R = this.R;
			parameter.logspiral_k = this.k;
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
			parameter.logspiral_R = this.R;
			parameter.logspiral_k = this.k;
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
			parameter.logspiral_R = this.R;
			parameter.logspiral_k = this.k;
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
			str = ['Logspiral(', sprintf(format, this.R), ', ', sprintf(format, this.k), this.printshift(this.reshift, this.imshift, format, true), ')'];
		end
		
		function [L, M, success] = toinstanceLMIregion(~)
			%TOINSTANCELMIREGION convert area to LMI region that can be used by Matlab functions for object
			%	Input:
			%		this:		instance
			%	Output:
			%		L:			L = L^T real matrix of constant coefficients in the LMI, where the L(1, n) element contains a imaginary part indicating the dimension of the LMI variable
			%		M:			real matrix of linear coefficients in the LMI
			%		success:	indicator, if conversion to LMI was successful
			L = [];
			M = [];
			success = false;
		end
	end
end
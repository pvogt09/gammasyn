classdef PolyEllipse < control.design.gamma.area.GammaArea
	%POLYELLIPSE class for representation of a polyellipsoidal pole area w_1 sqrt((re - re_1)^2 + w_1(im - im_1)^2) + ... + w_n sqrt((re - re_n)^2 + w_n(im - im_n)^2) - 1 = 0

	properties(Dependent=true)
		% weights of the focal points
		w;
		% focal points of the polyellipse
		f
	end

	properties(Access=protected)
		% weights of the focal points used in order to check compatibility with focal point size
		weights;
		% focal points of the polyellipse used in order to check compatibility with weight size
		focals
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
				[f, dfdre, dfdim, d2fdredre, d2fdimdre, d2fdredim, d2fdimdim] = control.design.gamma.area.PolyEllipse_border(re, im, parameter);
			elseif nargout >= 6
				[f, dfdre, dfdim, d2fdredre, d2fdimdre, d2fdredim] = control.design.gamma.area.PolyEllipse_border(re, im, parameter);
			elseif nargout >= 5
				[f, dfdre, dfdim, d2fdredre, d2fdimdre] = control.design.gamma.area.PolyEllipse_border(re, im, parameter);
			elseif nargout >= 4
				[f, dfdre, dfdim, d2fdredre] = control.design.gamma.area.PolyEllipse_border(re, im, parameter);
			elseif nargout >= 3
				[f, dfdre, dfdim] = control.design.gamma.area.PolyEllipse_border(re, im, parameter);
			elseif nargout >= 2
				[f, dfdre] = control.design.gamma.area.PolyEllipse_border(re, im, parameter);
			else
				f = control.design.gamma.area.PolyEllipse_border(re, im, parameter);
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
			[~, dfdre, dfdim] = control.design.gamma.area.PolyEllipse.border(re, im, parameter);
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
			[~, ~, ~, d2fdredre, d2fdimdre, d2fdredim, d2fdimdim] = control.design.gamma.area.PolyEllipse.border(re, im, parameter);
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
				'w',	this.w,...
				'f',	this.f...
			);
		end
	end

	methods
		function [this] = PolyEllipse(f, w, varargin)
			%POLYELLIPSE return new polyellipse pole area with specified weights and focal points and shift
			%	Input:
			%		f:		focal points
			%		w:		weights of focal points
			%		shift:	shift of ellipse center
			%	Output:
			%		this:	instance
			shift = getshift(varargin{:});
			this@control.design.gamma.area.GammaArea(GammaArea.POLYELLIPSE, shift);
			this = this.setFocals(f, w);
		end

		function [w] = get.w(this)
			%W getter for focal points
			%	Input:
			%		this:	instance
			%	Output:
			%		w:		weights
			w = this.weights;
		end

		function [f] = get.f(this)
			%F getter for focal points
			%	Input:
			%		this:	instance
			%	Output:
			%		f:		focal points
			f = this.focals;
		end

		function [this] = setFocals(this, f, w)
			%SETFOCALS setter for focal points and weights
			%	Input:
			%		this:	instance
			%		f:		list of focal points
			%		w:		list of weights
			%	Output:
			%		this:	instance
			if size(f, 1) ~= 1 || ~isnumeric(f)
				error('control:design:gamma:area', 'Focal points must be numeric values.');
			end
			if size(w, 1) ~= 1 || ~isnumeric(w) || any(w(:) < 0) || ~isreal(w)
				error('control:design:gamma:area', 'Weights must be a real positive values.');
			end
			while(~isempty(w) && isnan(w(end)))
				w = w(1:end - 1);
			end
			while(~isempty(f) && isnan(f(end)))
				f = f(1:end - 1);
			end
			if any(isnan(f)) || any(isnan(w))
				error('control:design:gamma:area', 'Focal points and weights must not be NaN.');
			end
			if (~isempty(this.weights) && size(w, 2) ~= size(this.weights, 2)) && (~isempty(this.focals) && size(f, 2) ~= size(this.focals, 2))
				error('control:design:gamma:area', 'Focal points and weights must have size 1X%d.', size(this.weights, 2));
			else
				this.focals = f;
				this.weights = w;
			end
		end

		function [this] = set.f(this, f)
			%F setter for focal points
			%	Input:
			%		this:	instance
			%		f:		focal points to set
			%	Output:
			%		this:	instance
			if size(f, 1) ~= 1 || ~isnumeric(f)
				error('control:design:gamma:area', 'Focal points must be numeric values.');
			end
			while(~isempty(f) && isnan(f(end)))
				f = f(1:end - 1);
			end
			if size(f, 2) ~= size(this.focals, 2)
				error('control:design:gamma:area', 'Focal points must have size 1X%d.', size(this.focals, 2));
			end
			if any(isnan(f))
				error('control:design:gamma:area', 'Focal points must not be NaN.');
			end
			this.focals = f;
		end

		function [this] = set.w(this, w)
			%W setter for weights
			%	Input:
			%		this:	instance
			%		w:		weights to set
			%	Output:
			%		this:	instance
			if size(w, 1) ~= 1 || ~isnumeric(w) || any(w(:) < 0) || ~isreal(w)
				error('control:design:gamma:area', 'Weights must be a real positive values.');
			end
			while(~isempty(w) && isnan(w(end)))
				w = w(1:end - 1);
			end
			if size(w, 2) ~= size(this.weights, 2)
				error('control:design:gamma:area', 'Weights must have size 1X%d.', size(this.weights, 2));
			end
			if any(isnan(w))
				error('control:design:gamma:area', 'Weights must not be NaN.');
			end
			this.weights = w;
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
			border = this.a*cos(p) - this.reshift + this.b*1i*sin(p) - 1i*this.imshift;
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
			parameter.polyellipse_f = this.f;
			parameter.polyellipse_w = this.w;
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
			parameter.polyellipse_f = this.f;
			parameter.polyellipse_w = this.w;
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
			parameter.polyellipse_f = this.f;
			parameter.polyellipse_w = this.w;
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
			fformated = arrayfun(@(x) sprintf(format, x), this.f, 'UniformOutput', false);
			wformated = arrayfun(@(x) sprintf(format, x), this.w, 'UniformOutput', false);
			str = ['PolyEllipse([', strjoin(fformated, ', '), '], [', strjoin(wformated, ', '), ']', this.printshift(this.reshift, this.imshift, format, true), ')'];
		end

		function [L, M, success] = toinstanceLMIregion(this)
			%TOINSTANCELMIREGION convert area to LMI region that can be used by Matlab functions for object
			%	Input:
			%		this:		instance
			%	Output:
			%		L:			L = L^T real matrix of constant coefficients in the LMI, where the L(1, n) element contains a imaginary part indicating the dimension of the LMI variable
			%		M:			real matrix of linear coefficients in the LMI
			%		success:	indicator, if conversion to LMI was successful
			if this.imshift ~= 0
				L = [];
				M = [];
				success = false;
			else
				% TODO: to be implemented
				L = [];
				M = [];
				success = false;
			end
		end
	end
end
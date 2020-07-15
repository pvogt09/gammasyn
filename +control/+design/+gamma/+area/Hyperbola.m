classdef Hyperbola < control.design.gamma.area.GammaArea
	%HYPERBOLA class for representation of a hyperbolic pole area re + a/b*sqrt(im^2 + b^2) = 0

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
				[f, dfdre, dfdim, d2fdredre, d2fdimdre, d2fdredim, d2fdimdim] = control.design.gamma.area.Hyperbola_border(re, im, parameter);
			elseif nargout >= 6
				[f, dfdre, dfdim, d2fdredre, d2fdimdre, d2fdredim] = control.design.gamma.area.Hyperbola_border(re, im, parameter);
			elseif nargout >= 5
				[f, dfdre, dfdim, d2fdredre, d2fdimdre] = control.design.gamma.area.Hyperbola_border(re, im, parameter);
			elseif nargout >= 4
				[f, dfdre, dfdim, d2fdredre] = control.design.gamma.area.Hyperbola_border(re, im, parameter);
			elseif nargout >= 3
				[f, dfdre, dfdim] = control.design.gamma.area.Hyperbola_border(re, im, parameter);
			elseif nargout >= 2
				[f, dfdre] = control.design.gamma.area.Hyperbola_border(re, im, parameter);
			else
				f = control.design.gamma.area.Hyperbola_border(re, im, parameter);
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
			[~, dfdre, dfdim] = control.design.gamma.area.Hyperbola.border(re, im, parameter);
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
			[~, ~, ~, d2fdredre, d2fdimdre, d2fdredim, d2fdimdim] = control.design.gamma.area.Hyperbola.border(re, im, parameter);
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
		function [this] = Hyperbola(a, b, varargin)
			%HYPERBOLA return new hyperboloidal pole area with specified semi axes and shift
			%	Input:
			%		a:		semi major axis of hyperbola
			%		b:		semi minor axis of hyperbola
			%		shift:	shift of hyperbola
			%	Output:
			%		this:	instance
			shift = getshift(varargin{:});
			this@control.design.gamma.area.GammaArea(GammaArea.HYPERBOLA, shift);
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
			border = -this.a/this.b*sqrt(p.^2 + this.b^2) + this.reshift + 1i*p + 1i*this.imshift;
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
			str = ['Hyperbola(', sprintf(format, this.a), ', ', sprintf(format, this.b), this.printshift(this.reshift, this.imshift, format, true), ')'];
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
				%%% Chilali et Gahinet, 1996 and Eric Lenz
				% parameters for general form
				%syms a1 b1 a2 b2 a12 b12 b21 real
				%%% real and imaginary part
				%syms x y real
				%%%
				%A = [a1, a12; a12, a2];
				%B = [(1/2) * b1, b12; b21, (1/2) * b2];
				%assert(all(all(logical(A == A'))))
				%F = A + (x + 1j * y) * B + (x - 1j * y) * B.';
				%%%
				%%% parameters for hyperbola
				%syms a b x0 real
				%%%
				%%% eig(F) = x * C1 + C0 +- sqrt( x^2 * DX2 + x * DX1 + y^2 * DY2 + D0)
				%%% left sides:		general for of coefficients, manually chosen from eig(F)
				%%% right sides:	coefficients for hyperbola parametrization
				%f = [ ...
				%	(b1 + b2)/2 == 1; ...   % C1
				%	(a1 + a2)/2 == -x0; ...   % C0
				%	(1/4) * (b1 - b2)^2 + (b12 + b21)^2 == 0; ...   % DX2
				%	(1/2) * (a1 - a2) * (b1 - b2) + 2 * a12 * (b12 + b21) == 0; ... % DX1
				%	(b12 - b21)^2 == a^2/b^2; ... % DY2
				%	(1/4) * (a1 - a2)^2 + a12^2 == a^2; ... % D0
				%];
				%sol = solve(f, [a1, a2, a12, b1, b2, b12, b21], 'ReturnConditions', true)
				%solutionidx = 1;
				%L_sym = subs([
				%	sol.a1(solutionidx),	sol.a12(solutionidx);
				%	sol.a12(solutionidx),	sol.a2(solutionidx)
				%], sol.parameters, 0);
				%M_sym = subs([
				%	sol.b1(solutionidx),	sol.b12(solutionidx);
				%	sol.b21(solutionidx),	sol.b2(solutionidx)
				%], sol.parameters, 0);
				%lyap_sym = L_sym - M_sym*(x + 1i*y) - M_sym'*(x - 1i*y)
				%lyap_sym = subs(lyap_sym, x0, -x0)
				%conditions = [
				%	det(lyap_sym)
				%	det(lyap_sym(1, 1))
				%	det(-lyap_sym(1, 2))
				%	det(-lyap_sym(2, 1))
				%	det(lyap_sym(2, 2))
				%] >= 0

				%%% might also work, but does not result in a LMI form that Matlaab can handle
				%a = sym('a', 'real');
				%b = sym('b', 'real');
				%x_0 = sym('x_0', 'real');
				%y_0 = sym('y_0', 'real');
				%y_0 = 0;
				%z = sym('z');
				%omega = sym('omega', 'real'); sigma = sym('sigma', 'real');
				%realp = (z + conj(z))/2;
				%imagp = (z - conj(z))/2/1j;
				%hyperbola = ((realp - x_0)/a)^2 - ((imagp - y_0)/b)^2 - 1;
				%hyperbola = subs(hyperbola, z, sigma + 1i*omega);
				%%% analogy to LMI representation of ellipse
				%L = [
				%	-2*a,	-2*x_0;
				%	-2*x_0,	-2*a
				%];
				%M = [
				%	0,			1 + a/b*1i;
				%	1 - a/b*1i,	0
				%];
				%C1 = [
				%	-2*a, -2*x_0 + (1 + a/b*1j)*z + (1 - a/b*1j)*conj(z);
				%	-2*x_0 + (1 - a/b*1j)*z + (1 + a/b*1j)*conj(z), -2*a
				%]
				%C = L + M*z + transpose(M)*conj(z);
				%simplify(det(C))
				%C_ = subs(C, z, sigma + 1i*omega);
				%detC_ = simplify(det(C_))
				%diffdetC_ = detC_ + 4*a^2*hyperbola
				%%% result is a complex valued LMI that has to be converted to real
				%%% CC = [
				%%% 	-2*a, -2*x_0 + 2, 0, -a/b*z + a/b*conj(z);
				%%% 	-2*x_0 + 2, -2*a, a/b*z - a/b*conj(z), 0;
				%%% 	0, a/b*z - a/b*conj(z), -2*a, -2*x_0 + 2;
				%%% 	-a/b*z + a/b*conj(z), 0, -2*x_0 + 2, -2*a
				%%% ]
				%%% Handbook on Semidefinite, Conic and Polynomial Optimization, Anjos, Lasserre, p. 729
				%CC = blkdiag(L, L) + [
				%	real(M),	-imag(M);
				%	imag(M),	real(M)
				%]*z + [
				%	-imag(M),	-real(M);
				%	real(M),	-imag(M)
				%]*z*0 + [
				%	real(transpose(M)),	-imag(transpose(M));
				%	imag(transpose(M)),	real(transpose(M))
				%]*conj(z) + [
				%	-imag(transpose(M)),	-real(transpose(M));
				%	real(transpose(M)),		-imag(transpose(M))
				%]*conj(z)*0;
				%simplify(det(CC))
				%CC_ = subs(CC, z, sigma + 1i*omega);
				%detCC_ = simplify(det(CC_))
				%diffdet = detCC_ - detC_^2
				%L_final = blkdiag(L, L);
				%M_final = [
				%	real(M),	-imag(M);
				%	imag(M),	real(M)
				%];
				%M_final_transpose = [
				%	real(transpose(M)),	-imag(transpose(M));
				%	imag(transpose(M)),	real(transpose(M))
				%];
				%%% M_final and M_final_transpose are skew symmetric, which is not supported by Matlab, but determinant is equal to hyperbola equation^4
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
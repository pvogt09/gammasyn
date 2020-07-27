classdef Custom < control.design.gamma.area.GammaArea
	%CUSTOM class for representation of a custom area border function f(re, im) = 0 or f(re + 1i im) = 0 with return arguments f, dfdre, dfdim, d2fdredre, d2fdimdre, d2fdredim and d2fdimdim

	properties(Dependent=true)
		% function handle of custom area border function
		fun
	end
	properties(Access=protected)
		% function handle
		func
	end
	properties(SetAccess=protected)
		% indicator if function handle has gradient information
		hasgrad = false;
		% indicator if function handle has hessian information
		hashess = false
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
				[f, dfdre, dfdim, d2fdredre, d2fdimdre, d2fdredim, d2fdimdim] = control.design.gamma.area.Custom_border(re, im, parameter);
			elseif nargout >= 6
				[f, dfdre, dfdim, d2fdredre, d2fdimdre, d2fdredim] = control.design.gamma.area.Custom_border(re, im, parameter);
			elseif nargout >= 5
				[f, dfdre, dfdim, d2fdredre, d2fdimdre] = control.design.gamma.area.Custom_border(re, im, parameter);
			elseif nargout >= 4
				[f, dfdre, dfdim, d2fdredre] = control.design.gamma.area.Custom_border(re, im, parameter);
			elseif nargout >= 3
				[f, dfdre, dfdim] = control.design.gamma.area.Custom_border(re, im, parameter);
			elseif nargout >= 2
				[f, dfdre] = control.design.gamma.area.Custom_border(re, im, parameter);
			else
				f = control.design.gamma.area.Custom_border(re, im, parameter);
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
			[~, dfdre, dfdim] = control.design.gamma.area.Custom.border(re, im, parameter);
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
			[~, ~, ~, d2fdredre, d2fdimdre, d2fdredim, d2fdimdim] = control.design.gamma.area.Custom.border(re, im, parameter);
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
				'fun',		this.fun,...
				'hasgrad',	this.hasgrad,...
				'hashess',	this.hashess...
			);
		end
	end

	methods
		function [this] = Custom(fun, varargin)
			%CIRCLE return new circle pole area with specified radius and shift
			%	Input:
			%		fun:	function handle for custom area border function f(re, im) = 0 or f(re + 1i im) = 0 with return arguments f, dfdre, dfdim, d2fdredre, d2fdimdre, d2fdredim and d2fdimdim
			%		shift:	shift of circle center
			%	Output:
			%		this:	instance
			shift = getshift(varargin{:});
			this@control.design.gamma.area.GammaArea(GammaArea.CUSTOM, shift);
			this.fun = fun;
		end

		function [this] = set.fun(this, fun)
			%FUN setter for custom function handle
			%	Input:
			%		this:	instance
			%		fun:	function handle to set
			%	Output:
			%		this:	instance
			if ~isfunctionhandle(fun)
				error('control:design:gamma:input', 'Polearea function must must be a function handle, not a ''%s''.', class(fun));
			end
			number_areaargs = nargin(fun);
			area_hasgrad = nargout(fun) >= 2 || nargout(fun) <= -1;
			area_hashess = nargout(fun) >= 4 || nargout(fun) <= -1;
			if number_areaargs > 2
				error('control:design:gamma:input', 'Polearea function must have at most 2 input arguments.');
			end
			temp = NaN; %#ok<NASGU> used to prevent parfor warning for uninitialized variable
			tempgraddelta = NaN; %#ok<NASGU> used to prevent parfor warning for uninitialized variable
			tempgradomega = NaN; %#ok<NASGU> used to prevent parfor warning for uninitialized variable
			temphessdeltadelta = NaN;
			temphessdeltaomega = NaN;
			temphessomegadelta = NaN;
			temphessomegaomega = NaN;
			if area_hashess
				if nargout(fun) == -1
					if number_areaargs == 1
						try
							[temp, tempgraddelta, tempgradomega, temphessdeltadelta, temphessdeltaomega, temphessomegadelta, temphessomegaomega] = fun(0);
						catch ehess
							if strcmpi(ehess.identifier, 'MATLAB:maxlhs') || strcmpi(ehess.identifier, 'MATLAB:TooManyOutputs') || strcmpi(ehess.identifier, 'MATLAB:unassignedOutputs')
								try
									[temp, tempgraddelta, tempgradomega] = fun(0);
								catch egrad
									if strcmpi(egrad.identifier, 'MATLAB:maxlhs') || strcmpi(egrad.identifier, 'MATLAB:TooManyOutputs') || strcmpi(egrad.identifier, 'MATLAB:unassignedOutputs')
										area_hasgrad = false;
										area_hashess = false;
										[temp] = fun(0);
										tempgraddelta = NaN(1, size(temp, 1));
										tempgradomega = NaN(1, size(temp, 1));
										temphessdeltadelta = NaN(1, size(temp, 1));
										temphessdeltaomega = NaN(1, size(temp, 1));
										temphessomegadelta = NaN(1, size(temp, 1));
										temphessomegaomega = NaN(1, size(temp, 1));
									else
										rethrow(egrad);
									end
								end
							else
								rethrow(ehess);
							end
						end
					else
						try
							[temp, tempgraddelta, tempgradomega, temphessdeltadelta, temphessdeltaomega, temphessomegadelta, temphessomegaomega] = fun(0, 0);
						catch ehess
							if strcmpi(ehess.identifier, 'MATLAB:maxlhs') || strcmpi(ehess.identifier, 'MATLAB:TooManyOutputs') || strcmpi(ehess.identifier, 'MATLAB:unassignedOutputs')
								try
									[temp, tempgraddelta, tempgradomega] = fun(0, 0);
								catch egrad
									if strcmpi(egrad.identifier, 'MATLAB:maxlhs') || strcmpi(egrad.identifier, 'MATLAB:TooManyOutputs') || strcmpi(egrad.identifier, 'MATLAB:unassignedOutputs')
										area_hasgrad = false;
										area_hashess = false;
										[temp] = fun(0, 0);
										tempgraddelta = NaN(1, size(temp, 1));
										tempgradomega = NaN(1, size(temp, 1));
										temphessdeltadelta = NaN(1, size(temp, 1));
										temphessdeltaomega = NaN(1, size(temp, 1));
										temphessomegadelta = NaN(1, size(temp, 1));
										temphessomegaomega = NaN(1, size(temp, 1));
									else
										rethrow(egrad);
									end
								end
							else
								rethrow(ehess);
							end
						end
					end
				else
					if number_areaargs == 1
						[temp, tempgraddelta, tempgradomega, temphessdeltadelta, temphessdeltaomega, temphessomegadelta, temphessomegaomega] = fun(0);
					else
						[temp, tempgraddelta, tempgradomega, temphessdeltadelta, temphessdeltaomega, temphessomegadelta, temphessomegaomega] = fun(0, 0);
					end
				end
				if ~isrow(tempgraddelta)
					error('control:design:gamma:input', 'Polearea function gradient must return row vectors.');
				end
				if ~isrow(tempgradomega)
					error('control:design:gamma:input', 'Polearea function gradient must return row vectors.');
				end
				if size(temp, 2) ~= size(tempgraddelta, 2)
					error('control:design:gamma:input', 'Polearea function gradient must return row vectors of the same size as polearea function.');
				end
				if size(temp, 2) ~= size(tempgradomega, 2)
					error('control:design:gamma:input', 'Polearea function gradient must return row vectors of the same size as polearea function.');
				end
				if ~isrow(temphessdeltadelta)
					error('control:design:gamma:input', 'Polearea function hessian must return row vectors.');
				end
				if ~isrow(temphessdeltaomega)
					error('control:design:gamma:input', 'Polearea function hessian must return row vectors.');
				end
				if ~isrow(temphessomegadelta)
					error('control:design:gamma:input', 'Polearea function hessian must return row vectors.');
				end
				if ~isrow(temphessomegaomega)
					error('control:design:gamma:input', 'Polearea function hessian must return row vectors.');
				end
				if size(temp, 2) ~= size(temphessdeltadelta, 2)
					error('control:design:gamma:input', 'Polearea function hessian must return row vectors of the same size as polearea function.');
				end
				if size(temp, 2) ~= size(temphessdeltaomega, 2)
					error('control:design:gamma:input', 'Polearea function hessian must return row vectors of the same size as polearea function.');
				end
				if size(temp, 2) ~= size(temphessomegadelta, 2)
					error('control:design:gamma:input', 'Polearea function hessian must return row vectors of the same size as polearea function.');
				end
				if size(temp, 2) ~= size(temphessomegaomega, 2)
					error('control:design:gamma:input', 'Polearea function hessian must return row vectors of the same size as polearea function.');
				end
			elseif area_hasgrad
				if nargout(fun) == -1
					if number_areaargs == 1
						try
							[temp, tempgraddelta, tempgradomega] = fun(0);
						catch egrad
							if strcmpi(egrad.identifier, 'MATLAB:maxlhs') || strcmpi(egrad.identifier, 'MATLAB:TooManyOutputs') || strcmpi(egrad.identifier, 'MATLAB:unassignedOutputs')
								area_hasgrad = false;
								[temp] = fun(0);
								tempgraddelta = NaN(1, size(temp, 1));
								tempgradomega = NaN(1, size(temp, 1));
							else
								rethrow(egrad);
							end
						end
					else
						try
							[temp, tempgraddelta, tempgradomega] = fun(0, 0);
						catch egrad
							if strcmpi(egrad.identifier, 'MATLAB:maxlhs') || strcmpi(egrad.identifier, 'MATLAB:TooManyOutputs') || strcmpi(egrad.identifier, 'MATLAB:unassignedOutputs')
								area_hasgrad = false;
								[temp] = fun(0, 0);
								tempgraddelta = NaN(1, size(temp, 1));
								tempgradomega = NaN(1, size(temp, 1));
							else
								rethrow(egrad);
							end
						end
					end
				else
					if number_areaargs == 1
						[temp, tempgraddelta, tempgradomega] = fun(0);
					else
						[temp, tempgraddelta, tempgradomega] = fun(0, 0);
					end
				end
				if ~isrow(tempgraddelta)
					error('control:design:gamma:input', 'Polearea function gradient must return row vectors.');
				end
				if ~isrow(tempgradomega)
					error('control:design:gamma:input', 'Polearea function gradient must return row vectors.');
				end
				if size(temp, 2) ~= size(tempgraddelta, 2)
					error('control:design:gamma:input', 'Polearea function gradient must return row vectors of the same size as polearea function.');
				end
				if size(temp, 2) ~= size(tempgradomega, 2)
					error('control:design:gamma:input', 'Polearea function gradient must return row vectors of the same size as polearea function.');
				end
				area_hashess = false;
			else
				if number_areaargs == 1
					temp = fun(0);
				else
					temp = fun(0, 0);
				end
				area_hashess = false;
			end
			if ~isrow(temp)
				error('control:design:gamma:input', 'Polearea function must return row vectors.');
			end
			if size(temp, 2) ~= 1
				error('control:design:gamma:input', 'Polearea function must have return scalars.');
			end
			if nargin(fun) == 1
				this.func = fun22arg(fun, area_hasgrad, area_hashess);
			else
				this.func = fun;
			end
			this.hasgrad = area_hasgrad;
			this.hashess = area_hashess;
		end

		function [fun] = get.fun(this)
			%FUN getter for custom function handle
			%	Input:
			%		this:	instance
			%	Output:
			%		fun:	function handle to set
			fun = this.func;
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
			border = zeros(0, numpoints);
			% TODO: add implementation
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
			parameter.custom_fun = this.fun;
			parameter.custom_hasgrad = this.hasgrad;
			parameter.custom_hashess = this.hashess;
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
			parameter.custom_fun = this.fun;
			parameter.custom_hasgrad = this.hasgrad;
			parameter.custom_hashess = this.hashess;
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
			parameter.custom_fun = this.fun;
			parameter.custom_hasgrad = this.hasgrad;
			parameter.custom_hashess = this.hashess;
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
			str = ['Custom(', sprintf('%s', func2str(this.fun)), this.printshift(this.reshift, this.imshift, format, true), ')'];
		end

		function [L, M, success] = toinstanceLMIregion(~)
			%TOINSTANCELMIREGION convert area to LMI region that can be used by Matlab functions for object
			%	Input:
			%		this:		instance
			%	Output:
			%		L:			L = L^T real matrix of constant coefficients in the LMI, where the L(1, 1) element contains a imaginary part indicating the dimension of the LMI variable
			%		M:			real matrix of linear coefficients in the LMI
			%		success:	indicator, if conversion to LMI was successful
			L = [];
			M = [];
			success = false;
		end
	end
end

function [handle] = fun22arg(fun, hasgrad, hashess)
	%FUN22ARG convert area function with one input argument to function with 2 input arguments
	%	Input:
	%		fun:		function to convert
	%		hasgrad:	indicator if function returns gradient information
	%		hashess:	indicator if function returns hessian information
	%	Output:
	%		handle:		function handle with 2 input arguments
	function [f, dfdre, dfdim, d2fdredre, d2fdimdre, d2fdredim, d2fdimdim] = areafun_hess(re, im)
		if nargout >= 4
			[f, dfdre, dfdim, d2fdredre, d2fdimdre, d2fdredim, d2fdimdim] = fun(re + 1i*im);
		elseif nargout >= 2
			[f, dfdre, dfdim] = fun(re + 1i*im);
		else
			f = fun(re + 1i*im);
		end
	end
	function [f, dfdre, dfdim] = areafun_grad(re, im)
		if nargout >= 2
			[f, dfdre, dfdim] = fun(re + 1i*im);
		else
			f = fun(re + 1i*im);
		end
	end
	function [f] = areafun_fun(re, im)
		f = fun(re + 1i*im);
	end
	if hasgrad && hashess
		handle = @areafun_hess;
	elseif hasgrad && ~hashess
		handle = areafun_grad;
	else
		handle = areafun_fun;
	end
end
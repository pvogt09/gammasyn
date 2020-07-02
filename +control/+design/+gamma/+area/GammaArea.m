classdef(Abstract) GammaArea < matlab.mixin.Heterogeneous
	%GAMMAAREA class for representation of a pole area
	
	properties(Constant=true, Access=protected)
		% number of points to use by default in plotborder
		NUMBEROFPOINTS = 100;
		% default format for numbers
		NUMBERFORMAT = '%g'
	end
	properties(Constant=true, Hidden=true)
		% prototype parameter structure
		PARAMETERPROTOTYPE = struct(...
			'type',				GammaArea.empty(),...
			'reshift',			0,...
			'imshift',			0,...
			'circle_R',			NaN,...
			'circlediscrete_R',	NaN,...
			'ellipse_a',		NaN,...
			'ellipse_b',		NaN,...
			'hyperbola_a',		NaN,...
			'hyperbola_b',		NaN,...
			'line_a',			NaN,...
			'line_b',			NaN,...
			'logspiral_R',		NaN,...
			'logspiral_k',		NaN,...
			'polyellipse_f',	NaN + 1i*NaN,...
			'polyellipse_w',	NaN,...
			'custom_fun',		NaN,...
			'custom_hasgrad',	false,...
			'custom_hashess',	false...
		);
		% prototype parameter structure for codegen
		PARAMETERPROTOTYPEEMPTY = struct(...
			'type',				{},...
			'reshift',			{},...
			'imshift',			{},...
			'circle_R',			{},...
			'circlediscrete_R',	{},...
			'ellipse_a',		{},...
			'ellipse_b',		{},...
			'hyperbola_a',		{},...
			'hyperbola_b',		{},...
			'line_a',			{},...
			'line_b',			{},...
			'logspiral_R',		{},...
			'logspiral_k',		{},...
			'polyellipse_f',	{},...
			'polyellipse_w',	{},...
			'custom_fun',		{},...
			'custom_hasgrad',	{},...
			'custom_hashess',	{}...
		);
		% prototype parameter structure with parameter sizes and complexity for codegen
		PARAMETERPROTOTYPESIZE = struct(...
			'type',				[1, 1],...
			'reshift',			[1, 1],...
			'imshift',			[1, 1],...
			'circle_R',			[1, 1],...
			'circlediscrete_R',	[1, 1],...
			'ellipse_a',		[1, 1],...
			'ellipse_b',		[1, 1],...
			'hyperbola_a',		[1, 1],...
			'hyperbola_b',		[1, 1],...
			'line_a',			[1, 1],...
			'line_b',			[1, 1],...
			'logspiral_R',		[1, 1],...
			'logspiral_k',		[1, 1],...
			'polyellipse_f',	[1, Inf] + 1i*[1, Inf],...
			'polyellipse_w',	[1, Inf],...
			'custom_fun',		[1, 1],...
			'custom_hasgrad',	[1, 1],...
			'custom_hashess',	[1, 1]...
		);
	end
	
	properties(SetAccess=protected)
		% type of pole area
		type,
		% shift on real axis
		reshift,
		% shift on imaginary axis
		imshift
	end
	
	methods(Static=true)
		function [is] = isfunctionstruct(functionstruct)
			%ISFUNCTIONSTRUCT return if a structure is a valid functionstruct
			%	Input:
			%		functionsstruct:	structure to test
			%	Output:
			%		is:					true, if the structure is a valid function structure, else false
			is = true;
			if ~isstruct(functionstruct)
				is = false;
				return;
			end
			if any(~isfield(functionstruct, {
					'type';
					'reshift';
					'imshift';
					'circle_R';
					'circlediscrete_R';
					'ellipse_a';
					'ellipse_b';
					'hyperbola_a';
					'hyperbola_b';
					'line_a';
					'line_b';
					'logspiral_R';
					'logspiral_k';
					'polyellipse_f';
					'polyellipse_w';
					'custom_fun';
					'custom_hasgrad';
					'custom_hashess'
				}))
				is = false;
				return;
			end
		end
		
		function [this] = fromfunctionstruct(functionstruct)
			%TOFUNCTIONSTRUCT convert parameter structure into object
			%	Input:
			%		functionstruct:	objcet as parameter structure
			%	Output:
			%		this:			instance
			if ~isstruct(functionstruct)
				error('control:design:gamma:area', 'Structure must be of type ''struct'', not ''%s''.', class(functionstruct));
			end
			if any(~isfield(functionstruct, {
					'type';
					'reshift';
					'imshift';
					'circle_R';
					'circlediscrete_R';
					'ellipse_a';
					'ellipse_b';
					'hyperbola_a';
					'hyperbola_b';
					'line_a';
					'line_b';
					'logspiral_R';
					'logspiral_k';
					'polyellipse_f';
					'polyellipse_w';
					'custom_fun';
					'custom_hasgrad';
					'custom_hashess'
				}))
				error('control:design:gamma:area', 'Structure is not a valid parameter structure for a GammaArea.');
			end
			if isscalar(functionstruct)
				switch functionstruct.type
					case GammaArea.CIRCLE
						this = control.design.gamma.area.Circle(functionstruct.circle_R, functionstruct.reshift + 1i*functionstruct.imshift);
					case GammaArea.CIRCLEDISCRETE
						this = control.design.gamma.area.CircleDiscrete(functionstruct.circle_R, functionstruct.reshift + 1i*functionstruct.imshift);
					case GammaArea.CIRCLESQUARE
						this = control.design.gamma.area.Circlesquare(functionstruct.circle_R, functionstruct.reshift + 1i*functionstruct.imshift);
					case GammaArea.ELLIPSE
						this = control.design.gamma.area.Ellipse(functionstruct.ellipse_a, functionstruct.ellipse_b, functionstruct.reshift + 1i*functionstruct.imshift);
					case GammaArea.ELLIPSESQUARE
						this = control.design.gamma.area.Ellipsesquare(functionstruct.ellipse_a, functionstruct.ellipse_b, functionstruct.reshift + 1i*functionstruct.imshift);
					case GammaArea.HYPERBOLA
						this = control.design.gamma.area.Hyperbola(functionstruct.hyperbola_a, functionstruct.hyperbola_b, functionstruct.reshift + 1i*functionstruct.imshift);
					case GammaArea.HYPERBOLASQUARE
						this = control.design.gamma.area.Hyperbolasquare(functionstruct.hyperbola_a, functionstruct.hyperbola_b, functionstruct.reshift + 1i*functionstruct.imshift);
					case GammaArea.IMAG
						this = control.design.gamma.area.Imag(functionstruct.line_a, functionstruct.line_b, functionstruct.reshift + 1i*functionstruct.imshift);
					case GammaArea.LINE
						this = control.design.gamma.area.Line(functionstruct.line_a, functionstruct.line_b, functionstruct.reshift + 1i*functionstruct.imshift);
					case GammaArea.LOGSPIRAL
						this = control.design.gamma.area.LogSpiral(functionstruct.logspiral_R, functionstruct.logspiral_k, functionstruct.reshift + 1i*functionstruct.imshift);
					case GammaArea.POLYELLIPSE
						this = control.design.gamma.area.PolyEllipse(functionstruct.polyellipse_f, functionstruct.polyellipse_w, functionstruct.reshift + 1i*functionstruct.imshift);
					case GammaArea.POLYELLIPSESQUARE
						this = control.design.gamma.area.PolyEllipsesquare(functionstruct.polyellipse_f, functionstruct.polyellipse_w, functionstruct.reshift + 1i*functionstruct.imshift);
					case GammaArea.NONE
						this = control.design.gamma.area.None(functionstruct.reshift + 1i*functionstruct.imshift);
					case GammaArea.CUSTOM
						this = control.design.gamma.area.Custom(functionstruct.custom_fun, functionstruct.reshift + 1i*functionstruct.imshift);
					otherwise
						error('control:design:gamma:area', 'Type must be a GammaArea, not a ''%s''.', class(type));
				end
			else
				temp = reshape(functionstruct, numel(functionstruct), 1);
				val = cell(size(temp, 1), 1);
				parfor ii = 1:length(temp)
					switch temp(ii).type
						case GammaArea.CIRCLE
							val{ii, 1} = control.design.gamma.area.Circle(temp(ii, 1).circle_R, temp(ii, 1).reshift + 1i*temp(ii, 1).imshift);
						case GammaArea.CIRCLEDISCRETE
							val{ii, 1} = control.design.gamma.area.CircleDiscrete(temp(ii, 1).circle_R, temp(ii, 1).reshift + 1i*temp(ii, 1).imshift);
						case GammaArea.CIRCLESQUARE
							val{ii, 1} = control.design.gamma.area.Circlesquare(temp(ii, 1).circle_R, temp(ii, 1).reshift + 1i*temp(ii, 1).imshift);
						case GammaArea.ELLIPSE
							val{ii, 1} = control.design.gamma.area.Ellipse(temp(ii, 1).ellipse_a, temp(ii, 1).ellipse_b, temp(ii, 1).reshift + 1i*temp(ii, 1).imshift);
						case GammaArea.ELLIPSESQUARE
							val{ii, 1} = control.design.gamma.area.Ellipsesquare(temp(ii, 1).ellipse_a, temp(ii, 1).ellipse_b, temp(ii, 1).reshift + 1i*temp(ii, 1).imshift);
						case GammaArea.HYPERBOLA
							val{ii, 1} = control.design.gamma.area.Hyperbola(temp(ii, 1).hyperbola_a, temp(ii, 1).hyperbola_b, temp(ii, 1).reshift + 1i*temp(ii, 1).imshift);
						case GammaArea.HYPERBOLASQUARE
							val{ii, 1} = control.design.gamma.area.Hyperbolasquare(temp(ii, 1).hyperbola_a, temp(ii, 1).hyperbola_b, temp(ii, 1).reshift + 1i*temp(ii, 1).imshift);
						case GammaArea.IMAG
							val{ii, 1} = control.design.gamma.area.Imag(temp(ii, 1).line_a, temp(ii, 1).line_b, temp(ii, 1).reshift + 1i*temp(ii, 1).imshift);
						case GammaArea.LINE
							val{ii, 1} = control.design.gamma.area.Line(temp(ii, 1).line_a, temp(ii, 1).line_b, temp(ii, 1).reshift + 1i*temp(ii, 1).imshift);
						case GammaArea.LOGSPIRAL
							val{ii, 1} = control.design.gamma.area.LogSpiral(temp(ii, 1).logspiral_R, temp(ii, 1).logspiral_k, temp(ii, 1).reshift + 1i*temp(ii, 1).imshift);
						case GammaArea.POLYELLIPSE
							val{ii, 1} = control.design.gamma.area.PolyEllipse(temp(ii, 1).polyellipse_f, temp(ii, 1).polyellipse_w, temp(ii, 1).reshift + 1i*temp(ii, 1).imshift);
						case GammaArea.POLYELLIPSESQUARE
							val{ii, 1} = control.design.gamma.area.PolyEllipsesquare(temp(ii, 1).polyellipse_f, temp(ii, 1).polyellipse_w, temp(ii, 1).reshift + 1i*temp(ii, 1).imshift);
						case GammaArea.NONE
							val{ii, 1} = control.design.gamma.area.None(temp(ii, 1).reshift + 1i*temp(ii, 1).imshift);
						case GammaArea.CUSTOM
							val{ii, 1} = control.design.gamma.area.Custom(temp(ii, 1).custom_fun, temp(ii, 1).reshift + 1i*temp(ii, 1).imshift);
						otherwise
							error('control:design:gamma:area', 'Type must be a GammaArea, not a ''%s''.', class(type));
					end
				end
				this = reshape([val{:}], size(functionstruct));
			end
		end
		
		function [uni, idxthis, idxuni] = unique(this)
			%UNIQUE return uniquely parametrized GammaAreas
			%	Input:
			%		this:		instance or functionstruct representation of a GammaArea
			%	Output:
			%		uni:		unique occurences of this
			%		idxthis:	indices such that uni = this(idxthis)
			%		idxuni:		indices such that this = uni(idxuni)
			if isa(this, 'control.design.gamma.area.GammaArea')
				parameters = this.tofunctionstruct();
				isgammaarea = true;
			else
				if ~isstruct(this) || any(~isfield(this, {
					'type';
					'reshift';
					'imshift';
					'circle_R';
					'circlediscrete_R';
					'ellipse_a';
					'ellipse_b';
					'hyperbola_a';
					'hyperbola_b';
					'line_a';
					'line_b';
					'logspiral_R';
					'logspiral_k';
					'polyellipse_f';
					'polyellipse_w';
					'custom_fun';
					'custom_hasgrad';
					'custom_hashess'
				}))
					error('control:design:gamma:area', 'Structure is not a valid parameter structure for a GammaArea.');
				end
				parameters = this;
				isgammaarea = false;
			end
			names = fieldnames(control.design.gamma.area.GammaArea.PARAMETERPROTOTYPEEMPTY);
			sizes = control.design.gamma.area.GammaArea.PARAMETERPROTOTYPESIZE;
			values = NaN(numel(parameters), size(names, 1)) + 1i*NaN(numel(parameters), size(names, 1));
			varsizevalues = cell(1, size(names, 1));
			customvalues = cell(numel(parameters), size(names, 1));
			isvarsize = false(1, size(names, 1));
			iscustom = strcmpi(names, 'custom_fun')';
			varlen = zeros(1, size(names, 1));
			parfor ii = 1:size(names, 1)
				isvarsize(1, ii) = any(isinf(sizes.(names{ii})));
				if iscustom(1, ii)
					vals = {parameters(:).(names{ii})};
					customvalues(:, ii) = vals;
					vals = cellfun(@function_handle2unique, vals, 'UniformOutput', true);
					nanvalue = isnan(vals);
					% make function handles unique
					replace = 1:sum(nanvalue);
					vals(nanvalue) = replace;
					values(:, ii) = vals(:);
				elseif isvarsize(1, ii)
					vals = {parameters(:).(names{ii})};
					len = cellfun(@length, vals, 'UniformOutput', true);
					maxlen = max(len);
					if all(maxlen == len)
						vals = cat(1, vals{:});
					else
						for jj = 1:length(vals)
							vals{jj} = [vals{jj} NaN(size(vals{jj}, 1), maxlen - len(jj))];
						end
						vals = cat(1, vals{:});
					end
					varsizevalues{:, ii} = vals;
					varlen(1, ii) = size(vals, 2);
				else
					vals = [parameters(:).(names{ii})];
					if strcmpi(names{ii}, 'type')
						vals = double(vals);
					end
					values(:, ii) = vals(:);
				end
			end
			customvalues = customvalues(:, iscustom);
			if any(isvarsize)
				allvalues = NaN(numel(parameters), size(values, 2) - sum(isvarsize) + sum(varlen));
				valuesidx = 1;
				varsizevalueidx = 1;
				parametermappingidx = NaN(1, size(allvalues, 2));
				for ii = 1:size(allvalues, 2)
					parametermappingidx(1, ii) = valuesidx;
					if ~isvarsize(1, valuesidx)
						allvalues(:, ii) = values(:, valuesidx);
						valuesidx = valuesidx + 1;
					else
						allvalues(:, ii) = varsizevalues{1, valuesidx}(:, varsizevalueidx);
						if varsizevalueidx == size(varsizevalues{1, valuesidx}, 2)
							varsizevalueidx = 1;
							valuesidx = valuesidx + 1;
						else
							varsizevalueidx = varsizevalueidx + 1;
						end
					end
				end
				values = allvalues;
			end
			wasinf = isinf(values);
			values(isnan(values)) = Inf;
			[v, idxthis, idxuni] = unique(values, 'rows', 'stable');
			uni = this(idxthis);
			customvalues = customvalues(idxthis, 1);
			if ~isgammaarea
				v(isinf(v)) = NaN;
				v(wasinf(idxthis, :)) = Inf;
				areafunsunique = control.design.gamma.area.GammaArea.PARAMETERPROTOTYPEEMPTY;
				parfor jj = 1:size(v, 1)
					temp = control.design.gamma.area.GammaArea.PARAMETERPROTOTYPEEMPTY;
					for ii = 1:size(names, 1)
						if strcmpi(names{ii}, 'type')
							temp(1).(names{ii, 1}) = GammaArea(v(jj, parametermappingidx == ii));
						elseif iscustom(1, ii)
							temp(1).(names{ii, 1}) = customvalues{jj, 1};
						else
							temp(1).(names{ii, 1}) = v(jj, parametermappingidx == ii);
						end
					end
					areafunsunique(jj, 1) = temp;
				end
				uni = control.design.gamma.area.GammaArea.fromfunctionstruct(areafunsunique);
			end
		end
	end
	
	methods(Static=true, Access=protected)
		function [str] = printshift(reshift, imshift, format, asargument)
			%PRINTSHIFT return shift as string
			%	Input:
			%		reshift:	real shift to print
			%		imshift:	imaginary shift to print
			%		format:		format to use for printing numbers
			%		asargument:	return string as argument
			%	Output:
			%		str:	shift as string
			if nargin <= 2
				format = control.design.gamma.area.GammaArea.NUMBERFORMAT;
			end
			if nargin <= 3
				asargument = false;
			end
			if ~ischar(format)
				error('control:design:gamma:area', 'Format string must be of type ''char'', not a ''%s''.', class(format));
			end
			if reshift ~= 0 || imshift ~= 0
				if abs(imshift) > 0
					str = sprintf([format, ' + i', format], reshift, imshift);
				else
					str = sprintf([format, ' + i0'], reshift);
				end
				if asargument
					str = [', ', str];
				end
			else
				str = '';
			end
		end
	end
	
	methods%(Access=protected)
		function [this] = GammaArea(type, shift)
			%GAMMAAREA return new pole area of specified type and shift
			%	Input:
			%		type:	type of pole area
			%		shift:	shift pole area
			%	Output:
			%		this:	instance
			if nargin <= 1
				shift = 0;
			end
			if ~isa(type, 'GammaArea')
				error('control:design:gamma:area', 'Type must be a GammaArea, not a ''%s''.', class(type));
			end
			if ~isscalar(shift) || ~isnumeric(shift)
				error('control:design:gamma:area', 'Shift must be a numeric scalar.');
			end
			this.type = type;
			this.reshift = real(shift);
			this.imshift = imag(shift);
		end
		
		%PLOTINSTANCEBORDER plot border of area function
		%	This is a template method for specialization of ploting functions as matlab.mixin.Heterogeneous does not allow overloading methods.
		%	Input:
		%		this:		instance
		%		limit:		maximum and minimum values for plotting
		%		numpoints:	number of points to plot
		%	Output:
		%		border:		border points
		[border] = plotinstanceborder(this, limit, numpoints);
	end
	
	methods(Access=protected)
		%GETPARAMETERS return structure with parameters of current object unknown to this class to parameterize instances and allow code generation
		%	Input:
		%		this:		instance
		%	Output:
		%		parameter:	structure with parameters
		[parameters] = getparameters(this);
		
		%GETINSTANCEBORDER return border of polearea with f < 0 for points left of the border, f = 0 for points on the border and f > 0 for points right of the border
		%	This is a template method for specialization of function calculations as matlab.mixin.Heterogeneous does not allow overloading methods.
		%	Input:
		%		this:		instance
		%		re:			real part of point
		%		im:			imaginary part of the point
		%	Output:
		%		f:			border function value at point [re, im]
		[f] = getinstanceborder(this, re, im);
		
		%GETINSTANCEGRADBORDER return gradient of border of polearea with f < 0 for points left of the border, f = 0 for points on the border and f > 0 for points right of the border
		%	This is a template method for specialization of gradient calculations as matlab.mixin.Heterogeneous does not allow overloading methods.
		%	Input:
		%		this:		instance
		%		re:			real part of point
		%		im:			imaginary part of the point
		%	Output:
		%		dfdre:		gradient of border function value at point [re, im] for coordinate re
		%		dfdim:		gradient of border function value at point [re, im] for coordinate im
		[dfdre, dfdim] = getinstancegradborder(this, re, im);
		
		%GETINSTANCEHESSBORDER return hessian of border of polearea with f < 0 for points left of the border, f = 0 for points on the border and f > 0 for points right of the border
		%	This is a template method for specialization of hessian calculations as matlab.mixin.Heterogeneous does not allow overloading methods.
		%	Input:
		%		this:		instance
		%		re:			real part of point
		%		im:			imaginary part of the point
		%	Output:
		%		d2fdredre:	partial derivative of the gradient dfdre of border function value at [re, im] for coordinate re
		%		d2fdimdre:	partial derivative of the gradient dfdre of border function value at [re, im] for coordinate im
		%		d2fdredim:	partial derivative of the gradient dfdim of border function value at [re, im] for coordinate re
		%		d2fdimdim:	partial derivative of the gradient dfdim of border function value at [re, im] for coordinate im
		[d2fdredre, d2fdimdre, d2fdredim, d2fdimdim] = getinstancehessborder(this, re, im);
		
		%GETINSTANCESTRING return string representation of object
		%	This is a template method for specialization of gradient calculations as matlab.mixin.Heterogeneous does not allow overloading methods.
		%	Input:
		%		this:		instance
		%		format:		format to use for printing numbers
		%	Output:
		%		str:		string representation of the instance
		[str] = getinstancestring(this, format);
		
		%TOINSTANCELMIREGION convert area to LMI region that can be used by Matlab functions for object
		%	Input:
		%		this:		instance
		%	Output:
		%		L:			L = L^T real matrix of constant coefficients in the LMI, where the L(1, n) element contains a imaginary part indicating the dimension of the LMI variable
		%		M:			real matrix of linear coefficients in the LMI
		%		success:	indicator, if conversion to LMI was successful
		[L, M, success] = toinstanceLMIregion(this);
	end
	
	methods(Sealed=true)
		function [f] = getborder(this, re, im)
			%GETBORDER return border of polearea with f < 0 for points left of the border, f = 0 for points on the border and f > 0 for points right of the border
			%	Input:
			%		this:		instance
			%		re:			real part of point
			%		im:			imaginary part of the point
			%	Output:
			%		f:			border function value at point [re, im]
			if numel(this) > 1
				t = reshape(this, [], 1);
				f = NaN(size(t, 1), 1);
				for ii = 1:size(t, 1)
					f(ii, 1) = t(ii).getinstanceborder(re, im);
				end
				f = reshape(f, size(this));
			else
				f = this.getinstanceborder(re, im);
			end
		end
		
		function [dfdre, dfdim] = getgradborder(this, re, im)
			%GETGRADBORDER return gradient of border of polearea with f < 0 for points left of the border, f = 0 for points on the border and f > 0 for points right of the border
			%	Input:
			%		this:		instance
			%		re:			real part of point
			%		im:			imaginary part of the point
			%	Output:
			%		dfdre:		gradient of border function value at point [re, im] for coordinate re
			%		dfdim:		gradient of border function value at point [re, im] for coordinate im
			if numel(this) > 1
				t = reshape(this, [], 1);
				dfdre = NaN(size(t, 1), 1);
				dfdim = NaN(size(t, 1), 1);
				for ii = 1:size(t, 1)
					[dfdre(ii, 1), dfdim(ii, 1)] = t(ii).getinstancegradborder(re, im);
				end
				dfdre = reshape(dfdre, size(this));
				dfdim = reshape(dfdim, size(this));
			else
				[dfdre, dfdim] = this.getinstancegradborder(re, im);
			end
		end
		
		function [d2fdredre, d2fdimdre, d2fdredim, d2fdimdim] = gethessborder(this, re, im)
			%GETHESSBORDER return hessian of border of polearea with f < 0 for points left of the border, f = 0 for points on the border and f > 0 for points right of the border
			%	Input:
			%		this:		instance
			%		re:			real part of point
			%		im:			imaginary part of the point
			%	Output:
			%		d2fdredre:	partial derivative of the gradient dfdre of border function value at [re, im] for coordinate re
			%		d2fdimdre:	partial derivative of the gradient dfdre of border function value at [re, im] for coordinate im
			%		d2fdredim:	partial derivative of the gradient dfdim of border function value at [re, im] for coordinate re
			%		d2fdimdim:	partial derivative of the gradient dfdim of border function value at [re, im] for coordinate im
			if numel(this) > 1
				t = reshape(this, [], 1);
				d2fdredre = NaN(size(t, 1), 1);
				d2fdimdim = NaN(size(t, 1), 1);
				for ii = 1:size(t, 1)
					[d2fdredre(ii, 1), d2fdredim(ii, 1), d2fdredim(ii, 1), d2fdimdim(ii, 1)] = t(ii).getinstancegradborder(re, im);
				end
				d2fdredre = reshape(d2fdredre, size(this));
				d2fdredim = reshape(d2fdredim, size(this));
				d2fdredim = reshape(d2fdredim, size(this));
				d2fdimdim = reshape(d2fdimdim, size(this));
			else
				[d2fdredre, d2fdimdre, d2fdredim, d2fdimdim] = this.getinstancehessborder(re, im);
			end
		end
		
		function [border] = plotborder(this, limit, numpoints)
			%PLOTBORDER plot border of area function
			%	Input:
			%		this:		instance
			%		limit:		maximum and minimum values for plotting
			%		numpoints:	number of points to plot
			%	Output:
			%		border:		border points
			if nargin <= 1
				limit = [
					1 + 1i;
					-1 - 1i
				];
			end
			if numel(this) > 1
				t = reshape(this, [], 1);
				if nargin <= 2
					numpoints = this.NUMBEROFPOINTS;
				end
				fig = get(groot, 'CurrentFigure');
				tempborder = zeros(0, numpoints);
				nargin1 = nargin == 1;
				nargin2 = nargin == 2;
				nargout0 = nargout > 0;
				lim = limit;
				for ii = 1:size(t, 1) %#ok<FORPF> tempborder changes size every iteration
					if nargout0
						if nargin1
							tempborder = [
								tempborder;
								t(ii).plotinstanceborder()
							];
						elseif nargin2
							tempborder = [
								tempborder;
								t(ii).plotinstanceborder(lim)
							];
						else
							tempborder = [
								tempborder;
								t(ii).plotinstanceborder(lim, numpoints)
							];
						end
					else
						if nargin1
							tmp = t(ii).plotinstanceborder();
						elseif nargin2
							tmp = t(ii).plotinstanceborder(lim);
						else
							tmp = t(ii).plotinstanceborder(lim, numpoints);
						end
						plot(real(tmp), imag(tmp));
						tempborder = [
							tempborder;
							tmp
						];
					end
					if isempty(fig)
						hold('on');
					end
					lim = [
						max(real([
							lim(:);
							tempborder(:)
						])) + 1i*max(imag([
							lim(:);
							tempborder(:)
						]));
						min(real([
							lim(:);
							tempborder(:)
						])) + 1i*min(imag([
							lim(:);
							tempborder(:)
						]))
					];
				end
				lim = [
					max(real([
						lim(:);
						tempborder(:)
					])) + 1i*max(imag([
						lim(:);
						tempborder(:)
					]));
					min(real([
						lim(:);
						tempborder(:)
					])) + 1i*min(imag([
						lim(:);
						tempborder(:)
					]))
				];
				if nargin >= 1
					lim = [
						min([
							max(real(lim(:)));
							max(real(limit(:)))
						]) + 1i*min([
							max(imag(lim(:)));
							max(imag(limit(:)))
						]);
						max([
							min(real(lim(:)));
							min(real(limit(:)))
						]) + 1i*max([
							min(imag(lim(:)));
							min(imag(limit(:)))
						])
					];
				end
				tempborder = zeros(0, numpoints);
				for ii = 1:size(t, 1) %#ok<FORPF> tempborder changes size every iteration
					if nargout0
						if nargin1
							tempborder = [
								tempborder;
								t(ii).plotinstanceborder(lim)
							];
						elseif nargin2
							tempborder = [
								tempborder;
								t(ii).plotinstanceborder(lim)
							];
						else
							tempborder = [
								tempborder;
								t(ii).plotinstanceborder(lim, numpoints)
							];
						end
					else
						if nargin1
							t(ii).plotinstanceborder(lim);
						elseif nargin2
							t(ii).plotinstanceborder(lim);
						else
							t(ii).plotinstanceborder(lim, numpoints);
						end
					end
					if isempty(fig)
						hold('on');
					end
				end
				if isempty(fig)
					hold('off');
				end
				if nargout0
					border = tempborder;
				end
			else
				if nargout > 0
					if nargin == 1
						border = this.plotinstanceborder();
					elseif nargin == 2
						border = this.plotinstanceborder(limit);
					else
						border = this.plotinstanceborder(limit, numpoints);
					end
				else
					if nargin == 1
						this.plotinstanceborder();
					elseif nargin == 2
						this.plotinstanceborder(limit);
					else
						this.plotinstanceborder(limit, numpoints);
					end
				end
			end
		end
		
		function [functionstruct] = tofunctionstruct(this)
			%TOFUNCTIONSTRUCT convert object into parameter structure
			%	Input:
			%		this:			instance
			%	Output:
			%		functionstruct:	objcet as parameter structure
			if isscalar(this)
				parameters = this.getparameters();
				if isfield(parameters, 'R')
					circle_R = parameters.R;
				else
					circle_R = NaN;
				end
				if this.type == GammaArea.LOGSPIRAL
					if isfield(parameters, 'k')
						logspiral_k = parameters.k;
					else
						logspiral_k = NaN;
					end
				else
					logspiral_k = NaN;
				end
				logspiral_R = circle_R;
				circlediscrete_R = circle_R;
				if this.type == GammaArea.ELLIPSE || this.type == GammaArea.ELLIPSESQUARE
					if isfield(parameters, 'a')
						ellipse_a = parameters.a;
					else
						ellipse_a = NaN;
					end
					if isfield(parameters, 'b')
						ellipse_b = parameters.b;
					else
						ellipse_b = NaN;
					end
				else
					ellipse_a = NaN;
					ellipse_b = NaN;
				end
				if this.type == GammaArea.HYPERBOLA || this.type == GammaArea.HYPERBOLASQUARE
					if isfield(parameters, 'a')
						hyperbola_a = parameters.a;
					else
						hyperbola_a = NaN;
					end
					if isfield(parameters, 'b')
						hyperbola_b = parameters.b;
					else
						hyperbola_b = NaN;
					end
				else
					hyperbola_a = NaN;
					hyperbola_b = NaN;
				end
				if this.type == GammaArea.LINE || this.type == GammaArea.IMAG
					if isfield(parameters, 'a')
						line_a = parameters.a;
					else
						line_a = NaN;
					end
					if isfield(parameters, 'b')
						line_b = parameters.b;
					else
						line_b = NaN;
					end
				else
					line_a = NaN;
					line_b = NaN;
				end
				if this.type == GammaArea.POLYELLIPSE || this.type == GammaArea.POLYELLIPSESQUARE
					if isfield(parameters, 'f')
						polyellipse_f = parameters.f;
					else
						polyellipse_f = NaN;
					end
					if isfield(parameters, 'w')
						polyellipse_w = parameters.w;
					else
						polyellipse_w = NaN;
					end
				else
					polyellipse_f = NaN;
					polyellipse_w = NaN;
				end
				if this.type == GammaArea.CUSTOM
					if isfield(parameters, 'fun')
						custom_fun = parameters.fun;
					else
						custom_fun = NaN;
					end
					if isfield(parameters, 'hasgrad')
						custom_hasgrad = parameters.hasgrad;
					else
						custom_hasgrad = false;
					end
					if isfield(parameters, 'hashess')
						custom_hashess = parameters.hashess;
					else
						custom_hashess = false;
					end
				else
					custom_fun = NaN;
					custom_hasgrad = false;
					custom_hashess = false;
				end
				functionstruct = struct(...
					'type',				this.type,...
					'reshift',			this.reshift,...
					'imshift',			this.imshift,...
					'circle_R',			circle_R,...
					'circlediscrete_R',	circlediscrete_R,...
					'ellipse_a',		ellipse_a,...
					'ellipse_b',		ellipse_b,...
					'hyperbola_a',		hyperbola_a,...
					'hyperbola_b',		hyperbola_b,...
					'line_a',			line_a,...
					'line_b',			line_b,...
					'logspiral_R',		logspiral_R,...
					'logspiral_k',		logspiral_k,...
					'polyellipse_f',	polyellipse_f,...
					'polyellipse_w',	polyellipse_w,...
					'custom_fun',		custom_fun,...
					'custom_hasgrad',	custom_hasgrad,...
					'custom_hashess',	custom_hashess...
				);
			else
				if isempty(this)
					functionstruct = control.design.gamma.area.GammaArea.PARAMETERPROTOTYPEEMPTY;
				else
					functionstruct = this.PARAMETERPROTOTYPEEMPTY;
				end
				temp = reshape(this, numel(this), 1);
				parfor ii = 1:length(temp)
					parameters = temp(ii).getparameters();
					if isfield(parameters, 'R')
						circle_R = parameters.R;
					else
						circle_R = NaN;
					end
					if temp(ii).type == GammaArea.ELLIPSE || temp(ii).type == GammaArea.ELLIPSESQUARE
						if isfield(parameters, 'a')
							ellipse_a = parameters.a;
						else
							ellipse_a = NaN;
						end
						if isfield(parameters, 'b')
							ellipse_b = parameters.b;
						else
							ellipse_b = NaN;
						end
					else
						ellipse_a = NaN;
						ellipse_b = NaN;
					end
					if temp(ii).type == GammaArea.LOGSPIRAL
						if isfield(parameters, 'k')
							logspiral_k = parameters.k;
						else
							logspiral_k = NaN;
						end
					else
						logspiral_k = NaN;
					end
					logspiral_R = circle_R;
					circlediscrete_R = circle_R;
					if temp(ii).type == GammaArea.HYPERBOLA || temp(ii).type == GammaArea.HYPERBOLASQUARE
						if isfield(parameters, 'a')
							hyperbola_a = parameters.a;
						else
							hyperbola_a = NaN;
						end
						if isfield(parameters, 'b')
							hyperbola_b = parameters.b;
						else
							hyperbola_b = NaN;
						end
					else
						hyperbola_a = NaN;
						hyperbola_b = NaN;
					end
					if temp(ii).type == GammaArea.LINE || temp(ii).type == GammaArea.IMAG
						if isfield(parameters, 'a')
							line_a = parameters.a;
						else
							line_a = NaN;
						end
						if isfield(parameters, 'b')
							line_b = parameters.b;
						else
							line_b = NaN;
						end
					else
						line_a = NaN;
						line_b = NaN;
					end
					if temp(ii).type == GammaArea.POLYELLIPSE || temp(ii).type == GammaArea.POLYELLIPSESQUARE
						if isfield(parameters, 'f')
							polyellipse_f = parameters.f;
						else
							polyellipse_f = NaN;
						end
						if isfield(parameters, 'w')
							polyellipse_w = parameters.w;
						else
							polyellipse_w = NaN;
						end
					else
						polyellipse_f = NaN;
						polyellipse_w = NaN;
					end
					if temp(ii).type == GammaArea.CUSTOM
						if isfield(parameters, 'fun')
							custom_fun = parameters.fun;
						else
							custom_fun = NaN;
						end
						if isfield(parameters, 'hasgrad')
							custom_hasgrad = parameters.hasgrad;
						else
							custom_hasgrad = false;
						end
						if isfield(parameters, 'hashess')
							custom_hashess = parameters.hashess;
						else
							custom_hashess = false;
						end
					else
						custom_fun = NaN;
						custom_hasgrad = false;
						custom_hashess = false;
					end
					functionstruct(ii) = struct(...
						'type',				temp(ii).type,...
						'reshift',			temp(ii).reshift,...
						'imshift',			temp(ii).imshift,...
						'circle_R',			circle_R,...
						'circlediscrete_R',	circlediscrete_R,...
						'ellipse_a',		ellipse_a,...
						'ellipse_b',		ellipse_b,...
						'hyperbola_a',		hyperbola_a,...
						'hyperbola_b',		hyperbola_b,...
						'line_a',			line_a,...
						'line_b',			line_b,...
						'logspiral_R',		logspiral_R,...
						'logspiral_k',		logspiral_k,...
						'polyellipse_f',	polyellipse_f,...
						'polyellipse_w',	polyellipse_w,...
						'custom_fun',		custom_fun,...
						'custom_hasgrad',	custom_hasgrad,...
						'custom_hashess',	custom_hashess...
					);
				end
				functionstruct = reshape(functionstruct, size(this));
			end
		end
			
		function [str] = getstring(this, format)
			%GETSTRING return string representation of object
			%	Input:
			%		this:		instance
			%		format:		format to use for printing numbers
			%	Output:
			%		str:		string representation of the instance
			if nargin <= 1
				format = this.NUMBERFORMAT;
			end
			if isscalar(this)
				str = this.getinstancestring(format);
			else
				temp = reshape(this, numel(this), 1);
				str = cell(size(temp, 1), 1);
				parfor ii = 1:length(temp)
					str{ii} = temp(ii).getinstancestring(format);
				end
				str = reshape(str, size(this));
			end
		end
		
		function [L, M, success] = toLMIregion(this)
			%TOLMIREGION convert area to LMI region that can be used by Matlab functions
			%	Input:
			%		this:		instance
			%	Output:
			%		L:			L = L^T real matrix of constant coefficients in the LMI, where the L(1, n) element contains a imaginary part indicating the dimension of the LMI variable
			%		M:			real matrix of linear coefficients in the LMI
			%		success:	indicator, if conversion to LMI was successful
			if isscalar(this)
				[L, M, success] = this.toinstanceLMIregion();
			else
				temp = reshape(this, numel(this), 1);
				success = false(size(temp, 1), 1);
				lmiL = cell(size(temp, 1), 1);
				lmiM = cell(size(temp, 1), 1);
				parfor ii = 1:length(temp)
					[lmiL{ii, 1}, lmiM{ii, 1}, success(ii, 1)] = temp(ii).toinstanceLMIregion();
				end
				if ~any(success)
					L = [];
					M = [];
					success = false;
					return;
				end
				lmiL(~success) = [];
				lmiM(~success) = [];
				L = blkdiag(lmiL{:});
				M = blkdiag(lmiM{:});
				success = reshape(success, size(this));
			end
		end
	end
end

function [x] = function_handle2unique(x)
	%FUNCTION_HANDLE2UNIQUE convert function handle to NaN for comparison with unique
	%	Input:
	%		x:	value to convert
	%	Output:
	%		x:	NaN if function handle else 0
	if isfunctionhandle(x)
		x = NaN;
	else
		x = 0;
	end
end
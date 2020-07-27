classdef InitialValue < handle
	%INITIALVALUE class for representing an initial value for gammasyn

	properties(Access=protected)
		% initial values
		x_0 = cell(0, 1);
	end

	properties(Access=protected, Transient)
		% inidicator, if initial value isnot numeric
		isspecial = false(0, 1);
		% calculated initial values
		x_0_calculated = cell(0, 1);
		% indicator, if initial values are calculated
		x_0_iscalculated = false(0, 1);
	end

	properties
		% indicator to ignore errors for non numeric initial values
		ignoreerror = false;
		% indicaotr, if cached initial values should be used
		usecache = true;
	end

	methods(Static=true)
		function [this] = loadobj(load)
			%LOADOBJ load object from file
			%	Input:
			%		load:	object as structure loaded from file
			%	Output:
			%		this:	instance
			if isstruct(load)
				if iscell(load.x_0)
					this = control.design.gamma.InitialValue(load.x_0{:});
				else
					this = control.design.gamma.InitialValue(load.x_0);
				end
			else
				this = load;
				this.x_0 = load.x_0;
				this.isspecial = cellfun(@isInitialValueElement, load.x_0, 'UniformOutput', true);
				this.x_0_calculated = cell(size(this.x_0, 1), 1);
				this.x_0_iscalculated = false(size(this.x_0, 1), 1);
			end
		end
	end

	methods(Access=protected)
		function [valid] = checkInitialValue(this, value)
			%CHECKINITIALVALUE check initial value for correct dimensions and type
			%	Input:
			%		this:	instance
			%		value:	iniial value to check
			%	Output:
			%		valid:	indicator, if value is valid
			if isnumeric(value) && ndims(value) <= 3
				valid = true;
				return;
			end
			if isa(value, 'control.design.gamma.InitialValueElement')
				valid = true;
				return;
			end
			if iscell(value)
				val = value(:);
				valid = false(size(value, 2), 1);
				for ii = 1:size(val, 1)
					valid(ii) = this.checkInitialValue(val{ii});
				end
				valid = all(valid(:));
			else
				valid = false;
			end
		end
	end

	methods
		function [this] = InitialValue(varargin)
			%INITIALVALUE create new initial value object
			%	Input:
			%		varargin:	variable number of numerical or object initial values
			%	Output:
			%		this:		instance
			this.add(varargin{:});
		end

		function [] = set.ignoreerror(this, ignoreerror)
			%IGNOREERROR setter for ignoreerror indicator
			%	Input:
			%		this:			instance
			%		ignoreerror:	indicator, if errors for non numeric initial values should be ignored
			if ~isscalar(ignoreerror) || ~islogical(ignoreerror)
				error('control:design:gamma:initial', 'Indicator for ignoring errors must be a scalar logical.');
			end
			this.ignoreerror = ignoreerror;
		end

		function [] = set.usecache(this, usecache)
			%USECACHE setter for usecache indicator
			%	Input:
			%		this:			instance
			%		usecache:		indicator, if already calculated initial values should be used
			if ~isscalar(usecache) || ~islogical(usecache)
				error('control:design:gamma:initial', 'Indicator for cache usage must be a scalar logical.');
			end
			this.usecache = usecache;
		end

		function [] = clearcache(this)
			%CLEARCACHE clear already calculated initial values
			%	Input:
			%		this:	instance
			this.x_0_calculated = cell(size(this.x_0_calculated, 1), 1);
			this.x_0_iscalculated = false(size(this.x_0_iscalculated, 1), 1);
		end

		function [] = add(this, varargin)
			%ADD add initial values
			%	Input:
			%		this:		instance
			%		varargin:	variable number of numeric and object initial values to add
			if nargin <= 1
				return;
			end
			points = cell(nargin - 1, 1);
			empty = false(size(points, 1), 1);
			multiplenumeric = ones(size(points, 1), 1);
			numericvalues = false(size(points, 1), 1);
			sizenumeric = zeros(size(points, 1), 2);
			for ii = 1:nargin - 1
				if this.checkInitialValue(varargin{ii})
					points(ii, 1) = varargin(ii);
					empty(ii, 1) = isempty(varargin{ii});
					if isnumeric(varargin{ii})
						numericvalues(ii, 1) = true;
						sz = size(varargin{ii});
						multiplenumeric(ii, 1) = size(varargin{ii}, 3);
						sizenumeric(ii, :) = [sz(1), sz(2)];
					end
				else
					error('control:design:gamma:initial', 'Initial value to add must be a valid initial value.');
				end
			end
			if any(empty(:))
				error('control:design:gamma:initial', 'Initial value must not be empty.');
			end
			if any(numericvalues(:))
				temp = sizenumeric(numericvalues, :);
				if any(temp(:, 1) ~= temp(1, 1))
					error('control:design:gamma:initial', 'Numerical initial values must have the same number of columns.');
				end
				if any(temp(:, 2) ~= temp(1, 2))
					error('control:design:gamma:initial', 'Numerical initial values must have the same number of rows.');
				end
				for ii = 1:size(points, 1)
					if numericvalues(ii, 1) && ~multiplenumeric(ii, 1)
						[R, K, F] = input_initial_value2RKF(points{ii, 1});
						points{ii, 1} = {
							R,	K,	F
						};
					end
				end
			end
			points(empty) = [];
			if any(multiplenumeric > 1)
				idx = 1 + cumsum(multiplenumeric).';
				idx = [
					[1 , idx(1:end - 1)];
					idx - 1;
				];
				temp = cell(sum(multiplenumeric), 1);
				for ii = 1:size(points, 1)
					var = points{ii, 1};
					if multiplenumeric(ii, 1) > 1
						temp(idx(1, ii):idx(2, ii)) = squeeze(mat2cell(var, size(var, 1), size(var, 2), ones(size(var, 3), 1)));
					else
						temp{idx(1, ii):idx(2, ii), 1} = var;
					end
				end
				points = temp;
			end
			special = cellfun(@isInitialValueElement, points, 'UniformOutput', true);
			if any(~special) && any(~this.isspecial)
				sample = this.x_0(this.isspecial);
				check = points(~special);
				for ii = 1:size(check, 1)
					if ndims(sample{1}) ~= ndims(check{ii, 1})
						error('control:design:gamma:initial', 'Numerical initial values must not have more than 2 dimensions.');
					end
					if any(size(sample{1}) ~= size(check{ii, 1}))
						error('control:design:gamma:initial', 'Sizes of initial values must be compatible.');
					end
				end
			end
			this.isspecial = [
				this.isspecial;
				special
			];
			this.x_0 = [
				this.x_0;
				points
			];
			this.x_0_calculated = [
				this.x_0_calculated;
				cell(size(points, 1), 1);
			];
			this.x_0_iscalculated = [
				this.x_0_iscalculated;
				false(size(points, 1), 1)
			];
		end

		function [] = remove(this, n)
			%REMOVE remove initial values from list
			%	Input:
			%		this:	instance
			%		n:		number of initial values to remove from end
			if ~isnumeric(n) || ~isscalar(n)
				error('control:design:gamma:initial', 'Number of initial values to remove must be a numeric scalar.');
			end
			if n > size(this.x_0) || n <= 0
				error('control:design:gamma:initial', 'Number of initial values to remove must be a positive number not exceeding the number of initial values.');
			end
			this.x_0(end - n + 1:end) = [];
			this.isspecial(end - n + 1:end) = [];
			this.x_0_calculated(end - n + 1:end) = [];
			this.x_0_iscalculated(end - n + 1:end) = [];
		end

		function [initialvalues] = get(this, idx, options, systems, areafun, weights, R_fixed, R_bounds, R_nonlin, varargin)
			%GET get initial values from list at specified indices
			%	Input:
			%		this:			instance
			%		idx:			indices of initial values to return
			%		options:		options for creatin of dependent initial values in list
			%		systems:		structure/cell array or matrix with dynamic systems to take into consideration
			%		areafun:		area border functions and gradients as cell array with number of system entries, each function returns a row vector with as much elements as area border functions are defined for the corresponding system
			%		weights:		weighting matrix with number of systems columns and number of pole area border functions rows
			%		R_fixed:		cell array with indicator matrix for gain elements that should be fixed and the values the fixed gains have, empty if no fixed elements are needed. For linear dependent gain components the first element is a 3D matrix with relation in the third dimension and the second element a vector of constant constraint elements
			%		R_bounds:		cell array with indicator matrix for gain elements that should be bounded and the values the gains are bounded by, empty if no bounded elements are needed. For linear dependent gain components the first element is a 3D matrix with relation in the third dimension and the second element a vector of upper bound constraint elements
			%		R_nonlin:		function pointer to a function of nonlinear inequality and equality constraints on gains with signature [c_R, ceq_R, c_K, ceq_K, c_F, ceq_F, gradc_R, gradceq_R, gradc_K, gradceq_K, gradc_F, gradceq_F] = constraint(R, K, F)
			%		varargin:		additional arguments needed for calculation
			%	Output:
			%		initialvalues:	cell array with 3D matrices of initial proportional, derivative and prefilter values
			x_init = this.x_0;
			special = this.isspecial;
			cache = this.x_0_calculated;
			cached = this.x_0_iscalculated;
			if islogical(idx) && size(x_init, 1) == length(idx)
				x_init = x_init(idx, 1);
				special = special(idx, 1);
				cache = cache(idx, 1);
				cached = cached(idx, 1);
				indices = idx;
			elseif isnumeric(idx)
				if all(idx(:) > 0) && all(idx(:) <= size(x_init, 1))
					tempR = floor(idx) == ceil(idx);
					if ~all(tempR(:))
						error('control:design:gamma:initial', 'Indices must be real positive values.');
					else
						indices = false(size(x_init, 1), 1);
						indices(idx(:)) = true;
						x_init = x_init(idx(:), 1);
						special = special(idx(:), 1);
						cache = cache(idx(:), 1);
						cached = cached(idx(:), 1);
					end
				else
					error('control:design:gamma:initial', 'Indices must be real positive values.');
				end
			elseif ischar(idx)
				if strcmpi(idx, 'first')
					indices = false(size(x_init, 1), 1);
					indices(1, 1) = true;
					x_init = x_init(1, 1);
					special = special(1, 1);
					cache = cache(1, 1);
					cached = cached(1, 1);
				elseif strcmpi(idx, 'last')
					indices = false(size(x_init, 1), 1);
					indices(end, 1) = true;
					x_init = x_init(end, 1);
					special = special(end, 1);
					cache = cache(end, 1);
					cached = cached(end, 1);
				else
					error('control:design:gamma:initial', 'Indices must be either ''first'' or ''last''.');
				end
			else
				error('control:design:gamma:initial', 'Indices must be real positive values, logicals or characters.');
			end
			if isempty(x_init) || isempty(special)
				initialvalues = [];
				return;
			end
			if any(special) && (this.usecache && any(~cached(special)))
				if nargin <= 6
					[R_fixed, K_fixed, F_fixed, RKF_fixed] = input_gain_constraint2RKF();
				else
					[R_fixed, K_fixed, F_fixed, RKF_fixed] = input_gain_constraint2RKF(R_fixed);
				end
				if nargin <= 10
					[R_bounds, K_bounds, F_bounds, RKF_bounds] = input_gain_constraint2RKF();
				else
					[R_bounds, K_bounds, F_bounds, RKF_bounds] = input_gain_constraint2RKF(R_bounds);
				end
				if nargin <= 7
					R_nonlin = [];
				end
				allowvarorder = true;
				allownegativeweight = true;
				[systems, areafun_strict, ~, weight_strict, ~, dimensions_strict, ~, ~, bounds, nonlcon] = checkandtransformargs(systems, areafun, weights, [], R_fixed, K_fixed, F_fixed, RKF_fixed, allowvarorder, allownegativeweight, R_bounds, K_bounds, F_bounds, RKF_bounds, R_nonlin);
			end
			x_init_special = x_init(special, 1);
			initspecial = cell(size(x_init_special, 1), 1);
			emptyR = false(size(x_init_special, 1), 1);
			emptyK = false(size(x_init_special, 1), 1);
			emptyF = false(size(x_init_special, 1), 1);
			multiplenumericR = ones(size(x_init_special, 1), 1);
			numericvaluesR = false(size(x_init_special, 1), 1);
			sizenumericR = zeros(size(x_init_special, 1), 2);
			multiplenumericK = ones(size(x_init_special, 1), 1);
			numericvaluesK = false(size(x_init_special, 1), 1);
			sizenumericK = zeros(size(x_init_special, 1), 2);
			multiplenumericF = ones(size(x_init_special, 1), 1);
			numericvaluesF = false(size(x_init_special, 1), 1);
			sizenumericF = zeros(size(x_init_special, 1), 2);
			numberofargs = nargin;
			haserror = true(size(x_init_special, 1), 1);
			errorid = cell(size(x_init_special, 1), 1);
			cachespecial = cache(special, 1);
			cachedspecial = cached(special, 1);
			if this.usecache && all(cached(special))
				haserror = false(size(x_init_special, 1), 1);
				numericvaluesR = true(size(x_init_special, 1), 1);
				numericvaluesK = true(size(x_init_special, 1), 1);
				numericvaluesF = true(size(x_init_special, 1), 1);
				initspecial = cachespecial;
				for ii = 1:size(initspecial, 1)
					multiplenumericR(ii, 1) = size(initspecial{ii, 1}{1}, 3);
					multiplenumericK(ii, 1) = size(initspecial{ii, 1}{2}, 3);
					multiplenumericF(ii, 1) = size(initspecial{ii, 1}{3}, 3);
					sizenumericR(ii, :) = [size(initspecial{ii, 1}{1}, 1), size(initspecial{ii, 1}{1}, 2)];
					sizenumericK(ii, :) = [size(initspecial{ii, 1}{2}, 1), size(initspecial{ii, 1}{2}, 2)];
					sizenumericF(ii, :) = [size(initspecial{ii, 1}{3}, 1), size(initspecial{ii, 1}{3}, 2)];
				end
			else
				usecachedvalues = this.usecache;
				parfor ii = 1:size(x_init_special, 1)
					if usecachedvalues && cachedspecial(ii, 1)
						val = cachespecial{ii, 1};
						haserror(ii, 1) = false;
					else
						tempR = x_init_special{ii, 1};
						fun = @tempR.get;
						val = [];
						try
							switch min([nargin(fun), numberofargs])
								case 0
									val = fun();
								case 1
									val = fun(options);
								case 2
									val = fun(options, systems);
								case 3
									val = fun(options, systems, areafun_strict);
								case 4
									val = fun(options, systems, areafun_strict, weight_strict);
								case 5
									val = fun(options, systems, areafun_strict, weight_strict, dimensions_strict);
								case 6
									val = fun(options, systems, areafun_strict, weight_strict, dimensions_strict, bounds);
								case 7
									val = fun(options, systems, areafun_strict, weight_strict, dimensions_strict, bounds, nonlcon);
								otherwise
									val = fun(options, systems, areafun_strict, weight_strict, dimensions_strict, bounds, nonlcon, varargin{:});
							end
							haserror(ii, 1) = false;
						catch e
							haserror(ii, 1) = true;
							errorid{ii, 1} = e;
						end
					end
					numericvaluesR(ii, 1) = isnumeric(val{1}); %#ok<PFOUS> is filtered with empty
					multiplenumericR(ii, 1) = size(val{1}, 3); %#ok<PFOUS> is filtered with empty
					sizenumericR(ii, :) = [size(val{1}, 1), size(val{1}, 2)]; %#ok<PFOUS> is filtered with empty
					emptyR(ii, 1) = isempty(val{1});
					numericvaluesK(ii, 1) = isnumeric(val{2}); %#ok<PFOUS> is filtered with empty
					multiplenumericK(ii, 1) = size(val{2}, 3); %#ok<PFOUS> is filtered with empty
					sizenumericK(ii, :) = [size(val{2}, 1), size(val{2}, 2)]; %#ok<PFOUS> is filtered with empty
					emptyK(ii, 1) = isempty(val{2});
					numericvaluesF(ii, 1) = isnumeric(val{3}); %#ok<PFOUS> is filtered with empty
					multiplenumericF(ii, 1) = size(val{3}, 3); %#ok<PFOUS> is filtered with empty
					sizenumericF(ii, :) = [size(val{3}, 1), size(val{3}, 2)]; %#ok<PFOUS> is filtered with empty
					emptyF(ii, 1) = isempty(val{3});
					if isrow(val)
						initspecial{ii, 1} = val; %#ok<PFOUS> is filtered with empty
					else
						initspecial{ii, 1} = val';
					end
				end
			end
			if this.ignoreerror
				empty = (emptyR & emptyK & emptyF) | haserror;
			else
				empty = (emptyR & emptyK & emptyF);
				if any(haserror(:))
					idx = find(haserror, 1, 'first');
					rethrow(errorid{idx, 1});
				end
			end
			removeemptyidx = 1:size(x_init, 1);
			removeemptyidx(~special) = [];
			x_init(removeemptyidx(empty), :) = [];
			special(removeemptyidx(empty), :) = [];
			numericvaluesR(empty, :) = [];
			numericvaluesK(empty, :) = [];
			numericvaluesF(empty, :) = [];
			multiplenumericR(empty, :) = [];
			multiplenumericK(empty, :) = [];
			multiplenumericF(empty, :) = [];
			sizenumericR(empty, :) = [];
			sizenumericK(empty, :) = [];
			sizenumericF(empty, :) = [];
			initspecial(emptyR, :) = [];
			if any(numericvaluesR(:))
				tempR = sizenumericR(numericvaluesR, :);
				if any(tempR(:, 1) ~= tempR(1, 1))
					if this.ignoreerror
					else
						error('control:design:gamma:initial', 'Sizes of proportional initial values must be compatible.');
					end
				end
				if any(tempR(:, 2) ~= tempR(1, 2))
					if this.ignoreerror
					else
						error('control:design:gamma:initial', 'Sizes of proportional initial values must be compatible.');
					end
				end
			end
			if any(numericvaluesK(:))
				tempK = sizenumericK(numericvaluesK, :);
				if any(tempK(:, 1) ~= tempK(1, 1))
					if this.ignoreerror
					else
						error('control:design:gamma:initial', 'Sizes of derivative initial values must be compatible.');
					end
				end
				if any(tempK(:, 2) ~= tempK(1, 2))
					if this.ignoreerror
					else
						error('control:design:gamma:initial', 'Sizes of derivative initial values must be compatible.');
					end
				end
			end
			if any(numericvaluesF(:))
				tempF = sizenumericF(numericvaluesF, :);
				if any(tempF(:, 1) ~= tempF(1, 1))
					if this.ignoreerror
					else
						error('control:design:gamma:initial', 'Sizes of prefilter initial values must be compatible.');
					end
				end
				if any(tempF(:, 2) ~= tempF(1, 2))
					if this.ignoreerror
					else
						error('control:design:gamma:initial', 'Sizes of prefilter initial values must be compatible.');
					end
				end
			end
			if any(multiplenumericR ~= multiplenumericK) || any(multiplenumericK ~= multiplenumericF)
				error('control:design:gamma:initial', 'Number of proportional, derivative and prefilter initial values must be equal.');
			end
			x_init_idx = 1:size(x_init, 1);
			x_init_numeric_idx = x_init_idx(~special);
			x_init_idx(~special) = [];
			x_init(x_init_idx) = initspecial;
			cacheidx = 1:size(this.x_0_calculated, 1);
			cacheidx(~indices) = [];
			this.x_0_calculated(cacheidx(x_init_idx)) = initspecial;
			this.x_0_iscalculated(cacheidx(x_init_idx)) = true;
			this.x_0_calculated(cacheidx(x_init_numeric_idx)) = x_init(~special);
			this.x_0_iscalculated(cacheidx(x_init_numeric_idx)) = true;
			% handle multiple values for special initial values
			if any(multiplenumericR > 1)
				numericR = ones(size(x_init, 1), 1);
				numericR(x_init_idx) = multiplenumericR;
				multiplenumericR = numericR;
				numericK = ones(size(x_init, 1), 1);
				numericK(x_init_idx) = multiplenumericK;
				multiplenumericK = numericK;
				numericF = ones(size(x_init, 1), 1);
				numericF(x_init_idx) = multiplenumericF;
				multiplenumericF = numericF;
				idx = 1 + cumsum(multiplenumericR).';
				idx = [
					[1 , idx(1:end - 1)];
					idx - 1;
				];
				tempR = cell(sum(multiplenumericR), 1);
				tempK = cell(sum(multiplenumericK), 1);
				tempF = cell(sum(multiplenumericF), 1);
				for ii = 1:size(x_init, 1)
					var = x_init{ii, 1};
					if multiplenumericR(ii, 1) > 1
						tempR(idx(1, ii):idx(2, ii)) = squeeze(mat2cell(var{1}, size(var{1}, 1), size(var{1}, 2), ones(size(var{1}, 3), 1)));
						tempK(idx(1, ii):idx(2, ii)) = squeeze(mat2cell(var{2}, size(var{2}, 1), size(var{2}, 2), ones(size(var{2}, 3), 1)));
						tempF(idx(1, ii):idx(2, ii)) = squeeze(mat2cell(var{3}, size(var{3}, 1), size(var{3}, 2), ones(size(var{3}, 3), 1)));
					else
						[tempR{idx(1, ii):idx(2, ii), 1}, tempK{idx(1, ii):idx(2, ii), 1}, tempF{idx(1, ii):idx(2, ii), 1}] = input_initial_value2RKF(var);
					end
				end
				x_init = [
					tempR,	tempK,	tempF
				];
			else
				tempR = cell(sum(multiplenumericR), 1);
				tempK = cell(sum(multiplenumericK), 1);
				tempF = cell(sum(multiplenumericF), 1);
				for ii = 1:size(x_init, 1)
					var = x_init{ii, 1};
					[tempR{ii, 1}, tempK{ii, 1}, tempF{ii, 1}] = input_initial_value2RKF(var);
				end
				x_init = [
					tempR,	tempK,	tempF
				];
			end
			% unwrap multiple numeric values
			multiplenumericR = ones(size(x_init, 1), 1);
			multiplenumericK = ones(size(x_init, 1), 1);
			multiplenumericF = ones(size(x_init, 1), 1);
			emptyR = false(size(x_init, 1), 1);
			emptyK = false(size(x_init, 1), 1);
			emptyF = false(size(x_init, 1), 1);
			usemultiple = ones(size(x_init, 1), 1);
			for ii = 1:size(x_init, 1)
				multiplenumericR(ii, 1) = size(x_init{ii, 1}, 3);
				multiplenumericK(ii, 1) = size(x_init{ii, 2}, 3);
				multiplenumericF(ii, 1) = size(x_init{ii, 3}, 3);
				emptyR(ii, 1) = isempty(x_init{ii, 1});
				emptyK(ii, 1) = isempty(x_init{ii, 2});
				emptyF(ii, 1) = isempty(x_init{ii, 3});
				if multiplenumericR(ii, 1) > 1
					if ~emptyK(ii, 1) && multiplenumericR(ii, 1) ~= multiplenumericK(ii, 1)
						error('control:design:gamma:initial', 'Number of proportional and derivative initial values must be equal.');
					end
					if ~emptyF(ii, 1) && multiplenumericR(ii, 1) ~= multiplenumericF(ii, 1)
						error('control:design:gamma:initial', 'Number of proportional and prefilter initial values must be equal.');
					end
					usemultiple(ii, 1) = multiplenumericR(ii, 1);
				end
				if multiplenumericK(ii, 1) > 1
					if ~emptyR(ii, 1) && multiplenumericK(ii, 1) ~= multiplenumericR(ii, 1)
						error('control:design:gamma:initial', 'Number of derivative and proportional initial values must be equal.');
					end
					if ~emptyF(ii, 1) && multiplenumericK(ii, 1) ~= multiplenumericF(ii, 1)
						error('control:design:gamma:initial', 'Number of derivative and prefilter initial values must be equal.');
					end
					usemultiple(ii, 1) = multiplenumericK(ii, 1);
				end
				if multiplenumericF(ii, 1) > 1
					if ~emptyR(ii, 1) && multiplenumericF(ii, 1) ~= multiplenumericR(ii, 1)
						error('control:design:gamma:initial', 'Number of proportional and prefilter initial values must be equal.');
					end
					if ~emptyK(ii, 1) && multiplenumericF(ii, 1) ~= multiplenumericK(ii, 1)
						error('control:design:gamma:initial', 'Number of prefilter and derivative initial values must be equal.');
					end
					usemultiple(ii, 1) = multiplenumericF(ii, 1);
				end
			end
			if any(usemultiple)
				idx = 1 + cumsum(multiplenumericR).';
				idx = [
					[1 , idx(1:end - 1)];
					idx - 1;
				];
				tempR = cell(sum(usemultiple), 1);
				tempK = cell(sum(usemultiple), 1);
				tempF = cell(sum(usemultiple), 1);
				for ii = 1:size(x_init, 1)
					var = x_init(ii, :);
					if usemultiple(ii, 1) > 1
						tempR(idx(1, ii):idx(2, ii)) = squeeze(mat2cell(var{1}, size(var{1}, 1), size(var{1}, 2), ones(size(var{1}, 3), 1)));
						tempK(idx(1, ii):idx(2, ii)) = squeeze(mat2cell(var{2}, size(var{2}, 1), size(var{2}, 2), ones(size(var{2}, 3), 1)));
						tempF(idx(1, ii):idx(2, ii)) = squeeze(mat2cell(var{3}, size(var{3}, 1), size(var{3}, 2), ones(size(var{3}, 3), 1)));
					else
						[tempR{idx(1, ii):idx(2, ii), 1}, tempK{idx(1, ii):idx(2, ii), 1}, tempF{idx(1, ii):idx(2, ii), 1}] = input_initial_value2RKF(var);
					end
				end
				x_init = [
					tempR,	tempK,	tempF
				];
			end
			% handle empty matrices
			sizenumericR = zeros(size(x_init, 1), 2);
			sizenumericK = zeros(size(x_init, 1), 2);
			sizenumericF = zeros(size(x_init, 1), 2);
			for ii = 1:size(x_init, 1)
				sizenumericR(ii, :) = [size(x_init{ii, 1}, 1), size(x_init{ii, 1}, 2)];
				sizenumericK(ii, :) = [size(x_init{ii, 2}, 1), size(x_init{ii, 2}, 2)];
				sizenumericF(ii, :) = [size(x_init{ii, 3}, 1), size(x_init{ii, 3}, 2)];
			end
			emptyR = sizenumericR == 0;
			if any(emptyR(:))
				maxdim = max(sizenumericR, [], 1);
				if any(emptyR(:, 1))
					if ~all(emptyR(:, 1))
						emptyRval = zeros(maxdim);
						x_init(emptyR(:, 1), 1) = {emptyRval};
						sizenumericR(emptyR(:, 1), :) = repmat(maxdim, sum(emptyR(:, 1)), 1);
					end
				end
				if any(emptyR(:, 2))
					if ~all(emptyR(:, 2))
						emptyRval = zeros(maxdim);
						x_init(emptyR(:, 2), 1) = {emptyRval};
						sizenumericR(emptyR(:, 2), :) = repmat(maxdim, sum(emptyR(:, 2)), 1);
					end
				end
			end
			emptyK = sizenumericK == 0;
			if any(emptyK(:))
				maxdim = max(sizenumericK, [], 1);
				if any(emptyK(:, 1))
					if ~all(emptyK(:, 1))
						emptyKval = zeros(maxdim);
						x_init(emptyK(:, 1), 2) = {emptyKval};
						sizenumericK(emptyK(:, 1), :) = repmat(maxdim, sum(emptyK(:, 1)), 1);
					end
				end
				if any(emptyK(:, 2))
					if ~all(emptyK(:, 2))
						emptyKval = zeros(maxdim);
						x_init(emptyK(:, 2), 2) = {emptyKval};
						sizenumericK(emptyK(:, 2), :) = repmat(maxdim, sum(emptyK(:, 2)), 1);
					end
				end
			end
			emptyF = sizenumericF == 0;
			if any(emptyF(:))
				maxdim = max(sizenumericF, [], 1);
				if any(emptyF(:, 1))
					if ~all(emptyF(:, 1))
						emptyFval = zeros(maxdim);
						x_init(emptyF(:, 1), 3) = {emptyFval};
						sizenumericF(emptyF(:, 1), :) = repmat(maxdim, sum(emptyF(:, 1)), 1);
					end
				end
				if any(emptyF(:, 2))
					if ~all(emptyF(:, 2))
						emptyFval = zeros(maxdim);
						x_init(emptyF(:, 2), 3) = {emptyFval};
						sizenumericF(emptyF(:, 2), :) = repmat(maxdim, sum(emptyF(:, 2)), 1);
					end
				end
			end
			% check sizes
			if any(sizenumericR(:, 1) ~= sizenumericR(1, 1))
				if this.ignoreerror
					[~, idx] = min(abs(sizenumericR(:, 1) - mean(sizenumericR(:, 1))));
					remove = sizenumericR(:, 1) ~= sizenumericR(idx, 1);
					x_init(remove, :) = [];
					sizenumericR(remove, :) = [];
				else
					error('control:design:gamma:initial', 'Sizes of proportional initial values must be compatible.');
				end
			end
			if any(sizenumericR(:, 2) ~= sizenumericR(1, 2))
				if this.ignoreerror
					[~, idx] = min(abs(sizenumericR(:, 2) - mean(sizenumericR(:, 2))));
					remove = sizenumericR(:, 2) ~= sizenumericR(idx, 2);
					x_init(remove, 1) = [];
					%sizenumericK(remove, :) = [];
				else
					error('control:design:gamma:initial', 'Sizes of proportional initial values must be compatible.');
				end
			end
			if any(sizenumericK(:, 1) ~= sizenumericK(1, 1))
				if this.ignoreerror
					[~, idx] = min(abs(sizenumericK(:, 1) - mean(sizenumericK(:, 1))));
					remove = sizenumericK(:, 1) ~= sizenumericK(idx, 1);
					x_init(remove, :) = [];
					sizenumericK(remove, :) = [];
				else
					error('control:design:gamma:initial', 'Sizes of derivative initial values must be compatible.');
				end
			end
			if any(sizenumericK(:, 2) ~= sizenumericK(1, 2))
				if this.ignoreerror
					[~, idx] = min(abs(sizenumericK(:, 2) - mean(sizenumericK(:, 2))));
					remove = sizenumericK(:, 2) ~= sizenumericK(idx, 2);
					x_init(remove, 1) = [];
					%sizenumericK(remove, :) = [];
				else
					error('control:design:gamma:initial', 'Sizes of derivative initial values must be compatible.');
				end
			end
			if any(sizenumericF(:, 1) ~= sizenumericF(1, 1))
				if this.ignoreerror
					[~, idx] = min(abs(sizenumericF(:, 1) - mean(sizenumericF(:, 1))));
					remove = sizenumericF(:, 1) ~= sizenumericF(idx, 1);
					x_init(remove, :) = [];
					sizenumericF(remove, :) = [];
				else
					error('control:design:gamma:initial', 'Sizes of prefilter initial values must be compatible.');
				end
			end
			if any(sizenumericF(:, 2) ~= sizenumericF(1, 2))
				if this.ignoreerror
					[~, idx] = min(abs(sizenumericF(:, 2) - mean(sizenumericF(:, 2))));
					remove = sizenumericF(:, 2) ~= sizenumericF(idx, 2);
					x_init(remove, 1) = [];
					%sizenumericF(remove, :) = [];
				else
					error('control:design:gamma:initial', 'Sizes of prefilter initial values must be compatible.');
				end
			end
			initialvalues = {
				cat(3, x_init{:, 1}),	cat(3, x_init{:, 2}),	cat(3, x_init{:, 3})
			};
		end

		function [initialvalues] = getall(this, options, systems, areafun, weights, R_fixed, R_bounds, R_nonlin, varargin)
			%GETALL get all initial values from list
			%	Input:
			%		this:			instance
			%		options:		options for creatin of dependent initial values in list
			%		systems:		structure/cell array or matrix with dynamic systems to take into consideration
			%		areafun:		area border functions and gradients as cell array with number of system entries, each function returns a row vector with as much elements as area border functions are defined for the corresponding system
			%		weights:		weighting matrix with number of systems columns and number of pole area border functions rows
			%		R_fixed:		cell array with indicator matrix for gain elements that should be fixed and the values the fixed gains have, empty if no fixed elements are needed. For linear dependent gain components the first element is a 3D matrix with relation in the third dimension and the second element a vector of constant constraint elements
			%		R_bounds:		cell array with indicator matrix for gain elements that should be bounded and the values the gains are bounded by, empty if no bounded elements are needed. For linear dependent gain components the first element is a 3D matrix with relation in the third dimension and the second element a vector of upper bound constraint elements
			%		R_nonlin:		function pointer to a function of nonlinear inequality and equality constraints on gains with signature [c_R, ceq_R, c_K, ceq_K, c_F, ceq_F, gradc_R, gradceq_R, gradc_K, gradceq_K, gradc_F, gradceq_F] = constraint(R, K, F)
			%		varargin:		additional arguments needed for calculation
			%	Output:
			%		initialvalues:	cell array with 3D matrices of initial proportional, derivative and prefilter values
			idx = 1:size(this.x_0, 1);
			switch nargin
				case 0
					error('control:design:gamma:initial', 'Not enough input arguments.');
				case 1
					initialvalues = this.get(idx);
				case 2
					initialvalues = this.get(idx, options);
				case 3
					initialvalues = this.get(idx, options, systems);
				case 4
					initialvalues = this.get(idx, options, systems, areafun);
				case 5
					initialvalues = this.get(idx, options, systems, areafun, weights);
				case 6
					initialvalues = this.get(idx, options, systems, areafun, weights, R_fixed);
				case 7
					initialvalues = this.get(idx, options, systems, areafun, weights, R_fixed, R_bounds);
				case 8
					initialvalues = this.get(idx, options, systems, areafun, weights, R_fixed, R_bounds, R_nonlin);
				otherwise
					initialvalues = this.get(idx, options, systems, areafun, weights, R_fixed, R_bounds, R_nonlin, varargin{:});
			end
		end
	end

end

function [is] = isInitialValueElement(element)
	%ISINITIALVALUEELEMENT return if a variable is an initial value element
	%	Input:
	%		element:	element to check
	%	Output:
	%		is:			true, if the element is an initial value element else false
	is = isa(element, 'control.design.gamma.InitialValueElement');
end
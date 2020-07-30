function [value, validvalue, errmsg, errid, validfield] = objectiveoptions_checkoption(options, field, value)
	%CHECKPROPERTY set a value for a solver option
	%	Input:
	%		options:		instance
	%		name:			name of option to set
	%		value:			value to set
	%		possValues:		possible values to set
	%		replacechar:	indicator, if char values should be replaced by an interpreted version
	%	Output:
	%		value:			value to set
	%		validvalue:		indicator, if value is valid
	%		errmsg:			error message, if value is not valid
	%		errid:			error identifier, if value is not valid
	if isempty(value) && isnumeric(value)
		validvalue = true;
		errmsg = '';
		errid = '';
		validfield = false;
		return;
	end

	% Some fields are checked in optimset/checkfield: Display, MaxFunEvals, MaxIter,
	% OutputFcn, TolFun, TolX. Some are checked in both (e.g., MaxFunEvals).
	validfield = true;
	switch field
		case {'type'}
			if ~isa(value, 'GammaJType')
				try
					value = GammaJType.fromchar(value);
					validvalue = true;
					errid = '';
					errmsg = '';
					if ~iscolumn(value)
						value = value';
					end
					if ~isempty(value) && size(value, 2) ~= 1
						validvalue = false;
						errid = 'control:design:gamma:input';
						errmsg = 'Objective option ''type'' must be a column vector.';
					end
				catch e
					validvalue = false;
					errid = 'control:design:gamma:input';
					errmsg = e.message;
				end
			else
				if ~iscolumn(value)
					value = value';
				end
				validvalue = true;
				errid = '';
				errmsg = '';
			end
		case {'weight'}
			if ~iscolumn(value)
				value = value';
			end
			if ~isnumeric(value)
				validvalue = false;
				errid = 'control:design:gamma:input';
				errmsg = 'Objective option ''weight'' must be numeric.';
			elseif ~isempty(value) && size(value, 2) ~= 1
				validvalue = false;
				errid = 'control:design:gamma:input';
				errmsg = 'Objective option ''weight'' must be a column vector.';
			else
				if any(imag(value) ~= 0)
					validvalue = false;
					errid = 'control:design:gamma:input';
					errmsg = 'Objective option ''weight'' must be not be complex.';
				else
					validvalue = true;
					errid = '';
					errmsg = '';
				end
			end
		case {'eigenvaluederivative'}
			if ~isa(value, 'GammaEigenvalueDerivativeType')
				try
					value = GammaEigenvalueDerivativeType.fromchar(value);
					validvalue = true;
					errid = '';
					errmsg = '';
				catch e
					validvalue = false;
					errid = 'control:design:gamma:input';
					errmsg = e.message;
				end
			else
				validvalue = true;
				errid = '';
				errmsg = '';
			end
			if ~isscalar(value)
				validvalue = false;
				errid = 'control:design:gamma:input';
				errmsg = 'Objective option ''eigenvaluederivative'' must be scalar.';
			end
		case {'eigenvaluefilter'}
			if ~isa(value, 'GammaEigenvalueFilterType')
				try
					value = GammaEigenvalueFilterType.extract(value);
					validvalue = true;
					errid = '';
					errmsg = '';
				catch e
					validvalue = false;
					errid = 'control:design:gamma:input';
					errmsg = e.message;
				end
			else
				validvalue = true;
				errid = '';
				errmsg = '';
			end
		case {'strategy'}
			if ~isa(value, 'GammaSolutionStrategy')
				try
					value = GammaSolutionStrategy.fromchar(value);
					validvalue = true;
					errid = '';
					errmsg = '';
				catch e
					validvalue = false;
					errid = 'control:design:gamma:input';
					errmsg = e.message;
				end
			else
				validvalue = true;
				errid = '';
				errmsg = '';
			end
			if ~isscalar(value)
				validvalue = false;
				errid = 'control:design:gamma:input';
				errmsg = 'Objective option ''strategy'' must be scalar.';
			end
		case {'errorhandler'}
			if ~isa(value, 'GammaErrorHandler')
				try
					value = GammaErrorHandler.fromchar(value);
					validvalue = true;
					errid = '';
					errmsg = '';
				catch e
					validvalue = false;
					errid = 'control:design:gamma:input';
					errmsg = e.message;
				end
			else
				validvalue = true;
				errid = '';
				errmsg = '';
			end
			if ~isscalar(value)
				validvalue = false;
				errid = 'control:design:gamma:input';
				errmsg = 'Objective option ''errorhandler'' must be scalar.';
			end
		case {'rho', 'max'}
			% real scalar or NaN
			if isscalar(value) && isnan(value)
				validvalue = true;
				errid = '';
				errmsg = '';
			else
				[validvalue, errmsg, errid] = boundedReal(field, value, [-Inf, Inf]);
			end
			if ~validvalue
				errid = 'control:design:gamma:input';
			end
		case {'Q'}
			% real matrix
			if isempty(value)
				if ~isnumeric(value)
					value = zeros(size(value));
				end
				validvalue = true;
				errid = '';
				errmsg = '';
			else
				[validvalue, errmsg, errid] = boundedNumericMatrix3d('Laypunov equation parameter Q', value);
			end
			if ~validvalue
				errid = 'control:design:gamma:input';
			end
		case {'R', 'K', 'F', 'R_shift', 'K_shift', 'F_shift'}
			% real matrix
			if isempty(value)
				if ~isnumeric(value)
					value = zeros(size(value));
				end
				validvalue = true;
				errid = '';
				errmsg = '';
			else
				if strcmpi(field, 'R')
					gain = 'Weight for proportional gain';
				elseif strcmpi(field, 'K')
					gain = 'Weight for derivative gain';
				elseif strcmpi(field, 'F')
					gain = 'Weight for prefilter gain';
				elseif strcmpi(field, 'R_shift')
					gain = 'Shift for proportional gain';
				elseif strcmpi(field, 'K_shift')
					gain = 'Shift for derivative gain';
				elseif strcmpi(field, 'F_shift')
					gain = 'Shift for prefilter gain';
				else
					gain = '';
				end
				[validvalue, errmsg, errid] = boundedNumericMatrix(gain, value);
			end
			if ~validvalue
				errid = 'control:design:gamma:input';
			end
		case {'usecompiled', 'allowvarorder', 'allownegativeweight', 'usereferences', 'usemeasurements_xdot', 'preventNaN', 'eigenvalueignoreinf', 'solvesymbolic'}
			if ~isscalar(value)
				validvalue = false;
				errmsg = sprintf('Value for option ''%s'' must be scalar.', field);
				errid = 'control:design:gamma:input';
			else
				if islogical(value)
					if value
						value = 'on';
					else
						value = 'off';
					end
				end
				[validvalue, errmsg, errid] = stringsType(field, value, {'on';'off'});
				if validvalue
					value = onofftological(value);
				end
			end
			if ~validvalue
				errid = 'control:design:gamma:input';
			end
		case {'numthreads'}
			% non-negative integer excluding inf or -1
			[validvalue, errmsg, errid] = boundedInteger(field, value, [-1, Inf]);
			if ~validvalue
				errid = 'control:design:gamma:input';
			end
		case {'round_equations_to_digits'}
			% integer excluding inf
			if isscalar(value) && isnan(value)
				validvalue = true;
				errid = '';
				errmsg = '';
			else
				[validvalue, errmsg, errid] = boundedInteger(field, value, [-Inf, Inf]);
			end
			if ~validvalue
				errid = 'control:design:gamma:input';
			end
		case {'errorhandler_function'}
			% function or empty
			if isempty(value)
				validvalue = true;
				errmsg = '';
				errid = '';
			else
				[validvalue, errmsg, errid] = functionType(field, value);
			end
			if ~validvalue
				errid = 'control:design:gamma:input';
			end
		case {'samples', 'Blocks'}
			% structure or empty
			if isempty(value)
				validvalue = true;
				errmsg = '';
				errid = '';
			else
				if isscalar(value) && isstruct(value)
					names = fieldnames(value);
					validvalue = true;
					errmsg = '';
					errid = '';
					for ii = 1:size(names, 1) %#ok<FORPF> no parfor because of break
						number = value.(names{ii, 1});
						if ~isnumeric(number)
							validvalue = false;
							errid = 'control:design:gamma:input';
							errmsg = sprintf('Number of samples for parameter ''%s'' must be numeric.', names{ii, 1});
							break;
						end
						if number < 0
							validvalue = false;
							errid = 'control:design:gamma:input';
							errmsg = sprintf('Number of samples for parameter ''%s'' must be nonnegative.', names{ii, 1});
							break;
						end
						if isinf(number)
							validvalue = false;
							errid = 'control:design:gamma:input';
							errmsg = sprintf('Number of samples for parameter ''%s'' must not be Inf.', names{ii, 1});
							break;
						end
					end
				else
					validvalue = false;
					errmsg = 'Number of samples must be supplied as scalar structure';
					errid = '';
				end
			end
			if ~validvalue
				errid = 'control:design:gamma:input';
			end
		case {'couplingconditions'}
			% non-negative integer excluding inf or -1
			[validvalue, errmsg, errid] = nonNegInteger(field, value);
			if ~validvalue
				errid = 'control:design:gamma:input';
			end
		case {'couplingstrategy'}
			if ~isa(value, 'GammaCouplingStrategy')
				try
					value = GammaCouplingStrategy.fromchar(value);
					validvalue = true;
					errid = '';
					errmsg = '';
				catch e
					validvalue = false;
					errid = 'control:design:gamma:input';
					errmsg = e.message;
				end
			else
				validvalue = true;
				errid = '';
				errmsg = '';
			end
			if ~isscalar(value)
				validvalue = false;
				errid = 'control:design:gamma:input';
				errmsg = 'Coupling option ''couplingstrategy'' must be scalar.';
			end
		case {'sortingstrategy_coupling'}
			if ~isa(value, 'GammaCouplingconditionSortingStrategy')
				try
					value = GammaCouplingconditionSortingStrategy.fromchar(value);
					validvalue = true;
					errid = '';
					errmsg = '';
				catch e
					validvalue = false;
					errid = 'control:design:gamma:input';
					errmsg = e.message;
				end
			else
				validvalue = true;
				errid = '';
				errmsg = '';
			end
			if ~isscalar(value)
				validvalue = false;
				errid = 'control:design:gamma:input';
				errmsg = 'Coupling option ''sortingstrategy_coupling'' must be scalar.';
			end
		case {'tolerance_coupling', 'tolerance_prefilter'}
			% non-negative real or empty or NaN
			if isempty(value)
				value = NaN;
			end
			wasnan = isnan(value);
			oldvalue = value;
			if wasnan
				value = zeros(size(value));
			end
			[validvalue, errmsg, errid] = nonNegReal(field, value);
			if wasnan
				value = oldvalue;
			end
			if ~validvalue
				errid = 'control:design:gamma:input';
			end
		case {'weight_coupling', 'weight_prefilter'}
			% non-negative real or empty
			oldvalue = value;
			if isempty(value)
				value = 1;
			end
			[validvalue, errmsg, errid] = nonNegReal(field, value);
			value = oldvalue;
			if ~validvalue
				errid = 'control:design:gamma:input';
			end
		otherwise
			% External users should not get here. We throw an error to remind
			% internal callers that they need to add new options to this
			% function.
			validfield = false;
			validvalue = false;
			errid = 'control:design:gamma:input';
			errmsg = sprintf('Unknown option ''%s''.', field);
	end
end

% TODO: remove optimization toolbox error messages and remove unneccessary functions

%-----------------------------------------------------------------------------------------
function [valid, errmsg, errid] = boundedNumericMatrix3d(field, value)
	valid = isnumeric(value) && ~islogical(value);
	errmsg = '';
	if ~valid
		errmsg = sprintf('%s must be numeric.', field);
	else
		valid = valid && ~any(isinf(value(:)));
		if ~valid
			errmsg = sprintf('%s must be finite.', field);
		else
			valid = valid && ndims(value) <= 3;
			if ~valid
				errmsg = sprintf('%s must be a matrix.', field);
			else
				valid = valid && all(imag(value(:)) == 0);
				if ~valid
					errmsg = sprintf('%s must be real.', field);
				end
			end
		end
	end
	if ~valid
		errid = 'control:design:gamma';
	else
		errid = '';
	end
end
function [valid, errmsg, errid] = boundedNumericMatrix(field, value)
	valid = isnumeric(value);
	errmsg = '';
	if ~valid
		errmsg = sprintf('%s must be numeric.', field);
	else
		valid = valid && ~any(isnan(value(:))) && ~any(isinf(value(:)));
		if ~valid
			errmsg = sprintf('%s must be finite.', field);
		else
			valid = valid && ndims(value) <= 2;
			if ~valid
				errmsg = sprintf('%s must be a matrix.', field);
			end
		end
	end
	if ~valid
		errid = 'control:design:gamma';
	else
		errid = '';
	end
end

function [valid, errmsg, errid] = nonNegReal(field,value,string)
	% Any nonnegative real scalar or sometimes a special string
	valid =  isreal(value) && isscalar(value) && (value >= 0) ;
	if nargin > 2
		valid = valid || isequal(value,string);
	end
	if ~valid
		if ischar(value)
			if isoptimtoolboxR2016B()
				msgid = 'MATLAB:optimfun:optimoptioncheckfield:nonNegRealStringType';
			else
				msgid = 'MATLAB:optimoptioncheckfield:nonNegRealStringType';
			end
			%errid = 'optimlib:options:checkfield:nonNegRealStringType';
		else
			if isoptimtoolboxR2016B()
				msgid = 'MATLAB:optimfun:optimoptioncheckfield:notAnonNegReal';
			else
				msgid = 'MATLAB:optimoptioncheckfield:notAnonNegReal';
			end
			%errid = 'optimlib:options:checkfield:notAnonNegReal';
		end
		errmsg = getString(message(msgid, field));
		errid = 'control:design:gamma:input';
	else
		errid = '';
		errmsg = '';
	end
end
%-----------------------------------------------------------------------------------------

function [valid, errmsg, errid] = nonNegInteger(field,value)
	% Any nonnegative real integer scalar or sometimes a special string
	valid =  isreal(value) && isscalar(value) && (value >= 0) && value == floor(value) ;
	if ~valid
		if isoptimtoolboxR2016B()
			msgid = 'MATLAB:optimfun:optimoptioncheckfield:notANonNegInteger';
		else
			msgid = 'MATLAB:optimoptioncheckfield:notANonNegInteger';
		end
		%errid = 'optimlib:options:checkfield:notANonNegInteger';
		errmsg = getString(message(msgid, field));
		errid = 'control:design:gamma:input';
	else
		errid = '';
		errmsg = '';
	end
end
%-----------------------------------------------------------------------------------------

function [valid, errmsg, errid] = boundedInteger(field,value,bounds)
	% Any positive real integer scalar or sometimes a special string
	valid = isnumeric(value) && isreal(value) && isscalar(value) && ...
		value == floor(value) && (value >= bounds(1)) && (value <= bounds(2));
	if ~valid
		errid = 'optimlib:options:checkfield:notABoundedInteger';
		errmsg = getString(message(errid, field, sprintf('[%6.3g, %6.3g]', bounds(1), bounds(2))));
		errid = 'control:design:gamma:input';
	else
		errid = '';
		errmsg = '';
	end
end
%--------------------------------------------------------------------------------

function [valid, errmsg, errid] = sameSignRange(field,value)
	% A two-element vector in ascending order; cannot mix positive and negative
	% numbers.
	valid = isnumeric(value) && isreal(value) && numel(value) == 2 && ...
		value(1) <= value(2) && (all(value>=0) || all(value<=0));
	if ~valid
		errid = 'optimlib:options:checkfield:notSameSignRange';
		errmsg = getString(message(errid, field));
		errid = 'control:design:gamma:input';
	else
		errid = '';
		errmsg = '';
	end
end
%-----------------------------------------------------------------------------------------

function [valid, errmsg, errid] = twoDimensionalMatrixType(field,value,strings)
	% Any matrix
	valid =  isa(value,'double') && ismatrix(value);
	if nargin > 2
		valid = valid || any(strcmp(value,strings));
	end
	if ~valid
		if ischar(value)
			errid = 'optimlib:options:checkfield:twoDimTypeStringType';
		else
			errid = 'optimlib:options:checkfield:notATwoDimMatrix';
		end
		errmsg = getString(message(errid, field));
		errid = 'control:design:gamma:input';
	else
		errid = '';
		errmsg = '';
	end
end
%-----------------------------------------------------------------------------------------

function [valid, errmsg, errid] = matrixType(field,value)
	% Any non-empty double (this "matrix" can have more 2 dimensions)
	valid = ~isempty(value) && ismatrix(value) && isa(value,'double');
	if ~valid
		if isoptimtoolboxR2016B()
			msgid = 'MATLAB:optimfun:optimoptioncheckfield:notAMatrix';
		else
			msgid = 'MATLAB:optimoptioncheckfield:notAMatrix';
		end
		%errid = 'optimlib:options:checkfield:notAMatrix';
		errmsg = getString(message(msgid, field));
		errid = 'control:design:gamma:input';
	else
		errid = '';
		errmsg = '';
	end
end
%-----------------------------------------------------------------------------------------

function [valid, errmsg, errid] = posVectorType(field,value)
	% Any non-empty positive scalar or all positive vector
	valid = ~isempty(value) && isa(value,'double') && isvector(value) && all(value > 0) ;
	if ~valid
		if isoptimtoolboxR2016B()
			msgid = 'MATLAB:optimfun:optimoptioncheckfield:notAPosMatrix';
		else
			msgid = 'MATLAB:optimoptioncheckfield:notAPosMatrix';
		end
		%errid = 'optimlib:options:checkfield:notAPosMatrix';
		errmsg = getString(message(msgid, field));
		errid = 'control:design:gamma:input';
	else
		errid = '';
		errmsg = '';
	end
end
%-----------------------------------------------------------------------------------------

function [valid, errmsg, errid] = rangeType(field,value)
	% A 2-row, double, all finite, non-empty array
	valid = isa(value,'double') && isempty(value) || ...
		(size(value,1) == 2) && all(isfinite(value(:)));
	if ~valid
		errid = 'optimlib:options:checkfield:notARange';
		errmsg = getString(message(errid, field));
		errid = 'control:design:gamma:input';
	else
		errid = '';
		errmsg = '';
	end
end
%-----------------------------------------------------------------------------------------

function [valid, errmsg, errid] = openRangeReal(field,value,range)
	% Any scalar
	valid = isscalar(value) && isa(value,'double') && ~isempty(value) && ...
		(value > range(1)) && (value < range(2));
	if ~valid
		errid = 'optimlib:options:checkfield:notInAnOpenRangeReal';
		errmsg = getString(message(errid, field, sprintf('%6.3g',range(1)), sprintf('%6.3g',range(2))));
		errid = 'control:design:gamma:input';
	else
		errid = '';
		errmsg = '';
	end
end
%-----------------------------------------------------------------------------------------

function [valid, errmsg, errid] = nonNegIntegerVector(field,value)
	% A vector of positive integers
	valid = isnumeric(value) && isvector(value) && all(value >= 0) && ...
		all(round(value) - value == 0);
	if ~valid
		if isoptimtoolboxR2016B()
			msgid = 'MATLAB:optimfun:optimoptioncheckfield:notANonNegIntVector';
		else
			msgid = 'optimlib:options:checkfield:notANonNegIntVector';
		end
		%errid = 'optimlib:options:checkfield:notANonNegIntVector';
		errmsg = getString(message(msgid, field));
		errid = 'control:design:gamma:input';
	else
		errid = '';
		errmsg = '';
	end
end
%-----------------------------------------------------------------------------------------

function [valid, errmsg, errid] = logicalType(field,value)
	% Any function handle or string (we do not test if the string is a function name)
	valid =  isscalar(value) && islogical(value);
	if ~valid
		if isoptimtoolboxR2016B()
			msgid = 'MATLAB:optimfun:optimoptioncheckfield:NotLogicalScalar';
		else
			msgid = 'MATLAB:optimoptioncheckfield:NotLogicalScalar';
		end
		%errid = 'optimlib:options:checkfield:NotLogicalScalar';
		errmsg = getString(message(msgid, field));
		errid = 'control:design:gamma:input';
	else
		errid = '';
		errmsg = '';
	end
end
%-----------------------------------------------------------------------------------------

function [valid, errmsg, errid] = functionType(field,value)
	% Any function handle or string (we do not test if the string is a function name)
	valid =  ischar(value) || isa(value, 'function_handle');
	if ~valid
		if isoptimtoolboxR2016B()
			msgid = 'MATLAB:optimfun:optimoptioncheckfield:notAFunction';
		else
			msgid = 'MATLAB:optimoptioncheckfield:notAFunction';
		end
		%errid = 'optimlib:options:checkfield:notAFunction';
		errmsg = getString(message(msgid, field));
		errid = 'control:design:gamma:input';
	else
		errid = '';
		errmsg = '';
	end
end
%-----------------------------------------------------------------------------------------
function [valid, errmsg, errid] = stringsType(field,value,strings)
	% One of the strings in cell array strings
	valid =  ischar(value) && any(strcmpi(value,strings));

	if ~valid
		% Format strings for error message
		allstrings = formatCellArrayOfStrings(strings);

		if isoptimtoolboxR2016B()
			msgid = 'MATLAB:optimfun:optimoptioncheckfield:notAStringsType';
		else
			msgid = 'MATLAB:optimoptioncheckfield:notAStringsType';
		end
		%errid = 'optimlib:options:checkfield:notAStringsType';
		errmsg = getString(message(msgid, field, allstrings));
		errid = 'control:design:gamma:input';
	else
		errid = '';
		errmsg = '';
	end
end
%-----------------------------------------------------------------------------------------
function [valid, errmsg, errid] = boundedReal(field,value,bounds)
	% Scalar in the bounds
	valid =  isa(value,'double') && isscalar(value) && ...
		(value >= bounds(1)) && (value <= bounds(2));
	if ~valid
		if isoptimtoolboxR2016B()
			msgid = 'MATLAB:optimfun:optimoptioncheckfield:notAboundedReal';
		else
			msgid = 'MATLAB:optimoptioncheckfield:notAboundedReal';
		end
		%errid = 'optimlib:options:checkfield:notAboundedReal';
		errmsg = getString(message(msgid, field, sprintf('[%6.3g, %6.3g]', bounds(1), bounds(2))));
		errid = 'control:design:gamma:input';
	else
		errid = '';
		errmsg = '';
	end
end
%-----------------------------------------------------------------------------------------
function [valid, errmsg, errid] = stringPosIntegerCellType(field,value,strings)
	% A cell array that is either {strings,positive integer} or {strings}
	valid = numel(value) == 1 && any(strcmp(value{1},strings)) || numel(value) == 2 && ...
		any(strcmp(value{1},strings)) && isreal(value{2}) && isscalar(value{2}) && value{2} > 0 && value{2} == floor(value{2});

	if ~valid
		if isoptimtoolboxR2016B()
			msgid = 'MATLAB:optimfun:optimoptioncheckfield:notAStringPosIntegerCellType';
		else
			msgid = 'MATLAB:optimoptioncheckfield:notAStringPosIntegerCellType';
		end
		%errid = 'optimlib:options:checkfield:notAStringPosIntegerCellType';
		errmsg = getString(message(msgid, field, strings));
		errid = 'control:design:gamma:input';
	else
		errid = '';
		errmsg = '';
	end
end
%-----------------------------------------------------------------------------------------
function [valid, errmsg, errid] = stringPosRealCellType(field,value,strings)
	% A cell array that is either {strings,positive real} or {strings}
	valid = (numel(value) >= 1) && any(strcmpi(value{1},strings));
	if (numel(value) == 2)
		valid = valid && isreal(value{2}) && (value{2} >= 0);
	end

	if ~valid
		% Format strings for error message
		allstrings = formatCellArrayOfStrings(strings);

		if isoptimtoolboxR2016B()
			msgid = 'MATLAB:optimfun:optimoptioncheckfield:notAStringPosRealCellType';
		else
			msgid = 'MATLAB:optimoptioncheckfield:notAStringPosRealCellType';
		end
		%errid = 'optimlib:options:checkfield:notAStringPosRealCellType';
		errmsg = getString(message(msgid, field,allstrings));
		errid = 'control:design:gamma:input';
	else
		errid = '';
		errmsg = '';
	end
end
%-----------------------------------------------------------------------------------------
function [valid, errmsg, errid] = posReal(field,value)
	% Any positive real scalar or sometimes a special string
	valid =  isnumeric(value) && isreal(value) && isscalar(value) && (value > 0) ;
	if ~valid
		if isoptimtoolboxR2016B()
			msgid = 'MATLAB:optimfun:optimoptioncheckfield:nonPositiveNum';
		else
			msgid = 'MATLAB:optimoptioncheckfield:nonPositiveNum';
		end
		%errid = 'optimlib:options:checkfield:nonPositiveNum';
		errmsg = getString(message(msgid, field));
		errid = 'control:design:gamma:input';
	else
		errid = '';
		errmsg = '';
	end
end
%-----------------------------------------------------------------------------------------

function [valid, errmsg, errid] = posInteger(field,value)
	% Any positive real scalar or sometimes a special string
	valid =  isnumeric(value) && isreal(value) && isscalar(value) && ...
		(value > 0) && value == floor(value);
	if ~valid
		errid = 'optimlib:options:checkfield:nonPositiveInteger';
		errmsg = getString(message(errid, field));
		errid = 'control:design:gamma:input';
	else
		errid = '';
		errmsg = '';
	end
end
%-----------------------------------------------------------------------------------------

function [valid, errmsg, errid] = realLessThanPlusInf(field,value,string)
	% Any real scalar that is less than +Inf, or sometimes a special string
	valid =  isnumeric(value) && isreal(value) && isscalar(value) && (value < +Inf);
	if nargin > 2
		valid = valid || strcmpi(value,string);
	end
	if ~valid
		if ischar(value)
			if isoptimtoolboxR2016B()
				msgid = 'MATLAB:optimfun:optimoptioncheckfield:realLessThanPlusInfStringType';
			else
				msgid = 'MATLAB:optimoptioncheckfield:realLessThanPlusInfStringType';
			end
			%errid = 'optimlib:options:checkfield:realLessThanPlusInfStringType';
		else
			if isoptimtoolboxR2016B()
				msgid = 'MATLAB:optimfun:optimoptioncheckfield:PlusInfReal';
			else
				msgid = 'MATLAB:optimoptioncheckfield:PlusInfReal';
			end
			%errid = 'optimlib:options:checkfield:PlusInfReal';
		end
		errmsg = getString(message(msgid, field));
		errid = 'control:design:gamma:input';
	else
		errid = '';
		errmsg = '';
	end
end
%-----------------------------------------------------------------------------------------

function [valid, errmsg, errid] = realGreaterThanMinusInf(field,value)
	% Any real scalar that is greater than -Inf
	valid =  isnumeric(value) && isreal(value) && isscalar(value) && (value > -Inf);
	if ~valid
		errid = 'optimlib:options:checkfield:minusInfReal';
		errmsg = getString(message(errid, field));
		errid = 'control:design:gamma:input';
	else
		errid = '';
		errmsg = '';
	end
end
%---------------------------------------------------------------------------------
function allstrings = formatCellArrayOfStrings(strings)
	%formatCellArrayOfStrings converts cell array of strings "strings" into an
	% array of strings "allstrings", with correct punctuation and "or"
	% depending on how many strings there are, in order to create readable
	% error message.

	% To print out the error message beautifully, need to get the commas and
	% "or"s in all the correct places while building up the string of possible
	% string values.

	% Add quotes around each string in the cell array
	strings = cellfun(@(x) sprintf('''%s''', x), strings, 'UniformOutput', false);

	% Create comma separated list from cell array. Note that strjoin requires
	% the cell array to be a 1xN row vector.
	allstrings = strjoin(strings(:)', ', ');

	% Replace last comma with ', or ' or ' or ' depending on the length of the
	% list. If there is only one string then there is no string match and 'or'
	% is not inserted into the string.
	numStrings = length(strings);
	if numStrings > 2
		finalConjunction = ', or ';
	elseif numStrings == 2
		finalConjunction = ' or ';
	else
		% For one string, there is no comma. The following call to regexprep
		% does nothing in this case. As such, we can set finalConjunction
		% arbitrarily to an empty string.
		%finalConjunction = '';
		return;
	end
	allstrings = regexprep(allstrings, ', ', finalConjunction, numStrings-1);
end
%--------------------------------------------------------------------------------

function [valid, errmsg, errid] = functionOrCellArray(field,value)
	% Any function handle, string or cell array of functions
	valid = ischar(value) || isa(value, 'function_handle') || iscell(value);
	if ~valid
		if isoptimtoolboxR2016B()
			msgid = 'MATLAB:optimfun:optimset:notAFunctionOrCellArray';
		else
			msgid = 'MATLAB:optimset:notAFunctionOrCellArray';
		end
		%errid = 'optimlib:options:checkfield:notAFunctionOrCellArray';
		errmsg = getString(message(msgid, field));
		errid = 'control:design:gamma:input';
	else
		errid = '';
		errmsg = '';
	end
end
%---------------------------------------------------------------------------------
function [is] = isoptimtoolboxR2016B()
	%ISOPTIMTOOLBOXR2016B return if the optimization toolbox has a version above of 2016B
	%	Output:
	%		is:	true, if the matlab version is newer than 2016B, else false
	persistent is2016B;
	if isempty(is2016B)
		is2016B = matlab.Version.CURRENT >= matlab.Version.R2016B;
	end
	is = is2016B;
end
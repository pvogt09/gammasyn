function [areaval, areaval_derivative, areaval_2_derivative] = calculate_areas(areafun, weight, eigenvalues, dimensions, options)
	%CALCULATE_AREAS helper function for calculation of border function, gradient and hessian values for gamma pole placement
	%	Input:
	%		areafun:				border functions as cell array of function handles or matrix of GammaArea objects
	%		weight:					weighting matrix with number of systems columns and number of pole area border functions rows
	%		eigenvalues:			eigenvalues used of all systems
	%		dimensions:				structure with information about dimensions of the different variables and systems
	%		options:				structure with options for objective function
	%	Output:
	%		areaval:				area border function value for current optimization value
	%		areaval_derivative:		gradient of area border function for current optimization value
	%		areaval_2_derivative:	hessian of area border function for current optimization value
	usecompiled = false;
	numthreads = configuration.matlab.numthreads();
	if nargin >= 5
		if islogical(options)
			usecompiled = options;
		elseif isstruct(options) && isfield(options, 'usecompiled')
			usecompiled = options.usecompiled;
			if isfield(options, 'numthreads')
				numthreads = options.numthreads;
			end
		elseif isstruct(options) && isfield(options, 'numthreads')
			numthreads = options.numthreads;
			if isfield(options, 'usecompiled')
				usecompiled = options.usecompiled;
			end
		elseif isnumeric(options)
			numthreads = options;
		end
	end
	numthreads = uint32(floor(max([0, numthreads])));
	if iscell(areafun)
		if nargout >= 3
			[areaval, areaval_derivative, areaval_2_derivative] = calculate_areas_handle(areafun, weight, eigenvalues, dimensions, numthreads);
		elseif nargout >= 2
			[areaval, areaval_derivative] = calculate_areas_handle(areafun, weight, eigenvalues, dimensions, numthreads);
		else
			areaval = calculate_areas_handle(areafun, weight, eigenvalues, dimensions, numthreads);
		end
	else
		if ndims(areafun) ~= ndims(dimensions.area_parameters) || any(size(areafun) ~= size(dimensions.area_parameters))
			error('control:design:gamma:area', 'Not enough parameters for all pole areas.');
		end
		if nargout >= 3
			if nargin >= 5
				[areaval, areaval_derivative, areaval_2_derivative] = calculate_areas_fixed(areafun, weight, eigenvalues, dimensions, usecompiled, numthreads);
			else
				[areaval, areaval_derivative, areaval_2_derivative] = calculate_areas_fixed(areafun, weight, eigenvalues, dimensions, numthreads);
			end
		elseif nargout >= 2
			if nargin >= 5
				[areaval, areaval_derivative] = calculate_areas_fixed(areafun, weight, eigenvalues, dimensions, usecompiled, numthreads);
			else
				[areaval, areaval_derivative] = calculate_areas_fixed(areafun, weight, eigenvalues, dimensions, numthreads);
			end
		else
			if nargin >= 5
				areaval = calculate_areas_fixed(areafun, weight, eigenvalues, dimensions, usecompiled, numthreads);
			else
				areaval = calculate_areas_fixed(areafun, weight, eigenvalues, dimensions, numthreads);
			end
		end
	end
end
function [areaval, areaval_derivative, areaval_2_derivative] = calculate_areas_fixed(areafun, weight, eigenvalues, dimensions, usecompiled, numthreads)
	%CALCULATE_AREAS_FIXED helper function for calculation of border function, gradient and hessian values for gamma pole placement for use with generated code
	%	Input:
	%		areafun:				border functions as cell array of function handles or matrix of GammaArea objects
	%		weight:					weighting matrix with number of systems columns and number of pole area border functions rows
	%		eigenvalues:			eigenvalues used of all systems
	%		dimensions:				structure with information about dimensions of the different variables and systems
	%		numthreads:				number of threads to run loops in
	%	Output:
	%		areaval:				area border function value for current optimization value
	%		areaval_derivative:		gradient of area border function for current optimization value
	%		areaval_2_derivative:	hessian of area border function for current optimization value
	if nargin <= 4
		usecompiled = false;
	end
	if nargin <= 5
		numthreads = configuration.matlab.numthreads();
	end
	numthreads = uint32(floor(max([0, numthreads])));
	if usecompiled
		if nargout >= 3
			[areaval, areaval_derivative, areaval_2_derivative] = calculate_areas_fixed_mex(areafun, weight, eigenvalues, dimensions, numthreads);
		elseif nargout >= 2
			[areaval, areaval_derivative] = calculate_areas_fixed_mex(areafun, weight, eigenvalues, dimensions, numthreads);
		else
			areaval = calculate_areas_fixed_mex(areafun, weight, eigenvalues, dimensions, numthreads);
		end
	else
		if nargout >= 3
			[areaval, areaval_derivative, areaval_2_derivative] = calculate_areas_fixed_m(areafun, weight, eigenvalues, dimensions, numthreads);
		elseif nargout >= 2
			[areaval, areaval_derivative] = calculate_areas_fixed_m(areafun, weight, eigenvalues, dimensions, numthreads);
		else
			areaval = calculate_areas_fixed_m(areafun, weight, eigenvalues, dimensions, numthreads);
		end
	end
end
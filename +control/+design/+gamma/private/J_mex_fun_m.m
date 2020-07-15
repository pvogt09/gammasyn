function [J] = J_mex_fun_m(x, system, weight, areafun, dimensions, options)
	%J_MEX_FUN_M calculate objective function for gamma pole placement (wrapper for J_mex_m for different number of output arguments)
	%	Input:
	%		x:			current optimization value (gain reshaped as column vector)
	%		system:		structure with system matrices of systems to take into consideration
	%		weight:		weighting matrix with number of systems columns and number of pole area border functions rows
	%		areafun:	area border functions and gradients as cell array with number of system entries, each function returns a row vector with as much elements as area border functions are defined for the corresponding system
	%		options:				structure with options for objective function
	%		dimensions:	structure with information about dimensions of the different variables and systems
	%	Output:
	%		J:			objective function value for current optimization value
	J = J_mex_m(x, system, weight, areafun, dimensions, options);
end
function [c, ceq] = c_mex_fun_m(x, system, weight, areafun, dimensions, options)
	%C_MEX_FUN_M calculate constraint function for gamma pole placement
	%	Input:
	%		x:			current optimization value (gain reshaped as column vector)
	%		system:		structure with system matrices of systems to take into consideration
	%		weight:		weighting matrix with number of systems columns and number of pole area border functions rows
	%		areafun:	area border functions and gradients as cell array with number of system entries, each function returns a row vector with as much elements as area border functions are defined for the corresponding system
	%		dimensions:	structure with information about dimensions of the different variables and systems
	%		options:	structure with options for objective function
	%	Output:
	%		c:			inequality constraint function value for current optimization value
	%		ceq:		equality constraint function value for current optimization value
	[c, ceq] = c_mex_m(x, system, weight, areafun, dimensions, options);
end
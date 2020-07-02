function [J, gradJ] = J_mex_grad_m(x, system, weight, areafun, dimensions, options)
	%J_MEX_GRAD_M calculate objective function and gradient for gamma pole placement (wrapper for J_mex_m for different number of output arguments)
	%	Input:
	%		x:			current optimization value (gain reshaped as column vector)
	%		system:		structure with system matrices of systems to take into consideration
	%		weight:		weighting matrix with number of systems columns and number of pole area border functions rows
	%		areafun:	area border functions and gradients as cell array with number of system entries, each function returns a row vector with as much elements as area border functions are defined for the corresponding system
	%		options:				structure with options for objective function
	%		dimensions:	structure with information about dimensions of the different variables and systems
	%	Output:
	%		J:			objective function value for current optimization value
	%		gradJ:		gradient of objective function value for current optimization value
	if nargout >= 2 && ~dimensions.area_hasgrad
		error('control:design:gamma:gradient', 'Gradient for polearea functions was not supplied.');
	end
	if nargout >= 2
		[J, gradJ] = J_mex_m(x, system, weight, areafun, dimensions, options);
	else
		J = J_mex_m(x, system, weight, areafun, dimensions, options);
	end
end

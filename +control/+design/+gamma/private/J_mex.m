function [J, gradJ, hessianJ] = J_mex(x, system, weight, areafun, dimensions, options, usecompiled)
	%J_MEX calculate objective function and gradient for gamma pole placement
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
	%		hessianJ:	hessian of objective function value for current optimization value
	if nargin <= 6
		usecompiled = options.usecompiled;
	end
	if nargout >= 2 && ~dimensions.area_hasgrad
		error('control:design:gamma:gradient', 'Gradient for polearea functions was not supplied.');
	end
	if nargout >= 3 && ~dimensions.area_hashess
		error('control:design:gamma:hessian', 'Hessian for polearea functions was not supplied.');
	end
	if nargout >= 3
		if usecompiled
			% J_mex_hess_mex is needed for different number of output arguments of c_mex_m in generated code
			[J, gradJ, hessianJ] = J_mex_hess_mex(x, system, weight, areafun, dimensions, options);
		else
			[J, gradJ, hessianJ] = J_mex_m(x, system, weight, areafun, dimensions, options);
		end
	elseif nargout >= 2
		if usecompiled
			% J_mex_grad_mex is needed for different number of output arguments of c_mex_m in generated code
			[J, gradJ] = J_mex_grad_mex(x, system, weight, areafun, dimensions, options);
		else
			[J, gradJ] = J_mex_m(x, system, weight, areafun, dimensions, options);
		end
	else
		if usecompiled
			% J_mex_fun_mex is needed for different number of output arguments of c_mex_m in generated code
			J = J_mex_fun_mex(x, system, weight, areafun, dimensions, options);
		else
			J = J_mex_m(x, system, weight, areafun, dimensions, options);
		end
	end
end
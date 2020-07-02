function [c, ceq, gradc, gradceq, hessc, hessceq] = c_mex(x, system, weight, areafun, dimensions, options, usecompiled)
	%C_MEX calculate constraint function and gradient for gamma pole placement
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
	%		gradc:		gradient of inequality constraint function value for current optimization value
	%		gradceq:	gradient of equality constraint function value for current optimization value
	%		hessc:		hessian of inequality constraint function value for current optimization value
	%		hessceq:	hessian of equality constraint function value for current optimization value
	if nargin <= 6
		usecompiled = options.usecompiled;
	end
	if nargout >= 3 && ~dimensions.area_hasgrad
		error('control:design:gamma:gradient', 'Gradient for polearea functions was not supplied.');
	end
	if nargout >= 5 && ~dimensions.area_hashess
		error('control:design:gamma:hessian', 'Hessian for polearea functions was not supplied.');
	end
	if nargout >= 5
		if usecompiled
			% c_mex_hess_mex is needed for different number of output arguments of c_mex_m in generated code
			[c, ceq, gradc, gradceq, hessc, hessceq] = c_mex_hess_mex(x, system, weight, areafun, dimensions, options);
		else
			[c, ceq, gradc, gradceq, hessc, hessceq] = c_mex_m(x, system, weight, areafun, dimensions, options);
		end
	elseif nargout >= 3
		if usecompiled
			% c_mex_grad_mex is needed for different number of output arguments of c_mex_m in generated code
			[c, ceq, gradc, gradceq] = c_mex_grad_mex(x, system, weight, areafun, dimensions, options);
		else
			[c, ceq, gradc, gradceq] = c_mex_m(x, system, weight, areafun, dimensions, options);
		end
	else
		if usecompiled
			% c_mex_fun_mex is needed for different number of output arguments of c_mex_m in generated code
			[c, ceq] = c_mex_fun_mex(x, system, weight, areafun, dimensions, options);
		else
			[c, ceq] = c_mex_m(x, system, weight, areafun, dimensions, options);
		end
	end
end
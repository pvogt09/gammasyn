function [hessian] = callback_hessian(x, sigma, lambda)
	%CALLBACK_HESSIAN callback function for hessian function handle for ipopt
	%	Input:
	%		x:			current optimization variable
	%	Output:
	%		hessian:	hessian function value
	hessian = callback(x, 'hess', sigma, lambda);
end
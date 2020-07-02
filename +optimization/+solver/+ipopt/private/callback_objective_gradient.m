function [gradJ] = callback_objective_gradient(x)
	%CALLBACK_OBJECTIVE_GRADIENT callback function for objective function handle for ipopt
	%	Input:
	%		x:			current optimization variable
	%	Output:
	%		gradJ:		function value of objective function gradient
	gradJ = callback(x, 'gradfun');
end
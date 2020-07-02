function [gradconstr] = callback_constraint_gradient(x)
	%CALLBACK_CONSTRAINT_GRADIENT callback function for contraint gradient function handle for ipopt
	%	Input:
	%		x:			current optimization variable
	%	Output:
	%		gradconstr:	constraint gradient function value
	gradconstr = sparse(callback(x, 'gradcon'));
end
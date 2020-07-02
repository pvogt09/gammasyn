function [gradconstr] = callback_constraint_gradient_structure()
	%CALLBACK_CONSTRAINT_GRADIENT_STRUCTURE callback function for contraint gradient structure function handle for ipopt
	%	Input:
	%		x:			current optimization variable
	%	Output:
	%		gradconstr:	constraint gradient function value
	gradconstr = callback([], 'gradconstruct');
end
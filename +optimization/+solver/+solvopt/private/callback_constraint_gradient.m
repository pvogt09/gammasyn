function [gradconstr] = callback_constraint_gradient(x)
	%CALLBACK_CONSTRAINT_GRADIENT callback function for contraint gradient function handle for solvopt
	%	Input:
	%		x:			current optimization variable
	%	Output:
	%		gradconstr:	constraint gradient function value
	gradconstr = callback(x, 'gradcon');
end
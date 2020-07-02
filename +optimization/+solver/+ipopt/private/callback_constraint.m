function [constr] = callback_constraint(x)
	%CALLBACK_CONSTRAINT callback function for contraint function handle for ipopt
	%	Input:
	%		x:			current optimization variable
	%	Output:
	%		constr:		constraint function value
	constr = callback(x, 'con');
end
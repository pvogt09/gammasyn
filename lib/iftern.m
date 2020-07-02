function [out] = iftern(cond, a, b)
	%IFTERN ternary operator
	%	Input:
	%		cond:	condition to return a
	%		a:		variable that is returned, if condition is true
	%		b:		variable that is returned, if condition is false
	%	Output:
	%		out:	a if condition is true, else b
	if cond
		out = a;
	else
		out = b;
	end
end


function [J] = callback_objective(x)
	%CALLBACK_OBJECTIVE callback function for objective function handle for solvopt
	%	Input:
	%		x:			current optimization variable
	%	Output:
	%		J:			function value of objective function
	J = callback(x, 'fun');
end
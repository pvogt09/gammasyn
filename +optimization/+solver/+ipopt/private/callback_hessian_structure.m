function [hessian_structure] = callback_hessian_structure()
	%CALLBACK_HESSIAN_STRUCTURE callback function for hessian structure function handle for ipopt
	%	Input:
	%		x:						current optimization variable
	%	Output:
	%		hessian_structure:		hessian structure value
	hessian_structure = callback([], 'hessstruct');
end
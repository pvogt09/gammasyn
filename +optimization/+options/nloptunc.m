classdef nloptunc < optimization.options.nlopt
	%NLOPTUNC solver options for nloptunc
	
	methods
		function [this] = nloptunc(varargin)
			%NLOPTUNC create new optimization option set
			%	Input:
			%		varargin:	options to set
			%	Output:
			%		this:		instance
			this@optimization.options.nlopt(optimization.solver.Optimizer.NLOPTUNC, optimization.options.ProblemType.UNCONSTRAINED, optimization.options.ProblemType.UNCONSTRAINED, varargin{:});
		end
		
		function [algorithms] = possiblealgorithms(~)
			%POSSIBLEALGORITHMS list with possible algorithms for optimizer
			%	Input:
			%		this:		instance
			%	Output:
			%		algorithms:	possible algorithms
			algorithms = {
				'stogo';
				'stogo_rand';
				'crs2';
				'direct';
				'direct_l';
				'direct_l_noscal';
				'direct_l_rand';
				'direct_l_rand_noscal';
				'direct_noscal';
				'esch';
				'direct_orig';
				'direct_l_orig';
				'ccsaq';
				'lbfgs';
				'lbfgs_nocedal';
				'tnewton';
				'tnewton_precond';
				'tnewton_precond_restart';
				'tnewton_restart';
				'var1';
				'var2';
				'bobyoa';
				'neldermead';
				'newuoa';
				'newuoa_bound';
				'praxis';
				'subplex'
			};
		end
	end
	
end
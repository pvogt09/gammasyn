classdef nloptuncglobal < optimization.options.nlopt
	%NLOPTUNCGLOBAL solver options for nloptuncglobal
	
	methods
		function [this] = nloptuncglobal(varargin)
			%NLOPTUNCGLOBAL create new optimization option set
			%	Input:
			%		varargin:	options to set
			%	Output:
			%		this:		instance
			this@optimization.options.nlopt(optimization.solver.Optimizer.NLOPTUNCGLOBAL, optimization.options.ProblemType.UNCONSTRAINED, optimization.options.ProblemType.UNCONSTRAINED, varargin{:});
			if isempty(this.SubproblemAlgorithm)
				this.SubproblemAlgorithm = 'neldermead';
			end
		end
		
		function [algorithms] = possiblealgorithms(~)
			%POSSIBLEALGORITHMS list with possible algorithms for optimizer
			%	Input:
			%		this:		instance
			%	Output:
			%		algorithms:	possible algorithms
			algorithms = {
				'mlsl';
				'mlsl_lds'
			};
		end
		
		function [subalgorithms] = possiblesubalgorithms(~)
			%POSSIBLESUBALGORITHMS list with possible algorithms for subproblems for optimizer
			%	Input:
			%		this:			instance
			%	Output:
			%		subalgorithms:	possible algorithms for subproblems
			subalgorithms = {
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
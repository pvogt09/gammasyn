classdef nloptcon < optimization.options.nlopt
	%NLOPTCON solver options for nloptcon
	
	methods
		function [this] = nloptcon(varargin)
			%NLOPTCON create new optimization option set
			%	Input:
			%		varargin:	options to set
			%	Output:
			%		this:		instance
			this@optimization.options.nlopt(optimization.solver.Optimizer.NLOPTCON, [
				optimization.options.ProblemType.UNCONSTRAINED;
				optimization.options.ProblemType.CONSTRAINED
			], optimization.options.ProblemType.CONSTRAINED, varargin{:});
		end
		
		function [algorithms] = possiblealgorithms(~)
			%POSSIBLEALGORITHMS list with possible algorithms for optimizer
			%	Input:
			%		this:		instance
			%	Output:
			%		algorithms:	possible algorithms
			algorithms = {
				'isres';
				'auglag';
				'auglag_eq';
				'mma';
				'slsqp';
				'cobyla'
			};
		end
	end
	
end
classdef nloptconglobal < optimization.options.nlopt
	%NLOPTCONGLOBAL solver options for nloptconglobal
	
	methods
		function [this] = nloptconglobal(varargin)
			%NLOPTCONGLOBAL create new optimization option set
			%	Input:
			%		varargin:	options to set
			%	Output:
			%		this:		instance
			this@optimization.options.nlopt(optimization.solver.Optimizer.NLOPTCONGLOBAL, [
				optimization.options.ProblemType.UNCONSTRAINED;
				optimization.options.ProblemType.CONSTRAINED
			], optimization.options.ProblemType.CONSTRAINED, varargin{:});
			if isempty(this.SubproblemAlgorithm)
				this.SubproblemAlgorithm = 'cobyla';
			end
		end
		
		function [algorithms] = possiblealgorithms(~)
			%POSSIBLEALGORITHMS list with possible algorithms for optimizer
			%	Input:
			%		this:		instance
			%	Output:
			%		algorithms:	possible algorithms
			algorithms = {
				'auglag';
				'auglag_eq'
			};
		end
		
		function [subalgorithms] = possiblesubalgorithms(~)
			%POSSIBLESUBALGORITHMS list with possible algorithms for subproblems for optimizer
			%	Input:
			%		this:			instance
			%	Output:
			%		subalgorithms:	possible algorithms for subproblems
			subalgorithms = {
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
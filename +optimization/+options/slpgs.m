classdef slpgs < optimization.options.slqpgs
	%SLPGS solver options for slpgs
	
	methods(Static=true)
		function [this] = fromDCM(value, ~)
			%FROMDCM convert structure from DCM import to instance
			%	Input:
			%		value:	value imported from DCM file
			%		name:	optional name of parameter
			%	Output:
			%		this:	instance
			this = optimization.options.sqpgs();
			if nargin == 0 || isempty(value)
				return;
			end
			if isstruct(value)
				this.useoptions(value);
			end
		end
		
		function [success] = registerDCMHandler()
			%REGISTERDCMHANDLER register this class in the DCM handler class
			%	Output:
			%		success:	indicator, if registration was successful
			success = export.parameter.DCMHandler.instance.registerHandler(?optimization.options.slpgs, ?optimization.options.slpgs);
		end
	end
	
	methods
		function [this] = slpgs(varargin)
			%SQPGS create new optimization option set
			%	Input:
			%		varargin:	options to set
			%	Output:
			%		this:		instance
			this@optimization.options.slqpgs(optimization.solver.Optimizer.SLPGS);
			if nargin >= 1
				this.useoptions(varargin{:});
			end
		end
		
		function [algorithms] = possiblealgorithms(~)
			%POSSIBLEALGORITHMS list with possible algorithms for optimizer
			%	Input:
			%		this:		instance
			%	Output:
			%		algorithms:	possible algorithms
			algorithms = {
				'slpgs'
			};
		end
		
		function [subalgorithms] = possiblesubalgorithms(~)
			%POSSIBLESUBALGORITHMS list with possible algorithms for subproblems for optimizer
			%	Input:
			%		this:			instance
			%	Output:
			%		subalgorithms:	possible algorithms for subproblems
			subalgorithms = {
				'linprog';
			};
		end
	end
	
end
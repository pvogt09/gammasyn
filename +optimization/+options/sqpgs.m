classdef sqpgs < optimization.options.slqpgs
	%SQPGS solver options for sqpgs

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
			success = export.parameter.DCMHandler.instance.registerHandler(?optimization.options.sqpgs, ?optimization.options.sqpgs);
		end
	end

	methods
		function [this] = sqpgs(varargin)
			%SQPGS create new optimization option set
			%	Input:
			%		varargin:	options to set
			%	Output:
			%		this:		instance
			this@optimization.options.slqpgs(optimization.solver.Optimizer.SQPGS);
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
				'sqpgs'
			};
		end

		function [subalgorithms] = possiblesubalgorithms(~)
			%POSSIBLESUBALGORITHMS list with possible algorithms for subproblems for optimizer
			%	Input:
			%		this:			instance
			%	Output:
			%		subalgorithms:	possible algorithms for subproblems
			subalgorithms = {
				'quadprog';
				'gurobi'
			};
		end
	end

end
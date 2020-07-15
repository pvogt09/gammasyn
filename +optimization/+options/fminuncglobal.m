classdef fminuncglobal < optimization.options.fminunc
	%FMINUNCGLOBAL solver options for global search with fminunc

	methods
		function [this] = fminuncglobal(varargin)
			%FMINUNCGLOBAL create new optimization option set
			%	Input:
			%		varargin:	options to set
			%	Output:
			%		this:		instance
			this@optimization.options.fminunc();
			this.Solver = optimization.solver.Optimizer.FMINUNCGLOBAL;
			if nargin >= 1
				this.useoptions(varargin{:});
			end
		end

		function [information] = formatOutput(this, errorcode, time, xmin, fmin, nvars, overalliterations, overallfunevals, retries, output, alloutputs)
			%FORMATOUTPUT unify output of optimization
			%	Input:
			%		this:				instance
			%		errorcode:			exit code of optimization
			%		time:				time for optimization
			%		xmin:				solution minimizing the objective function
			%		fmin:				minimum objective function value
			%		nvars:				number of optimization variables
			%		overalliterations:	total number of iterations over all optimization retries
			%		overallfunevals:	total number of function evaluations over all optimization retries
			%		retries:			number of retries
			%		output:				optimization output
			%		alloutputs:			cell array with outputs for different optimization runs to add to the overall optimization output
			%	Output:
			%		information:		unified optimization output
			information = this.OUTPUTPROTOTYPE;
			outputstruct = nargin >= 10 && isstruct(output);
			if outputstruct && isfield(output, 'localSolverTotal')
				information.iterations		= output.localSolverTotal;
			end
			if outputstruct && isfield(output, 'funcCount')
				information.funcCount		= output.funcCount;
			end
			if outputstruct && isfield(output, 'algorithm')
				information.algorithm		= output.algorithm;
			else
				information.algorithm		= 'FMINCONGLOBAL';
			end
			if outputstruct && isfield(output, 'message')
				information.message			= output.message;
			end
			if nargin >= 10
				information.additional		= output;
			end
			information.information.t					= time;
			information.information.Nvar				= nvars;
			if outputstruct && isfield(output, 'localSolverTotal')
				information.information.iterations		= output.localSolverTotal;
			end
			if outputstruct && isfield(output, 'funcCount')
				information.information.funcCount		= output.funcCount;
			end
			information.information.overalliterations	= overalliterations;
			information.information.overallfunCount		= overallfunevals;
			information.information.retries				= retries;
			information.information.feasibility			= errorcode;
			information.information.xmin				= xmin;
			information.information.fmin				= fmin;
			if outputstruct && isfield(output, 'message')
				information.information.output			= output.message;
			end
			if nargin >= 11
				if iscell(alloutputs) && numel(alloutputs) > 1
					information.runs = alloutputs;
				else
					information.runs = {};
				end
			end
		end
	end

end
function [iv,dv] = set_opt_para(iv, dv, options)

	% Set options for the optimization function minsearch called from oplace

	IPRINT=3;
	MAXITN=4;
	MAXIFN=5;
	IEPSG=2;
	IEPSF=3;
	IEPSX=4;
	IDELX=5;
	[iv, dv] = swerte(iv, dv);  %Defaultwerte setzen
	if nargin > 2 && ~isempty(options)
		if ~isstruct(options)
			if isa(options, 'optim.options.SolverOptions')
				options = struct(...
					'MaxIter',		options.MaxIterations,...
					'MaxFunEvals',	options.MaxFunctionsEvaluations,...
					'Display',		options.Display,...
					'TolX',			options.StepTolerance,...
					'TolFun',		options.FunctionTolerance...
				);
			elseif isa(options, 'optimization.options.Options')
				options = options.getoptions(optimization.options.OptionType.STRUCT, struct());
			else
				error('oplace:optimize:options', 'Undefined option type for oplace.');
			end
		end
		if isfield(options, 'optimization')
			if isstruct(options.optimization)
				options = options.optimization;
			else
				if isa(options.optimization, 'optim.options.SolverOptions')
					options = struct(...
						'MaxIter',		options.optimization.MaxIterations,...
						'MaxFunEvals',	options.optimization.MaxFunctionsEvaluations,...
						'Display',		options.optimization.Display,...
						'TolX',			options.optimization.StepTolerance,...
						'TolFun',		options.optimization.FunctionTolerance...
					);
				elseif isa(options.optimization, 'optimization.options.Options')
					options = options.optimization.getoptions(optimization.options.OptionType.STRUCT, struct());
				else
					error('oplace:optimize:options', 'Undefined option type for oplace.');
				end
			end
		end
		if isfield(options, 'TolFun')
			dv(IEPSF) = options.TolFun;
		end
		if isfield(options, 'FunctionTolerance')
			dv(IEPSF) = options.FunctionTolerance;
		end
		if isfield(options, 'TolX')
			dv(IEPSX) = options.TolX;
		end
		if isfield(options, 'StepTolerance')
			dv(IEPSX) = options.StepTolerance;
		end
		if isfield(options, 'TolFun')
			dv(IEPSG) = options.TolFun;
		end
		if isfield(options, 'OptimalityTolerance')
			dv(IEPSG) = options.OptimalityTolerance;
		end
		if isfield(options, 'MaxIter')
			iv(MAXITN) = options.MaxIter;
		end
		if isfield(options, 'MaxIterations')
			iv(MAXITN) = options.MaxIterations;
		end
		if isfield(options, 'MaxFun')
			iv(MAXIFN) = options.MaxFun;
		end
		if isfield(options, 'MaxFunEvals')
			iv(MAXIFN) = options.MaxFunEvals;
		end
		if isfield(options, 'MaxFunEvaluations')
			iv(MAXIFN) = options.MaxFunEvaluations;
		end
		if isfield(options, 'Display')
			if any(strcmpi(options.Display, {'iter', 'notify'}))
				iv(IPRINT) = -1;
			elseif any(strcmpi(options.Display, {'iter-detailed', 'notify-detailed'}))
				iv(IPRINT) = 1;
			else
				iv(IPRINT) = 0;
			end
		end
	else
		op_dat = read_optim_dat;
		if ~isempty(op_dat)
			fnames = fieldnames(op_dat);
			index = strmatch('TolFun', fnames, 'exact');
			if (~isempty(index))
				dv(IEPSF) = op_dat.TolFun;
			end
			index = strmatch('TolX', fnames, 'exact');
			if (~isempty(index))
				dv(IEPSX) = op_dat.TolX;
			end
			index = strmatch('TolGrad', fnames, 'exact');
			if (~isempty(index))
				dv(IEPSG) = op_dat.TolGrad;
			end
			index = strmatch('MaxIter', fnames, 'exact');
			if (~isempty(index))
				iv(MAXITN) = op_dat.MaxIter;
			end
			index = strmatch('MaxFun', fnames, 'exact');
			if (~isempty(index))
				iv(MAXIFN) = op_dat.MaxFun;
			end
			index = strmatch('Print', fnames, 'exact');
			if (~isempty(index))
				iv(IPRINT) = op_dat.Print;
			end
		end
	end
end
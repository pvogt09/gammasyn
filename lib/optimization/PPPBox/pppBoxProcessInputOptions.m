function [numRandVecs,maxFunEvals,maxIter,optTol,progTol,...
          DerivativeCheck,verbose,verboseI,debug,doPlot,...
          useGurobi,alpha,beta,delta,armijoSimple,rho, numDiffType, maxTime, maxSQPIter] = pppBoxProcessInputOptions(o)

	% Set all provided fields in 'o' to upper case letters
	o = toUpper(o);

	% Check if derivatives shall be checked
	DerivativeCheck = 0;
	if isfield(o, 'DERIVATIVECHECK')
		if ischar(o.DERIVATIVECHECK)
			switch(upper(o.DERIVATIVECHECK))
				case 1
					DerivativeCheck = true;
				case 'ON'
					DerivativeCheck = true;
				case 0
					DerivativeCheck = false;
				case 'OFF'
					DerivativeCheck = false;
			end
		else
			DerivativeCheck = logical(o.DERIVATIVECHECK);
		end
	end

	verbose = 1;    % Show final information
	verboseI = 0;    % Show information during iteration
	debug = 0;      % even more information during iteration
	doPlot = ~isempty(getOpt(o, 'PLOTFCN', {}));     % also show nice plots during the line search
	
	numDiffType = 2;
	if isfield(o, 'FINDIFFTYPE')
		if strcmpi(o.FINDIFFTYPE, 'forward')
			numDiffType = 1;
		elseif strcmpi(o.FINDIFFTYPE, 'central')
			numDiffType = 2;
		end
	end
	

	% Specify depth of display
	if isfield(o, 'DISPLAY')
		switch(upper(o.DISPLAY))
			case 0
				verbose = false;
				verboseI = false;
			case 'ITER'
				verboseI = true;
			case {'FINAL', 'FINAL-DETAILED', 'NOTIFY', 'NOTIFY-DETAILED'}
				verboseI = false;
			case 'OFF'
				verbose = false;
				verboseI = false;
			case 'NONE'
				verbose = false;
				verboseI = false;
			case {'FULL', 'ITER-DETAILED'}
				debug = true;
			case 'EXCESSIVE'
				debug = true;
				doPlot = true;
		end
	end

	numRandVecs = getOpt(o, 'NUMRANDVECS', 0);
	maxFunEvals	= getOpt(o, 'MAXFUNEVALS', 1000);
	maxIter		= getOpt(o, 'MAXITER', 200);
	maxSQPIter	= getOpt(o, 'MAXSQPITER', 200);
	optTol		= getOpt(o, 'OPTTOL', 1e-6);
	progTol		= getOpt(o, 'PROGTOL', 1e-6);
	maxTime		= getOpt(o, 'MAXTIME', Inf);
	hasGurobi = any(exist('gurobi') == [2, 3, 5, 6]); %#ok<EXIST> exist('gurobi', 'function')
	if hasGurobi
		subproblemAlgorithm = getOpt(o, 'SUBPROBLEMALGORITHM', 'gurobi');
	else
		subproblemAlgorithm = getOpt(o, 'SUBPROBLEMALGORITHM', 'quadprog');
	end
	useGurobi	= getOpt(o, 'GUROBI', hasGurobi && strcmpi(subproblemAlgorithm, 'gurobi'));
	alpha		= getOpt(o, 'ALPHA', 0.3); %1e-4 minFunc Std
	beta		= getOpt(o, 'BETA', 0.75);
	delta		= getOpt(o, 'DELTA', .01);
	gam			= getOpt(o, 'GAM', 1);
	armijoSimple= getOpt(o, 'ARMIJOSIMPLE', 1);
	rho			= getOpt(o, 'RHO', 1);
end

function [v] = getOpt(options, opt, default)
	if isfield(options,opt)
		if ~isempty(options.(opt))
			v = options.(opt);
		else
			v = default;
		end
	else
		v = default;
	end
end

function [o] = toUpper(o)
	if ~isempty(o)
		fn = fieldnames(o);
		for i = 1:length(fn)
			o.(upper(fn{i})) = o.(fn{i});
		end
	end
end
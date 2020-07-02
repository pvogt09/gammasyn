function [x, fval, exitflag, output] = pppBox(fun, x0, options, Aiq, biq, varargin)
	%PPP-BOX Summary of this function goes here
	%
	% Author:   Lukas Stein
	% Modified: Jan Strubel (AUG 2014)


	if nargin < 2
		error('PPPBox:input', 'Not enough input arguments. Provide at least a funObj handle and a start vector!');
	end
	if nargin < 3
		options = [];
		Aiq = [];
		biq = [];
	end
	if nargin < 5
		Aiq = [];
		biq = [];
	end
	isfunvararg = nargin(fun) ~= 1;

	% Get Parameters
	[numRandVecs, maxFunEvals, maxIter, optTol, progTol,...
		DerivativeCheck, verbose, verboseI, debug, doPlot,...
		useGurobi, alpha, beta, delta, armijoSimple, ~, numDiffType, maxTime, maxSQPIter] = pppBoxProcessInputOptions(options);

	numOptVars = length(x0);
	hasgradient = 2 <= nargout(fun);
	if nargout(fun) == -1
		try
			if isfunvararg
				[f, g] = fun(x0, varargin{:}); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
			else
				[f, g] = fun(x0); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
			end
			hasgradient = true;
		catch e
			if strcmpi(e.identifier, 'MATLAB:maxlhs') || strcmpi(e.identifier, 'MATLAB:TooManyOutputs') || strcmpi(e.identifier, 'MATLAB:unassignedOutputs')
				hasgradient = false;
			else
				rethrow(e);
			end
		end
	end

	% Check start vector
	if (~isempty(x0))
	%     if (max(max(abs(x0))) > 1 || ~isreal(x0))
	%         error('Optimization variables x_i have to be scaled such that |x_i| <= 1');
	%     end
		if (size(x0,1) ~= numOptVars)
			error('PPPBox:input', 'Number of rows in third argument must equal the number of optimization variables');
		end
	end

	% Derivative Check with inital point
	if DerivativeCheck
		order = 1;  % Only check gradient
		derivativeCheckMinMax(fun, x0, order, numDiffType, varargin{:}); % Checks gradient
	end

	% Generate multiple start vectors
	x0 = [
		x0, 2*rand(numOptVars,numRandVecs)-1
	];

	%% Check start vectors
	% Set up QP solver
	quadprogoptions = optimset(optimset('quadprog'), 'MaxIter', maxSQPIter);
	if verboseI
		quadprogoptions.Display = 'final';
	else
		quadprogoptions.Display = 'none';
	end
	params = struct(...
		'OutputFlag',		0,...
		'IterationLimit',	maxSQPIter...
	);
	model.sense = '<';
	model.lb = -2*ones(numOptVars, 1);
	model.ub =  2*ones(numOptVars, 1);

	iqConsFlag = false;
	if nargin > 4
		% Check if affine (in)equalitiy constraints are provided
		if ((isempty(Aiq) && ~isempty(biq)) || (~isempty(Aiq) && isempty(biq)))
			error('PPPBox:input', 'Supply both arguments for inequality constraints or none.');
		end

		% Check Dimensions
		if ~isempty(Aiq) && ~isempty(biq)
			if (size(Aiq, 2) ~= numOptVars) || (size(Aiq, 1) ~= size(biq, 1)) || size(biq, 2) ~= 1
				error('PPPBox:input', 'Dimension mismatch in arguments 5 and 6.');
			end
			% Set Flag
			iqConsFlag = true;

			% Check if start vector results in a fesible problem
			for count = 1:size(x0, 2)
				model.Q = speye(numOptVars);
				model.obj = -2*x0(:, count);
				model.A = sparse(Aiq);
				model.rhs = biq;
				if useGurobi
					result = gurobi(model, params);
					if (strcmp(result.status, 'INFEASIBLE') || strcmp(result.status, 'INF_OR_UNBD'))
						error('PPPBox:input', 'Inequality constraints result in an infeasible problem.');
					elseif ~strcmp(result.status, 'OPTIMAL')
						error('PPPBox:input', 'Optimization failed');
					end
				else
					% Use quadprog
					[result.x, ~, initialexitflag] = quadprog(model.Q, model.obj, model.A, model.rhs, [], [], model.lb, model.ub, [], quadprogoptions);
					if initialexitflag == -2
						error('PPPBox:input', 'Inequality constraints result in an infeasible problem.');
					elseif initialexitflag < 0
						error('PPPBox:input', 'Optimization failed');
					end
				end
				x0(:, count) = result.x;
			end
		end
	end

	%% Main Optimization Loop 
	% fixed optimizazion matrices
	model.lb = [
		model.lb;
		-1e200
	];
	model.ub = [
		model.ub;
		1e200
	];

	% Get inital function value
	if isfunvararg
		f = fun(x0(:, 1), varargin{:});
	else
		f = fun(x0(:, 1));
	end
	funEvals = 1;
	fval_best = max(f);
	x_best = x0(:, 1);
	n = length(f);

	if nargout > 3
		% Initialize Trace
		trace.fval = fval_best;
		trace.funcCount = 1;
		trace.optCond = -1e200;

		msg = 'Initial values of the optimization';
		% Init output struct
		output = struct(...
			'iterations',		0,...
			'funcCount',		1,...
			'algorithm',		'PPPBOX single',...
			'firstorderopt',	trace.optCond,...
			'message',			msg,...
			'time',				0,...
			'trace',			trace...
		);
	end

	% Output Log
	if verboseI
		fprintf('%10s %10s %15s %15s %15s\n', 'Iteration', 'FunEvals6', 'Step Length', 'Function Val', 'Opt Cond');
	end

	solvertime = tic;
	for count = 1:size(x0, 2)
		% initial data; initial value is not necessarily a feasible point 
		x = x0(:, count);
		x_old = x;
		fval_old = 0;
		for count1 = 1:maxIter
			% Evaluate user supplied function
			if hasgradient
				if isfunvararg
					[f, gradf] = fun(x, varargin{:});
				else
					[f, gradf] = fun(x);
				end
			else
				if isfunvararg
					f = fun(x, varargin{:});
				else
					f = fun(x);
				end
				[~, gradf] = autoGradMinMax(x, numDiffType, fun, varargin{:});
			end
			[psi, indg] = max(f);
			g = gradf(:, indg);
			if any(isnan(f(:))) || any(isnan(gradf(:)))
				exitflag = -3;
				msg = 'Problem grows unbounded';
				break;
			end
			if any(isinf(f(:))) || any(isinf(gradf(:)))
				exitflag = -3;
				msg = 'Problem grows unbounded';
				break;
			end

			% solve QP to obtain the value (theta) of the optimality function and the
			% corresponding search direction (h) 
			if count1 == 1
				H           = delta*speye(numOptVars);
				model.Q     = [
					.5*H, zeros(numOptVars,1);
					zeros(1, numOptVars+1)
				];
				model.obj   = [
					-delta*x;
					1
				];
			else
				% Compute difference vectors
				y = g - g_old;
				s = t*h;
				ys = y'*s;
				% BFGS Update
				if ys > 1e-10
					Hy = H*y;
					yHy = y'*Hy;
					gamma = ys/yHy;
					v = sqrt(yHy)*(s/ys - Hy/yHy);
					H = gamma*(H - Hy*Hy'/yHy + v*v') + (s*s')/ys;
				else
					if debug
						fprintf('Skipping Update\n');
					end
				end
				model.obj = [
					-H*x;
					1
				];
				model.Q = 0.5*sparse(blkdiag(H, 0));
			end

			% Set up QP subproblem, check if (in)equality constraints are
			% present
			if iqConsFlag
				model.A = sparse([
					gradf',	-ones(n, 1);
					Aiq,	zeros(size(Aiq, 1), 1)
				]);
				model.rhs = [
					(-f+psi) + gradf'*x;
					biq
				];
			else
				model.A = sparse([
					gradf', -ones(n, 1)
				]);
				model.rhs = (-f + psi) + gradf'*x;
			end

			if useGurobi
				% Call GUROBI
				result = gurobi(model, params);
				if (strcmp(result.status, 'INFEASIBLE') || strcmp(result.status, 'INF_OR_UNBD'))
					error('PPPBox:input', 'Optimization terminated, infeasible problem.');
				elseif ~strcmp(result.status, 'OPTIMAL')
					error('PPPBox:input', 'Optimization failed');
				end

				% Get new point and the value of the optimality function theta
				xnew = result.x;
				theta = result.objval;
			else
				% Use quadprog
				[xnew, theta, ~, ~, LAMBDA] = quadprog(model.Q, model.obj, model.A, model.rhs, [], [], model.lb, model.ub, [
					x;
					0
				], quadprogoptions);
			end


			% compute descent direction h
			h = xnew(1:end-1) - x;
			theta = theta +.5*delta*(x'*x);

			% Step Size 
			if armijoSimple
			% Armijo type step size rule
				k = 0;
				if isfunvararg
					while (max(fun(x + beta^k*h, varargin{:})) - psi - beta^k*alpha*theta > 0) && k < 100
						k = k + 1;
					end
				else
					while (max(fun(x + beta^k*h)) - psi - beta^k*alpha*theta > 0) && k < 100
						k = k + 1;
					end
				end

				% Compute new x and function value
				t = beta^k;
				x = x + t*h;
				if isfunvararg
					fval = max(fun(x, varargin{:}));
				else
					fval = max(fun(x));
				end
				funEvals = funEvals + k;
			else
				% From minFunc
				LS_interp = 1;
				LS_multi = 1;
				if count1 == 1
					t = 1;
				else
					t = min(1, t*2);
				end
				if isfunvararg
					fval = max(fun(x, varargin{:})); 
				else
					fval = max(fun(x)); 
				end
				[t, x, fval, ~, k] = ArmijoBacktrackMinMax(x, t, h, fval, psi, g, theta, alpha, LS_interp, LS_multi, progTol, debug, doPlot, 1, fun, varargin{:});
			end
			funEvals = funEvals + k;

			% Output iteration information
			if verboseI
				fprintf('%10d %10d %15.5e %15.5e %15.5e\n', count1, k, t, fval, theta);
			end

			if nargout > 3
				% Update Trace
				trace.fval(end+1, 1) = fval;
				trace.funcCount(end+1, 1) = funEvals;
				trace.optCond(end+1, 1) = theta;
			end

			% Check Optimality Condition
			if theta >= -optTol
				exitflag = 1;
				msg = 'Optimality Condition below optTol';
				break;
			end

			% ******************* Check for lack of progress *******************

			if abs(fval_old - fval) < optTol 
				exitflag = 2;
				msg = 'Function Value changing by less than optTol'; 
				break;
			end
			if norm(x - x_old) < progTol
				exitflag = 2;
				msg = 'Change in x below progTol';
				break;
			end

			% ******** Check for going over iteration/evaluation limit *******************

			if funEvals >= maxFunEvals
				exitflag = 0;
				msg = 'Reached Maximum Number of Function Evaluations';
				break;
			end

			if count1 >= maxIter
				exitflag = 0;
				msg = 'Reached Maximum Number of Iterations';
				break;
			end
			
			if toc(solvertime) > maxTime
				exitflag = 0;
				msg = 'Reached Maximum Time';
				break;
			end

			% Save values for next iteration
			x_old = x;
			fval_old = fval; 
			g_old = g;

		end % end iteration

		% Display final information
		if verbose
			fprintf('\n%s\n', msg);
		end
		if nargout > 3
			output = struct(...
				'iterations',		count1,...
				'funcCount',		funEvals,...
				'algorithm',		'PPPBOX single',...
				'firstorderopt',	theta,...
				'message',			msg,...
				'time',				toc(solvertime),...
				'trace',			trace...
			);
		end

		% Keep track of best point so far
		if fval_best > fval
			fval_best = fval;
			x_best = x;
	%         theta_best= theta;
		end
	end

	%% Set output
	x       = x_best;
	fval    = fval_best;

	% Print final information
	if verbose
		fprintf('\n%10s %10s %15s %15s %15s\n', 'Iteration', 'FunEvals', 'Step Length', 'Function Val', 'Opt Cond');
		fprintf('%10s %10d %15.5e %15.5e %15.5e\n', 'final', funEvals, t, fval, theta);
	end

end
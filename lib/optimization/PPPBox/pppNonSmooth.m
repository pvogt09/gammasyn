function [x, fval, exitflag, output] = pppNonSmooth(fun, x0, options, varargin)
	%PPP-BOX Summary of this function goes here
	%
	% Author:   Lukas Stein
	% Modified: Jan Strubel (AUG 2014)
	if nargin < 2
		error('PPPBox:input', 'Not enough input arguments. Provide at least a funObj handle and a start vector!');
	end
	if nargin < 3
		options = [];
	end

	% Get Parameters
	[numRandVecs, maxFunEvals, maxIter, optTol, progTol,...
		DerivativeCheck, verbose, verboseI, debug, doPlot,...
		useGurobi, alpha, beta, delta, armijoSimple, rho, numDiffType, maxTime, maxSQPIter] = pppBoxProcessInputOptions(options);

	numOptVars = length(x0);
	hasgradient = 2 <= nargout(fun);
	if nargout(fun) == -1
		try
			[f, g] = fun(x0, varargin{:}); %#ok<ASGLU> call with two output arguments to check if the second one is present and catch error if not
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
		if (size(x0, 1) ~= numOptVars)
			error('PPPBox:input', 'Number of rows in third argument must equal the number of optimization variables');
		end
	end

	% Derivative Check with inital point
	if hasgradient && DerivativeCheck
		order = 1;  % Only check gradient
		derivativeCheckMinMax(fun, x0, order, numDiffType, varargin{:}); % Checks gradient
	end

	% Generate multiple start vectors
	gain = 2*max(max(abs(x0)));
	x0 = [
		x0, gain*(2*rand(numOptVars,numRandVecs)-1)
	];

	%% Main Optimization Loop
	% Get inital function value
	f = fun(x0(:, 1), varargin{:});
	funEvals = 1;
	fval_best = max(f);
	x_best = x0(:, 1);


	% Set up QP solver
	params = struct(...
		'OutputFlag',		0,...
		'IterationLimit',	maxSQPIter...
	);
	% params.Method = -1;
	model.sense = '<';

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
			'algorithm',		'PPPBOX multi',...
			'firstorderopt',	trace.optCond,...
			'message',			msg,...
			'time',				0,...
			'trace',			trace...
		);
	end

	% Output Log
	if verboseI
		fprintf('%10s %10s %15s %15s %15s\n', 'Iteration', 'FunEvals', 'Step Length', 'Function Val', 'Opt Cond');
	end

	quadprogoptions = optimset(optimset('quadprog'), 'MaxIter', maxSQPIter);
	if verboseI
		quadprogoptions.Display = 'final';
	else
		quadprogoptions.Display = 'none';
	end
	solvertime = tic;
	for count = 1:size(x0, 2)
		% initial data; initial value is not necessarily a feasible point 
		x           = x0(:, count);
		x_old       = x;
		fval_old    = 0;
		H           = delta*speye(numOptVars);

		for count1 = 1:maxIter
			% Evaluate user supplied function
			if hasgradient
				[f, gradf] = fun(x, varargin{:});
			else
				f = fun(x, varargin{:});
				[~, gradf] = autoGradMinMax(x, numDiffType, fun, varargin{:});
			end
			if any(isnan(f(:))) || any(isnan(gradf(:)))
				exitflag = -3;
				msg = 'Problem grows unbounded';
				break;
			end
			[psi, indg] = max(f);
			if count1 == 1
				g_old = gradf(:, indg);
			end

			% Generate rho-active set
			% rho = 0 - > steepest descent
			% rho = 1 - > original problem
			threshold = (1 - rho)*(psi - min(f)) + min(f);
			if isnan(threshold)
				threshold = min(f);
			end
			inds = find(f >= threshold);
			f = f(inds);
			gradf = gradf(:, inds);
			n = length(f);
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

			% Set up QP subproblem, check if (in)equality constriants are
			% present
			model.Q     = sparse(blkdiag(0.5*H, 0));
			model.obj   = [
				zeros(1, numOptVars), 1
			];
			model.alpha = -psi;
			model.A     = sparse([
				gradf', -ones(n,1)
			]);
			model.rhs   = -f + psi;

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

				% Also get the Lagrange multipliers for the computation of the
				% hessian / BFGS update
				lagrangeMult = result.pi;
			else
				% Use quadprog
				[xnew, theta, ~, ~, LAMBDA] = quadprog(model.Q, model.obj, model.A, model.rhs, [], [], [], [], [], quadprogoptions);

				% Also get the Lagrange multipliers for the computation of the
				% hessian / BFGS update
				lagrangeMult = LAMBDA.ineqlin;
			end

			% compute descent direction h and 'gradient'
			g = gradf*lagrangeMult;
			h = xnew(1:end-1);

			%% Step Size 
			switch armijoSimple
				case 1
					% Armijo type step size rule
					k = 0;  
					while (max(fun(x + beta^k*h, varargin{:})) - psi - beta^k*alpha*theta > 0) && k < 100
						k = k + 1;
					end

					% Compute new x and function value
					t = beta^k;
					x = x + t*h;
					fval = max(fun(x, varargin{:}));
					funEvals = funEvals + k;
				case 2
					% From minFunc
					LS_interp = 1;
					LS_multi = 1;
					if count1 == 1
						t = 1;
					else
						t = min(1, t*2);
					end
					fval = max(fun(x, varargin{:})); 
					[t, x, fval, ~, k] = ArmijoBacktrackMinMax(x, t, h, fval, psi, g, theta, alpha, LS_interp, LS_multi, progTol, debug, doPlot, 1, fun, varargin{:});
				case 3
					% Non-monotone line search
					% STEP 1, 
					% i.) already done -> h
					% ii.)
					ahHh = alpha*h'*H*h;
					if hasgrad
						[fh, gradfh] = fun(x + h, varargin{:});
					else
						fh = fun(x + h, varargin{:});
						[~, gradfh] = autoGradMinMax(x + h, numDiffType, fun, varargin{:});
					end
					[psih, indgh] = max(fh);

					% Initialize x1, x2
					if count1 == 1
						psi1 = psi;
						psi2 = psi;
					end

					% Consider rho active set
					threshold = (1 - rho)*(psih - min(fh)) + min(fh);
					indsh = find(fh >= threshold);
					fh = fh(indsh);
					gradfh = gradfh(:, indsh);
					nh = length(fh);

					if psih <= max([psi, psi1, psi2]) - ahHh
						% Newton step
						t = 1;
						htilde = 0*h;
						k = 1;
					else
						% iii.) Solve another QP to get htilde
						% solve QP to obtain search direction (htilde) 

						% Set up QP subproblem, check if (in)equality constriants are
						% present
						model.Q     = sparse(blkdiag(0.5*H, 0));
						model.obj   = [
							2*h'*H, 1
						];
						model.alpha = -psih + ahHh/alpha;
						model.A     = sparse([
							gradfh', -ones(nh,1)
						]);
						model.rhs   = -fh + psih;

						if useGurobi
							% Call GUROBI
							result = gurobi(model, params);
							if (strcmp(result.status, 'INFEASIBLE') || strcmp(result.status, 'INF_OR_UNBD'))
								error('PPPBox:input', 'Optimization terminated, infeasible problem.');
							elseif ~strcmp(result.status, 'OPTIMAL')
								error('PPPBox:input', 'Optimization failed');
							end

							% Get new point and the value of the optimality function theta
							htilde = result.x(1:end-1);
						else
							% Use quadprog
							[xnew, theta, ~, ~, LAMBDA] = quadprog(model.Q, model.obj, model.A, model.rhs, [], [], [], [], [], quadprogoptions);
							htilde = xnew(1:end-1);
						end

						% Compare norm of htilde and
						if norm(htilde) > norm(h)
							htilde = 0*h;
						end

						% iv) compute step size
						k = 0; 
						t = beta^k;
						while ( max(fun(x + t*h + t^2*htilde, varargin{:})) > max([psi, psi1, psi2]) - t*ahHh ) && k < 500
							k = k + 1;
							t = beta^k;
						end
					end

					% STEP 2 - Update
					x        = x + t*h + t^2*htilde;
					fval     = max(fun(x, varargin{:}));
					funEvals = funEvals + k;
					psi2 = psi1;
					psi1 = psi;

			end % switch - line search

			% Approximate Hessian - BFGS Update
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

			% Save Function Evaluations
			funEvals = funEvals + k + 1;

			% Output iteration information
			if verboseI
				fprintf('%10d %10d %15.5e %15.5e %15.5e\n', count1, funEvals, t, fval, theta);
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
			
			if toc(solvertime) >= maxTime
				exitflag = 0;
				msg = 'Reached Maximum Time';
				break;
			end

			% Save values for next iteration
			x_old    = x;
			fval_old = fval; 
			g_old    = g;

			% Keep track of best point so far
			if fval_best > fval
				fval_best = fval;
				x_best = x;
			end

		end % end iteration

		% Display final information
		if verbose
			fprintf('\n%s\n', msg);
		end
		if nargout > 3
			output = struct(...
				'iterations',		count1,...
				'funcCount',		funEvals,...
				'algorithm',		'PPPBOX multi',...
				'firstorderopt',	theta,...
				'message',			msg,...
				'time',				toc(solvertime),...
				'trace',			trace...
			);
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
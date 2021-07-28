function [pass] = SolverTest(~)
	%SOLVERTEST test cases for checking solvers for correct argument handling
	%	Input:
	%		silent:	idicator, if information should be output
	%	Output:
	%		pass:	indicator, if test was passed
	pass = identifier.TestResult.PASSED;
	solvers = enumeration('optimization.solver.Optimizer');
	functiontype = {
		'fun';
		'fungrad';
		'funhess';
		'anonfun';
		'anonfungrad';
		'anonfunhess';
		'funvar';
		'fungradvar';
		'funhessvar';
		'anonfunvar';
		'anonfungradvar';
		'anonfunhessvar'
	};
	constrainttype = {
		'con';
		'congrad';
		'conhess';
		'anoncon';
		'anoncongrad';
		'anonconhess';
		'convar';
		'congradvar';
		'conhessvar';
		'anonconvar';
		'anoncongradvar';
		'anonconhessvar';
		'nocon'
	};
	anonfunsingle = @(x) x(1)^2 + x(2) - x(3)^4;
	anonfungradsingle = @(x) flexargout(x(1)^2 + x(2) - x(3)^4, [
		2*x(1);
		1;
		-4*x(3)^3
	]);
	anonfunhesssingle = @(x) flexargout(x(1)^2 + x(2) - x(3)^4, [
		2*x(1);
		1;
		-4*x(3)^3
	], [
		2,	0,	0;
		0,	0,	0;
		0,	0,	-12*x(3)^2
	]);
	anonfunsinglevar = @(x, y) y*x(1)^2 + x(2) - x(3)^4;
	anonfungradsinglevar = @(x, y) flexargout(y*x(1)^2 + x(2) - x(3)^4, [
		y*2*x(1);
		1;
		-4*x(3)^3
	]);
	anonfunhesssinglevar = @(x, y) flexargout(y*x(1)^2 + x(2) - x(3)^4, [
		y*2*x(1);
		1;
		-4*x(3)^3
	], [
		2*y,	0,	0;
		0,		0,	0;
		0,		0,	-12*x(3)^2
	]);
	anonfunmulti = @(x) [
		x(1)^2 + x(2) - x(3)^4;
		(x(1) - 3)^2;
		x(2)^2 + x(3);
		x(1)
	];
	anonfungradmulti = @(x) flexargout([
		x(1)^2 + x(2) - x(3)^4;
		(x(1) - 3)^2;
		x(2)^2 + x(3);
		x(1)
	], [
		2*x(1),			1,		-4*x(3)^3;
		2*(x(1) - 3),	0,		0;
		0,				2*x(2),	1;
		1,				0,		0
	]');
	anonfunmultivar = @(x, y) [
		y*x(1)^2 + x(2) - x(3)^4;
		(x(1) - 3)^2;
		x(2)^2 + x(3);
		x(1)
	];
	anonfungradmultivar = @(x, y) flexargout([
		y*x(1)^2 + x(2) - x(3)^4;
		(x(1) - 3)^2;
		x(2)^2 + x(3);
		x(1)
	], [
		y*2*x(1),		1,		-4*x(3)^3;
		2*(x(1) - 3),	0,		0;
		0,				2*x(2),	1;
		1,				0,		0
	]');
	anoncon = @(x) flexargout([
		x(3) - 100;
		x(1) + x(2) - 1000;
		-x(1) - x(2) + 50;
		x(1)^2 + x(2)^2 - 75^2
	], []);
	anoncongrad = @(x) flexargout([
		x(3) - 100;
		x(1) + x(2) - 1000;
		-x(1) - x(2) + 50;
		x(1)^2 + x(2)^2 - 75^2
	], [], [
		0,	0,	1;
		1,	1,	0;
		-1,	-1,	0;
		2*x(1), 2*x(2), 0
	]', []);
	anonconhess = @(x) flexargout([
		x(3) - 100;
		x(1) + x(2) - 1000;
		-x(1) - x(2) + 50;
		x(1)^2 + x(2)^2 - 75^2
	], [], [
		0,	0,	1;
		1,	1,	0;
		-1,	-1,	0;
		2*x(1), 2*x(2), 0
	]', [], cat(3, [
		0,	0,	0;
		0,	0,	0;
		0,	0,	0
	], [
		0,	0,	0;
		0,	0,	0;
		0,	0,	0
	], [
		0,	0,	0;
		0,	0,	0;
		0,	0,	0
	], [
		2,	0,	0;
		0,	2,	0;
		0,	0,	0
	]), []);
	anonconvar = @(x, y) flexargout([
		y*x(3) - 100;
		x(1) + x(2) - 1000;
		-x(1) - x(2) + 50;
		x(1)^2 + x(2)^2 - 75^2
	], []);
	anoncongradvar = @(x, y) flexargout([
		y*x(3) - 100;
		x(1) + x(2) - 1000;
		-x(1) - x(2) + 50;
		x(1)^2 + x(2)^2 - 75^2
	], [], [
		0,	0,	y;
		1,	1,	0;
		-1,	-1,	0;
		2*x(1), 2*x(2), 0
	]', []);
	anonconhessvar = @(x, y) flexargout([
		y*x(3) - 100;
		x(1) + x(2) - 1000;
		-x(1) - x(2) + 50;
		x(1)^2 + x(2)^2 - 75^2
	], [], [
		0,	0,	y;
		1,	1,	0;
		-1,	-1,	0;
		2*x(1), 2*x(2), 0
	]', [], cat(3, [
		0,	0,	0;
		0,	0,	0;
		0,	0,	0
	], [
		0,	0,	0;
		0,	0,	0;
		0,	0,	0
	], [
		0,	0,	0;
		0,	0,	0;
		0,	0,	0
	], [
		2,	0,	0;
		0,	2,	0;
		0,	0,	0
	]), []);
	lb = [
		-100;
		-100;
		-100
	]; %#ok<NASGU>used in assertNoException
	ub = [
		100;
		100;
		100
	]; %#ok<NASGU>used in assertNoException
	A_ineq = {
		[];
		[
			1, 1, 1;
			2, 2, 2
		]
	};
	b_ineq = {
		[];
		[
			0;
			0
		]
	};
	A_eq = {
		[];
		[
			1, 1, 1
		]
	};
	b_eq = {
		[];
		0
	};
	hasallowedexceptions = false(size(solvers, 1), 1);
	allowedexceptions = cell(size(solvers, 1), 1);
	allowedexceptionstack = cell(size(solvers, 1), 1);
	% allow 'trust-region-reflective' but this algorithm does not solve problems with the constraints you have specified
	hasallowedexceptions(solvers == optimization.solver.Optimizer.FMINCON) = true;
	if any(solvers == optimization.solver.Optimizer.FMINCON)
		allowedexceptions{solvers == optimization.solver.Optimizer.FMINCON} = {
			'optimlib:fmincon:ConstrTRR'
		};
		allowedexceptionstack{solvers == optimization.solver.Optimizer.FMINCON} = [
			struct(...
				'file',	which('fmincon'),...
				'line',	'any'...
			)
		];
	end
	% allow bad index for objective function becoming Inf in SLP-GS
	hasallowedexceptions(solvers == optimization.solver.Optimizer.SLPGS) = true;
	if any(solvers == optimization.solver.Optimizer.SLPGS)
		allowedexceptions{solvers == optimization.solver.Optimizer.SLPGS} = {
			'MATLAB:badsubscript';
			'optim:ipqpcommon:ipConvexQP:InfNaNComplexDetected';
			'optim:linprog:FinalConstraintsViolated'% introduced in R2016A for new linprog
		};
		allowedexceptionstack{solvers == optimization.solver.Optimizer.SLPGS} = [
			struct(...
				'file',	which('slqpgs.Direction'),...
				'line',	'any'...
			);
			struct(...
				'file',	which('slqpgs.Direction'),...
				'line',	'any'...
			);
			struct(...
				'file',	which('slqpgs.Direction'),...
				'line',	'any'...
			)
		];
	end
	% allow bad index for objective function becoming Inf in SQP-GS
	hasallowedexceptions(solvers == optimization.solver.Optimizer.SQPGS) = true;
	if any(solvers == optimization.solver.Optimizer.SQPGS)
		allowedexceptions{solvers == optimization.solver.Optimizer.SQPGS} = {
			'MATLAB:badsubscript';
			'optim:ipqpcommon:ipConvexQP:InfNaNComplexDetected';
			'optim:linprog:FinalConstraintsViolated'% introduced in R2016A for new linprog
		};
		allowedexceptionstack{solvers == optimization.solver.Optimizer.SQPGS} = [
			struct(...
				'file',	which('slqpgs.Direction'),...
				'line',	'any'...
			);
			struct(...
				'file',	which('slqpgs.Direction'),...
				'line',	'any'...
			);
			struct(...
				'file',	which('slqpgs.Direction'),...
				'line',	'any'...
			)
		];
	end
	% allow bad index for objective function becoming Inf in SLP-GS
	hasallowedexceptions(solvers == optimization.solver.Optimizer.SLPGS) = true;
	if any(solvers == optimization.solver.Optimizer.SLPGS)
		allowedexceptions{solvers == optimization.solver.Optimizer.SLPGS} = {
			'MATLAB:badsubscript';
			'optim:ipqpcommon:ipConvexQP:InfNaNComplexDetected';
			'optim:linprog:FinalConstraintsViolated'% introduced in R2016A for new linprog
		};
		allowedexceptionstack{solvers == optimization.solver.Optimizer.SLPGS} = [
			struct(...
				'file',	which('slqpgs.Direction'),...
				'line',	'any'...
			);
			struct(...
				'file',	which('slqpgs.Direction'),...
				'line',	'any'...
			);
			struct(...
				'file',	which('slqpgs.Direction'),...
				'line',	'any'...
			)
		];
	end
	warnstate = warning('off', 'MATLAB:nearlySingularMatrix');
	iter = 0;
	for ii = 1:size(solvers, 1) %#ok<FORPF> parfor is used in optimization functions and parfor here would prevent parfor in the optimization functions from beeing parallelized
		algorithms = solvers(ii, 1).getAlgorithmChoices();
		if isempty(algorithms)
			algorithms = {'default'};
		end
		problemtype = solvers(ii, 1).getSupportedProblemTypes();
		for jj = 1:size(algorithms, 2)
			for kk = 1:size(problemtype, 1)
				for ll = 1:2
					for mm = 1:size(functiontype, 1)
						for nn = 1:size(constrainttype, 1)
							for oo = 0:1
								for pp = 1:2
									for rr = 1:size(A_ineq, 1)
										for ss = 1:size(A_eq, 1)
											if solvers(ii, 1) == optimization.solver.Optimizer.PPPBOX && strcmpi(algorithms{1, jj}, 'multi') && any(problemtype(kk, 1) == [
												optimization.options.ProblemType.CONSTRAINED;
												optimization.options.ProblemType.UNCONSTRAINED
											])
												continue;
											end
											if solvers(ii, 1) == optimization.solver.Optimizer.PPPBOX && strcmpi(algorithms{1, jj}, 'single') && any(problemtype(kk, 1) == [
												optimization.options.ProblemType.CONSTRAINEDMULTI;
												optimization.options.ProblemType.UNCONSTRAINEDMULTI
											])
												continue;
											end
											if solvers(ii, 1) == optimization.solver.Optimizer.GA && strcmpi(algorithms{1, jj}, 'gamulti') && any(problemtype(kk, 1) == [
												optimization.options.ProblemType.CONSTRAINED;
												optimization.options.ProblemType.UNCONSTRAINED
											])
												continue;
											end
											if solvers(ii, 1) == optimization.solver.Optimizer.GA && strcmpi(algorithms{1, jj}, 'ga') && any(problemtype(kk, 1) == [
												optimization.options.ProblemType.CONSTRAINEDMULTI;
												optimization.options.ProblemType.UNCONSTRAINEDMULTI
											])
												continue;
											end
											if any(solvers(ii, 1) == [
												optimization.solver.Optimizer.FMINCONGLOBAL;
												optimization.solver.Optimizer.FMINUNCGLOBAL
											]) && (oo > 0 || ll > 1)
												continue;
											end
											if any(solvers(ii, 1) == [
												optimization.solver.Optimizer.GA;
												optimization.solver.Optimizer.PARTICLESWARM
											])&& oo > 0
												continue;
											end
											if any(solvers(ii, 1) == [
												optimization.solver.Optimizer.FMINCON;
												optimization.solver.Optimizer.FMINUNC
											])&& ~configuration.optimization.hasoptimization()
												continue;
											end
											if any(solvers(ii, 1) == [
												optimization.solver.Optimizer.GA;
												optimization.solver.Optimizer.PARTICLESWARM;
												optimization.solver.Optimizer.PATTERNSEARCH;
												optimization.solver.Optimizer.SIMULANNEAL;
												optimization.solver.Optimizer.FMINCONGLOBAL;
												optimization.solver.Optimizer.FMINUNCGLOBAL
											])&& ~configuration.optimization.hasglobaloptimization()
												continue;
											end
											if solvers(ii, 1) == optimization.solver.Optimizer.IPOPT && strcmpi(algorithms{1, jj}, 'mumps')
												continue;
											end
											if any(solvers(ii, 1) == [
												optimization.solver.Optimizer.GA;
												optimization.solver.Optimizer.PARTICLESWARM;
												optimization.solver.Optimizer.FMINCONGLOBAL;
												optimization.solver.Optimizer.FMINUNCGLOBAL
											])&& pp > 1
												continue;
											end
											if any(solvers(ii, 1) == [
													optimization.solver.Optimizer.SLPGS;
													optimization.solver.Optimizer.SQPGS
												])
												rng('default');
											end
											hasfungrad = false;
											isfunvararg = false;
											hascongrad = false;
											isconvararg = false;
											hasfunhess = false;
											hasconhess = false;
											switch problemtype(kk, 1)
												case {optimization.options.ProblemType.CONSTRAINED, optimization.options.ProblemType.UNCONSTRAINED}
													switch lower(functiontype{mm, 1})
														case 'fun'
															J = @funsingle;
														case 'fungrad'
															J = @fungradsingle;
															hasfungrad = true;
														case 'funhess'
															J = @funhesssingle;
															hasfungrad = true;
															hasfunhess = true;
														case 'anonfun'
															J = anonfunsingle;
														case 'anonfungrad'
															J = anonfungradsingle;
															hasfungrad = true;
														case 'anonfunhess'
															J = anonfunhesssingle;
															hasfungrad = true;
															hasfunhess = true;
														case 'funvar'
															J = @funsinglevar;
															isfunvararg = true;
														case 'fungradvar'
															J = @fungradsinglevar;
															hasfungrad = true;
															isfunvararg = true;
														case 'funhessvar'
															J = @funhesssinglevar;
															hasfungrad = true;
															hasfunhess = true;
															isfunvararg = true;
														case 'anonfunvar'
															J = anonfunsinglevar;
															isfunvararg = true;
														case 'anonfungradvar'
															J = anonfungradsinglevar;
															hasfungrad = true;
															isfunvararg = true;
														case 'anonfunhessvar'
															J = anonfunhesssinglevar;
															hasfungrad = true;
															hasfunhess = true;
															isfunvararg = true;
														otherwise
															error('optimization:solver:test', 'Undefined objective function type.');
													end
													switch lower(constrainttype{nn, 1})
														case 'con'
															c = @con;
														case 'congrad'
															c = @congrad;
															hascongrad = true;
														case 'conhess'
															c = @conhess;
															hascongrad = true;
															hasconhess = true;
														case 'anoncon'
															c = anoncon;
														case 'anoncongrad'
															c = anoncongrad;
															hascongrad = true;
														case 'anonconhess'
															c = anonconhess;
															hascongrad = true;
															hasconhess = true;
														case 'convar'
															c = @convar;
															isconvararg = true;
														case 'congradvar'
															c = @congradvar;
															hascongrad = true;
															isconvararg = true;
														case 'conhessvar'
															c = @conhessvar;
															hascongrad = true;
															hasconhess = true;
															isconvararg = true;
														case 'anonconvar'
															c = anonconvar;
															isconvararg = true;
														case 'anoncongradvar'
															c = anoncongradvar;
															hascongrad = true;
															isconvararg = true;
														case 'anonconhessvar'
															c = anonconhessvar;
															hascongrad = true;
															hasconhess = true;
															isconvararg = true;
														case 'nocon'
															c = [];
															hascongrad = true;
															isconvararg = false;
														otherwise
															error('optimization:solver:test', 'Undefined constraint function type.');
													end
												case {optimization.options.ProblemType.CONSTRAINEDMULTI, optimization.options.ProblemType.UNCONSTRAINEDMULTI}
													%c = [];
													%hascongrad = true;
													switch lower(functiontype{mm, 1})
														case 'fun'
															J = @funmulti;
														case {'fungrad', 'funhess'}
															J = @fungradmulti;
															hasfungrad = true;
														case 'anonfun'
															J = anonfunmulti;
														case {'anonfungrad', 'anonfunhess'}
															J = anonfungradmulti;
															hasfungrad = true;
														case 'funvar'
															J = @funmultivar;
															isfunvararg = true;
														case {'fungradvar', 'funhessvar'}
															J = @fungradmultivar;
															hasfungrad = true;
															isfunvararg = true;
														case 'anonfunvar'
															J = anonfunmultivar;
															isfunvararg = true;
														case {'anonfungradvar', 'anonfunhessvar'}
															J = anonfungradmultivar;
															hasfungrad = true;
															isfunvararg = true;
														otherwise
															error('optimization:solver:test', 'Undefined objective function type.');
													end
													switch lower(constrainttype{nn, 1})
														case 'con'
															c = @con;
														case 'congrad'
															c = @congrad;
															hascongrad = true;
														case 'conhess'
															c = @conhess;
															hascongrad = true;
															hasconhess = true;
														case 'anoncon'
															c = anoncon;
														case 'anoncongrad'
															c = anoncongrad;
															hascongrad = true;
														case 'anonconhess'
															c = anonconhess;
															hascongrad = true;
															hasconhess = true;
														case 'convar'
															c = @convar;
															isconvararg = true;
														case 'congradvar'
															c = @congradvar;
															hascongrad = true;
															isconvararg = true;
														case 'conhessvar'
															c = @conhessvar;
															hascongrad = true;
															hasconhess = true;
															isconvararg = true;
														case 'anonconvar'
															c = anonconvar;
															isconvararg = true;
														case 'anoncongradvar'
															c = anoncongradvar;
															hascongrad = true;
															isconvararg = true;
														case 'anonconhessvar'
															c = anonconhessvar;
															hascongrad = true;
															hasconhess = true;
															isconvararg = true;
														case 'nocon'
															c = [];
															hascongrad = false;
															isconvararg = false;
														otherwise
															error('optimization:solver:test', 'Undefined constraint function type.');
													end
												otherwise
													error('optimization:solver:test', 'Undefined problem type.');
											end
											if any(solvers(ii, 1) == [
												optimization.solver.Optimizer.FMINCON;
												optimization.solver.Optimizer.FMINCONGLOBAL
											]) && strcmpi(algorithms{1, jj}, 'trust-region-reflective') && ~hasfungrad
												continue;
											end
											if any(solvers(ii, 1) == [
												optimization.solver.Optimizer.FMINCON;
												optimization.solver.Optimizer.FMINCONGLOBAL
											]) && strcmpi(algorithms{1, jj}, 'trust-region-reflective')
												c = [];
												hascongrad = true;
												isconvararg = false;
											end
											if any(solvers(ii, 1) == [
												optimization.solver.Optimizer.FMINUNC;
												optimization.solver.Optimizer.FMINUNCGLOBAL
											]) && strcmpi(algorithms{1, jj}, 'trust-region') && ~hasfungrad
												continue;
											end
											if solvers(ii, 1) == optimization.solver.Optimizer.IPOPT && (~hasfungrad || ~hascongrad)
												continue;
											end
											if solvers(ii, 1) == optimization.solver.Optimizer.MINFUNC && ~hasfungrad
												continue;
											end
											if solvers(ii, 1) == optimization.solver.Optimizer.MINFUNC
												if ~hasfunhess && any(strcmpi(algorithms{1, jj}, {'mnewton', 'newton'}))
													continue;
												end
											end
											if solvers(ii, 1) == optimization.solver.Optimizer.MINIMIZE && ~hasfungrad
												continue;
											end
											if (hasfunhess || hasconhess) && ~solvers(ii, 1).getHessianSupport()
												continue;
											end
											if ~isfunctionhandle(J)
												error('optimization:solver:test', 'Undefined objective function type.');
											end
											if ~isempty(c) && ~isfunctionhandle(c)
												error('optimization:solver:test', 'Undefined constraint function type.');
											end
											options = optimization.options.OptionFactory.instance.options(solvers(ii, 1),...
												'ProblemType',					problemtype(kk, 1),...
												'Retries',						ll,...
												'Algorithm',					algorithms{1, jj},...
												'FunctionTolerance',			1E-3,...
												'StepTolerance',				1E-3,...
												'ConstraintTolerance',			1E-3,...
												'MaxFunctionEvaluations',		50,...
												'MaxIterations',				20,...
												'MaxSQPIter',					500,...
												'SpecifyObjectiveGradient',		hasfungrad,...
												'SpecifyObjectiveHessian',		hasfunhess,...
												'SpecifyConstraintGradient',	hascongrad,...
												'SpecifyConstraintHessian',		hasconhess,...
												'CheckGradients',				false,...
												'FunValCheck',					false,...
												'FiniteDifferenceType',			'forward',...
												'Diagnostics',					false,...
												'Display',						'final-detailed',...
												'MaxTime',						1,...
												'UseParallel',					logical(oo)...
											);
											x = [];
											fval = [];
											exitflag = [];
											output = [];
											lambda = [];
											grad = [];
											hessian = [];
											if any(solvers(ii, 1) == [
												optimization.solver.Optimizer.NLOPTCON;
												optimization.solver.Optimizer.NLOPTCONGLOBAL;
												optimization.solver.Optimizer.NLOPTUNC;
												optimization.solver.Optimizer.NLOPTUNCGLOBAL
											])
												globalflag = any(solvers(ii, 1) == [
													optimization.solver.Optimizer.NLOPTCONGLOBAL;
													optimization.solver.Optimizer.NLOPTUNCGLOBAL
												]);
												[algorithm, needsgradient, isglobal] = optimization.options.nlopt.getalgorithm(options.Algorithm, hasfungrad, globalflag, options.ProblemType);
												if isnan(algorithm)
													[algorithm, needsgradient, isglobal] = optimization.options.nlopt.getalgorithm(options.Algorithm, ~hasfungrad, globalflag, options.ProblemType);
												end
												if isnan(algorithm)
													continue;
												end
												if needsgradient && ~hasfungrad
													continue;
												end
												if needsgradient && ~hascongrad && any(solvers(ii, 1) == [
													optimization.solver.Optimizer.NLOPTCON;
													optimization.solver.Optimizer.NLOPTCONGLOBAL
												])
													continue;
												end
												if solvers(ii, 1) == optimization.solver.Optimizer.NLOPTCONGLOBAL && ~isglobal
													continue;
												end
											end
											iter = iter + 1
											x_0 = [
												7;
												8;
												-10
											];
											x_0_init = repmat(x_0, 1, pp);
											y = 12; %#ok<NASGU>used in assertNoException
											A = A_ineq{rr, 1};
											b = b_ineq{rr, 1};
											Aeq = A_eq{ss, 1};
											beq = b_eq{ss, 1};
											lambda_expected = struct(...
												'lower',		[],...
												'upper',		[],...
												'ineqlin',		[],...
												'eqlin',		[],...
												'ineqnonlin',	[],...
												'eqnonlin',		[]...
											);
											if isfunvararg || isconvararg
												if hasallowedexceptions(ii, 1)
													test.TestSuite.assertNoExceptionExceptStack(['optimization.solver.', lower(char(solvers(ii, 1))), '.optimize(J, x_0_init, A, b, Aeq, beq, [], [], [], [], y);'], allowedexceptions{ii, 1}, allowedexceptionstack{ii, 1}, 'control:outputfeedback:test', 'optimize must not throw an exception for six input arguments.');
												else
													test.TestSuite.assertNoException(['optimization.solver.', lower(char(solvers(ii, 1))), '.optimize(J, x_0_init, A, b, Aeq, beq, [], [], [], [], y);'], 'control:outputfeedback:test', 'optimize must not throw an exception for six input arguments.');
												end
												if hasallowedexceptions(ii, 1)
													test.TestSuite.assertNoExceptionExceptStack(['optimization.solver.', lower(char(solvers(ii, 1))), '.optimize(J, x_0_init, A, b, Aeq, beq, lb, [], [], [], y);'], allowedexceptions{ii, 1}, allowedexceptionstack{ii, 1}, 'control:outputfeedback:test', 'optimize must not throw an exception for seven input arguments.');
												else
													test.TestSuite.assertNoException(['optimization.solver.', lower(char(solvers(ii, 1))), '.optimize(J, x_0_init, A, b, Aeq, beq, lb, [], [], [], y);'], 'control:outputfeedback:test', 'optimize must not throw an exception for seven input arguments.');
												end
												if hasallowedexceptions(ii, 1)
													test.TestSuite.assertNoExceptionExceptStack(['optimization.solver.', lower(char(solvers(ii, 1))), '.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub, [], [], y);'], allowedexceptions{ii, 1}, allowedexceptionstack{ii, 1}, 'control:outputfeedback:test', 'optimize must not throw an exception for eight input arguments.');
												else
													test.TestSuite.assertNoException(['optimization.solver.', lower(char(solvers(ii, 1))), '.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub, [], [], y);'], 'control:outputfeedback:test', 'optimize must not throw an exception for eight input arguments.');
												end
												if hasallowedexceptions(ii, 1)
													test.TestSuite.assertNoExceptionExceptStack(['optimization.solver.', lower(char(solvers(ii, 1))), '.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub, c, [], y);'], allowedexceptions{ii, 1}, allowedexceptionstack{ii, 1}, 'control:outputfeedback:test', 'optimize must not throw an exception for nine input arguments.');
												else
													test.TestSuite.assertNoException(['optimization.solver.', lower(char(solvers(ii, 1))), '.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub, c, [], y);'], 'control:outputfeedback:test', 'optimize must not throw an exception for nine input arguments.');
												end
												if hasallowedexceptions(ii, 1)
													test.TestSuite.assertNoExceptionExceptStack('optimization.solver.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options, y);', allowedexceptions{ii, 1}, allowedexceptionstack{ii, 1}, 'control:outputfeedback:test', 'optimize must not throw an exception for zero output arguments.');
												else
													test.TestSuite.assertNoException('optimization.solver.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options, y);', 'control:outputfeedback:test', 'optimize must not throw an exception for zero output arguments.');
												end
												initialize_outputs(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options, y);
												if hasallowedexceptions(ii, 1)
													test.TestSuite.assertNoExceptionExceptStack('[x] = optimization.solver.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options, y);', allowedexceptions{ii, 1}, allowedexceptionstack{ii, 1}, 'control:outputfeedback:test', 'optimize must not throw an exception for one output argument.');
												else
													test.TestSuite.assertNoException('[x] = optimization.solver.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options, y);', 'control:outputfeedback:test', 'optimize must not throw an exception for one output argument.');
												end
												test.TestSuite.assertSameSize(x, x_0, 'control:outputfeedback:test', 'optimal value must have same dimensions as initial value.');

												initialize_outputs(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options, y);
												if hasallowedexceptions(ii, 1)
													test.TestSuite.assertNoExceptionExceptStack('[x, fval] = optimization.solver.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options, y);', allowedexceptions{ii, 1}, allowedexceptionstack{ii, 1}, 'control:outputfeedback:test', 'optimize must not throw an exception for two output arguments.');
												else
													test.TestSuite.assertNoException('[x, fval] = optimization.solver.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options, y);', 'control:outputfeedback:test', 'optimize must not throw an exception for two output arguments.');
												end
												test.TestSuite.assertSameSize(x, x_0, 'control:outputfeedback:test', 'optimal value must have same dimensions as initial value.');
												test.TestSuite.assertSameSize(fval, 1, 'control:outputfeedback:test', 'optimal function value must be scalar.');

												initialize_outputs(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options, y);
												if hasallowedexceptions(ii, 1)
													test.TestSuite.assertNoExceptionExceptStack('[x, fval, exitflag] = optimization.solver.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options, y);', allowedexceptions{ii, 1}, allowedexceptionstack{ii, 1}, 'control:outputfeedback:test', 'optimize must not throw an exception for three output arguments.');
												else
													test.TestSuite.assertNoException('[x, fval, exitflag] = optimization.solver.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options, y);', 'control:outputfeedback:test', 'optimize must not throw an exception for three output arguments.');
												end
												test.TestSuite.assertSameSize(x, x_0, 'control:outputfeedback:test', 'optimal value must have same dimensions as initial value.');
												test.TestSuite.assertSameSize(fval, 1, 'control:outputfeedback:test', 'optimal function value must be scalar.');
												test.TestSuite.assertSameSize(exitflag, 1, 'control:outputfeedback:test', 'exitflag must be scalar.');

												initialize_outputs(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options, y);
												if hasallowedexceptions(ii, 1)
													test.TestSuite.assertNoExceptionExceptStack('[x, fval, exitflag, output] = optimization.solver.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options, y);', allowedexceptions{ii, 1}, allowedexceptionstack{ii, 1}, 'control:outputfeedback:test', 'optimize must not throw an exception for four output arguments.');
												else
													test.TestSuite.assertNoException('[x, fval, exitflag, output] = optimization.solver.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options, y);', 'control:outputfeedback:test', 'optimize must not throw an exception for four output arguments.');
												end
												test.TestSuite.assertSameSize(x, x_0, 'control:outputfeedback:test', 'optimal value must have same dimensions as initial value.');
												test.TestSuite.assertSameSize(fval, 1, 'control:outputfeedback:test', 'optimal function value must be scalar.');
												test.TestSuite.assertSameSize(exitflag, 1, 'control:outputfeedback:test', 'exitflag must be scalar.');
												test.TestSuite.assertFieldnames(output, optimization.options.Options.OUTPUTPROTOTYPEMULTIPLE, 'control:outputfeedback:test');
												if pp > 1
													test.TestSuite.assertEqual(length(output.runs), size(x_0_init, 2), 'control:outputfeedback:test');
												end

												initialize_outputs(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options, y);
												if hasallowedexceptions(ii, 1)
													test.TestSuite.assertNoExceptionExceptStack('[x, fval, exitflag, output, lambda] = optimization.solver.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options, y);', allowedexceptions{ii, 1}, allowedexceptionstack{ii, 1}, 'control:outputfeedback:test', 'optimize must not throw an exception for five output arguments.');
												else
													test.TestSuite.assertNoException('[x, fval, exitflag, output, lambda] = optimization.solver.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options, y);', 'control:outputfeedback:test', 'optimize must not throw an exception for five output arguments.');
												end
												test.TestSuite.assertSameSize(x, x_0, 'control:outputfeedback:test', 'optimal value must have same dimensions as initial value.');
												test.TestSuite.assertSameSize(fval, 1, 'control:outputfeedback:test', 'optimal function value must be scalar.');
												test.TestSuite.assertSameSize(exitflag, 1, 'control:outputfeedback:test', 'exitflag must be scalar.');
												test.TestSuite.assertFieldnames(output, optimization.options.Options.OUTPUTPROTOTYPEMULTIPLE, 'control:outputfeedback:test');
												if pp > 1
													test.TestSuite.assertEqual(length(output.runs), size(x_0_init, 2), 'control:outputfeedback:test');
												end
												test.TestSuite.assertFieldnames(lambda, lambda_expected, 'control:outputfeedback:test');

												initialize_outputs(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options, y);
												if hasallowedexceptions(ii, 1)
													test.TestSuite.assertNoExceptionExceptStack('[x, fval, exitflag, output, lambda, grad] = optimization.solver.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options, y);', allowedexceptions{ii, 1}, allowedexceptionstack{ii, 1}, 'control:outputfeedback:test', 'optimize must not throw an exception for six output arguments.');
												else
													test.TestSuite.assertNoException('[x, fval, exitflag, output, lambda, grad] = optimization.solver.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options, y);', 'control:outputfeedback:test', 'optimize must not throw an exception for six output arguments.');
												end
												test.TestSuite.assertSameSize(x, x_0, 'control:outputfeedback:test', 'optimal value must have same dimensions as initial value.');
												test.TestSuite.assertSameSize(fval, 1, 'control:outputfeedback:test', 'optimal function value must be scalar.');
												test.TestSuite.assertSameSize(exitflag, 1, 'control:outputfeedback:test', 'exitflag must be scalar.');
												test.TestSuite.assertFieldnames(output, optimization.options.Options.OUTPUTPROTOTYPEMULTIPLE, 'control:outputfeedback:test');
												if pp > 1
													test.TestSuite.assertEqual(length(output.runs), size(x_0_init, 2), 'control:outputfeedback:test');
												end
												test.TestSuite.assertFieldnames(lambda, lambda_expected, 'control:outputfeedback:test');
												test.TestSuite.assertSameSize(grad, x_0, 'control:outputfeedback:test', 'gradient must have same dimensions as initial value.');

												initialize_outputs(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options, y);
												if hasallowedexceptions(ii, 1)
													test.TestSuite.assertNoExceptionExceptStack('[x, fval, exitflag, output, lambda, grad, hessian] = optimization.solver.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options, y);', allowedexceptions{ii, 1}, allowedexceptionstack{ii, 1}, 'control:outputfeedback:test', 'optimize must not throw an exception for seven output arguments.');
												else
													test.TestSuite.assertNoException('[x, fval, exitflag, output, lambda, grad, hessian] = optimization.solver.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options, y);', 'control:outputfeedback:test', 'optimize must not throw an exception for seven output arguments.');
												end
												test.TestSuite.assertSameSize(x, x_0, 'control:outputfeedback:test', 'optimal value must have same dimensions as initial value.');
												test.TestSuite.assertSameSize(fval, 1, 'control:outputfeedback:test', 'optimal function value must be scalar.');
												test.TestSuite.assertSameSize(exitflag, 1, 'control:outputfeedback:test', 'exitflag must be scalar.');
												test.TestSuite.assertFieldnames(output, optimization.options.Options.OUTPUTPROTOTYPEMULTIPLE, 'control:outputfeedback:test');
												if pp > 1
													test.TestSuite.assertEqual(length(output.runs), size(x_0_init, 2), 'control:outputfeedback:test');
												end
												test.TestSuite.assertFieldnames(lambda, lambda_expected, 'control:outputfeedback:test');
												test.TestSuite.assertSameSize(grad, x_0, 'control:outputfeedback:test', 'gradient must have same dimensions as initial value.');
												test.TestSuite.assertSameSize(hessian, x_0*x_0', 'control:outputfeedback:test', 'hessian matrix must have same dimensions as initial value.');
											else
												if hasallowedexceptions(ii, 1)
													test.TestSuite.assertNoExceptionExceptStack(['optimization.solver.', lower(char(solvers(ii, 1))), '.optimize(J, x_0_init);'], allowedexceptions{ii, 1}, allowedexceptionstack{ii, 1}, 'control:outputfeedback:test', 'optimize must not throw an exception for two input arguments.');
												else
													test.TestSuite.assertNoException(['optimization.solver.', lower(char(solvers(ii, 1))), '.optimize(J, x_0_init);'], 'control:outputfeedback:test', 'optimize must not throw an exception for two input arguments.');
												end
												if hasallowedexceptions(ii, 1)
													test.TestSuite.assertNoExceptionExceptStack(['optimization.solver.', lower(char(solvers(ii, 1))), '.optimize(J, x_0_init, A, b);'], allowedexceptions{ii, 1}, allowedexceptionstack{ii, 1}, 'control:outputfeedback:test', 'optimize must not throw an exception for four input arguments.');
												else
													test.TestSuite.assertNoException(['optimization.solver.', lower(char(solvers(ii, 1))), '.optimize(J, x_0_init, A, b);'], 'control:outputfeedback:test', 'optimize must not throw an exception for four input arguments.');
												end
												if hasallowedexceptions(ii, 1)
													test.TestSuite.assertNoExceptionExceptStack(['optimization.solver.', lower(char(solvers(ii, 1))), '.optimize(J, x_0_init, A, b, Aeq, beq);'], allowedexceptions{ii, 1}, allowedexceptionstack{ii, 1}, 'control:outputfeedback:test', 'optimize must not throw an exception for six input arguments.');
												else
													test.TestSuite.assertNoException(['optimization.solver.', lower(char(solvers(ii, 1))), '.optimize(J, x_0_init, A, b, Aeq, beq);'], 'control:outputfeedback:test', 'optimize must not throw an exception for six input arguments.');
												end
												if hasallowedexceptions(ii, 1)
													test.TestSuite.assertNoExceptionExceptStack(['optimization.solver.', lower(char(solvers(ii, 1))), '.optimize(J, x_0_init, A, b, Aeq, beq, lb);'], allowedexceptions{ii, 1}, allowedexceptionstack{ii, 1}, 'control:outputfeedback:test', 'optimize must not throw an exception for seven input arguments.');
												else
													test.TestSuite.assertNoException(['optimization.solver.', lower(char(solvers(ii, 1))), '.optimize(J, x_0_init, A, b, Aeq, beq, lb);'], 'control:outputfeedback:test', 'optimize must not throw an exception for seven input arguments.');
												end
												if hasallowedexceptions(ii, 1)
													test.TestSuite.assertNoExceptionExceptStack(['optimization.solver.', lower(char(solvers(ii, 1))), '.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub);'], allowedexceptions{ii, 1}, allowedexceptionstack{ii, 1}, 'control:outputfeedback:test', 'optimize must not throw an exception for eight input arguments.');
												else
													test.TestSuite.assertNoException(['optimization.solver.', lower(char(solvers(ii, 1))), '.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub);'], 'control:outputfeedback:test', 'optimize must not throw an exception for eight input arguments.');
												end
												if hasallowedexceptions(ii, 1)
													test.TestSuite.assertNoExceptionExceptStack(['optimization.solver.', lower(char(solvers(ii, 1))), '.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub, c);'], allowedexceptions{ii, 1}, allowedexceptionstack{ii, 1}, 'control:outputfeedback:test', 'optimize must not throw an exception for nine input arguments.');
												else
													test.TestSuite.assertNoException(['optimization.solver.', lower(char(solvers(ii, 1))), '.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub, c);'], 'control:outputfeedback:test', 'optimize must not throw an exception for nine input arguments.');
												end
												if hasallowedexceptions(ii, 1)
													test.TestSuite.assertNoExceptionExceptStack('optimization.solver.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options);', allowedexceptions{ii, 1}, allowedexceptionstack{ii, 1}, 'control:outputfeedback:test', 'optimize must not throw an exception for zero output arguments.');
												else
													test.TestSuite.assertNoException('optimization.solver.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options);', 'control:outputfeedback:test', 'optimize must not throw an exception for zero output arguments.');
												end
												initialize_outputs(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options, y);
												if hasallowedexceptions(ii, 1)
													test.TestSuite.assertNoExceptionExceptStack('[x] = optimization.solver.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options);', allowedexceptions{ii, 1}, allowedexceptionstack{ii, 1}, 'control:outputfeedback:test', 'optimize must not throw an exception for one output argument.');
												else
													test.TestSuite.assertNoException('[x] = optimization.solver.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options);', 'control:outputfeedback:test', 'optimize must not throw an exception for one output argument.');
												end
												test.TestSuite.assertSameSize(x, x_0, 'control:outputfeedback:test', 'optimal value must have same dimensions as initial value.');

												initialize_outputs(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options, y);
												if hasallowedexceptions(ii, 1)
													test.TestSuite.assertNoExceptionExceptStack('[x, fval] = optimization.solver.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options);', allowedexceptions{ii, 1}, allowedexceptionstack{ii, 1}, 'control:outputfeedback:test', 'optimize must not throw an exception for two output arguments.');
												else
													test.TestSuite.assertNoException('[x, fval] = optimization.solver.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options);', 'control:outputfeedback:test', 'optimize must not throw an exception for two output arguments.');
												end
												test.TestSuite.assertSameSize(x, x_0, 'control:outputfeedback:test', 'optimal value must have same dimensions as initial value.');
												test.TestSuite.assertSameSize(fval, 1, 'control:outputfeedback:test', 'optimal function value must be scalar.');

												if hasallowedexceptions(ii, 1)
													test.TestSuite.assertNoExceptionExceptStack('[x, fval, exitflag] = optimization.solver.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options);', allowedexceptions{ii, 1}, allowedexceptionstack{ii, 1}, 'control:outputfeedback:test', 'optimize must not throw an exception for three output arguments.');
												else
													test.TestSuite.assertNoException('[x, fval, exitflag] = optimization.solver.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options);', 'control:outputfeedback:test', 'optimize must not throw an exception for three output arguments.');
												end
												test.TestSuite.assertSameSize(x, x_0, 'control:outputfeedback:test', 'optimal value must have same dimensions as initial value.');
												test.TestSuite.assertSameSize(fval, 1, 'control:outputfeedback:test', 'optimal function value must be scalar.');
												test.TestSuite.assertSameSize(exitflag, 1, 'control:outputfeedback:test', 'exitflag must be scalar.');

												initialize_outputs(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options, y);
												if hasallowedexceptions(ii, 1)
													test.TestSuite.assertNoExceptionExceptStack('[x, fval, exitflag, output] = optimization.solver.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options);', allowedexceptions{ii, 1}, allowedexceptionstack{ii, 1}, 'control:outputfeedback:test', 'optimize must not throw an exception for four output arguments.');
												else
													test.TestSuite.assertNoException('[x, fval, exitflag, output] = optimization.solver.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options);', 'control:outputfeedback:test', 'optimize must not throw an exception for four output arguments.');
												end
												test.TestSuite.assertSameSize(x, x_0, 'control:outputfeedback:test', 'optimal value must have same dimensions as initial value.');
												test.TestSuite.assertSameSize(fval, 1, 'control:outputfeedback:test', 'optimal function value must be scalar.');
												test.TestSuite.assertSameSize(exitflag, 1, 'control:outputfeedback:test', 'exitflag must be scalar.');
												test.TestSuite.assertFieldnames(output, optimization.options.Options.OUTPUTPROTOTYPEMULTIPLE, 'control:outputfeedback:test');
												if pp > 1
													test.TestSuite.assertEqual(length(output.runs), size(x_0_init, 2), 'control:outputfeedback:test');
												end

												initialize_outputs(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options, y);
												if hasallowedexceptions(ii, 1)
													test.TestSuite.assertNoExceptionExceptStack('[x, fval, exitflag, output, lambda] = optimization.solver.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options);', allowedexceptions{ii, 1}, allowedexceptionstack{ii, 1}, 'control:outputfeedback:test', 'optimize must not throw an exception for five output arguments.');
												else
													test.TestSuite.assertNoException('[x, fval, exitflag, output, lambda] = optimization.solver.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options);', 'control:outputfeedback:test', 'optimize must not throw an exception for five output arguments.');
												end
												test.TestSuite.assertSameSize(x, x_0, 'control:outputfeedback:test', 'optimal value must have same dimensions as initial value.');
												test.TestSuite.assertSameSize(fval, 1, 'control:outputfeedback:test', 'optimal function value must be scalar.');
												test.TestSuite.assertSameSize(exitflag, 1, 'control:outputfeedback:test', 'exitflag must be scalar.');
												test.TestSuite.assertFieldnames(output, optimization.options.Options.OUTPUTPROTOTYPEMULTIPLE, 'control:outputfeedback:test');
												if pp > 1
													test.TestSuite.assertEqual(length(output.runs), size(x_0_init, 2), 'control:outputfeedback:test');
												end
												test.TestSuite.assertFieldnames(lambda, lambda_expected, 'control:outputfeedback:test');

												initialize_outputs(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options, y);
												if hasallowedexceptions(ii, 1)
													test.TestSuite.assertNoExceptionExceptStack('[x, fval, exitflag, output, lambda, grad] = optimization.solver.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options);', allowedexceptions{ii, 1}, allowedexceptionstack{ii, 1}, 'control:outputfeedback:test', 'optimize must not throw an exception for six output arguments.');
												else
													test.TestSuite.assertNoException('[x, fval, exitflag, output, lambda, grad] = optimization.solver.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options);', 'control:outputfeedback:test', 'optimize must not throw an exception for six output arguments.');
												end
												test.TestSuite.assertSameSize(x, x_0, 'control:outputfeedback:test', 'optimal value must have same dimensions as initial value.');
												test.TestSuite.assertSameSize(fval, 1, 'control:outputfeedback:test', 'optimal function value must be scalar.');
												test.TestSuite.assertSameSize(exitflag, 1, 'control:outputfeedback:test', 'exitflag must be scalar.');
												test.TestSuite.assertFieldnames(output, optimization.options.Options.OUTPUTPROTOTYPEMULTIPLE, 'control:outputfeedback:test');
												if pp > 1
													test.TestSuite.assertEqual(length(output.runs), size(x_0_init, 2), 'control:outputfeedback:test');
												end
												test.TestSuite.assertFieldnames(lambda, lambda_expected, 'control:outputfeedback:test');
												test.TestSuite.assertSameSize(grad, x_0, 'control:outputfeedback:test', 'gradient must have same dimensions as initial value.');

												initialize_outputs(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options, y);
												if hasallowedexceptions(ii, 1)
													test.TestSuite.assertNoExceptionExceptStack('[x, fval, exitflag, output, lambda, grad, hessian] = optimization.solver.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options);', allowedexceptions{ii, 1}, allowedexceptionstack{ii, 1}, 'control:outputfeedback:test', 'optimize must not throw an exception for seven output arguments.');
												else
													test.TestSuite.assertNoException('[x, fval, exitflag, output, lambda, grad, hessian] = optimization.solver.optimize(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options);', 'control:outputfeedback:test', 'optimize must not throw an exception for seven output arguments.');
												end
												test.TestSuite.assertSameSize(x, x_0, 'control:outputfeedback:test', 'optimal value must have same dimensions as initial value.');
												test.TestSuite.assertSameSize(fval, 1, 'control:outputfeedback:test', 'optimal function value must be scalar.');
												test.TestSuite.assertSameSize(exitflag, 1, 'control:outputfeedback:test', 'exitflag must be scalar.');
												test.TestSuite.assertFieldnames(output, optimization.options.Options.OUTPUTPROTOTYPEMULTIPLE, 'control:outputfeedback:test');
												if pp > 1
													test.TestSuite.assertEqual(length(output.runs), size(x_0_init, 2), 'control:outputfeedback:test');
												end
												test.TestSuite.assertFieldnames(lambda, lambda_expected, 'control:outputfeedback:test');
												test.TestSuite.assertSameSize(grad, x_0, 'control:outputfeedback:test', 'gradient must have same dimensions as initial value.');
												test.TestSuite.assertSameSize(hessian, x_0*x_0', 'control:outputfeedback:test', 'hessian matrix must have same dimensions as initial value.');
											end
										end
									end
								end
							end
						end
					end
				end
			end
		end
	end
	warning(warnstate);
end

function [] = initialize_outputs(J, x_0_init, A, b, Aeq, beq, lb, ub, c, options, varargin)
	x = x_0_init(:, 1);
	if nargin(J) > 1
		fval = J(x, varargin{:});
	else
		fval = J(x);
	end
	exitflag = 0;
	output = optimization.options.Options.OUTPUTPROTOTYPEMULTIPLE;
	if size(x_0_init, 2) > 1
		output.runs = cell(size(x_0_init, 2));
	end
	lambda = struct(...
		'lower',		[],...
		'upper',		[],...
		'ineqlin',		[],...
		'eqlin',		[],...
		'ineqnonlin',	[],...
		'eqnonlin',		[]...
	);
	grad = x;
	hessian = x*x';
	assignin('caller', 'x', x);
	assignin('caller', 'fval', fval);
	assignin('caller', 'exitflag', exitflag);
	assignin('caller', 'output', output);
	assignin('caller', 'lambda', lambda);
	assignin('caller', 'grad', grad);
	assignin('caller', 'hessian', hessian);
end

function [J, gradJ] = fungradsingle(x)
	J = x(1)^2 + x(2) - x(3)^4;
	if nargout >= 2
		gradJ = [
			2*x(1)
			1;
			-4*x(3)^3
		];
	end
end

function [J, gradJ, hessJ] = funhesssingle(x)
	J = x(1)^2 + x(2) - x(3)^4;
	if nargout >= 2
		gradJ = [
			2*x(1)
			1;
			-4*x(3)^3
		];
		if nargout >= 3
			hessJ = [
				2,	0,	0;
				0,	0,	0;
				0,	0,	-12*x(3)^2
			];
		end
	end
end

function [J, gradJ] = fungradsinglevar(x, y)
	J = y*x(1)^2 + x(2) - x(3)^4;
	if nargout >= 2
		gradJ = [
			y*2*x(1);
			1;
			-4*x(3)^3
		];
	end
end

function [J, gradJ, hessJ] = funhesssinglevar(x, y)
	J = y*x(1)^2 + x(2) - x(3)^4;
	if nargout >= 2
		gradJ = [
			y*2*x(1);
			1;
			-4*x(3)^3
		];
		if nargout >= 3
			hessJ = [
				2,	0,	0;
				0,	0,	0;
				0,	0,	-12*x(3)^2
			];
		end
	end
end

function [J] = funsingle(x)
	J = x(1)^2 + x(2) - x(3)^4;
end

function [J] = funsinglevar(x, y)
	J = y*x(1)^2 + x(2) - x(3)^4;
end

function [J, gradJ] = fungradmulti(x)
	J = [
		x(1)^2 + x(2) - x(3)^4;
		(x(1) - 3)^2;
		x(2)^2 + x(3);
		x(1)
	];
	if nargout >= 2
		gradJ = [
			2*x(1),			1,		-4*x(3)^3;
			2*(x(1) - 3),	0,		0;
			0,				2*x(2),	1;
			1,				0,		0
		]';
	end
end

function [J, gradJ] = fungradmultivar(x, y)
	J = [
		y*x(1)^2 + x(2) - x(3)^4;
		(x(1) - 3)^2;
		x(2)^2 + x(3);
		x(1)
	];
	if nargout >= 2
		gradJ = [
			y*2*x(1),		1,		-4*x(3)^3;
			2*(x(1) - 3),	0,		0;
			0,				2*x(2),	1;
			1,				0,		0
		]';
	end
end

function [J] = funmulti(x)
	J = [
		x(1)^2 + x(2) - x(3)^4;
		(x(1) - 3)^2;
		x(2)^2 + x(3)
	];
end

function [J] = funmultivar(x, y)
	J = [
		y*x(1)^2 + x(2) - x(3)^4;
		(x(1) - 3)^2;
		x(2)^2 + x(3)
	];
end

function [c, ceq, gradc, gradceq, hessc, hessceq] = conhess(x)
	c = [
		x(3) - 100;
		x(1) + x(2) - 1000;
		-x(1) - x(2) + 50;
		x(1)^2 + x(2)^2 - 75^2
	];
	ceq = [];
	if nargout >= 3
		gradc = [
			0,	0,	1;
			1,	1,	0;
			-1,	-1,	0;
			2*x(1), 2*x(2), 0
		]';
		gradceq = [];
		if nargout >= 5
			hessc = cat(3, [
				0,	0,	0;
				0,	0,	0;
				0,	0,	0
			], [
				0,	0,	0;
				0,	0,	0;
				0,	0,	0
			], [
				0,	0,	0;
				0,	0,	0;
				0,	0,	0
			], [
				2,	0,	0;
				0,	2,	0;
				0,	0,	0
			]);
			hessceq = [];
		end
	end
end

function [c, ceq, gradc, gradceq] = congrad(x)
	c = [
		x(3) - 100;
		x(1) + x(2) - 1000;
		-x(1) - x(2) + 50;
		x(1)^2 + x(2)^2 - 75^2
	];
	ceq = [];
	if nargout >= 3
		gradc = [
			0,	0,	1;
			1,	1,	0;
			-1,	-1,	0;
			2*x(1), 2*x(2), 0
		]';
		gradceq = [];
	end
end

function [c, ceq] = con(x)
	c = [
		x(3) - 100;
		x(1) + x(2) - 1000;
		-x(1) - x(2) + 50;
		x(1)^2 + x(2)^2 - 75^2
	];
	ceq = [];
end

function [c, ceq, gradc, gradceq, hessc, hessceq] = conhessvar(x, y)
	c = [
		y*x(3) - 100;
		x(1) + x(2) - 1000;
		-x(1) - x(2) + 50;
		x(1)^2 + x(2)^2 - 75^2
	];
	ceq = [];
	if nargout >= 3
		gradc = [
			0,	0,	y;
			1,	1,	0;
			-1,	-1,	0;
			2*x(1), 2*x(2), 0
		]';
		gradceq = [];
		if nargout >= 3
			hessc = cat(3, [
				0,	0,	0;
				0,	0,	0;
				0,	0,	0
			], [
				0,	0,	0;
				0,	0,	0;
				0,	0,	0
			], [
				0,	0,	0;
				0,	0,	0;
				0,	0,	0
			], [
				2,	0,	0;
				0,	2,	0;
				0,	0,	0
			]);
			hessceq = [];
		end
	end
end

function [c, ceq, gradc, gradceq] = congradvar(x, y)
	c = [
		y*x(3) - 100;
		x(1) + x(2) - 1000;
		-x(1) - x(2) + 50;
		x(1)^2 + x(2)^2 - 75^2
	];
	ceq = [];
	if nargout >= 3
		gradc = [
			0,	0,	y;
			1,	1,	0;
			-1,	-1,	0;
			2*x(1), 2*x(2), 0
		]';
		gradceq = [];
	end
end

function [c, ceq] = convar(x, y)
	c = [
		y*x(3) - 100;
		x(1) + x(2) - 1000;
		-x(1) - x(2) + 50;
		x(1)^2 + x(2)^2 - 75^2
	];
	ceq = [];
end
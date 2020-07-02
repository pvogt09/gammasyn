function [algorithm, needsgradient, isglobal, supportsconstraints, needsbounds, algos] = getalgorithm(basealgorithm, gradientflag, globalflag, problemtype)
	%GETALGORITHM get number of algorithm for nlopt in dependence of user supplied settings
	%	Input:
	%		basealgorithm:			name of algorithm set by user
	%		gradientflag:			indicator, if gradient can be supplied
	%		globalflag:				indicator, if a global algorithm should be used
	%		problemtype:			type of optimization problem to solve
	%	Output:
	%		algorithm:				number of algorithm to use
	%		needsgradient:			indicator, if the selected algorithm needs gradient information
	%		isglobal:				indicator, if the algorithm is a global algorithm
	%		supportsconstraints:	indicator, if the algorithm supports constraints
	%		needsbounds:			indicator, if the algorithm needs finite lower and upper bounds
	%		algos:					list of all possible algorithms with their properties
	persistent algorithms;
	if isempty(algorithms)
		algorithms = {
			%algorithmname						name					gradient	global	constr needsbounds	optionvalue
			'NLOPT_AUGLAG',						'auglag',					true,	true,	true,	false;
			'NLOPT_AUGLAG_EQ',					'auglag_eq',				true,	true,	true,	false;
			'NLOPT_GD_MLSL',					'mlsl',						true,	true,	false,	true;
			'NLOPT_GD_MLSL_LDS',				'mlsl_lds',					true,	true,	false,	true;
			'NLOPT_GD_STOGO',					'stogo',					true,	false,	false,	true;
			'NLOPT_GD_STOGO_RAND',				'stogo_rand',				true,	false,	false,	true;
			'NLOPT_GN_CRS2_LM',					'crs2',						false,	false,	false,	true;
			'NLOPT_GN_DIRECT',					'direct',					false,	false,	false,	true;
			'NLOPT_GN_DIRECT_L',				'direct_l',					false,	false,	false,	true;
			'NLOPT_GN_DIRECT_L_NOSCAL',			'direct_l_noscal',			false,	false,	false,	true;
			'NLOPT_GN_DIRECT_L_RAND',			'direct_l_rand',			false,	false,	false,	true;
			'NLOPT_GN_DIRECT_L_RAND_NOSCAL',	'direct_l_rand_noscal',		false,	false,	false,	true;
			'NLOPT_GN_DIRECT_NOSCAL',			'direct_noscal',			false,	false,	false,	true;
			'NLOPT_GN_ESCH',					'esch',						false,	false,	false,	true;
			'NLOPT_GN_ISRES',					'isres',					false,	false,	true,	true;
			'NLOPT_GN_MLSL',					'mlsl',						false,	true,	false,	true;
			'NLOPT_GN_MLSL_LDS',				'mlsl_lds',					false,	true,	false,	true;
			'NLOPT_GN_ORIG_DIRECT',				'direct_orig',				false,	false,	false,	true;
			'NLOPT_GN_ORIG_DIRECT_L',			'direct_l_orig',			false,	false,	false,	true;
			%'NLOPT_G_MLSL',					'mlsl',						true,	true,	false,	true;
			%'NLOPT_G_MLSL_LDS',				'mlsl_lds',					true,	true,	false,	true;
			'NLOPT_LD_AUGLAG',					'auglag',					true,	false,	true,	false;
			'NLOPT_LD_AUGLAG_EQ',				'auglag_eq',				true,	false,	true,	false;
			'NLOPT_LD_CCSAQ',					'ccsaq',					true,	false,	false,	false;
			'NLOPT_LD_LBFGS',					'lbfgs',					true,	false,	false,	false;
			'NLOPT_LD_LBFGS_NOCEDAL',			'lbfgs_nocedal',			true,	false,	false,	false;
			'NLOPT_LD_MMA',						'mma',						true,	false,	true,	false;
			'NLOPT_LD_SLSQP',					'slsqp',					true,	false,	true,	false;
			'NLOPT_LD_TNEWTON',					'tnewton',					true,	false,	false,	false;
			'NLOPT_LD_TNEWTON_PRECOND',			'tnewton_precond',			true,	false,	false,	false;
			'NLOPT_LD_TNEWTON_PRECOND_RESTART',	'tnewton_precond_restart',	true,	false,	false,	false;
			'NLOPT_LD_TNEWTON_RESTART',			'tnewton_restart',			true,	false,	false,	false;
			'NLOPT_LD_VAR1',					'var1',						true,	false,	false,	false;
			'NLOPT_LD_VAR2',					'var2',						true,	false,	false,	false;
			'NLOPT_LN_AUGLAG',					'auglag',					false,	false,	true,	false;
			'NLOPT_LN_AUGLAG_EQ',				'auglag_eq',				false,	false,	true,	false;
			'NLOPT_LN_BOBYQA',					'bobyoa'					false,	false,	false,	false;
			'NLOPT_LN_COBYLA',					'cobyla',					false,	false,	true,	false;
			'NLOPT_LN_NELDERMEAD',				'neldermead',				false,	false,	false,	false;
			'NLOPT_LN_NEWUOA',					'newuoa',					false,	false,	false,	false;
			'NLOPT_LN_NEWUOA_BOUND',			'newuoa_bound',				false,	false,	false,	false;
			'NLOPT_LN_PRAXIS',					'praxis',					false,	false,	false,	false;
			'NLOPT_LN_SBPLX',					'subplex',					false,	false,	false,	false
		};
		algorithms = [algorithms, cell(size(algorithms, 1), 1)];
		for ii = 1:size(algorithms, 1) %#ok<FORPF> no parfor for function pointer creation
			fun = str2func(['optimization.options.nlopt.', algorithms{ii, 1}]);
			algorithms{ii, 7} = fun();
		end
	end
	if nargin <= 4
		problemtype = optimization.options.ProblemType.UNCONSTRAINED;
	end
	if nargout >= 6
		algos = algorithms;
	end
	possiblealgorithms = algorithms(strcmpi(basealgorithm, algorithms(:, 2)), :);
	if isempty(possiblealgorithms)
		error('optimization:solver:nlopt:input', 'Current algorithm is not supported.');
	end
	if problemtype == optimization.options.ProblemType.CONSTRAINED && any([possiblealgorithms{:, 5}])
		possiblealgorithms = possiblealgorithms([possiblealgorithms{:, 5}], :);
	end
	if isempty(possiblealgorithms)
		algorithm = NaN;
		needsgradient = false;
		isglobal = false;
		supportsconstraints = false;
		needsbounds = false;
		return;
	end
	if size(possiblealgorithms, 1) == 1
		algorithm = possiblealgorithms{1, 7};
		needsgradient = possiblealgorithms{1, 3};
		isglobal = possiblealgorithms{1, 4};
		supportsconstraints = possiblealgorithms{1, 5};
		needsbounds = possiblealgorithms{1, 6};
		return;
	end
	if gradientflag && any([possiblealgorithms{:, 3}])
		gradientalgorithms = possiblealgorithms([possiblealgorithms{:, 3}], :);
		if size(gradientalgorithms, 1) == 1
			algorithm = gradientalgorithms{1, 7};
			needsgradient = gradientalgorithms{1, 3};
			isglobal = gradientalgorithms{1, 4};
			supportsconstraints = gradientalgorithms{1, 5};
			needsbounds = gradientalgorithms{1, 6};
			return;
		end
		possiblealgorithms = gradientalgorithms;
	end
	if ~gradientflag && any(~[possiblealgorithms{:, 3}])
		gradientalgorithms = possiblealgorithms(~[possiblealgorithms{:, 3}], :);
		if size(gradientalgorithms, 1) == 1
			algorithm = gradientalgorithms{1, 7};
			needsgradient = gradientalgorithms{1, 3};
			isglobal = gradientalgorithms{1, 4};
			supportsconstraints = gradientalgorithms{1, 5};
			needsbounds = gradientalgorithms{1, 6};
			return;
		end
		possiblealgorithms = gradientalgorithms;
	end
	if globalflag && any([possiblealgorithms{:, 4}])
		possiblealgorithms = possiblealgorithms([possiblealgorithms{:, 4}], :);
	else
		possiblealgorithms = possiblealgorithms(~[possiblealgorithms{:, 4}], :);
	end
	if isempty(possiblealgorithms)
		algorithm = NaN;
		needsgradient = false;
		isglobal = false;
		supportsconstraints = false;
		needsbounds = false;
		return;
	end
	if size(possiblealgorithms, 1) == 1
		algorithm = possiblealgorithms{1, 7};
		needsgradient = possiblealgorithms{1, 3};
		isglobal = possiblealgorithms{1, 4};
		supportsconstraints = possiblealgorithms{1, 5};
		needsbounds = possiblealgorithms{1, 6};
		return;
	else
		error('optimization:solver:nlopt:input', 'Current algorithm is not supported.');
	end
end
function [pass] = gammasynTest(~)
	%GAMMASYNTEST test cases for checking gammasyn for correct argument handling
	%	Input:
	%		silent:	idicator, if information should be output
	%	Output:
	%		pass:	indicator, if test was passed
	pass = identifier.TestResult.PASSED;
	solvers = [
		optimization.solver.Optimizer.FMINCON;% should be used to check constrained optimization
		optimization.solver.Optimizer.IPOPT;% should be used because it does not allow complex return values and also needs structural information (i.e. isallKNaN == true)
		optimization.solver.Optimizer.FMINUNC;% should be used to check unconstrained optimization
		optimization.solver.Optimizer.FMINIMAX% should be used to test multiobjective optimization
	];
	derivativetype = [
		GammaEigenvalueDerivativeType.DEFAULT;
		GammaEigenvalueDerivativeType.RUDISILLCHU;
		GammaEigenvalueDerivativeType.VANDERAA
	];
	needsderivative = [
		false(1, 4);
		true(1, 4);
		perms([
			true, false, false, false
		]);
		perms([
			true, true, false, false
		]);
		perms([
			true, true, true, false
		])
	];

	R = 50;
	a = 0.65;
	b = 0.5;
	poleareatypes = {
		control.design.gamma.area.Circle(R);
		control.design.gamma.area.Circlesquare(R);
		control.design.gamma.area.CircleDiscrete(R);
		control.design.gamma.area.Ellipse(1, 2);
		control.design.gamma.area.Ellipsesquare(1, 2);
		control.design.gamma.area.Hyperbola(a, b);
		control.design.gamma.area.Hyperbolasquare(a, b);
		control.design.gamma.area.Imag(1, 0);
		control.design.gamma.area.Line(1, 1);
		control.design.gamma.area.LogSpiral(R, 1);
		control.design.gamma.area.None();
		control.design.gamma.area.PolyEllipse([1 + 1i, 1 - 1i, -2], [1, 1, 3]);
		control.design.gamma.area.PolyEllipsesquare([1 + 1i, 1 - 1i, -2], [1, 1, 3])
	};
	poleareafun = @(x) flexargout(2*real(x), 2, 0, 0, 0, 0, 0);
	number_models = [1, 2, 5];
	allowvarorder = [false, true];
	use_measurements_xdot = [false, true];
	use_references = [false, true];
	number_controls = [1, 2];
	number_measurements = [1, 2];
	number_measurements_xdot = [0, 1, 2];
	number_references = [0, 1, 2];
	number_states = 3;% number of states
	number_states_var = 2;% number of states when allowvarorder == true
	descriptor = [false, true];
	weight = 0.001;
	objective_types = enumeration('GammaJType');
	objective_types_cell = num2cell(objective_types);
	objective_types_cell(objective_types == GammaJType.EIGENVALUECONDITION, :) = [];
	objective_types_cell(objective_types == GammaJType.LOG, :) = [];% TODO: can not be used unsupervised automatically because J == Inf
	objective_types = cat(1, objective_types_cell{:});
	objectiveweight = ones(size(objective_types, 1), 1);
	objectiveweight(objective_types == GammaJType.LOG) = 1E-20;

	info = struct();
	areafunflag = [0, 1, 2, 3];
	areacombine = [false, true];
	areasizeflag = [0, 1, 2];
	eigenvectorobjective = [false, true];
	usecompiled = [false, true];
	for ii = 1:size(number_models, 2) %#ok<FORPF> parfor is used in optimization functions and parfor here would prevent parfor in the optimization functions from being parallelized
		for jj = 1:size(use_measurements_xdot, 2)
			for kk = 1:size(use_references, 2)
				for ll = 1:size(areafunflag, 2)
					for mm = 1:size(areacombine, 2)
						for oo = 1:size(areasizeflag, 2)
							for pp = 1:size(allowvarorder, 2)
								for qq = 1:size(derivativetype, 2)
									for rr = 1:size(needsderivative, 1)
										for ss = 1:size(solvers, 1)
											for tt = 1:size(eigenvectorobjective, 2)
												for uu = 1:size(usecompiled, 2)
													for vv = 1:size(number_controls, 2)
														for ww = 1:size(number_measurements, 2)
															for xx = 1:size(number_measurements_xdot, 2)
																for yy = 1:size(number_references, 2)
																	for zz = 1:size(descriptor, 2)
																		R_0 = zeros(number_controls(1, vv), number_measurements(1, ww));
																		K_0 = zeros(number_controls(1, vv), number_measurements_xdot(1, xx));
																		F_0 = zeros(number_controls(1, vv), number_references(1, yy));
																		systems = cell(number_models(1, ii), 1);
																		for sample_models = 1:number_models(1, ii)
																			if use_measurements_xdot(1, jj)
																				if use_references(1, kk)
																					if allowvarorder(1, pp) && sample_models == number_models(1, ii)
																						systems{sample_models, 1} = testsystem(descriptor(1, zz), number_states_var, number_controls(1, vv), number_measurements(1, ww), number_measurements_xdot(1, xx), number_references(1, yy));
																					else
																						systems{sample_models, 1} = testsystem(descriptor(1, zz), number_states, number_controls(1, vv), number_measurements(1, ww), number_measurements_xdot(1, xx), number_references(1, yy));
																					end
																				else
																					if allowvarorder(1, pp) && sample_models == number_models(1, ii)
																						systems{sample_models, 1} = testsystem(descriptor(1, zz), number_states_var, number_controls(1, vv), number_measurements(1, ww), number_measurements_xdot(1, xx));
																					else
																						systems{sample_models, 1} = testsystem(descriptor(1, zz), number_states, number_controls(1, vv), number_measurements(1, ww), number_measurements_xdot(1, xx));
																					end
																				end
																			else
																				if use_references(1, kk)
																					if allowvarorder(1, pp) && sample_models == number_models(1, ii)
																						systems{sample_models, 1} = testsystem(descriptor(1, zz), number_states_var, number_controls(1, vv), number_measurements(1, ww), 0, number_references(1, yy));
																					else
																						systems{sample_models, 1} = testsystem(descriptor(1, zz), number_states, number_controls(1, vv), number_measurements(1, ww), 0, number_references(1, yy));
																					end
																				else
																					if allowvarorder(1, pp) && sample_models == number_models(1, ii)
																						systems{sample_models, 1} = testsystem(descriptor(1, zz), number_states_var, number_controls(1, vv), number_measurements(1, ww));
																					else
																						systems{sample_models, 1} = testsystem(descriptor(1, zz), number_states, number_controls(1, vv), number_measurements(1, ww));
																					end
																				end
																			end
																		end
																		if use_measurements_xdot(1, jj)
																			if use_references(1, kk)
																				R_init = {R_0, K_0, F_0};
																			else
																				R_init = {R_0, K_0};
																			end
																		else
																			if use_references(1, kk)
																				R_init = {R_0, [], F_0};
																			else
																				R_init = R_0;
																			end
																		end
																		sys = cat(1, systems{:});
																		if areacombine(mm)
																			if areafunflag(ll) == 0
																				areafun = poleareafun;
																			elseif areafunflag(ll) == 1
																				areafun = cat(2, poleareatypes{:});
																			elseif areafunflag(ll) == 2
																				areafun = cat(2, poleareatypes', poleareafun);
																			elseif areafunflag(ll) == 3
																				areafun = cat(2, poleareafun, poleareatypes');
																			else
																				error('control:gamma:arguments:test', 'Undefined area type.');
																			end
																		else
																			if areafunflag(ll) == 0
																				areafun = {poleareafun, poleareafun};
																			elseif areafunflag(ll) == 1
																				areafun = cat(2, poleareatypes{:});
																				areafun = {areafun, areafun};
																			elseif areafunflag(ll) == 2
																				areafun = cat(2, poleareatypes', poleareafun);
																				areafun = {areafun, areafun};
																			elseif areafunflag(ll) == 3
																				areafun = cat(2, poleareatypes', poleareafun);
																				areafun = {poleareafun, areafun};
																			else
																				error('control:gamma:arguments:test', 'Undefined area type.');
																			end
																		end
																		if eigenvectorobjective
																			objective = [
																				objective_types;
																				GammaJType.EIGENVALUECONDITION
																			];
																		else
																			objective = objective_types;
																		end
																		objweight = objectiveweight;
																		if needsderivative(rr, 3) && solver.getHessianSupport()
																			if any(objective == GammaJType.KREISSELMEIER)
																				warning('Kreisselmeier objective does not have an implementation of the hessian matrix.');
																				objweight(objective == GammaJType.KREISSELMEIER) = [];
																				objective(objective == GammaJType.KREISSELMEIER) = [];
																			end
																		end
																		objectiveoptions = struct(...
																			'usecompiled',				usecompiled(1, uu),...% indicator, if compiled functions should be used
																			'eigenvaluederivative',		derivativetype(1, qq),...
																			'type',						objective,...% type of pole area weighting in objective function
																			'weight',					objweight,...
																			'allowvarorder',			allowvarorder(1, pp),...%allow variable state number for different multi models
																			'errorhandler',				GammaErrorHandler.USER,...
																			'errorhandler_function',	@control.design.gamma.test.errorhandler_log...
																		);
																		solver = solvers(ss, 1);
																		options = optimization.options.OptionFactory.instance.options(solver,...
																			'ProblemType',					solver.getPreferredProblemType(),...% can be CONSTRAINED for constrained optimization and UNCONSTRAINED for unconstrained optimization
																			'Retries',						1,...% number of retries
																			'Algorithm',					solver.getDefaultAlgorithm(),...% algorithm of solver, for not builtin solvers name of the solver, e.g. 'snopt' for SNOPT
																			'FunctionTolerance',			1E-8,...
																			'StepTolerance',				1E-8,...
																			'ConstraintTolerance',			1E-5,...
																			'MaxFunctionEvaluations',		5,...
																			'MaxIterations',				5,...
																			'MaxSQPIter',					5,...
																			'SpecifyObjectiveGradient',		needsderivative(rr, 1),...
																			'SpecifyObjectiveHessian',		needsderivative(rr, 3) && solver.getHessianSupport(),...
																			'SpecifyConstraintGradient',	needsderivative(rr, 2),...
																			'SpecifyConstraintHessian',		needsderivative(rr, 4) && solver.getHessianSupport(),...
																			'CheckGradients',				false,...
																			'FunValCheck',					false,...
																			'FiniteDifferenceType',			'forward',...
																			'Diagnostics',					false,...
																			'UseParallel',					false,...
																			'Display',						'final-detailed',...
																			'MaxTime',						(0.5)...
																		);

																		test.TestSuite.assertNoException('[R_opt, J_opt, info] = control.design.gamma.gammasyn(systems, areafun, weight, [], R_init, options, objectiveoptions);', 'control:gammasyn:test', 'gammasyn must not throw an exception.');
																		if use_measurements_xdot(1, jj)
																			if use_references(1, kk)
																				test.TestSuite.assertSameSize(length(R_opt), 3, 'control:gamma:arguments:test', 'Optimal gain matrix must have %d elements.', 3);
																				test.TestSuite.assertSameSize(R_opt{1}, R_0, 'control:gamma:arguments:test', 'Optimal proportional gain matrix must have have same dimension as inital proportional gain matrix.');
																				test.TestSuite.assertSameSize(R_opt{2}, K_0, 'control:gamma:arguments:test', 'Optimal derivative gain matrix must have have same dimension as inital derivative gain matrix.');
																				test.TestSuite.assertSameSize(R_opt{3}, F_0, 'control:gamma:arguments:test', 'Optimal prefilter matrix must have same dimension as inital prefilter matrix.');
																			else
																				test.TestSuite.assertSameSize(length(R_opt), 2, 'control:gamma:arguments:test', 'Optimal gain matrix must have %d elements.', 2);
																				test.TestSuite.assertSameSize(R_opt{1}, R_0, 'control:gamma:arguments:test', 'Optimal proportional gain matrix must have have same dimension as inital proportional gain matrix.');
																				test.TestSuite.assertSameSize(R_opt{2}, K_0, 'control:gamma:arguments:test', 'Optimal derivative gain matrix must have same dimension as inital derivative gain matrix.');
																			end
																		else
																			if use_references(1, kk)
																				test.TestSuite.assertSameSize(length(R_opt), 3, 'control:gamma:arguments:test', 'Optimal gain matrix must have %d elements.', 3);
																				test.TestSuite.assertSameSize(R_opt{1}, R_0, 'control:gamma:arguments:test', 'Optimal gain matrix must have have same dimension as inital gain matrix.');
																				test.TestSuite.assertSameSize(R_opt{3}, F_0, 'control:gamma:arguments:test', 'Optimal prefilter matrix must have same dimension as inital prefilter matrix.');
																			else
																				test.TestSuite.assertSameSize(R_opt, R_0, 'control:gamma:arguments:test', 'Optimal gain matrix must have %d elements.');
																			end
																		end
																		test.TestSuite.assert(~any(isnan(J_opt(:))), 'control:gamma:arguments:test', 'Optimal objective value must not contain NaN.');
																		test.TestSuite.assertFieldnames(info, optimization.options.Options.OUTPUTPROTOTYPEMULTIPLE, 'control:gamma:arguments:test', 'Fieldnames must match.');
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
						end
					end
				end
			end
		end
	end
end
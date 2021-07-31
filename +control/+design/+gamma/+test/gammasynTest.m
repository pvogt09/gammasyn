function [pass] = gammasynTest(silent)
	%GAMMASYNTEST test cases for checking gammasyn for correct argument handling
	%	Input:
	%		silent:	idicator, if information should be output
	%	Output:
	%		pass:	indicator, if test was passed
	if nargin <= 1
		silent = false;
	end
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
	number_states = [1, 2, 3];% number of states
	number_states_var = 2;% number of states when allowvarorder == true
	descriptor = [false, true];
	weight = 0.001;
	objective_types = enumeration('GammaJType');
	objective_types_cell = num2cell(objective_types);
	objective_types_cell(objective_types == GammaJType.DECOUPLING, :) = []; % has its own test
	objective_types_cell(objective_types == GammaJType.LYAPUNOV, :) = [];
	objective_types_cell(objective_types == GammaJType.EIGENVALUECONDITION, :) = [];
	objective_types_cell(objective_types == GammaJType.LOG, :) = [];% TODO: can not be used unsupervised automatically because J == Inf
	objective_types = cat(1, objective_types_cell{:});
	objectiveweight = ones(size(objective_types, 1), 1);
	objectiveweight(objective_types == GammaJType.LOG) = 1E-20;

	eigenvalueconditionweight = 0.1;
	nohessianobjectivetype = [
		GammaJType.LYAPUNOV
	];
	nohessianobjectiveweight = [
		0.25
	];

	info = struct();
	areafunflag = [0, 1, 2, 3];
	areacombine = [false, true];
	areasizeflag = 0;% TODO: use different sizes of areas [0, 1, 2];
	eigenvectorobjective = [false, true];
	nohessianobjective = [false, true];
	usecompiled = [false, true];
	preventnan = [false, true];
	combinations = size(number_models, 2)*size(use_measurements_xdot, 2)*size(use_references, 2)*size(areafunflag, 2)*size(areacombine, 2)*size(areasizeflag, 2)*size(allowvarorder, 2)*size(derivativetype, 2)*size(needsderivative, 1)*size(preventnan, 1)*size(solvers, 1)*size(eigenvectorobjective, 2)*size(nohessianobjective, 2)*size(usecompiled, 2)*size(number_states, 2)*size(number_controls, 2)*size(number_measurements, 2)*size(number_measurements_xdot, 2)*size(number_references, 2)*size(descriptor, 2);
	iter = 0;
	for gg = 1:size(number_models, 2) %#ok<FORPF> parfor is used in optimization functions and parfor here would prevent parfor in the optimization functions from being parallelized
		for hh = 1:size(use_measurements_xdot, 2)
			for ii = 1:size(use_references, 2)
				for jj = 1:size(areafunflag, 2)
					for kk = 1:size(areacombine, 2)
						for ll = 1:size(areasizeflag, 2)
							for mm = 1:size(allowvarorder, 2)
								for nn = 1:size(derivativetype, 2)
									for oo = 1:size(needsderivative, 1)
										for pp = 1:size(preventnan, 1)
											for qq = 1:size(solvers, 1)
												for rr = 1:size(eigenvectorobjective, 2)
													for ss = 1:size(nohessianobjective, 2)
														for tt = 1:size(usecompiled, 2)
															for uu = 1:size(number_states, 2)
																for vv = 1:size(number_controls, 2)
																	for ww = 1:size(number_measurements, 2)
																		for xx = 1:size(number_measurements_xdot, 2)
																			for yy = 1:size(number_references, 2)
																				for zz = 1:size(descriptor, 2)
																					iter = iter + 1;
																					R_0 = zeros(number_controls(1, vv), number_measurements(1, ww));
																					K_0 = zeros(number_controls(1, vv), number_measurements_xdot(1, xx));
																					F_0 = zeros(number_controls(1, vv), number_references(1, yy));
																					systems = cell(number_models(1, gg), 1);
																					for sample_models = 1:number_models(1, gg)
																						if use_measurements_xdot(1, hh)
																							if use_references(1, ii)
																								if allowvarorder(1, mm) && sample_models == number_models(1, gg)
																									systems{sample_models, 1} = testsystem(descriptor(1, zz), number_states_var, number_controls(1, vv), number_measurements(1, ww), number_measurements_xdot(1, xx), number_references(1, yy));
																								else
																									systems{sample_models, 1} = testsystem(descriptor(1, zz), number_states(1, uu), number_controls(1, vv), number_measurements(1, ww), number_measurements_xdot(1, xx), number_references(1, yy));
																								end
																							else
																								if allowvarorder(1, mm) && sample_models == number_models(1, gg)
																									systems{sample_models, 1} = testsystem(descriptor(1, zz), number_states_var, number_controls(1, vv), number_measurements(1, ww), number_measurements_xdot(1, xx));
																								else
																									systems{sample_models, 1} = testsystem(descriptor(1, zz), number_states(1, uu), number_controls(1, vv), number_measurements(1, ww), number_measurements_xdot(1, xx));
																								end
																							end
																						else
																							if use_references(1, ii)
																								if allowvarorder(1, mm) && sample_models == number_models(1, gg)
																									systems{sample_models, 1} = testsystem(descriptor(1, zz), number_states_var, number_controls(1, vv), number_measurements(1, ww), 0, number_references(1, yy));
																								else
																									systems{sample_models, 1} = testsystem(descriptor(1, zz), number_states(1, uu), number_controls(1, vv), number_measurements(1, ww), 0, number_references(1, yy));
																								end
																							else
																								if allowvarorder(1, mm) && sample_models == number_models(1, gg)
																									systems{sample_models, 1} = testsystem(descriptor(1, zz), number_states_var, number_controls(1, vv), number_measurements(1, ww));
																								else
																									systems{sample_models, 1} = testsystem(descriptor(1, zz), number_states(1, uu), number_controls(1, vv), number_measurements(1, ww));
																								end
																							end
																						end
																					end
																					if use_measurements_xdot(1, hh)
																						if use_references(1, ii)
																							R_init = {R_0, K_0, F_0};
																						else
																							R_init = {R_0, K_0};
																						end
																					else
																						if use_references(1, ii)
																							R_init = {R_0, [], F_0};
																						else
																							R_init = R_0;
																						end
																					end
																					sys = cat(1, systems{:});
																					if areacombine(kk)
																						if areafunflag(jj) == 0
																							areafun = poleareafun;
																						elseif areafunflag(jj) == 1
																							areafun = cat(2, poleareatypes{:});
																						elseif areafunflag(jj) == 2
																							areafun = cat(2, poleareatypes', {poleareafun});
																						elseif areafunflag(jj) == 3
																							areafun = cat(2, {poleareafun}, poleareatypes');
																						else
																							error('control:gamma:arguments:test', 'Undefined area type.');
																						end
																					else
																						if areafunflag(jj) == 0
																							areafun = {poleareafun, poleareafun};
																						elseif areafunflag(jj) == 1
																							areafun = cat(2, poleareatypes{:});
																							areafun = {areafun, areafun};
																						elseif areafunflag(jj) == 2
																							areafun = cat(2, poleareatypes', {poleareafun});
																							areafun = {areafun, areafun};
																						elseif areafunflag(jj) == 3
																							areafun = cat(2, poleareatypes', {poleareafun});
																							areafun = {poleareafun, areafun};
																						else
																							error('control:gamma:arguments:test', 'Undefined area type.');
																						end
																					end
																					objweight = objectiveweight;
																					if nohessianobjective(1, ss)
																						if eigenvectorobjective(1, rr)
																							objective = [
																								objective_types;
																								GammaJType.EIGENVALUECONDITION;
																								nohessianobjectivetype
																							];
																							objweight = [
																								objweight;
																								eigenvalueconditionweight;
																								nohessianobjectiveweight
																							];
																						else
																							objective = [
																								objective_types;
																								nohessianobjectivetype
																							];
																							objweight = [
																								objweight;
																								nohessianobjectiveweight
																							];
																						end
																					else
																						if eigenvectorobjective(1, rr)
																							objective = [
																								objective_types;
																								GammaJType.EIGENVALUECONDITION
																							];
																							objweight = [
																								objweight;
																								eigenvalueconditionweight
																							];
																						else
																							objective = objective_types;
																						end
																					end
																					prototype = control.design.gamma.GammasynOptions.PROTOTYPE;
																					objectiveoptions = struct(...
																						'usecompiled',				usecompiled(1, tt),...% indicator, if compiled functions should be used
																						'eigenvaluederivative',		derivativetype(1, nn),...
																						'type',						objective,...% type of pole area weighting in objective function
																						'weight',					objweight,...
																						'allowvarorder',			allowvarorder(1, mm),...%allow variable state number for different multi models
																						'objective',				prototype.objective,...
																						'errorhandler',				GammaErrorHandler.USER,...
																						'errorhandler_function',	@control.design.gamma.test.errorhandler_log...
																					);
																					objectiveoptions.objective.preventnan = preventnan(1, pp);
																					if allowvarorder(1, mm) && sample_models == number_models(1, gg)
																						number_usestates = number_states_var;
																					else
																						number_usestates = number_states(1, uu);
																					end
																					objectiveoptions.objective.lyapunov.Q = eye(number_usestates);
																					solver = solvers(qq, 1);
																					if solver == optimization.solver.Optimizer.IPOPT && (~needsderivative(oo, 1) || ~needsderivative(oo, 2))
																						continue;
																					end
																					if (eigenvectorobjective(1, rr) || nohessianobjective(1, ss)) && ((needsderivative(oo, 3) && solver.getHessianSupport()) || (needsderivative(oo, 4) && solver.getHessianSupport()))
																						continue;
																					end
																					options = optimization.options.OptionFactory.instance.options(solver,...
																						'ProblemType',					solver.getPreferredProblemType(),...% can be CONSTRAINED for constrained optimization and UNCONSTRAINED for unconstrained optimization
																						'Retries',						1,...% number of retries
																						'Algorithm',					solver.getDefaultAlgorithm(),...% algorithm of solver, for not builtin solvers name of the solver, e.g. 'snopt' for SNOPT
																						'FunctionTolerance',			1E-8,...
																						'StepTolerance',				1E-8,...
																						'ConstraintTolerance',			1E-5,...
																						'MaxFunctionEvaluations',		2,...
																						'MaxIterations',				2,...
																						'MaxSQPIter',					2,...
																						'SpecifyObjectiveGradient',		needsderivative(oo, 1),...
																						'SpecifyObjectiveHessian',		needsderivative(oo, 3) && solver.getHessianSupport(),...
																						'SpecifyConstraintGradient',	needsderivative(oo, 2),...
																						'SpecifyConstraintHessian',		needsderivative(oo, 4) && solver.getHessianSupport(),...
																						'CheckGradients',				false,...
																						'FunValCheck',					false,...
																						'FiniteDifferenceType',			'forward',...
																						'Diagnostics',					false,...
																						'UseParallel',					false,...
																						'Display',						'final-detailed',...
																						'MaxTime',						(0.5)...
																					);

																					if ~silent
																						fprintf('iteration: %d/%d\t(%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d)\n', iter, combinations, gg, hh, ii, jj, kk, ll, mm, nn, oo, pp, qq, rr, ss, tt, uu, vv, ww, xx, yy, zz);
																					end
																					test.TestSuite.assertNoException('[R_opt, J_opt, info] = control.design.gamma.gammasyn(systems, areafun, weight, [], R_init, options, objectiveoptions);', 'control:gammasyn:test', 'gammasyn must not throw an exception.');
																					if (number_references(1, yy) > 0 && use_references(1, ii)) || (number_measurements_xdot(1, xx) > 0 && use_measurements_xdot(1, hh))
																						if use_measurements_xdot(1, hh)
																							if use_references(1, ii)
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
																							if use_references(1, ii)
																								test.TestSuite.assertSameSize(length(R_opt), 3, 'control:gamma:arguments:test', 'Optimal gain matrix must have %d elements.', 3);
																								test.TestSuite.assertSameSize(R_opt{1}, R_0, 'control:gamma:arguments:test', 'Optimal gain matrix must have have same dimension as inital gain matrix.');
																								test.TestSuite.assertSameSize(R_opt{3}, F_0, 'control:gamma:arguments:test', 'Optimal prefilter matrix must have same dimension as inital prefilter matrix.');
																							else
																								test.TestSuite.assertSameSize(R_opt, R_0, 'control:gamma:arguments:test', 'Optimal gain matrix must have %d elements.');
																							end
																						end
																					else
																						test.TestSuite.assertSameSize(R_opt, R_0, 'control:gamma:arguments:test', 'Optimal gain matrix must have %d elements.');
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
		end
	end
end
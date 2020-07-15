function [pass] = cTest(silent)
	%CTEST test cases for checking constraint function for correct argument handling
	%	Input:
	%		silent:	idicator, if information should be output
	%	Output:
	%		pass:	indicator, if test was passed
	if nargin <= 1
		silent = false;
	end
	pass = identifier.TestResult.PASSED;
	derivativetype = enumeration('GammaEigenvalueDerivativeType');

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
		control.design.gamma.area.PolyEllipsesquare([1 + 1i, 1 - 1i, -2], [1, 1, 3]);
		%control.design.gamma.area.Custom(@(re, im) control.design.gamma.area.Hyperbola_border(re, im, struct('hyperbola_a', a, 'hyperbola_b', b, 'reshift', 0, 'imshift', 0)))
	};
	% TODO: add control.design.gamma.area.Custom as separate test case like poleareafun
	poleareas = enumeration('GammaArea');
	if size(poleareatypes, 1) ~= size(poleareas, 1) - 1
		error('control:gamma:arguments:test', 'Not all polearea types are used in the test.');
	end
	poleareafun = @(x) flexargout(2*real(x), 2, 0, 0, 0, 0, 0);
	poleareafun_multi = @(x) flexargout([2*real(x), real(x)^2 + imag(x)^2 - 1], [2, 2*real(x)], [0, 2*imag(x)], [0, 2], [0, 0], [0, 0], [0, 2]);
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
	objective_types = cat(1, objective_types_cell{:});
	objectiveweight = ones(size(objective_types, 1), 1);
	objectiveweight(objective_types == GammaJType.LOG) = 1E-20;

	areafunflag = [0, 1, 2, 3, 4, 5, 6, 7];
	areacombine = [false, true];
	% TODO: has no effect
	areasizeflag = 1;%[0, 1, 2];
	eigenvectorobjective = [false, true];
	usecompiled = [false, true];
	hascompiled = any(usecompiled) && true && all(~compile.control.design.gamma.c_mex_needupdate());
	checkandtransformargs = get_private_function(realpath(fullfile(mfilename('fullpath'), '..', '..', 'private')), 'checkandtransformargs');
	checkobjectiveoptions = get_private_function(realpath(fullfile(mfilename('fullpath'), '..', '..', 'private')), 'checkobjectiveoptions');
	checkinitialR = get_private_function(realpath(fullfile(mfilename('fullpath'), '..', '..', 'private')), 'checkinitialR');
	N = prod([
		size(number_models, 2);
		size(use_measurements_xdot, 2);
		size(use_references, 2);
		size(areafunflag, 2);
		size(areacombine, 2);
		size(areasizeflag, 2);
		size(allowvarorder, 2);
		size(derivativetype, 2);
		size(objective_types, 1) + 1;
		size(eigenvectorobjective, 2);
		size(usecompiled, 2);
		size(number_controls, 2);
		size(number_measurements, 2);
		size(number_measurements_xdot, 2);
		size(number_references, 2);
		size(descriptor, 2)
	]);
	if ~silent
		wait = Progress(N, 'Constraint function test', true, 1000);
	end
	M = 1;
	for ii = 1:size(number_models, 2)
		for jj = 1:size(use_measurements_xdot, 2)
			for kk = 1:size(use_references, 2)
				for ll = 1:size(areafunflag, 2)
					for mm = 1:size(areacombine, 2)
						for oo = 1:size(areasizeflag, 2)
							for pp = 1:size(allowvarorder, 2)
								for qq = 1:size(derivativetype, 2)
									for rr = 0:size(objective_types, 1)
										%for ss = 1:size(solvers, 1)
											for tt = 1:size(eigenvectorobjective, 2)
												for uu = 1:size(usecompiled, 2)
													for vv = 1:size(number_controls, 2)
														for ww = 1:size(number_measurements, 2)
															for xx = 1:size(number_measurements_xdot, 2)
																for yy = 1:size(number_references, 2)
																	for zz = 1:size(descriptor, 2)
																		if ~silent && wait.iscancelled()
																			break;
																		end
																		M = M + 1;
																		if usecompiled(1, uu)
																			if ~configuration.control.design.gamma.hascompiled()
																				test.TestSuite.assert(false, 'control:gammasyn:test', 'Constraint function has not been compiled.');
																			end
																			if ~hascompiled
																				test.TestSuite.assert(false, 'control:gammasyn:test', 'Constraint function must be recompiled.');
																			end
																		end
																		compiled = usecompiled(1, uu);
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
																		sys = cat(1, systems{:});
																		if areacombine(mm)
																			if areafunflag(ll) == 0
																				areafun = poleareafun;
																				compiled = false;
																			elseif areafunflag(ll) == 1
																				areafun = cat(2, poleareatypes{:});
																			elseif areafunflag(ll) == 2
																				areafun = cat(2, poleareatypes', {poleareafun});
																				compiled = false;
																			elseif areafunflag(ll) == 3
																				areafun = cat(2, {poleareafun}, poleareatypes');
																				compiled = false;
																			elseif areafunflag(ll) == 4
																				areafun = cat(2, poleareatypes', {poleareafun_multi});
																				compiled = false;
																			elseif areafunflag(ll) == 5
																				areafun = cat(2, {poleareafun_multi}, poleareatypes');
																				compiled = false;
																			elseif areafunflag(ll) == 6
																				areafun = cat(2, {poleareafun_multi}, poleareatypes', {poleareafun});
																				compiled = false;
																			elseif areafunflag(ll) == 7
																				areafun = poleareatypes(1);
																			else
																				error('control:gamma:arguments:test', 'Undefined area type.');
																			end
																		else
																			if areafunflag(ll) == 0
																				areafun = {poleareafun, poleareafun};
																				compiled = false;
																			elseif areafunflag(ll) == 1
																				areafun = cat(2, poleareatypes{:});
																				areafun = {areafun, areafun};
																			elseif areafunflag(ll) == 2
																				areafun = cat(2, poleareatypes', {poleareafun});
																				areafun = {areafun, areafun};
																				compiled = false;
																			elseif areafunflag(ll) == 3
																				areafun = cat(2, poleareatypes', {poleareafun});
																				areafun = {poleareafun, areafun};
																				compiled = false;
																			elseif areafunflag(ll) == 4
																				areafun = cat(2, poleareatypes', {poleareafun_multi});
																				areafun = {areafun, areafun};
																				compiled = false;
																			elseif areafunflag(ll) == 5
																				areafun = cat(2, poleareatypes', {poleareafun_multi});
																				areafun = {poleareafun_multi, areafun};
																				compiled = false;
																			elseif areafunflag(ll) == 6
																				areafun = cat(2, {poleareafun}, poleareatypes', {poleareafun_multi});
																				areafun = {{poleareafun_multi, poleareafun}, areafun};
																				compiled = false;
																			elseif areafunflag(ll) == 7
																				areafun = cat(2, poleareatypes', {poleareafun});
																				areafun = {poleareafun, areafun(1)};
																				compiled = false;
																			else
																				error('control:gamma:arguments:test', 'Undefined area type.');
																			end
																		end
																		if rr == 0
																			if eigenvectorobjective(1, tt)
																				objective = [
																					objective_types;
																					GammaJType.EIGENVALUECONDITION
																				];
																				objweight = [
																					objectiveweight;
																					1
																				];
																			else
																				objective = objective_types;
																				objweight = objectiveweight;
																			end
																		else
																			objective = objective_types(rr, 1);
																			objweight = objectiveweight;
																		end
																		objectiveoptions = struct(...
																			'usecompiled',				usecompiled(1, uu),...% indicator, if compiled functions should be used
																			'eigenvaluederivative',		derivativetype(1, qq),...
																			'type',						objective,...% type of pole area weighting in objective function
																			'weight',					objweight(1:size(objective), 1),...
																			'allowvarorder',			allowvarorder(1, pp),...%allow variable state number for different multi models
																			'errorhandler',				GammaErrorHandler.USER,...
																			'errorhandler_function',	@control.design.gamma.test.errorhandler_log...
																		);
																		[system, areafun_strict, areafun_loose, weight_strict, weight_loose, dimensions_strict, dimensions_loose] = feval(checkandtransformargs, systems, areafun, weight, [], [], [], [], [], objectiveoptions.allowvarorder);
																		R_0 = ones(dimensions_loose.controls, dimensions_loose.measurements);
																		K_0 = ones(dimensions_loose.controls, dimensions_loose.measurements_xdot);
																		F_0 = ones(dimensions_loose.controls, dimensions_loose.references);
																		objectiveoptions.objective.preventNaN = true;
																		objectiveoptions.objective.normgain.R = 1./magic(max(size(R_0)));
																		objectiveoptions.objective.normgain.K = 1./magic(max(size(K_0)));
																		objectiveoptions.objective.normgain.F = 1./magic(max(size(F_0)));
																		objectiveoptions.objective.normgain.R = objectiveoptions.objective.normgain.R(1:size(R_0, 1), 1:size(R_0, 2));
																		objectiveoptions.objective.normgain.K = objectiveoptions.objective.normgain.K(1:size(K_0, 1), 1:size(K_0, 2));
																		objectiveoptions.objective.normgain.F = objectiveoptions.objective.normgain.F(1:size(F_0, 1), 1:size(F_0, 2));
																		objectiveoptions.objective.lyapunov.Q = eye(dimensions_loose.states);
																		range = 1000;
																		testpoints = 3;
																		[objectiveoptions] = feval(checkobjectiveoptions, objectiveoptions, feval(checkinitialR, {R_0, K_0, F_0}, dimensions_loose), dimensions_loose.states, system, struct('ProblemType', optimization.options.ProblemType.CONSTRAINED), dimensions_strict, dimensions_loose, areafun_strict, areafun_loose, weight_strict, weight_loose);
																		cfun = gethandle(compiled, system, weight_loose, areafun_loose, dimensions_loose, objectiveoptions);
																		R = cell(length(R_0), 1);
																		for jjj = 1:length(R_0)
																			[r_index(1), r_index(2)] = ind2sub(size(R_0), jjj);
																			r_range = linspace(-abs(range), abs(range), testpoints);
																			R_temp = repmat(R_0, [1, 1, size(r_range, 2)]);
																			for iii = 1:size(r_range, 2)
																				R_temp(r_index(1), r_index(2), iii) = r_range(iii);
																			end
																			R{jjj, 1} = R_temp;
																		end
																		K = cell(length(K_0), 1);
																		for jjj = 1:length(K_0)
																			[k_index(1), k_index(2)] = ind2sub(size(K_0), jjj);
																			k_range = linspace(-abs(range), abs(range), testpoints);
																			K_temp = repmat(K_0, [1, 1, size(k_range, 2)]);
																			for iii = 1:size(k_range, 2)
																				K_temp(k_index(1), k_index(2), iii) = k_range(iii);
																			end
																			K{jjj, 1} = K_temp;
																		end
																		F = cell(length(F_0), 1);
																		for jjj = 1:length(F_0)
																			[f_index(1), f_index(2)] = ind2sub(size(F_0), jjj);
																			f_range = linspace(-abs(range), abs(range), testpoints);
																			F_temp = repmat(F_0, [1, 1, size(f_range, 2)]);
																			for iii = 1:size(f_range, 2)
																				F_temp(f_index(1), f_index(2), iii) = f_range(iii);
																			end
																			F{jjj, 1} = F_temp;
																		end
																		if isempty(K) && isempty(F)
																			combinations = [
																				R(:), cell(numel(R), 1), cell(numel(R), 1)
																			];
																		elseif isempty(K)
																			[r, f] = ndgrid(1:size(R, 1), 1:size(F, 1));
																			combinations = [
																				R(r(:)), cell(numel(r), 1),	F(f(:))
																			];
																		elseif isempty(F)
																			[r, k] = ndgrid(1:size(R, 1), 1:size(K, 1));
																			combinations = [
																				R(r(:)),	K(k(:)), cell(numel(r), 1)
																			];
																		else
																			[r, k, f] = ndgrid(1:size(R, 1), 1:size(K, 1), 1:size(F, 1));
																			combinations = [
																				R(r(:)),	K(k(:)),	F(f(:))
																			];
																		end
																		for jjj = 1:size(combinations, 1)
																			cc = combinations(jjj, :);
																			for iii = 1:size(cc{1}, 3)
																				if isempty(cc{2}) && isempty(cc{3})
																					x = feval(checkinitialR, cc{1}(:, :, iii), dimensions_loose);
																				elseif isempty(cc{2})
																					x = feval(checkinitialR, {cc{1}(:, :, iii), [], cc{3}(:, :, iii)}, dimensions_loose);
																				elseif isempty(cc{3})
																					x = feval(checkinitialR, {cc{1}(:, :, iii), cc{2}(:, :, iii)}, dimensions_loose);
																				else
																					x = feval(checkinitialR, {cc{1}(:, :, iii), cc{2}(:, :, iii), cc{3}(:, :, iii)}, dimensions_loose);
																				end
																				test.TestSuite.assertNoException('[c, ceq] = cfun(x);', 'control:gammasyn:test', 'Constraint function must not throw an exception for one input argument.');
																				test.TestSuite.assertEqual(size(c, 2), 1, 'control:gamma:arguments:test', 'Constraint function value must be a column vector.');
																				test.TestSuite.assert(~any(isnan(c(:))), 'control:gamma:arguments:test', 'Constraint function value must not be NaN.');
																				test.TestSuite.assert(isempty(ceq), 'control:gamma:arguments:test', 'Constraint function value for equalities must be empty.');

																				test.TestSuite.assertNoException('[c, ceq, gradc, gradceq] = cfun(x);', 'control:gammasyn:test', 'Constraint function must not throw an exception for one input argument.');
																				test.TestSuite.assertEqual(size(c, 2), 1, 'control:gamma:arguments:test', 'Constraint function value must be a column vector.');
																				test.TestSuite.assert(isempty(ceq), 'control:gamma:arguments:test', 'Constraint function value for equalities must be empty.');
																				test.TestSuite.assert(~any(isnan(c(:))), 'control:gamma:arguments:test', 'Constraint function value must not be NaN.');
																				test.TestSuite.assertEqual(size(gradc, 2), size(c, 1), 'control:gamma:arguments:test', 'Constraint function gradient value must have same dimension as c.');
																				test.TestSuite.assertEqual(size(gradc, 1), numel(x), 'control:gamma:arguments:test', 'Constraint function gradient value must have same dimension as x.');
																				test.TestSuite.assert(~any(isnan(gradc(:))), 'control:gamma:arguments:test', 'Constraint function gradient value must not be NaN.');
																				test.TestSuite.assert(isempty(gradceq), 'control:gamma:arguments:test', 'Constraint function gradient value for equalities must be empty.');

																				test.TestSuite.assertNoExceptionExcept('[c, ceq, gradc, gradceq, hessc, hessceq] = cfun(x);', 'control:design:gamma:hessian', 'control:gammasyn:test', 'Constraint function must not throw an exception for one input argument.');
																				test.TestSuite.assertEqual(size(c, 2), 1, 'control:gamma:arguments:test', 'Constraint function value must be a column vector.');
																				test.TestSuite.assert(isempty(ceq), 'control:gamma:arguments:test', 'Constraint function value for equalities must be empty.');
																				test.TestSuite.assert(~any(isnan(c(:))), 'control:gamma:arguments:test', 'Constraint function value must not be NaN.');
																				test.TestSuite.assertEqual(size(gradc, 2), size(c, 1), 'control:gamma:arguments:test', 'Constraint function gradient value must have same dimension as c.');
																				test.TestSuite.assertEqual(size(gradc, 1), numel(x), 'control:gamma:arguments:test', 'Constraint function gradient value must have same dimension as x.');
																				test.TestSuite.assert(~any(isnan(gradc(:))), 'control:gamma:arguments:test', 'Constraint function gradient value must not be NaN.');
																				test.TestSuite.assert(isempty(gradceq), 'control:gamma:arguments:test', 'Constraint function gradient value for equalities must be empty.');
																				test.TestSuite.assertEqual(size(hessc, 3), size(c, 1), 'control:gamma:arguments:test', 'Constraint function hessian value must have same dimension as c.');
																				test.TestSuite.assertEqual(size(hessc, 1), numel(x), 'control:gamma:arguments:test', 'Constraint function hessian value must have same dimension as x.');
																				test.TestSuite.assertEqual(size(hessc, 2), numel(x), 'control:gamma:arguments:test', 'Constraint function hessian value must have same dimension as x.');
																				test.TestSuite.assert(~any(isnan(hessc(:))), 'control:gamma:arguments:test', 'Constraint function hessian value must not be NaN.');
																				test.TestSuite.assert(isempty(hessceq), 'control:gamma:arguments:test', 'Constraint function hessian value for equalities must be empty.');
																			end
																		end
																		if ~silent
																			wait.step();
																		end
																	end
																end
															end
														end
													end
												end
											end
										%end
									end
								end
							end
						end
					end
				end
			end
		end
	end
	if ~silent
		clear wait;
	end
end

function [c] = gethandle(usecompiled, system, weight, areafun, dimensions, options)
	persistent c_mex_private;
	if isempty(c_mex_private)
		c_mex_private = get_private_function(realpath(fullfile(mfilename('fullpath'), '..', '..', 'private')), 'c_mex');
	end
	c_mex = c_mex_private;
	function [c, ceq, gradc, gradceq, hessc, hessceq] = cfun(x)
		x = reshape(x, [], 1);
		if nargout >= 5
			[c, ceq, gradc, gradceq, hessc, hessceq] = control.design.gamma.c(x, system, weight, areafun, dimensions, options);
		elseif nargout >= 3
			[c, ceq, gradc, gradceq] = control.design.gamma.c(x, system, weight, areafun, dimensions, options);
		else
			[c, ceq] = control.design.gamma.c(x, system, weight, areafun, dimensions, options);
		end
	end
	function [c, ceq, gradc, gradceq, hessc, hessceq] = cfun_mex(x)
		x = reshape(x, [], 1);
		if nargout >= 5
			[c, ceq, gradc, gradceq, hessc, hessceq] = c_mex(x, system, weight, areafun, dimensions, options);
		elseif nargout >= 3
			[c, ceq, gradc, gradceq] = c_mex(x, system, weight, areafun, dimensions, options);
		else
			[c, ceq] = c_mex(x, system, weight, areafun, dimensions, options);
		end
	end
	if usecompiled
		c = @cfun_mex;
	else
		c = @cfun;
	end
end
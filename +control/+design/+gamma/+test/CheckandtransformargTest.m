function [pass] = CheckandtransformargTest(~)
	%CHECKANDTRANSFORMARGTEST test cases for checking argument checking function
	%	Input:
	%		silent:	idicator, if information should be output
	%	Output:
	%		pass:	indicator, if test was passed
	pass = identifier.TestResult.PASSED;
	number_controls = 1;
	number_measurements = 2;
	number_states = 2;
	structsys = struct(...
		'E',		eye(number_states),...
		'A',		eye(number_states),...
		'B',		randi(10, [number_states, number_controls]),...
		'C',		randi(10, [number_measurements, number_states]),...
		'C_dot',	randi(10, [number_measurements, number_states]),...
		'D',		zeros(number_measurements, number_controls)...
	);
	structarray = repmat(structsys, 3, 1);
	ssscalar = ss(eye(number_states), randi(10, [number_states, number_controls]), randi(10, [number_measurements, number_states]), zeros(number_measurements, number_controls));
	tfscalar = tf(ssscalar);
	ssarray = rss(number_states, number_measurements, number_controls, 4, 2);
	tfarray = tf(ssarray);
	ltisysscalar = ltisys(eye(number_states), randi(10, [number_states, number_controls]), randi(10, [number_measurements, number_states]), zeros(number_measurements, number_controls));
	ltisysarray = cat(3, ltisysscalar, ltisysscalar);
	range = [
		1.8,	4.2;
		1.4,	2.6;
		0.8,	1.2
	];
	parametervector = pvec('box', range);
	psysscalar = psys(parametervector, repmat(ltisysscalar, 1, size(range, 1) + 1));
	psysarray = cat(3, psysscalar, psysscalar);
	a = realp('a', 1);
	a.Minimum = -1;
	a.Maximum = 4;
	a.Free = false;
	samples_a = 3;
	u = ureal('u', 5, 'Range',	[-3, 6]);
	samples_u = 3;
	ussscalar = ss(diag(u*ones(number_states, 1)), randi(10, [number_states, number_controls]), randi(10, [number_measurements, number_states]), zeros(number_measurements, number_controls));
	genssscalar = ss(a*eye(number_states), randi(10, [number_states, number_controls]), randi(10, [number_measurements, number_states]), zeros(number_measurements, number_controls));
	ugenssscalar = ss(a*eye(number_states), u*randi(10, [number_states, number_controls]), randi(10, [number_measurements, number_states]), zeros(number_measurements, number_controls));
	systemsscalar = {
		structsys,		1;
		ssscalar,		1;
		tfscalar,		1;
		ltisysscalar,	1;
		psysscalar,		2^size(range, 1)
	};
	systemsarray = {
		structarray,	size(structarray, 1);
		ssarray,		size(ssarray, 3)*size(ssarray, 4);
		tfarray,		size(tfarray, 3)*size(tfarray, 4);
		ltisysarray,	2;
		psysarray,		2^size(range, 1)*size(psysarray, 3)
	};
	systemsexpand = {
		ussscalar,		samples_u;
		genssscalar,	samples_a;
		ugenssscalar,	samples_a*samples_u;
		ssarray,		size(ssarray, 3)*size(ssarray, 4);
		tfarray,		size(tfarray, 3)*size(tfarray, 4);
		ltisysscalar,	1;
		ltisysarray,	2;
		psysscalar,		2^size(range, 1);
		psysarray,		2^size(range, 1)*size(psysarray, 3)
	};
	allowvarorder = true;
	poleareafun = @(x) 2*x;
	polearea = {
		control.design.gamma.area.Circlesquare(1-1E-5);
		[
			control.design.gamma.area.Circlesquare(1-1E-5),	control.design.gamma.area.Hyperbola(0.05, 0.14, 1 + 0i)
		]
	};
	systemoptions = struct('samples', struct(), 'usereferences', true);
	iter = 0;
	fun = get_private_function(realpath(fullfile(mfilename('fullpath'), '..', '..', 'private')), 'checkandtransformargs');
	dimensionfields = {
		'models';
		'states';
		'controls';
		'measurements';
		'measurements_xdot';
		'references';
		'descriptor';
		'isdiscrete';
		'areas_max';
		'area_args';
		'area_parts';
		'area_hasgrad';
		'area_hashess';
		'area_parameters';
		'R_fixed_has';
		'R_fixed_only';
		'R_fixed_constraints';
		'R_fixed';
		'R_fixed_values';
		'R_fixed_A';
		'R_fixed_b';
		'R_fixed_T';
		'R_fixed_T_inv';
		'R_isforced2zero'
		'K_fixed_has';
		'K_fixed_only';
		'K_fixed_constraints';
		'K_fixed';
		'K_fixed_values';
		'K_fixed_A';
		'K_fixed_b';
		'K_fixed_T';
		'K_fixed_T_inv';
		'K_isforced2zero'
		'F_fixed_has';
		'F_fixed_only';
		'F_fixed_constraints';
		'F_fixed';
		'F_fixed_values';
		'F_fixed_A';
		'F_fixed_b';
		'F_fixed_T';
		'F_fixed_T_inv';
		'F_isforced2zero'
		'RKF_fixed_has';
		'RKF_fixed_only';
		'RKF_fixed_constraints';
		'RKF_fixed';
		'RKF_fixed_values';
		'RKF_fixed_A';
		'RKF_fixed_b';
		'RKF_fixed_T';
		'RKF_fixed_T_inv';
		'index_R_free';
		'index_K_free';
		'index_F_free';
		'index_all_free';
		'index_RKF_free'
	};
	areafunflag = [0, 1, 2];
	areacombine = [false, true];
	areasizeflag = [0, 1, 2];
	weightsizeflag = [0, 1, 2];
	weightcombine = [false, true, NaN];
	weightareaflag = [false, true];
	for ii = 0:size(systemsscalar, 1) %#ok<FORPF> parfor is used in substitution functions
		for jj = 0:size(systemsarray, 1)
			for kk = 0:size(systemsexpand, 1)
				for ll = 1:size(areafunflag, 2)
					for mm = 1:size(areacombine, 2)
						for oo = 1:size(areasizeflag, 2)
							for pp = 1:size(weightsizeflag, 2)
								for qq = 1:size(weightcombine, 2)
									for rr = 1:size(weightareaflag, 2)
										if ii == 0 && jj == 0 && kk == 0
											continue;
										end
										sys = cell(0, 1);
										sysexpand = zeros(0, 1);
										if ii ~= 0
											sys = cat(1, sys, systemsscalar(ii, 1));
											sysexpand = cat(1, sysexpand, systemsscalar{ii, 2});
										end
										if jj ~= 0
											sys = cat(1, sys, systemsarray(jj, 1));
											sysexpand = cat(1, sysexpand, systemsarray{jj, 2});
										end
										if kk ~= 0
											sys = cat(1, sys, systemsexpand(kk, 1));
											sysexpand = cat(1, sysexpand, systemsexpand{kk, 2});
											systemoptions.samples = struct('a', samples_a, 'u', samples_u);
										else
											systemoptions.samples = struct();
										end
										if iscell(sys)
											systemoptions.usereferences = ~any(cellfun(@isstruct, sys, 'UniformOutput', true));
										end
										if areacombine(mm)
											if areafunflag(ll) == 0
												areafun = poleareafun;
											elseif areafunflag(ll) == 1
												areafun = polearea{1};
											elseif areafunflag(ll) == 2
												areafun = polearea{2};
											else
												error('control:gamma:arguments:test', 'Undefined area type.');
											end
											if areasizeflag(oo) == 1
												if isfunctionhandle(areafun)
													areafun = repmat({areafun}, size(sys, 1), 1);
												else
													areafun = repmat(areafun, size(sys, 1), 1);
												end
											elseif areasizeflag(oo) == 2
												if isfunctionhandle(areafun)
													areafun = repmat({areafun}, sum(sysexpand, 1), 1);
												else
													areafun = repmat(areafun, sum(sysexpand, 1), 1);
												end
											end
											numareafun_strict = size(areafun, 2);
											numareafun_loose = size(areafun, 2);
										else
											if areafunflag(ll) == 0
												areafun = {poleareafun, poleareafun};
											elseif areafunflag(ll) == 1
												areafun = {polearea{1}, polearea{1}};
											elseif areafunflag(ll) == 2
												areafun = {polearea{2}, polearea{2}};
											else
												error('control:gamma:arguments:test', 'Undefined area type.');
											end
											numareafun_strict = size(areafun{1}, 2);
											numareafun_loose = size(areafun{2}, 2);
											if areasizeflag(oo) == 1
												if isfunctionhandle(areafun{1})
													areafun{1} = repmat({areafun{1}}, size(sys, 1), 1);
												else
													areafun{1} = repmat(areafun{1}, size(sys, 1), 1);
												end
												if isfunctionhandle(areafun{2})
													areafun{2} = repmat({areafun{2}}, size(sys, 1), 1);
												else
													areafun{2} = repmat(areafun{2}, size(sys, 1), 1);
												end
											elseif areasizeflag(oo) == 2
												if isfunctionhandle(areafun{1})
													areafun{1} = repmat({areafun{1}}, sum(sysexpand, 1), 1);
												else
													areafun{1} = repmat(areafun{1}, sum(sysexpand, 1), 1);
												end
												if isfunctionhandle(areafun{2})
													areafun{2} = repmat({areafun{2}}, sum(sysexpand, 1), 1);
												else
													areafun{2} = repmat(areafun{2}, sum(sysexpand, 1), 1);
												end
											end
										end
										if isnan(weightcombine(qq))
											if ~weightareaflag(rr)
												weight = cat(3, 1, 1);
											else
												weight = cat(3, ones(1, numareafun_strict), ones(1, numareafun_strict));
											end
											if weightsizeflag(pp) == 1
												weight = repmat(weight, size(sys, 1), 1);
											elseif weightsizeflag(pp) == 2
												weight = repmat(weight, sum(sysexpand, 1), 1);
											end
										elseif weightcombine(qq)
											if ~weightareaflag(rr)
												weight = 1;
											else
												weight = ones(1, numareafun_strict);
											end
											if weightsizeflag(pp) == 1
												weight = repmat(weight, size(sys, 1), 1);
											elseif weightsizeflag(pp) == 2
												weight = repmat(weight, sum(sysexpand, 1), 1);
											end
										else
											if ~weightareaflag(rr)
												weight = {1, 1};
											else
												weight = {ones(1, numareafun_strict), ones(1, numareafun_loose)};
											end
											if weightsizeflag(pp) == 1
												weight{1} = repmat(weight{1}, size(sys, 1), 1);
												weight{2} = repmat(weight{2}, size(sys, 1), 1);
											elseif weightsizeflag(pp) == 2
												weight{1} = repmat(weight{1}, sum(sysexpand, 1), 1);
												weight{2} = repmat(weight{2}, sum(sysexpand, 1), 1);
											end
										end
										if iscell(sys) && numel(sys) == 1
											test.TestSuite.assertNoException('[system1, areafun_strict, areafun_loose, weight_strict, weight_loose, dimensions_strict, dimensions_loose, number_states_all, bounds, nonlcon] = fun(sys, areafun, weight, systemoptions, [], [], [], [], allowvarorder);', 'control:outputfeedback:test', 'optimize must not throw an exception for six input arguments.');
											test.TestSuite.assertNoException('[system2, areafun_strict2, areafun_loose2, weight_strict2, weight_loose2, dimensions_strict2, dimensions_loose2, number_states_all2, bounds2, nonlcon2] = fun(sys{1}, areafun, weight, systemoptions, [], [], [], [], allowvarorder);', 'control:outputfeedback:test', 'optimize must not throw an exception for six input arguments.');
											test.TestSuite.assertEqual(system1, system2, 'control:gamma:arguments:test', 'Systems for call as cell array and without cell array must be equal.');
											test.TestSuite.assertEqual(areafun_strict, areafun_strict2, 'control:gamma:arguments:test', 'Strict areafun for call as cell array and without cell array must be equal.');
											test.TestSuite.assertEqual(areafun_loose, areafun_loose2, 'control:gamma:arguments:test', 'Loose areafun for call as cell array and without cell array must be equal.');
											test.TestSuite.assertEqual(weight_strict, weight_strict2, 'control:gamma:arguments:test', 'Strict weight for call as cell array and without cell array must be equal.');
											test.TestSuite.assertEqual(weight_loose, weight_loose2, 'control:gamma:arguments:test', 'Loose weight for call as cell array and without cell array must be equal.');
										else
											test.TestSuite.assertNoException('[system1, areafun_strict, areafun_loose, weight_strict, weight_loose, dimensions_strict, dimensions_loose, number_states_all, bounds, nonlcon] = fun(sys, areafun, weight, systemoptions, [], [], [], [], allowvarorder);', 'control:outputfeedback:test', 'optimize must not throw an exception for six input arguments.');
										end
										test.TestSuite.assertEqual(isstruct(system1), true, 'control:gamma:arguments:test', 'Checkandtransformargs must return systems as structure.');
										test.TestSuite.assertFieldnames(system1, {'E', 'A', 'B', 'C', 'C_dot', 'D'}, 'control:gamma:arguments:test');
										test.TestSuite.assertEqual(size(system1, 1), sum(sysexpand), 'control:gamma:arguments:test', 'Checkandtransformargs must return %d systems.', sum(sysexpand));
										test.TestSuite.assertEqual(size(system1, 2), 1, 'control:gamma:arguments:test', 'Checkandtransformargs must return systems as column.');
										test.TestSuite.assertEqual(size(areafun_strict, 1), sum(sysexpand), 'control:gamma:arguments:test', 'Checkandtransformargs must return %d strict areas.', sum(sysexpand));
										if ~isempty(areafun_loose)
											test.TestSuite.assertEqual(size(areafun_loose, 1), sum(sysexpand), 'control:gamma:arguments:test', 'Checkandtransformargs must return %d loose areas.', sum(sysexpand));
											end
										test.TestSuite.assertEqual(size(weight_strict, 1), sum(sysexpand), 'control:gamma:arguments:test', 'Checkandtransformargs must return %d strict weights.', sum(sysexpand));
										test.TestSuite.assertEqual(size(weight_strict, 2), size(areafun_strict, 2), 'control:gamma:arguments:test', 'Checkandtransformargs must return strict weights as column.');
										test.TestSuite.assertEqual(size(weight_loose, 1), sum(sysexpand), 'control:gamma:arguments:test', 'Checkandtransformargs must return %d loose weights.', sum(sysexpand));
										if ~isempty(areafun_loose)
											test.TestSuite.assertEqual(size(weight_loose, 2), size(areafun_loose, 2), 'control:gamma:arguments:test', 'Checkandtransformargs must return loose weight as column.');
										end
										test.TestSuite.assertFieldnames(dimensions_strict, dimensionfields, 'control:gamma:arguments:test');
										test.TestSuite.assertEqual(dimensions_strict.models, sum(sysexpand), 'Checkandtransformargs must return %d systems.', sum(sysexpand));
										test.TestSuite.assertEqual(all(dimensions_strict.controls == number_controls), true, 'Checkandtransformargs must return systems with %d controls.', number_controls);
										test.TestSuite.assertEqual(all(dimensions_strict.measurements == number_measurements), true, 'Checkandtransformargs must return systems with %d measurements.', number_measurements);
										test.TestSuite.assertFieldnames(dimensions_strict, dimensionfields, 'control:gamma:arguments:test');
										test.TestSuite.assertEqual(dimensions_loose.models, sum(sysexpand), 'Checkandtransformargs must return %d systems.', sum(sysexpand));
										test.TestSuite.assertEqual(all(dimensions_loose.controls == number_controls), true, 'Checkandtransformargs must return systems with %d controls.', number_controls);
										test.TestSuite.assertEqual(all(dimensions_loose.measurements == number_measurements), true, 'Checkandtransformargs must return systems with %d measurements.', number_measurements);
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
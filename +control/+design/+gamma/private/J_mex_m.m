function [J, gradJ, hessianJ] = J_mex_m(x, system, weight, areafun, dimensions, options)
	%J_MEX_M calculate objective function, gradient and hessian for gamma pole placement
	%	Input:
	%		x:			current optimization value (gain reshaped as column vector)
	%		system:		structure with system matrices of systems to take into consideration
	%		weight:		weighting matrix with number of systems columns and number of pole area border functions rows
	%		areafun:	area border functions and gradients as cell array with number of system entries, each function returns a row vector with as much elements as area border functions are defined for the corresponding system
	%		dimensions:	structure with information about dimensions of the different variables and systems
	%		options:	structure with options for objective function
	%	Output:
	%		J:			objective function value for current optimization value
	%		gradJ:		gradient of objective function value for current optimization value
	%		hessianJ:	hessian of objective function value for current optimization value
	if nargout >= 2 && ~dimensions.area_hasgrad
		error('control:design:gamma:gradient', 'Gradient for polearea functions was not supplied.');
	end
	if nargout >= 3 && ~dimensions.area_hashess
		error('control:design:gamma:hessian', 'Hessian for polearea functions was not supplied.');
	end
	number_models = dimensions.models;
	number_states = dimensions.states;
	number_controls = dimensions.controls;
	number_measurements = dimensions.measurements;
	number_measurements_xdot = dimensions.measurements_xdot;
	number_references = dimensions.references;
	derivative_feedback = number_measurements_xdot > 0;
	R_fixed_has = dimensions.R_fixed_has;
	K_fixed_has = dimensions.K_fixed_has;
	F_fixed_has = dimensions.F_fixed_has;
	RKF_fixed_has = dimensions.RKF_fixed_has;
	T_inv = dimensions.R_fixed_T_inv;
	T_inv_xdot = dimensions.K_fixed_T_inv;
	T_inv_prefilter = dimensions.F_fixed_T_inv;
	T_inv_RKF = dimensions.RKF_fixed_T_inv;
	numthreads = options.numthreads;
	eigenvaluederivativetype = options.eigenvaluederivative;
	eigenvalueignoreinf = options.eigenvalueignoreinf;
	isgaintype = false;
	for ii  = 1:size(options.type, 1) %#ok<FORPF> no parfor because of break
		if GammaJType_isgainobjective(options.type(ii, 1))
			isgaintype = true;
			break;
		end
	end
	[R, K, F] = x2R(x, dimensions);
	if nargout >= 3
		if all(options.type == GammaJType.ZERO)
			J = 0;
			gradJ = zeros(number_controls*(number_measurements + number_measurements_xdot + number_references), 1);
			hessianJ = zeros(number_controls*(number_measurements + number_measurements_xdot + number_references), number_controls*(number_measurements + number_measurements_xdot + number_references));
		elseif any(options.type == GammaJType.EIGENVALUECONDITION)
			% TODO: calculate hessian for eigenvector dependent objective function
			error('control:design:gamma:hessian', 'Hessian for eigenvector dependent objective function is not yet implemented.');
			if derivative_feedback
				[eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative, eigenvalue_derivative_xdot, eigenvector_right_derivative, eigenvector_right_derivative_xdot, eigenvector_left_derivative, eigenvector_left_derivative_xdot] = calculate_eigenvalues_m(system, R, K, dimensions, eigenvaluederivativetype, numthreads, options.eigenvaluefilter);
				[eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative, eigenvalue_derivative_xdot, eigenvector_right_derivative, eigenvector_right_derivative_xdot, eigenvector_left_derivative, eigenvector_left_derivative_xdot] = calculate_eigenvalue_filter(options.eigenvaluefilter, eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative, eigenvalue_derivative_xdot, eigenvector_right_derivative, eigenvector_right_derivative_xdot, eigenvector_left_derivative, eigenvector_left_derivative_xdot);
				[areaval, areaval_derivative] = calculate_areas_fixed_m(areafun, weight, eigenvalues, dimensions, numthreads, eigenvalueignoreinf);
				[J, gradJtemp] = calculate_objective(areaval, weight, eigenvalue_derivative, eigenvalue_derivative_xdot, areaval_derivative, dimensions, options);
				[J_eigenvector, gradJtemp_eigenvector] = calculate_objective_eigenvector(system, R, K, F, dimensions, options, eigenvector_right, eigenvector_left, eigenvector_right_derivative, eigenvector_right_derivative_xdot, eigenvector_left_derivative, eigenvector_left_derivative_xdot);
				if isgaintype
					[J_gain, gradJ_gain] = calculate_objective_gain(system, R, K, F, dimensions, options);
				else
					J_gain = 0;
					gradJ_gain = zeros(number_controls, number_measurements + number_measurements_xdot + number_references, 1);
				end
				J = J + J_eigenvector + J_gain;
				gradJtemp = gradJtemp + gradJtemp_eigenvector + gradJ_gain;
				gradJ_row = reshape(sum(gradJtemp, 3), number_controls*(number_measurements + number_measurements_xdot + number_references), 1);
				if RKF_fixed_has
					if options.objective.preventNaN
						gradJ = mtimes_preventNaN(gradJ_row.', T_inv_RKF).';
					else
						gradJ = (gradJ_row.'*T_inv_RKF).';
					end
				elseif R_fixed_has || K_fixed_has || F_fixed_has
					T_inv_blk = blkdiag(T_inv, T_inv_xdot, T_inv_prefilter);
					if options.objective.preventNaN
						gradJ = mtimes_preventNaN(gradJ_row.', T_inv_blk).';
					else
						gradJ = (gradJ_row.'*T_inv_blk).';
					end
				else
					gradJ = gradJ_row;
				end
			else
				[eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative, ~, eigenvector_right_derivative, eigenvector_right_derivative_xdot, eigenvector_left_derivative, eigenvector_left_derivative_xdot] = calculate_eigenvalues_m(system, R, K, dimensions, eigenvaluederivativetype, numthreads, options.eigenvaluefilter);
				[eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative, ~, eigenvector_right_derivative, eigenvector_right_derivative_xdot, eigenvector_left_derivative, eigenvector_left_derivative_xdot] = calculate_eigenvalue_filter(options.eigenvaluefilter, eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative, [], eigenvector_right_derivative, eigenvector_right_derivative_xdot, eigenvector_left_derivative, eigenvector_left_derivative_xdot);
				[areaval, areaval_derivative] = calculate_areas_fixed_m(areafun, weight, eigenvalues, dimensions, numthreads, eigenvalueignoreinf);
				eigenvalue_derivative_xdot = zeros(number_states, number_controls, number_measurements_xdot, number_models) + 0i;
				[J, gradJtemp] = calculate_objective(areaval, weight, eigenvalue_derivative, eigenvalue_derivative_xdot, areaval_derivative, dimensions, options);
				[J_eigenvector, gradJtemp_eigenvector] = calculate_objective_eigenvector(system, R, K, F, dimensions, options, eigenvector_right, eigenvector_left, eigenvector_right_derivative, eigenvector_right_derivative_xdot, eigenvector_left_derivative, eigenvector_left_derivative_xdot);
				if isgaintype
					[J_gain, gradJ_gain] = calculate_objective_gain(system, R, K, F, dimensions, options);
				else
					J_gain = 0;
					gradJ_gain = zeros(number_controls, number_measurements + number_references, 1);
				end
				J = J + J_eigenvector + J_gain;
				gradJtemp = gradJtemp + gradJtemp_eigenvector + gradJ_gain;
				gradJ_row = reshape(sum(gradJtemp, 3), number_controls*(number_measurements + number_references), 1);
				if RKF_fixed_has
					if options.objective.preventNaN
						gradJ = mtimes_preventNaN(gradJ_row.', T_inv_RKF).';
					else
						gradJ = (gradJ_row.'*T_inv_RKF).';
					end
				elseif R_fixed_has || F_fixed_has
					T_inv_blk = blkdiag(T_inv, T_inv_prefilter);
					if options.objective.preventNaN
						gradJ = mtimes_preventNaN(gradJ_row.', T_inv_blk).';
					else
						gradJ = (gradJ_row.'*T_inv_blk).';
					end
				else
					gradJ = gradJ_row;
				end
			end
		else
			if derivative_feedback
				% HINT: matlab R2015B does not recognize size of needseigenvalues correctly and assumes :?x:? instead of :?x1
				needseigenvalues = GammaJType_needseigenvalues(options.type);
				if any(needseigenvalues(:))
					[eigenvalues, ~, ~, eigenvalue_derivative, eigenvalue_derivative_xdot, ~, ~, ~, ~, eigenvalue_2derivative, eigenvalue_2derivative_xdot, eigenvalue_2derivative_m, eigenvalue_2derivative_xdot_m] = calculate_eigenvalues_m(system, R, K, dimensions, eigenvaluederivativetype, numthreads, options.eigenvaluefilter);
					[eigenvalues, ~, ~, eigenvalue_derivative, eigenvalue_derivative_xdot, ~, ~, ~, ~, eigenvalue_2derivative, eigenvalue_2derivative_xdot, eigenvalue_2derivative_m, eigenvalue_2derivative_xdot_m] = calculate_eigenvalue_filter(options.eigenvaluefilter, eigenvalues, [], [], eigenvalue_derivative, eigenvalue_derivative_xdot, [], [], [], [], eigenvalue_2derivative, eigenvalue_2derivative_xdot, eigenvalue_2derivative_m, eigenvalue_2derivative_xdot_m);
					[areaval, areaval_derivative, areaval_2_derivative] = calculate_areas_fixed_m(areafun, weight, eigenvalues, dimensions, numthreads, eigenvalueignoreinf);
					[J, gradJtemp, hessianJ] = calculate_objective(areaval, weight, eigenvalue_derivative, eigenvalue_derivative_xdot, areaval_derivative, dimensions, options, eigenvalue_2derivative, areaval_2_derivative, eigenvalue_2derivative_xdot, eigenvalue_2derivative_m, eigenvalue_2derivative_xdot_m);
				else
					J = 0;
					gradJtemp = zeros(number_controls, number_measurements + number_measurements_xdot + number_references, 1);
					hessianJ = zeros(number_controls*(number_measurements + number_measurements_xdot + number_references), number_controls*(number_measurements + number_measurements_xdot + number_references));
				end
				if isgaintype
					[J_gain, gradJ_gain, hessianJ_gain] = calculate_objective_gain(system, R, K, F, dimensions, options);
				else
					J_gain = 0;
					gradJ_gain = zeros(number_controls, number_measurements + number_measurements_xdot + number_references, 1);
					hessianJ_gain = zeros(number_controls*(number_measurements + number_measurements_xdot + number_references), number_controls*(number_measurements + number_measurements_xdot + number_references));
				end
				J = J + J_gain;
				gradJtemp = gradJtemp + gradJ_gain;
				hessianJ = hessianJ + hessianJ_gain;
				gradJ_row = reshape(sum(gradJtemp, 3), number_controls*(number_measurements + number_measurements_xdot + number_references), 1);
				if RKF_fixed_has
					if options.objective.preventNaN
						gradJ = mtimes_preventNaN(gradJ_row.', T_inv_RKF).';
						hessianJ = mtimes_preventNaN(T_inv_RKF.', mtimes_preventNaN(hessianJ, T_inv_RKF));
					else
						gradJ = (gradJ_row.'*T_inv_RKF).';
						hessianJ = T_inv_RKF.'*hessianJ*T_inv_RKF;
					end
				elseif R_fixed_has || K_fixed_has || F_fixed_has
					T_inv_blk = blkdiag(T_inv, T_inv_xdot, T_inv_prefilter);
					if options.objective.preventNaN
						gradJ = mtimes_preventNaN(gradJ_row.', T_inv_blk).';
						hessianJ = mtimes_preventNaN(T_inv_blk.', mtimes_preventNaN(hessianJ, T_inv_blk));
					else
						gradJ = (gradJ_row.'*T_inv_blk).';
						hessianJ = T_inv_blk.'*hessianJ*T_inv_blk;
					end
				else
					gradJ = gradJ_row;
				end
			else
				% HINT: matlab R2015B does not recognize size of needseigenvalues correctly and assumes :?x:? instead of :?x1
				needseigenvalues = GammaJType_needseigenvalues(options.type);
				if any(needseigenvalues(:))
					[eigenvalues, ~, ~, eigenvalue_derivative, ~, ~, ~, ~, ~, eigenvalue_2derivative] = calculate_eigenvalues_m(system, R, K, dimensions, eigenvaluederivativetype, numthreads, options.eigenvaluefilter);
					[eigenvalues, ~, ~, eigenvalue_derivative, ~, ~, ~, ~, ~, eigenvalue_2derivative] = calculate_eigenvalue_filter(options.eigenvaluefilter, eigenvalues, [], [], eigenvalue_derivative, [], [], [], [], [], eigenvalue_2derivative);
					[areaval, areaval_derivative, areaval_2_derivative] = calculate_areas_fixed_m(areafun, weight, eigenvalues, dimensions, numthreads, eigenvalueignoreinf);
					eigenvalue_derivative_xdot = zeros(number_states, number_controls, number_measurements_xdot, number_models) + 0i;
					eigenvalue_2derivative_tmp = zeros(number_states, number_controls*number_measurements_xdot, number_controls*number_measurements_xdot, number_models) + 0i;
					[J, gradJtemp, hessianJ] = calculate_objective(areaval, weight, eigenvalue_derivative, eigenvalue_derivative_xdot, areaval_derivative, dimensions, options, eigenvalue_2derivative, areaval_2_derivative, eigenvalue_2derivative_tmp, eigenvalue_2derivative_tmp, eigenvalue_2derivative_tmp);
				else
					J = 0;
					gradJtemp = zeros(number_controls, number_measurements + number_references, 1);
					hessianJ = zeros(number_controls*(number_measurements + number_references), number_controls*(number_measurements + number_references));
				end
				if isgaintype
					[J_gain, gradJ_gain, hessianJ_gain] = calculate_objective_gain(system, R, K, F, dimensions, options);
				else
					J_gain = 0;
					gradJ_gain = zeros(number_controls, number_measurements + number_measurements_xdot + number_references, 1);
					hessianJ_gain = zeros(number_controls*(number_measurements + number_measurements_xdot + number_references), number_controls*(number_measurements + number_measurements_xdot + number_references));
				end
				J = J + J_gain;
				gradJtemp = gradJtemp + gradJ_gain;
				hessianJ = hessianJ + hessianJ_gain;
				gradJ_row = reshape(sum(gradJtemp, 3), number_controls*(number_measurements + number_references), 1);
				if RKF_fixed_has
					if options.objective.preventNaN
						gradJ = mtimes_preventNaN(gradJ_row.', T_inv_RKF).';
						hessianJ = mtimes_preventNaN(T_inv_RKF.', mtimes_preventNaN(hessianJ, T_inv_RKF));
					else
						gradJ = (gradJ_row.'*T_inv_RKF).';
						hessianJ = T_inv_RKF.'*hessianJ*T_inv_RKF;
					end
				elseif R_fixed_has || K_fixed_has || F_fixed_has
					T_inv_blk = blkdiag(T_inv, T_inv_prefilter);
					if options.objective.preventNaN
						gradJ = mtimes_preventNaN(gradJ_row.', T_inv_blk).';
						hessianJ = mtimes_preventNaN(T_inv_blk.', mtimes_preventNaN(hessianJ, T_inv_blk));
					else
						gradJ = (gradJ_row.'*T_inv_blk).';
						hessianJ = T_inv_blk.'*hessianJ*T_inv_blk;
					end
				else
					gradJ = gradJ_row;
				end
			end
		end
		if RKF_fixed_has
			gradJ = gradJ(dimensions.index_RKF_free, :);
			hessianJ = hessianJ(dimensions.index_RKF_free, dimensions.index_RKF_free);
		elseif R_fixed_has || K_fixed_has || F_fixed_has
			gradJ = gradJ(dimensions.index_all_free, :);
			hessianJ = hessianJ(dimensions.index_all_free, dimensions.index_all_free);
		end
	elseif nargout >= 2
		if all(options.type == GammaJType.ZERO)
			J = 0;
			gradJ = zeros(number_controls*(number_measurements + number_measurements_xdot), 1);
		elseif any(options.type == GammaJType.EIGENVALUECONDITION)
			if derivative_feedback
				[eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative, eigenvalue_derivative_xdot, eigenvector_right_derivative, eigenvector_right_derivative_xdot, eigenvector_left_derivative, eigenvector_left_derivative_xdot] = calculate_eigenvalues_m(system, R, K, dimensions, eigenvaluederivativetype, numthreads, options.eigenvaluefilter);
				[eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative, eigenvalue_derivative_xdot, eigenvector_right_derivative, eigenvector_right_derivative_xdot, eigenvector_left_derivative, eigenvector_left_derivative_xdot] = calculate_eigenvalue_filter(options.eigenvaluefilter, eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative, eigenvalue_derivative_xdot, eigenvector_right_derivative, eigenvector_right_derivative_xdot, eigenvector_left_derivative, eigenvector_left_derivative_xdot);
				[areaval, areaval_derivative] = calculate_areas_fixed_m(areafun, weight, eigenvalues, dimensions, numthreads, eigenvalueignoreinf);
				[J, gradJtemp] = calculate_objective(areaval, weight, eigenvalue_derivative, eigenvalue_derivative_xdot, areaval_derivative, dimensions, options);
				[J_eigenvector, gradJtemp_eigenvector] = calculate_objective_eigenvector(system, R, K, F, dimensions, options, eigenvector_right, eigenvector_left, eigenvector_right_derivative, eigenvector_right_derivative_xdot, eigenvector_left_derivative, eigenvector_left_derivative_xdot);
				if isgaintype
					[J_gain, gradJ_gain] = calculate_objective_gain(system, R, K, F, dimensions, options);
				else
					J_gain = 0;
					gradJ_gain = zeros(number_controls, number_measurements + number_measurements_xdot + number_references, 1);
				end
				J = J + J_eigenvector + J_gain;
				gradJtemp = gradJtemp + gradJtemp_eigenvector + gradJ_gain;
				gradJ_row = reshape(sum(gradJtemp, 3), number_controls*(number_measurements + number_measurements_xdot + number_references), 1);
				if RKF_fixed_has
					if options.objective.preventNaN
						gradJ = mtimes_preventNaN(gradJ_row.', T_inv_RKF).';
					else
						gradJ = (gradJ_row.'*T_inv_RKF).';
					end
				elseif R_fixed_has || K_fixed_has || F_fixed_has
					T_inv_blk = blkdiag(T_inv, T_inv_xdot, T_inv_prefilter);
					if options.objective.preventNaN
						gradJ = mtimes_preventNaN(gradJ_row.', T_inv_blk).';
					else
						gradJ = (gradJ_row.'*T_inv_blk).';
					end
				else
					gradJ = gradJ_row;
				end
			else
				[eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative, ~, eigenvector_right_derivative, eigenvector_right_derivative_xdot, eigenvector_left_derivative, eigenvector_left_derivative_xdot] = calculate_eigenvalues_m(system, R, K, dimensions, eigenvaluederivativetype, numthreads, options.eigenvaluefilter);
				[eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative, ~, eigenvector_right_derivative, eigenvector_right_derivative_xdot, eigenvector_left_derivative, eigenvector_left_derivative_xdot] = calculate_eigenvalue_filter(options.eigenvaluefilter, eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative, [], eigenvector_right_derivative, eigenvector_right_derivative_xdot, eigenvector_left_derivative, eigenvector_left_derivative_xdot);
				[areaval, areaval_derivative] = calculate_areas_fixed_m(areafun, weight, eigenvalues, dimensions, numthreads, eigenvalueignoreinf);
				eigenvalue_derivative_xdot = zeros(number_states, number_controls, number_measurements_xdot, number_models) + 0i;
				[J, gradJtemp] = calculate_objective(areaval, weight, eigenvalue_derivative, eigenvalue_derivative_xdot, areaval_derivative, dimensions, options);
				[J_eigenvector, gradJtemp_eigenvector] = calculate_objective_eigenvector(system, R, K, F, dimensions, options, eigenvector_right, eigenvector_left, eigenvector_right_derivative, eigenvector_right_derivative_xdot, eigenvector_left_derivative, eigenvector_left_derivative_xdot);
				if isgaintype
					[J_gain, gradJ_gain] = calculate_objective_gain(system, R, K, F, dimensions, options);
				else
					J_gain = 0;
					gradJ_gain = zeros(number_controls, number_measurements + number_references, 1);
				end
				J = J + J_eigenvector + J_gain;
				gradJtemp = gradJtemp + gradJtemp_eigenvector + gradJ_gain;
				gradJ_row = reshape(sum(gradJtemp, 3), number_controls*(number_measurements + number_references), 1);
				if RKF_fixed_has
					if options.objective.preventNaN
							gradJ = mtimes_preventNaN(gradJ_row.', T_inv_RKF).';
					else
						gradJ = (gradJ_row.'*T_inv_RKF).';
					end
				elseif R_fixed_has || K_fixed_has || F_fixed_has
					T_inv_blk = blkdiag(T_inv, T_inv_prefilter);
					if options.objective.preventNaN
							gradJ = mtimes_preventNaN(gradJ_row.', T_inv_blk).';
					else
						gradJ = (gradJ_row.'*T_inv_blk).';
					end
				else
					gradJ = gradJ_row;
				end
			end
		else
			if derivative_feedback
				% HINT: matlab R2015B does not recognize size of needseigenvalues correctly and assumes :?x:? instead of :?x1
				needseigenvalues = GammaJType_needseigenvalues(options.type);
				if any(needseigenvalues(:))
					[eigenvalues, ~, ~, eigenvalue_derivative, eigenvalue_derivative_xdot] = calculate_eigenvalues_m(system, R, K, dimensions, eigenvaluederivativetype, numthreads, options.eigenvaluefilter);
					[eigenvalues, ~, ~, eigenvalue_derivative, eigenvalue_derivative_xdot] = calculate_eigenvalue_filter(options.eigenvaluefilter, eigenvalues, [], [], eigenvalue_derivative, eigenvalue_derivative_xdot);
					[areaval, areaval_derivative] = calculate_areas_fixed_m(areafun, weight, eigenvalues, dimensions, numthreads, eigenvalueignoreinf);
					[J, gradJtemp] = calculate_objective(areaval, weight, eigenvalue_derivative, eigenvalue_derivative_xdot, areaval_derivative, dimensions, options);
				else
					J = 0;
					gradJtemp = zeros(number_controls, number_measurements + number_measurements_xdot + number_references, 1);
				end
				if isgaintype
					[J_gain, gradJ_gain] = calculate_objective_gain(system, R, K, F, dimensions, options);
				else
					J_gain = 0;
					gradJ_gain = zeros(number_controls, number_measurements + number_measurements_xdot + number_references, 1);
				end
				J = J + J_gain;
				gradJtemp = gradJtemp + gradJ_gain;
				gradJ_row = reshape(sum(gradJtemp, 3), number_controls*(number_measurements + number_measurements_xdot + number_references), 1);
				if RKF_fixed_has
					if options.objective.preventNaN
						gradJ = mtimes_preventNaN(gradJ_row.', T_inv_RKF).';
					else
						gradJ = (gradJ_row.'*T_inv_RKF).';
					end
				elseif R_fixed_has || K_fixed_has || F_fixed_has
					T_inv_blk = blkdiag(T_inv, T_inv_xdot, T_inv_prefilter);
					if options.objective.preventNaN
						gradJ = mtimes_preventNaN(gradJ_row.', T_inv_blk).';
					else
						gradJ = (gradJ_row.'*T_inv_blk).';
					end
				else
					gradJ = gradJ_row;
				end
			else
				% HINT: matlab R2015B does not recognize size of needseigenvalues correctly and assumes :?x:? instead of :?x1
				needseigenvalues = GammaJType_needseigenvalues(options.type);
				if any(needseigenvalues(:))
					[eigenvalues, ~, ~, eigenvalue_derivative] = calculate_eigenvalues_m(system, R, K, dimensions, eigenvaluederivativetype, numthreads, options.eigenvaluefilter);
					[eigenvalues, ~, ~, eigenvalue_derivative] = calculate_eigenvalue_filter(options.eigenvaluefilter, eigenvalues, [], [], eigenvalue_derivative);
					[areaval, areaval_derivative] = calculate_areas_fixed_m(areafun, weight, eigenvalues, dimensions, numthreads, eigenvalueignoreinf);
					eigenvalue_derivative_xdot = zeros(number_states, number_controls, number_measurements_xdot, number_models) + 0i;
					[J, gradJtemp] = calculate_objective(areaval, weight, eigenvalue_derivative, eigenvalue_derivative_xdot, areaval_derivative, dimensions, options);
				else
					J = 0;
					gradJtemp = zeros(number_controls, number_measurements + number_references, 1);
				end
				if isgaintype
					[J_gain, gradJ_gain] = calculate_objective_gain(system, R, K, F, dimensions, options);
				else
					J_gain = 0;
					gradJ_gain = zeros(number_controls, number_measurements + number_references, 1);
				end
				J = J + J_gain;
				gradJtemp = gradJtemp + gradJ_gain;
				gradJ_row = reshape(sum(gradJtemp, 3), number_controls*(number_measurements + number_references), 1);
				if RKF_fixed_has
					if options.objective.preventNaN
						gradJ = mtimes_preventNaN(gradJ_row.', T_inv_RKF).';
					else
						gradJ = (gradJ_row.'*T_inv_RKF).';
					end
				elseif R_fixed_has || K_fixed_has || F_fixed_has
					T_inv_blk = blkdiag(T_inv, T_inv_prefilter);
					if options.objective.preventNaN
						gradJ = mtimes_preventNaN(gradJ_row.', T_inv_blk).';
					else
						gradJ = (gradJ_row.'*T_inv_blk).';
					end
				else
					gradJ = gradJ_row;
				end
			end
		end
		if RKF_fixed_has
			gradJ = gradJ(dimensions.index_RKF_free, :);
		elseif R_fixed_has || K_fixed_has || F_fixed_has
			gradJ = gradJ(dimensions.index_all_free, :);
		end
	else
		if all(options.type == GammaJType.ZERO)
			J = 0;
		elseif any(options.type == GammaJType.EIGENVALUECONDITION)
			[eigenvalues, eigenvector_right, eigenvector_left] = calculate_eigenvalues_m(system, R, K, dimensions, eigenvaluederivativetype, numthreads, options.eigenvaluefilter);
			[eigenvalues, eigenvector_right, eigenvector_left] = calculate_eigenvalue_filter(options.eigenvaluefilter, eigenvalues, eigenvector_right, eigenvector_left);
			areaval = calculate_areas_fixed_m(areafun, weight, eigenvalues, dimensions, numthreads, eigenvalueignoreinf);
			J = calculate_objective(areaval, weight, [], [], [], dimensions, options);
			J_eigenvector = calculate_objective_eigenvector(system, R, K, F, dimensions, options, eigenvector_right, eigenvector_left);
			if isgaintype
				J_gain = calculate_objective_gain(system, R, K, F, dimensions, options);
			else
				J_gain = 0;
			end
			J = J + J_eigenvector + J_gain;
		else
			% HINT: matlab R2015B does not recognize size of needseigenvalues correctly and assumes :?x:? instead of :?x1
			needseigenvalues = GammaJType_needseigenvalues(options.type);
			if any(needseigenvalues(:))
				eigenvalues = calculate_eigenvalues_m(system, R, K, dimensions, eigenvaluederivativetype, numthreads, options.eigenvaluefilter);
				eigenvalues = calculate_eigenvalue_filter(options.eigenvaluefilter, eigenvalues);
				areaval = calculate_areas_fixed_m(areafun, weight, eigenvalues, dimensions, numthreads, eigenvalueignoreinf);
				J = calculate_objective(areaval, weight, [], [], [], dimensions, options);
			else
				J = 0;
			end
			if isgaintype
				J_gain = calculate_objective_gain(system, R, K, F, dimensions, options);
			else
				J_gain = 0;
			end
			J = J + J_gain;
		end
	end
end
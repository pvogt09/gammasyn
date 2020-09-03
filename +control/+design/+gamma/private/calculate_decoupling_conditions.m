function [c, ceq, gradc, gradceq] = calculate_decoupling_conditions(system, R, K, F, dimensions, options, eigenvalues, eigenvector_right, eigenvector_left, eigenvector_right_derivative, eigenvector_left_derivative)
	%CALCULATE_DECOUPLING_CONDITIONS calculate nonlinear output and input decoupling conditions for fully numeric decoupling controller design
	%	Input:
	%		system:							structure with dynamic systems to take into consideration
	%		R:								proportional gain matrix
	%		K:								derivative gain matrix
	%		F:								prefilter matrix
	%		dimensions:						structure with information about dimensions of the variables and systems
	%		options:						structure with options
	%		eigenvalues:					array with eigenvalues of all controlled models
	%		eigenvector_right:				3D-array containing the right eigenvector matrix of each controlled dynamic system
	%		eigenvector_left:				3D-array containing the left eigenvector matrix of each controlled dynamic system
	%		eigenvector_right_derivative:	5D-array containing the derivatives of the right eigenvector matrix of each controlled dynamic system with respect to every controller parameter
	%		eigenvector_left_derivative:	5D-array containing the derivatives of the left eigenvector matrix of each controlled dynamic system with respect to every controller parameter
	%	Output:
	%		c:								array of inequality constraints
	%		ceq:							array of equality constraints
	%		gradc:							array of gradients of inequality constraints
	%		gradceq:						array of gradients of equality constraints

	%% initialization
	needsgradient = nargout >= 3;
	numthreads = options.numthreads;
	number_models = dimensions.models;
	number_states = dimensions.states;
	number_controls = dimensions.controls;
	number_measurements = dimensions.measurements;
	number_measurements_xdot = dimensions.measurements_xdot;
	number_references = dimensions.references;
	number_coefficients = number_controls*(number_measurements + number_measurements_xdot + number_references);
	tf_structure = options.decouplingcontrol.tf_structure;
	number_decouplingconditions = dimensions.number_decouplingconditions.';
	indices_columns_with_zeros = find(number_decouplingconditions);
	number_columns_with_zeros = length(indices_columns_with_zeros);
	dim_invariant = dimensions.m_invariant.';
	hasfeedthrough = int32(dimensions.hasfeedthrough_decoupling).';
	sortingstrategy = options.decouplingcontrol.sortingstrategy_decoupling;
	minimumnormsorting = sortingstrategy == GammaDecouplingconditionSortingStrategy.MINIMUMNORM;
	eigenvaluetracking = sortingstrategy == GammaDecouplingconditionSortingStrategy.EIGENVALUETRACKING;
	if ~options.decouplingcontrol.allowoutputdecoupling
		if number_states ~= number_measurements
			error('control:design:gamma:decoupling', 'Number of states (%d) must match number of measurements (%d) for decoupling control.', number_states, number_measurements);
		end
	end
	if any(any(K ~= zeros(number_controls, number_measurements_xdot)))
		error('control:design:gamma:decoupling', 'Derivative gain must be zero for decoupling control.');
	end

	if nargin <= 9
		eigenvector_right_derivative = zeros(number_states, number_states, number_controls, number_measurements, number_models);
	end
	if nargin <= 10
		eigenvector_left_derivative = zeros(number_states, number_states, number_controls, number_measurements, number_models);
	end

	number_outputconditions_per_sys_path = 2*(number_decouplingconditions.*dim_invariant); % factor 2 because of real and imaginary parts
	number_inputconditions_per_sys_path = 2*(repmat(number_states, 1, number_references) - dim_invariant) + number_decouplingconditions.*hasfeedthrough; % factor 2 because of real and imaginary parts
	number_outputconditions_per_sys = sum(number_outputconditions_per_sys_path);
	number_inputconditions_per_sys = sum(number_inputconditions_per_sys_path);

	eigenvalue_map = int32(zeros(number_states, 2, number_models));
	permutations_VR1 = NaN(0, number_states, number_references);
	permutations_WR2 = NaN(0, number_states, number_references);
	number_permutations = zeros(1, number_references);

	if eigenvaluetracking
		eigenvalue_map = track_eigenvalues(system, R, F, eigenvalues, eigenvector_right, eigenvector_left, number_models, number_states, number_controls, number_measurements, number_coefficients);
	elseif minimumnormsorting
		[permutations_VR1, permutations_WR2, number_permutations] = set_permutations(number_states, dim_invariant, number_references, numthreads);
	end

	dFdxi = zeros(number_controls, number_references, number_controls*number_references);
	if needsgradient
		indices = 1:(number_controls*number_references+1):(number_controls*number_references)^2;
		dFdxi(indices) = 1;
	end

	ceq_out = zeros(number_outputconditions_per_sys, 1, number_models);% output decoupling conditions
	ceq_in = zeros(number_inputconditions_per_sys, 1, number_models);% input decoupling conditions
	if needsgradient
		ceq_out_grad = zeros(number_coefficients, number_outputconditions_per_sys, number_models);% gradients of output decoupling conditions
		ceq_in_grad  = zeros(number_coefficients, number_inputconditions_per_sys, number_models);% gradients of input decoupling conditions
	end

	% TODO: choice of VR1 and WR2 still problematic. Can be seen if system with feedthrough is used and number_states == dim_invariant. Then VR1 = V and WR2 = []. So no choice has to be done. This works well.
	parfor (ii = 1:number_models, numthreads)
% 	for ii = 1:number_models
		%% system variables
		B = system(ii).B;
		C = system(ii).C;
		C_ref = system(ii).C_ref;
		D_ref = system(ii).D_ref;
		eigenvector_r = eigenvector_right(:, :, ii);
		eigenvector_l = eigenvector_left(:, :, ii)';% eigenvector_l*eigenvector_r = eye(number_states)
		ceq_out_jj = zeros(number_outputconditions_per_sys, 1);
		ceq_in_jj = zeros(number_inputconditions_per_sys, 1);
		ceq_out_grad_ii = zeros(number_coefficients, number_outputconditions_per_sys);% gradients of output decoupling conditions per system
		ceq_in_grad_ii  = zeros(number_coefficients, number_inputconditions_per_sys);% gradients of input decoupling conditions per system
		for jj = 1:number_references
			Cjj = C_ref(tf_structure(:, jj) == 0, :); %#ok<PFBNS>
			Djj = D_ref(tf_structure(:, jj) == 0, :);
			if isempty(Cjj)
				continue;
			end
			f = F(:, jj); %#ok<PFBNS>
			%% determine VR1 and WR2
			if minimumnormsorting
				permutations_VR1_jj = permutations_VR1(1:number_permutations(jj), 1:dim_invariant(jj), jj); %#ok<PFBNS>
				permutations_WR2_jj = permutations_WR2(1:number_permutations(jj), 1:(number_states - dim_invariant(jj)), jj); %#ok<PFBNS>
				[idx_VR1, idx_WR2] = minimumnorm_sorting(R, f, B, C, Cjj, Djj, eigenvector_r, eigenvector_l, permutations_VR1_jj, permutations_WR2_jj, number_permutations(jj), number_states, dim_invariant(jj), hasfeedthrough(jj)); %#ok<PFBNS>
			elseif eigenvaluetracking
				idx_VR1 = double(eigenvalue_map(1:dim_invariant(jj), 2, ii).'); %#ok<PFBNS>
				idx_WR2 = double(eigenvalue_map(dim_invariant(jj) + 1:end, 2, ii).');
			else
				idx_VR1 = double(1:dim_invariant(jj));
				idx_WR2 = double(dim_invariant(jj) + 1:number_states);
			end
% 			if ii == 1 % ignored for parfor, has to be commented out for codegen
% 				plot_tracked_eigenvalues(number_states, eigenvalues([idx_VR1, idx_WR2], ii)); %#ok<PFBNS>
% 			end
			VR1 = eigenvector_r(:, idx_VR1);
			WR2 = eigenvector_l(idx_WR2, :);

			%% input and output decoupling conditions of the system
			WBF = WR2*B*f;
			if hasfeedthrough(jj)
				CVR = (Cjj - Djj*R*C)*VR1;
			else
				CVR = Cjj*VR1;
			end
			ceq_out_jj(sum(number_outputconditions_per_sys_path(1:jj-1)) + (1:number_outputconditions_per_sys_path(jj)), :) = [
				reshape(real(CVR), number_decouplingconditions(jj)*dim_invariant(jj), 1);
				reshape(imag(CVR), number_decouplingconditions(jj)*dim_invariant(jj), 1)
			]; %#ok<PFBNS>
			if hasfeedthrough(jj)
				ceq_in_jj(sum(number_inputconditions_per_sys_path(1:jj-1)) + (1:number_inputconditions_per_sys_path(jj)), :) = [
					reshape(real(WBF.'),	(number_states - dim_invariant(jj))*(number_references - number_decouplingconditions(jj)), 1);
					reshape(imag(WBF.'),	(number_states - dim_invariant(jj))*(number_references - number_decouplingconditions(jj)), 1);
					reshape(Djj*f,			(number_decouplingconditions(jj))*(number_references - number_decouplingconditions(jj)), 1)
				]; %#ok<PFBNS>
			else
				ceq_in_jj(sum(number_inputconditions_per_sys_path(1:jj-1)) + (1:number_inputconditions_per_sys_path(jj)), :) = [
					reshape(real(WBF.'),	(number_states - dim_invariant(jj))*(number_references - number_decouplingconditions(jj)), 1);
					reshape(imag(WBF.'),	(number_states - dim_invariant(jj))*(number_references - number_decouplingconditions(jj)), 1);
				];
			end

			%% gradients needed
			if needsgradient
				eigenvector_l_derv_sys = eigenvector_left_derivative(:, :, :, :, double(ii)); %#ok<PFBNS> % ii must not be int32, otherwise parfor runtime error
				eigenvector_r_derv_sys = eigenvector_right_derivative(:, :, :, :, double(ii)); %#ok<PFBNS>
				%% gradients of output decoupling conditions
				if dim_invariant(jj) > 0
					all_grads_rev_mixed = [
						reshape(eigenvector_r_derv_sys(:, idx_VR1, :, :), number_states, number_measurements*number_controls*dim_invariant(jj)), zeros(number_states, number_controls*number_references*dim_invariant(jj))
					];
					all_grads_rev_split = reshape(all_grads_rev_mixed, number_states, dim_invariant(jj), number_coefficients);
					all_grads_rev_stacked_below = reshape(all_grads_rev_split, number_states*dim_invariant(jj), number_coefficients);
					if hasfeedthrough(jj)
						DdR = zeros(number_controls*dim_invariant(jj), number_coefficients) + 1i*zeros(number_controls*dim_invariant(jj), number_coefficients);
						for kk = 1:number_controls*number_measurements
							matrix = zeros(number_controls, number_measurements);
							matrix(kk) = 1;
							DdR(:, kk) = reshape(matrix*C*VR1, number_controls*dim_invariant(jj), 1);
						end
						all_grads_coupl_out = kron(eye(dim_invariant(jj)), Cjj - Djj*R*C)*all_grads_rev_stacked_below - kron(eye(dim_invariant(jj)), Djj)*DdR; % roughly factor 5 faster than for
					else
						all_grads_coupl_out = kron(eye(dim_invariant(jj)), Cjj)*all_grads_rev_stacked_below;
					end
				else
					all_grads_coupl_out = zeros(number_coefficients, 0);
				end

				ceq_out_grad_ii(:, sum(number_outputconditions_per_sys_path(1:jj-1)) + (1:number_outputconditions_per_sys_path(jj))) = [
					real(all_grads_coupl_out).', imag(all_grads_coupl_out).'
				];

				%% gradients of input decoupling conditions
				if number_states > dim_invariant(jj)
					all_grads_lev_mixed = reshape(eigenvector_l_derv_sys(:, idx_WR2, :, :), number_states, number_measurements*number_controls*(number_states - dim_invariant(jj)));
					all_grads_lev_split = reshape(all_grads_lev_mixed, number_states, number_states - dim_invariant(jj), number_measurements*number_controls);
					all_grads_lev_stacked_below = reshape(all_grads_lev_split, number_states*(number_states - dim_invariant(jj)), number_measurements*number_controls);
					all_grads_coupl_in_first_part = (kron(eye(number_states - dim_invariant(jj)), f.'*B.')*conj(all_grads_lev_stacked_below)).'; % why conj()?
					all_grads_coupl_in_secnd_part = NaN((number_references - number_decouplingconditions(jj))*(number_states - dim_invariant(jj)), number_controls*(number_references - number_decouplingconditions(jj))) + 1i*NaN((number_references - number_decouplingconditions(jj))*(number_states - dim_invariant(jj)), number_controls*(number_references - number_decouplingconditions(jj)));
					dFdxi_tmp = dFdxi(:, jj, (jj - 1)*number_controls + 1:jj*number_controls); %#ok<PFBNS>
					for kk = 1:number_controls*(number_references - number_decouplingconditions(jj))
						all_grads_coupl_in_secnd_part(:, kk) = reshape(dFdxi_tmp(:, :, kk).'*B.'*WR2.', (number_references - number_decouplingconditions(jj))*(number_states - dim_invariant(jj)), 1); % dFdxi is constant and calculated before loop
					end
					all_grads_coupl_in = [
						all_grads_coupl_in_first_part;
						zeros((jj - 1)*number_controls, (number_states - dim_invariant(jj)));
						all_grads_coupl_in_secnd_part.';
						zeros((number_references - jj)*number_controls, (number_states - dim_invariant(jj)))
					];
				else
					all_grads_coupl_in = zeros(number_coefficients, 0);
				end
				if hasfeedthrough(jj)
					ceq_in_grad_ii(:, sum(number_inputconditions_per_sys_path(1:jj-1)) + (1:number_inputconditions_per_sys_path(jj))) = [
						real(all_grads_coupl_in), imag(all_grads_coupl_in), [
							zeros(number_decouplingconditions(jj), number_controls*number_measurements + (jj - 1)*number_controls), Djj, zeros(number_decouplingconditions(jj), (number_references - jj)*number_controls)
						].'
					];
				else
					ceq_in_grad_ii(:, sum(number_inputconditions_per_sys_path(1:jj-1)) + (1:number_inputconditions_per_sys_path(jj))) = [
						real(all_grads_coupl_in), imag(all_grads_coupl_in)
					];
				end
			end
		end % end FOR each column
		ceq_out(:, :, ii) = ceq_out_jj;
		ceq_in(:, :, ii) = ceq_in_jj;
		if needsgradient
			ceq_out_grad(:, :, ii) = ceq_out_grad_ii;
			ceq_in_grad(:, :, ii) = ceq_in_grad_ii;
		end
	end % end FOR each system

	ceq_raw = [
		reshape(ceq_out, number_outputconditions_per_sys*number_models, 1);
		reshape(ceq_in, number_inputconditions_per_sys*number_models, 1)
	];

	%% prefilter F1 should not be trivial
	normF = options.decouplingcontrol.tolerance_prefilter;% parameter to avoid trivial solution for prefilter
	c_F = zeros(number_columns_with_zeros, 1);
	for ii = 1:number_columns_with_zeros
		c_F(ii, 1) = -F(:, indices_columns_with_zeros(ii)).'*F(:, indices_columns_with_zeros(ii)) + normF;% has no complex values and no complex gradient
	end
	if options.decouplingcontrol.decouplingstrategy == GammaDecouplingStrategy.NUMERIC_NONLINEAR_EQUALITY
		c = options.decouplingcontrol.weight_decoupling*c_F;
		ceq = options.decouplingcontrol.weight_decoupling*ceq_raw;
	elseif options.decouplingcontrol.decouplingstrategy == GammaDecouplingStrategy.NUMERIC_NONLINEAR_INEQUALITY
		tolerance_vec = options.decouplingcontrol.tolerance_decoupling*ones(size(ceq_raw, 1), 1);
		c = [
			options.decouplingcontrol.weight_decoupling*(ceq_raw - tolerance_vec);
			options.decouplingcontrol.weight_decoupling*(-ceq_raw - tolerance_vec);
			options.decouplingcontrol.weight_prefilter*c_F
		];
		ceq = zeros(0, 1);
	else
		c = zeros(0, 1);
		ceq = options.decouplingcontrol.weight_decoupling*ceq_raw;
	end
	if needsgradient
		ceq_grad_pre = [
			reshape(ceq_out_grad, number_coefficients, number_outputconditions_per_sys*number_models), reshape(ceq_in_grad, number_coefficients, number_inputconditions_per_sys*number_models)
		];
		ceq_grad_r = ceq_grad_pre(1:number_measurements*number_controls, :);
		ceq_grad_f = ceq_grad_pre(number_measurements*number_controls + 1:end, :);
		if dimensions.R_fixed_has
			ceq_grad_r_T = ((ceq_grad_r.')*dimensions.R_fixed_T_inv).';
		else
			ceq_grad_r_T = ceq_grad_r;
		end
		if dimensions.F_fixed_has
			ceq_grad_f_T = ((ceq_grad_f.')*dimensions.F_fixed_T_inv).';
		else
			ceq_grad_f_T = ceq_grad_f;
		end
		gradceq_raw = [
			ceq_grad_r_T;
			zeros(number_controls*number_measurements_xdot, number_models*(number_inputconditions_per_sys + number_outputconditions_per_sys));
			ceq_grad_f_T
		];

		grad_F_pre = zeros(number_controls*number_references, number_columns_with_zeros);
		for ii = 1:number_columns_with_zeros
			column = indices_columns_with_zeros(ii);
			grad_F_pre(1:column*number_controls, ii) = [
				zeros((column - 1)*number_controls, 1);
				-2*F(:, column)
			];
		end
		if dimensions.F_fixed_has
			grad_F_T = ((grad_F_pre.')*dimensions.F_fixed_T_inv).';
		else
			grad_F_T = grad_F_pre;
		end
		grad_F1 =	[
			zeros(number_controls*number_measurements, number_columns_with_zeros);
			zeros(number_controls*number_measurements_xdot, number_columns_with_zeros);
			grad_F_T
		];
		if options.decouplingcontrol.decouplingstrategy == GammaDecouplingStrategy.NUMERIC_NONLINEAR_EQUALITY
			gradc = options.decouplingcontrol.weight_prefilter*grad_F1;
			gradceq = options.decouplingcontrol.weight_decoupling*gradceq_raw;
		elseif options.decouplingcontrol.decouplingstrategy == GammaDecouplingStrategy.NUMERIC_NONLINEAR_INEQUALITY
			gradc = options.decouplingcontrol.weight_decoupling*[
				options.decouplingcontrol.weight_decoupling*gradceq_raw, options.decouplingcontrol.weight_decoupling*-gradceq_raw, options.decouplingcontrol.weight_prefilter*grad_F1
			];
			gradceq = zeros(number_coefficients, 0);
		else
			gradc = zeros(number_coefficients, 0);
			gradceq = options.decouplingcontrol.weight_decoupling*gradceq_raw;
		end
	end
end

function eigenvalue_map = track_eigenvalues(system, R, F, eigenvalues, eigenvector_right, eigenvector_left, number_models, number_states, number_controls, number_measurements, number_coefficients)
	%TRACK_EIGENVALUES returns the sorting indices that order the eigenvalues according to their proceeding in state space using their gradient
	%	Input:
	%		system:							structure with dynamic systems to take into consideration
	%		R:								proportional gain matrix
	%		F:								prefilter matrix
	%		eigenvalues:					array with eigenvalues of all controlled models
	%		eigenvector_right:				3D-array containing the right eigenvector matrix of each controlled dynamic system
	%		eigenvector_left:				3D-array containing the left eigenvector matrix of each controlled dynamic system
	%		number_models:					number of models
	%		number_states:					number of states
	%		number_controls:				number of controls
	%		number_measurements:			number of measurements
	%		number_coefficients:			number of optimization variables
	%	Output:
	%		eigenvalue_map:					3D-array that lists indices of tracked eigenvalues in second column. Third dimension for every system

	persistent xi_all xi_last xi_last_forwardstep ew_last ew_grad_last % optimization variables, eigenvalues and gradient from last iterations

	% determine if this iteration is a forward step
	isforwardstep = false;
	xi = reshape([R, F], number_coefficients, 1); % current optimization variables
	if isempty(xi_last)
		xi_last = xi;
		xi_last_forwardstep = xi;
		xi_all = xi;
	else
		% determination of isforwardstep is the reason for declaration as "beta".
		if all(xi_last == xi) % if FMINCON, KSOPT, IPOPT and xi ~= xi_last, then solver performs tactile step(?)
			isforwardstep = true;
		end
		xi_last = xi;
		xi_all = [xi_all, xi];
	end

	% track eigenvalues
	if isempty(ew_last)
		ew_last = real(eigenvalues) + 1j*imag(eigenvalues);
		ew_grad_last = zeros(number_states, number_controls*number_measurements, number_models) + 1j*zeros(number_states, number_controls*number_measurements, number_models);
		eigenvalue_map = (1:number_states).';
		eigenvalue_map = repmat(eigenvalue_map, 1, 2, number_models);
	else
		delta_xi_R = xi(1:number_controls*number_measurements) - xi_last_forwardstep(1:number_controls*number_measurements);
		ew_current_estimate = zeros(number_states, number_models) + 1j*zeros(number_states, number_models);
		eigenvalue_map = [
			(1:number_states).', int32(zeros(number_states, 1))
		];
		eigenvalue_map = repmat(eigenvalue_map, 1, 1, number_models);
		for ii = 1:number_models
			ew_current_estimate(:, ii) = ew_last(:, ii) + ew_grad_last(:, :, ii)*delta_xi_R;
			for jj = 1:number_states
				ew_diff = abs(ew_current_estimate(:, ii) - eigenvalues(jj, ii)); % calculate distance of an eigenvalue to all other eigenvalues
				[~, idx] = sort(ew_diff);
				eigenvalue_map(jj, 2, ii) = int32(idx(1)); % map this eigenvalue to the closest one
			end
			% minimize the overall distance if the obtained map is not unique
			set_diff = setdiff(1:number_states, sort(eigenvalue_map(:, 2, ii).'));
			if ~isempty(set_diff)
				[occurence_map, occurence_ind] = count_occurences(eigenvalue_map(:, 2, ii), (1:number_states).');
				indices_involved_in_duplicates = occurence_map(occurence_map(:,2) ~= 1, 1).';
				ref_indices_without_partner_all = zeros(number_states, number_states);
				for cnt = 1:number_states
					len = sum(~isnan(occurence_ind(cnt, :)));
					if len > 1
						ref_indices_without_partner_all(cnt, 1:len) = occurence_ind(cnt, 1:len);
					end
				end
				ref_indices_without_partner_with_zero = reshape(ref_indices_without_partner_all, 1, []);
				ref_indices_without_partner = ref_indices_without_partner_with_zero(ref_indices_without_partner_with_zero ~= 0);

				permuts = perms_codegen(indices_involved_in_duplicates);
				eigenvalues_sys = eigenvalues(:, ii);
				delta_ew = abs(repmat(ew_last(ref_indices_without_partner, ii).', size(permuts, 1), 1) - eigenvalues_sys(permuts));
				Jsum = sum(delta_ew, 2);
				[~, idxJ] = sort(Jsum);
				bestPermut = permuts(idxJ(1), :);
				eigenvalue_map(ref_indices_without_partner, 2, ii) = int32(bestPermut.');
			end
		end
	end

	% update persistent variables
	if isforwardstep
		ew_last = eigenvalues;
		xi_last_forwardstep = xi;
		for ii = 1:number_models
			E = system(ii).E;
			B = system(ii).B;
			C = system(ii).C;
			rev = eigenvector_right(:, :, ii);
			lev = eigenvector_left(:, :, ii)';
			for jj = 1:number_states
				for kk = 1:number_controls*number_measurements
					eiej = zeros(number_controls, number_measurements);
					eiej(kk) = 1;
					ew_grad_last(jj, kk, ii) = -lev(jj,:)*B*eiej*C*rev(:,jj)/(lev(jj,:)*E*rev(:,jj));
				end
			end
		end
	end
end

function [map, occ_indices] = count_occurences(vec, query)
	%COUNT_OCCURENCES counts occurences of numbers
	%	Input:
	%		vec:					vector with numbers that should be counted
	%		query:					numbers whose occurence is counted
	%	Output:
	%		map:					array that lists number of occurences of demanded numbers
	%		occ_indices:			matrix of indices where to find demanded numbers in vec filled with NaN
	no_query = length(query);
	map = zeros(no_query, 2);
	map(:, 1) = query;
	occ_indices = NaN(no_query, no_query);
	for ii = 1:no_query
		query_check = vec == query(ii);
		no_occur = sum(query_check);
		fnd = find(query_check);
		occ_indices(ii, 1:length(fnd)) = fnd;
		map(ii, 2) = no_occur;
	end
end

function plot_tracked_eigenvalues(number_states, eigenvalues) %#ok<DEFNU>
	%PLOT_TRACKED_EIGENVALUES plots proceeding of eigenvalues
	%	Input:
	%		number_states:			number of states
	%		eigenvalues:			vector of eigenvalues to be plotted
	persistent eigs f scat count
	if isempty(eigs)
		count = 1;
		eigs = eigenvalues;
		f = figure();
		scatter(real(eigs), imag(eigs), 'filled','k');
		hold on;
		grid on;
		scat = scatter(real(eigs), imag(eigs), 'xk');
	else
		eigs = [eigs, eigenvalues];
		count = count + 1;
	end
	figure(f);
	for ii = 1:number_states
		delete(scat)
		plot(real(eigs(ii, :)), imag(eigs(ii, :)),'k');
		scat = scatter(real(eigenvalues), imag(eigenvalues), 'xk');
		title(sprintf('count: %d', count));
	end
end

function [P] = perms_codegen(V)
	%PERMS_CODEGEN  All possible permutations.
	%   PERMS(1:N), or PERMS(V) where V is a vector of length N, creates a
	%   matrix with N! rows and N columns containing all possible
	%   permutations of the N elements.
	%
	%   This function is only practical for situations where N is less
	%   than about 10 (for N=11, the output takes over 3 gigabytes).
	%
	%   Class support for input V:
	%      float: double, single
	%      integer: uint8, int8, uint16, int16, uint32, int32, uint64, int64
	%      logical, char
	%
	%   See also NCHOOSEK, RANDPERM, PERMUTE.
	%
	%	This function is based on perms() and changed to make it codegen-ready.

	%   Copyright 1984-2015 The MathWorks, Inc.

	[~, maxsize] = computer;
	n = numel(V);
	% Error if output dimensions are too large
	if n*factorial(n) > maxsize
		error('MATLAB_pmaxsize')
	end

	V = V(:).'; % Make sure V is a row vector
	n = length(V);
	if n <= 1
		P = V;
	else
		P = permsr_codegen(n);
		if isequal(V, 1:n)
			P = cast(P, 'like', V);
		else
			P = V(P);
		end
	end
end

function [P_out] = permsr_codegen(n)
	% subfunction to help with recursion
	% This function is based on permsr() and changed to make it codegen-ready.
	P = NaN(factorial(n), n, n);
	P(1, 1, 1) = 1;

	rows = 1;
	cols = 1;

	for nn=2:n

		Psmall = P(1:rows, 1:cols, nn - 1);
		m = size(Psmall,1);
		P(1:nn*m, 1:nn, nn) = zeros(nn*m,nn);

		P(1:m, 1, nn) = nn;
		P(1:m, 2:nn, nn) = Psmall;

		for i = nn-1:-1:1
			reorder = [1:i-1, i+1:nn];
			% assign the next m rows in P.
			P((nn-i)*m+1:(nn-i+1)*m,1, nn) = i;
			P((nn-i)*m+1:(nn-i+1)*m,2:nn, nn) = reorder(Psmall);
		end
		rows = nn*m;
		cols = nn;
	end
	P_out = P(:, :, n);
end

function [idx_VR1, idx_WR2] = minimumnorm_sorting(R, f, B, C, Cjj, Djj, eigenvector_right, eigenvector_left, permutations_VR1, permutations_WR2, number_permutations, number_states, dim_invariant, hasfeedthrough)
	%MINIMUM-SORTING sorts eigenvalues by minimizing the norm of the output and input decoupling conditions
	%	Input:
	%		R:						proportional gain matrix
	%		f:						prefilter matrix
	%		B:						input matrix
	%		C:						measurement output matrix
	%		Cjj:					reference output matrix
	%		Djj:					reference feedthrough matrix
	%		eigenvector_right:		right eigenvectors of controlled dynamic system
	%		eigenvector_left:		left eigenvectors of controlled dynamic system
	%		permutations_VR1:		permutations of inidices possible for VR1
	%		permutations_WR2:		permutations of inidices possible for WR2
	%		number_permutations:	number of permutations for VR1 and WR2
	%		number_states:			number of states
	%		dim_invariant:			size of invariant subspace
	%		hasfeedthrough:			indicates if Djj is unequal to zero
	%	Output:
	%		idx_VR1:				vector of eigenvalue indices for VR1 that minimize norm of decoupling conditions
	%		idx_WR2:				vector of eigenvalue indices for WR2 that minimize norm of decoupling conditions
	if hasfeedthrough
		C2VR = decoupling_vectorTwoNorm((Cjj - Djj*R*C)*eigenvector_right, 1).';
	else
		C2VR = decoupling_vectorTwoNorm(Cjj*eigenvector_right, 1).';
	end
	WLBF = decoupling_vectorTwoNorm(eigenvector_left*B*f, 2);
	J = zeros(number_permutations, number_states);
	for jj = 1:number_permutations
		for kk = 1:dim_invariant
			J(jj, kk) =	C2VR(permutations_VR1(jj, kk));
		end
		for kk = 1:number_states - dim_invariant
			J(jj, dim_invariant + kk) =	WLBF(permutations_WR2(jj, kk));
		end
	end
	[~, min_idx] = min(sum(J, 2));
	idx_VR1 = sort(permutations_VR1(min_idx, :));
	idx_WR2 = sort(permutations_WR2(min_idx, :));
end

function [permutations_VR1, permutations_WR2, number_permutations] = set_permutations(number_states, dim_invariant, number_references, numthreads)
	%SET_PERMUTATIONS calculate permutations for eigenvector matrices
	%	Input:
	%		number_states:			number of states
	%		dim_invriant:			vector of invariant subspace sizes
	%		number_references:		number of references
	%		numthreads:				number of threads to use
	%	Output:
	%		permutations_VR1:		3D-array with permutations matrix for right eigenvectors. Third dimension used for different references
	%		permutations_WR2:		3D-array with permutations matrix for left eigenvectors. Third dimension used for different references
	%		number_permutations:	number of permutations for each reference
	%	HINT: this functionality is only moved to a separate function to prevent a runtime crash in freeing variables used in parfor in generated code for R2015B
	% prepare determination of VR1 and WR2
	maximum_permutations = nchoosek(number_states, floor(number_states/2));
	permutations_VR1 = NaN(maximum_permutations, number_states, number_references);
	permutations_WR2 = NaN(maximum_permutations, number_states, number_references);
	number_permutations = zeros(1, number_references);

	for ii = 1:number_references
		permutations_VR1_tmp = nchoosek(1:number_states, dim_invariant(ii));% outside of loop because m is constant for every model
		number_permutations(ii) = int32(size(permutations_VR1_tmp, 1));
		permutations_WR2_tmp = int32(zeros(number_permutations(ii), number_states - dim_invariant(ii), 'like', permutations_VR1_tmp));
		parfor (jj = 1:number_permutations(ii), numthreads)
			permutations_WR2_tmp(jj, :) = setdiff(1:number_states, permutations_VR1_tmp(jj, :));
		end
		permutations_VR1(1:number_permutations(ii), 1:dim_invariant(ii), ii) = permutations_VR1_tmp;
		permutations_WR2(1:number_permutations(ii), 1:(number_states - dim_invariant(ii)), ii) = permutations_WR2_tmp;
	end
end
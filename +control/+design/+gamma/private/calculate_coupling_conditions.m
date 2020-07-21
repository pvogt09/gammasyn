function [c, ceq, gradc, gradceq] = calculate_coupling_conditions(system, R, ~, F, dimensions, options, eigenvector_right, eigenvector_left, eigenvector_right_derivative, eigenvector_left_derivative)
	%CALCULATE_COUPLING_CONDITIONS calculate nonlinear output and input coupling conditions for fully numeric coupling controller design
	%	Input:
	%		system:							structure with dynamic systems to take into consideration
	%		R:								proportional gain matrix
	%		K:								derivative gain matrix
	%		F:								prefilter matrix
	%		dimensions:						structure with information about dimensions of the variables and systems
	%		options:						structure with options
	%		eigenvector_right:				3D-array containing the right eigenvector matrix of each controlled dynamic system
	%		eigenvector_left:				3D-array containing the left eigenvector matrix of each controlled dynamic system
	%		eigenvector_right_derivative:	5D-array containing the derivatives of the right eigenvector matrix of each controlled dynamic system with respect to every controller parameter
	%		eigenvector_left_derivative:	5D-array containing the derivatives of the left eigenvector matrix of each controlled dynamic system with respect to every controller parameter
	%	Output:
	%		c:								array of inequality constraints
	%		ceq:							array of equality constraints
	%		gradc:							array of gradients of inequality constraints
	%		gradceq:						array of gradients of equality constraints
	needsgradient = nargout >= 3;
	numthreads = options.numthreads;
	number_models = dimensions.models;
	number_states = dimensions.states;
	number_controls = dimensions.controls;
	number_measurements = dimensions.measurements;
	number_measurements_xdot = dimensions.measurements_xdot;
	number_references = dimensions.references;
	number_coefficients = number_controls*(number_measurements + number_measurements_xdot + number_references);
	number_couplingconditions = dimensions.couplingconditions;
	dim_invariant = dimensions.m_invariant;
	hasfeedthrough = dimensions.hasfeedthrough_coupling;
	n_xi = number_states*number_controls + number_controls*number_references;% number of controller and prefilter coefficients

	if nargin <= 8
		eigenvector_right_derivative = zeros(number_states, number_states, number_controls, number_states, number_models);
	end
	if nargin <= 9
		eigenvector_left_derivative = zeros(number_states, number_states, number_controls, number_states, number_models);
	end

	number_outputconditions_per_sys = 2*dim_invariant*number_couplingconditions;
	if hasfeedthrough
		number_inputconditions_per_sys = (number_controls - number_couplingconditions)*(2*(number_states - dim_invariant) + number_couplingconditions);
	else
		number_inputconditions_per_sys = (number_controls - number_couplingconditions)*2*(number_states - dim_invariant);
	end

	[permutations_VR1, permutations_WR2, number_permutations] = set_permutations(number_states, dim_invariant, numthreads);

	dF1dxi = zeros(number_controls, number_controls - number_couplingconditions, number_controls*(number_controls - number_couplingconditions));
	if needsgradient
		% loop code in combination with set_permutations as literal code in this function crashes R2015B at runtime in compiled code, therefore linear indexing is used
		%parfor (ii = 1:number_controls*(number_controls - number_couplingconditions), numthreads)
		%	dF1dxi_tmp = zeros(number_controls, number_controls - number_couplingconditions);
		%	dF1dxi_tmp(ii) = 1;
		%	dF1dxi(:, :, ii) = dF1dxi_tmp;
		%end
		indices = (1:number_controls*(number_controls - number_couplingconditions)) + number_controls*(number_controls - number_couplingconditions)*(-1 + (1:number_controls*(number_controls - number_couplingconditions)));
		dF1dxi(indices) = 1;
	end
	F1 = F(:, 1:end - number_couplingconditions);

	ceq_out = zeros(number_outputconditions_per_sys, 1, number_models);% output coupling conditions
	ceq_in = zeros(number_inputconditions_per_sys, 1, number_models);% input coupling conditions
	if needsgradient
		ceq_out_grad = zeros(n_xi, number_outputconditions_per_sys, number_models);% gradients of output coupling conditions
		ceq_in_grad  = zeros(n_xi, number_inputconditions_per_sys, number_models);% gradients of input coupling conditions
	end

	% TODO: choice of VR1 and WR2 still problematic. Can be seen if system with feedthrough is used and number_states == dim_invariant. Then VR1 = V and WR2 = []. So no choice has to be done. This works well.
	parfor (ii = 1:number_models, numthreads)
		% system variables
		C2 = system(ii).C_ref(end - number_couplingconditions + 1:end, :);
		D2 = system(ii).D_ref(end - number_couplingconditions + 1:end, :);
		B = system(ii).B;
		eigenvector_r = eigenvector_right(:, :, ii);
		eigenvector_l = eigenvector_left(:, :, ii)';% ev_l*ev_r = eye(n)
		% system variables end

		% determine VR1 and WR2
		if hasfeedthrough
			C2VR = coupling_vectorTwoNorm((C2 - D2*R)*eigenvector_r, 1).';
		else
			C2VR = coupling_vectorTwoNorm(C2*eigenvector_r, 1).';
		end
		WLBF = coupling_vectorTwoNorm(eigenvector_l*B*F1, 2);
		J = zeros(number_permutations, number_states);
		for jj = 1:number_permutations
			for kk = 1:dim_invariant
				J(jj, kk) =	C2VR(permutations_VR1(jj, kk)); %#ok<PFBNS>
			end
			for kk = 1:number_states - dim_invariant
				J(jj, dim_invariant + kk) =	WLBF(permutations_WR2(jj, kk)); %#ok<PFBNS>
			end
		end
		[~, min_idx] = min(sum(J, 2));
		idx_VR1 = sort(permutations_VR1(min_idx, :));
		idx_WR2 = sort(permutations_WR2(min_idx, :));
		VR1 = eigenvector_r(:, idx_VR1);
		WR2 = eigenvector_l(idx_WR2, :);
		% determine VR1 and WR2 end

		% input and output coupling conditions of the system
		WBF = WR2*B*F1;
		if hasfeedthrough
			CVR = (C2 - D2*R)*VR1;
		else
			CVR = C2*VR1;
		end
		ceq_out(:, :, ii) = [
			reshape(real(CVR), number_couplingconditions*dim_invariant, 1);
			reshape(imag(CVR), number_couplingconditions*dim_invariant, 1)
		];
		if hasfeedthrough
			ceq_in(:, :, ii) = [
				reshape(real(WBF.'),	(number_states - dim_invariant)*(number_controls - number_couplingconditions), 1);
				reshape(imag(WBF.'),	(number_states - dim_invariant)*(number_controls - number_couplingconditions), 1);
				reshape(D2*F1,			(number_couplingconditions)*(number_controls - number_couplingconditions), 1)
			];
		else
			ceq_in(:, :, ii) = [
				reshape(real(WBF.'),	(number_states - dim_invariant)*(number_controls - number_couplingconditions), 1);
				reshape(imag(WBF.'),	(number_states - dim_invariant)*(number_controls - number_couplingconditions), 1);
			];
		end
		% input and output coupling conditions of the system end

		if needsgradient
			eigenvector_l_derv_sys = eigenvector_left_derivative(:, :, :, :, double(ii)); %#ok<PFBNS> % ii must not be int32, otherwise parfor runtime error
			eigenvector_r_derv_sys = eigenvector_right_derivative(:, :, :, :, double(ii)); %#ok<PFBNS>
			% gradients of output coupling conditions
			if dim_invariant > 0
				all_grads_rev_mixed = [
					reshape(eigenvector_r_derv_sys(:, idx_VR1, :, :), number_states, number_states*number_controls*dim_invariant), zeros(number_states, number_controls*number_references*dim_invariant)
				];
				all_grads_rev_split = reshape(all_grads_rev_mixed, number_states, dim_invariant, n_xi);
				all_grads_rev_stacked_below = reshape(all_grads_rev_split, number_states*dim_invariant, n_xi);
				if hasfeedthrough
					DdR = zeros(number_controls*dim_invariant, n_xi) + 1i*zeros(number_controls*dim_invariant, n_xi);
					for jj = 1:number_controls*number_states
						matrix = zeros(number_controls, number_states);
						matrix(jj) = 1;
						DdR(:, jj) = reshape(matrix*VR1, number_controls*dim_invariant, 1);
					end
					all_grads_coupl_out = kron(eye(dim_invariant), C2 - D2*R)*all_grads_rev_stacked_below - kron(eye(dim_invariant), D2)*DdR; % roughly factor 5 faster than for
				else
					all_grads_coupl_out = kron(eye(dim_invariant), C2)*all_grads_rev_stacked_below;
				end
			else
				all_grads_coupl_out = zeros(n_xi, 0);
			end

			ceq_out_grad(:, :, ii) = [
				real(all_grads_coupl_out).', imag(all_grads_coupl_out).'
			];
			% gradients of output coupling conditions end

			% gradients of input coupling conditions
			if number_states > dim_invariant
				all_grads_lev_mixed = reshape(eigenvector_l_derv_sys(:, idx_WR2, :, :), number_states, number_states*number_controls*(number_states - dim_invariant));
				all_grads_lev_split = reshape(all_grads_lev_mixed, number_states, number_states - dim_invariant, number_states*number_controls);
				all_grads_lev_stacked_below = reshape(all_grads_lev_split, number_states*(number_states - dim_invariant), number_states*number_controls);
				all_grads_coupl_in_first_part = (kron(eye(number_states - dim_invariant), F1.'*B.')*conj(all_grads_lev_stacked_below)).';
				all_grads_coupl_in_secnd_part = NaN((number_controls - number_couplingconditions)*(number_states - dim_invariant), number_controls*(number_controls - number_couplingconditions)) + 1i*NaN((number_controls - number_couplingconditions)*(number_states - dim_invariant), number_controls*(number_controls - number_couplingconditions));
				for kk = 1:number_controls*(number_controls - number_couplingconditions)
					all_grads_coupl_in_secnd_part(:, kk) = reshape(dF1dxi(:, :, kk).'*B.'*WR2.', (number_controls - number_couplingconditions)*(number_states - dim_invariant), 1); %#ok<PFBNS> dF1dxi is constant and calculated before loop
				end
				all_grads_coupl_in = [
					all_grads_coupl_in_first_part;
					all_grads_coupl_in_secnd_part.';
					zeros(number_controls*number_couplingconditions, (number_controls - number_couplingconditions)*(number_states - dim_invariant))
				];
			else
				all_grads_coupl_in = zeros(n_xi, 0);
			end
			if hasfeedthrough
				ceq_in_grad(:, :, ii) = [
					real(all_grads_coupl_in), imag(all_grads_coupl_in), [
						zeros(number_couplingconditions*(number_controls - number_couplingconditions), number_controls*number_states), kron(eye(number_controls - number_couplingconditions), D2), zeros(number_couplingconditions*(number_controls - number_couplingconditions), number_controls*number_couplingconditions)
					].'
				];
			else
				ceq_in_grad(:, :, ii) = [
					real(all_grads_coupl_in), imag(all_grads_coupl_in)
				];
			end
			% gradients of input coupling conditions end
		end
	end % end FOR each system

	ceq_raw = [
		reshape(ceq_out, number_outputconditions_per_sys*number_models, 1);
		reshape(ceq_in, number_inputconditions_per_sys*number_models, 1)
	];
	% prefilter F1 should not be trivial
	normF1 = options.couplingcontrol.tolerance_prefilter;% parameter to avoid trivial solution for prefilter
	c_F1 = -trace(F1.'*F1) + normF1;% has no complex values and no complex gradient
	if options.couplingcontrol.couplingstrategy == GammaCouplingStrategy.NUMERIC_NONLINEAR_EQUALITY
		c = c_F1;
		ceq = ceq_raw;
	elseif options.couplingcontrol.couplingstrategy == GammaCouplingStrategy.NUMERIC_NONLINEAR_INEQUALITY
		tolerance_vec = options.couplingcontrol.tolerance_coupling*ones(size(ceq_raw, 1), 1);
		c = [
			ceq_raw - tolerance_vec;
			-ceq_raw - tolerance_vec;
			c_F1
		];
		ceq = zeros(0, 1);
	else
		c = zeros(0, 1);
		ceq = ceq_raw;
	end
	if needsgradient
		ceq_grad_pre = [
			reshape(ceq_out_grad, n_xi, number_outputconditions_per_sys*number_models), reshape(ceq_in_grad, n_xi, number_inputconditions_per_sys*number_models)
		];
		ceq_grad_r = ceq_grad_pre(1:number_states*number_controls, :);
		ceq_grad_f = ceq_grad_pre(number_states*number_controls + 1:end, :);
		if dimensions.K_fixed_has
			ceq_grad_r_T = ((ceq_grad_r.')*dimensions.K_fixed_T_inv).';
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

		grad_F_pre = [
			reshape(-2*F1, number_controls*(number_controls - number_couplingconditions), 1);
			zeros(number_controls*number_couplingconditions, 1)
		];
		if dimensions.F_fixed_has
			grad_F_T = ((grad_F_pre.')*dimensions.F_fixed_T_inv).';
		else
			grad_F_T = grad_F_pre;
		end
		grad_F1 =	[
			zeros(number_controls*number_states, 1);
			zeros(number_controls*number_measurements_xdot, 1);
			grad_F_T
		];
		if options.couplingcontrol.couplingstrategy == GammaCouplingStrategy.NUMERIC_NONLINEAR_EQUALITY
			gradc = grad_F1;
			gradceq = gradceq_raw;
		elseif options.couplingcontrol.couplingstrategy == GammaCouplingStrategy.NUMERIC_NONLINEAR_INEQUALITY
			gradc = [
				gradceq_raw, -gradceq_raw, grad_F1
			];
			gradceq = zeros(number_coefficients, 0);
		else
			gradc = zeros(number_coefficients, 0);
			gradceq = gradceq_raw;
		end
	end
end

function [permutations_VR1, permutations_WR2, number_permutations] = set_permutations(number_states, dim_invariant, numthreads)
	%SET_PERMUTATIONS calculate permutations for eigenvector matrices
	%	Input:
	%		number_states:			number of states
	%		dim_invriant:			size of invariant subspace
	%		numthreads:				number of threads to use
	%	Output:
	%		permutations:VR1:		permutations matrix for right eigenvectors
	%		permutations_WR2:		permutation matrix for left eigenvectors
	%		number_permutations:	number of permutations
	%	HINT: this functionality is only moved to a separate function to prevent a runtime crash in freeing variables used in parfor in generated code for R2015B
	% prepare determination of VR1 and WR2
	permutations_VR1 = nchoosek(1:number_states, dim_invariant);% outside of loop because m is constant for every model
	number_permutations = size(permutations_VR1, 1);
	permutations_WR2_tmp = NaN(number_permutations, number_states - dim_invariant);
	parfor (ii = 1:number_permutations, numthreads)
		permutations_WR2_tmp(ii, :) = setdiff(1:number_states, permutations_VR1(ii, :));
	end
	permutations_WR2 = permutations_WR2_tmp;
end
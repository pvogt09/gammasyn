function [RKF_fixed, RKF_bounds, valid, message] = decoupling_RKF_fixed(systems, objectiveoptions, solveroptions, descriptor)
	%DECOUPLING_RKF_FIXED calculates structural constraints for proportional controller and prefilter coefficients, that are necessary for decoupling control
	%	Input:
	%		systems:					structure with dynamic systems to take into consideration
	%		objectiveoptions:			structure with objective options
	%		solveroptions:				options for optimization algorithm to use
	%		descriptor:					indicator if design should be performed for a DAE system
	%	Output:
	%		RKF_fixed:					cell array of three cell arrays providing the structural constraints of proportional, derivative and prefilter gain compatible with gammasyn
	%		RKF_bounds:					cell array of three cell arrays providing the structural inequality constraints of proportional, derivative and prefilter gain compatible with gammasyn
	%		valid:						indicator of failure
	%		message:					failure message
	if nargin <= 3
		descriptor = false;
	end
	RKF_fixed = [];
	RKF_bounds = [];
	R_bounds = {[], []};
	F_bounds = {[], []};
	valid = true;
	message = '';
	hassymbolic = configuration.matlab.hassymbolictoolbox();
	solvesymbolic = objectiveoptions.decouplingcontrol.solvesymbolic;
	round_to = objectiveoptions.decouplingcontrol.round_equations_to_digits; % parameter for rounding equation systems in case no solution of normal equations (which must have a solution) is found
	control_design_type = objectiveoptions.decouplingcontrol.decouplingstrategy;
	allowoutputdecoupling = objectiveoptions.decouplingcontrol.allowoutputdecoupling;
	tf_structure = objectiveoptions.decouplingcontrol.tf_structure;
	number_states = size(systems(1).A, 1);
	number_controls = size(systems(1).B, 2);
	number_references = size(systems(1).C_ref, 1);
	number_measurements = size(systems(1).C, 1);
	number_models = size(systems, 1);

	if solvesymbolic && ~hassymbolic
		warning('control:design:gamma:decoupling', 'Constraints shall be calculated symbolically, but Symbolic Toolbox was not found.');
		solvesymbolic = false;
	end
	if ~isa(control_design_type, 'GammaDecouplingStrategy')
		error('control:design:gamma:input', 'Decoupling controller design type must be of type ''GammaDecouplingStrategy''.');
	end
	if control_design_type == GammaDecouplingStrategy.EXACT
		allowApproxSol = false;
	elseif any(control_design_type == [
		GammaDecouplingStrategy.APPROXIMATE;
		GammaDecouplingStrategy.APPROXIMATE_INEQUALITY
	])
		allowApproxSol = true;
	else
		error('control:design:gamma:input', 'Wrong GammaDecouplingStrategy.');
	end
	if ~allowoutputdecoupling
		if number_states ~= number_measurements
			error('control:design:gamma:decoupling', 'Number of states (%d) must match number of measurements (%d) for decoupling control.', number_states, number_measurements);
		end
	end

	%% Constraint equations for controller and prefilter
	%  Equations have the form X_R*r = z_R and X_F*f = z_F
	dim_F_1 = number_controls;		% number of rows of prefilter F1
	dim_F_2 = number_references;	% number of columns of prefilter F1

	if solvesymbolic
		R = sym('r%d%d', [number_controls, number_measurements]);
		assume(R, 'real');
		r = reshape(R, number_controls*number_measurements, 1);
		F = sym('f%d%d', [dim_F_1, dim_F_2]);
		assume(F, 'real');
		f = reshape(F, dim_F_1*dim_F_2, 1);
	else
		R = [];
		r = [];
		f = [];
	end

	Q_orth_T_B_cell = cell(number_models, 1);
	Q_orth_jj_T_B_cell = cell(number_models, number_references);
	Q_orth_jj_T_B_cell(:) = {zeros(0, number_controls)};
	X_R_cell = cell(number_models, number_references);
	X_R_cell(:) = {zeros(0, number_controls*number_measurements)};
	z_R_cell = cell(number_models, number_references);
	dim_invariant_mat = zeros(number_models, number_references);
	sys_feedthrough_mat = false(number_models, number_references);
	not_con_invariant_mat = false(number_models, number_references);
	for ii = 1:number_models
		A  = systems(ii).A;
		B  = systems(ii).B;
		C  = systems(ii).C;
		C_ref = systems(ii).C_ref;
		D_ref = systems(ii).D_ref;
		kerC = null(C);
		for jj = 1:number_references
			g_structure = tf_structure(:, jj);
			Cjj = C_ref(g_structure == 0, :);
			Djj = D_ref(g_structure == 0, :);
			number_decouplingconditions_jj = size(Cjj, 1);
			if isempty(Cjj) || isempty(Djj) % no zeros in this column of the transfer matrix
				continue;
			end
			if any(Djj(:) ~= 0) % this decoupling output has a feedthrough
				sys_feedthrough_mat(ii, jj) = true;
			end
			E  = systems(ii).E;
			Q = []; %#ok<NASGU> needed for parfor

			if descriptor
				if rank(E) ~= number_states
					error('control:design:gamma:input', 'Matrix E must be invertible for decoupling controller design.');
				else
					if sys_feedthrough_mat(ii, jj)
						Q = vstar(E\A, E\B, Cjj, Djj);
					else
						Q = mainco(E\A, E\B, null(Cjj));
					end
				end
			else
				if sys_feedthrough_mat(ii, jj)
					Q = vstar(A, B, Cjj, Djj); % output nulling controlled invariant subspace. If Djj == 0, same as mainco(A, B, null(Cjj))
				else
					Q = mainco(A, B, null(Cjj)); % distinguish to improve numerical behavior
				end
			end
			if all(all(Q(:) == 0))
				Q = zeros(number_states, 0);
			end
			% check if output nulling controlled invariant subspace is also input containing conditioned invariant
			if size(kerC, 2) > 0 % only in this case, Q might not be conditioned invariant
				if rank([A*ints(Q, kerC), Q]) > rank(Q) % not the right condition for systems with feedthrough
					% controlled invariant subspace is not conditioned invariant
					not_con_invariant_mat(ii, jj) = true;
				end
			end
			m = size(Q, 2);
			dim_invariant_mat(ii, jj) = m;
			Q_orth = null(Q.');
			V = [
				Q, Q_orth
			];
			if sys_feedthrough_mat(ii, jj) % if Djj ~= 0: additional equations Djj*F(:, jj) = 0
				Q_orth_jj_T_B_cell{ii, jj} = [
					Q_orth'*B;
					Djj
				];
			else
				Q_orth_jj_T_B_cell{ii, jj} = Q_orth'*B;
			end
			if solvesymbolic
				ArT = V'*(A - B*R*C)*V; % V' == inv(V) because V is built as orthogonal matrix
				A21 = ArT(m + 1:end, 1:m);
				[X_calC_sym, z_calC_sym] = equationsToMatrix(A21 == 0, r);
				X_calC = double(X_calC_sym);
				z_calC = double(z_calC_sym);
			else
				X_calC = kron(Q'*C', Q_orth'*B);
				z_calC = reshape(Q_orth'*A*Q, (number_states - m)*m, 1);
			end
			if sys_feedthrough_mat(ii, jj) % if Djj == 0: kron(Q'*C', Djj) = 0 and Cjj*Q = 0 --> no new equations
				X = [
					X_calC;
					kron(Q'*C', Djj)
				];
				z = [
					z_calC;
					reshape(Cjj*Q, number_decouplingconditions_jj*m, 1)
				];
			else
				X = X_calC;
				z = z_calC;
			end
			X_R_cell{ii, jj} = X;
			z_R_cell{ii, jj} = z;
		end
		Q_orth_T_B_cell{ii, 1} = blkdiag(Q_orth_jj_T_B_cell{ii, :});
	end

	if any(any(sys_feedthrough_mat)) && allowoutputdecoupling && number_measurements < number_states
		warning('control:design:gamma:decoupling', 'Using feedthrough decoupling and outputfeedback is "beta" since in this case, controlled invariant subspace is not properly checked for being conditioned invariant.');
	end
	not_con_invariant = any(any(not_con_invariant_mat));
	if not_con_invariant
		warning('control:design:gamma:decoupling', 'No suitable output feedback exists because at least one controlled invariant subspace is not conditioned invariant. Use different measurement configuration.');
	end

	X_R = cat(1, X_R_cell{:});
	z_R = cat(1, z_R_cell{:});
	X_F = cat(1, Q_orth_T_B_cell{:});
	z_F = zeros(size(X_F, 1), 1);

	X_R(abs(X_R) < eps) = 0; % avoid basic numerical difficulties
	z_R(abs(z_R) < eps) = 0;
	X_F(abs(X_F) < eps) = 0;


	%% Choose constrained parameters
	if any(any((dim_invariant_mat == number_states) & ~sys_feedthrough_mat))
		error('RBABS:control:design:gamma:dimensions', 'Dimension m of controlled invariant subspace is equal to the system dimension %d, while there is no feedthrough in the decoupling condition. Specify non-trivial decoupling conditions.', number_states)
	end
	if control_design_type == GammaDecouplingStrategy.APPROXIMATE_INEQUALITY
		R_bounds = convert_vectorized2hadamard([
			X_R;
			-X_R
		], [
			z_R + objectiveoptions.decouplingcontrol.tolerance_decoupling;
			-z_R - objectiveoptions.decouplingcontrol.tolerance_decoupling
		], [number_controls, number_measurements]);
		R_fixed = {[], []};
	else
		if ~isempty(X_R)
			if solvesymbolic
				r_sol = solve(X_R*r == z_R, r, 'ReturnConditions', true);
				r_sol_fields = struct2cell(r_sol);
				r_sol_fields = r_sol_fields(1:number_measurements*number_controls);
				r_sol_empty = cellfun(@isempty, r_sol_fields, 'UniformOutput', true);
				r_sol_empty = any(r_sol_empty(:));

				if r_sol_empty
					if output_verbosity(solveroptions, 'notify')
						disp('------------X_R*r=z_R has no solution.------------------');
					end
					if allowApproxSol
						if output_verbosity(solveroptions)
							disp('------------Calculate approximate solution.-----------');
						end
						r_sol = solve(X_R.'*X_R*r == X_R.'*z_R, r, 'ReturnConditions', true);
						r_sol_fields = struct2cell(r_sol);
						r_sol_fields = r_sol_fields(1:number_measurements*number_controls);
						r_sol_empty_approx = cellfun(@isempty, r_sol_fields, 'UniformOutput', true);
						r_sol_empty_approx = any(r_sol_empty_approx(:));
						if r_sol_empty_approx
							if isnan(round_to)
								r_sol = solve(X_R.'*X_R*r == X_R.'*z_R, r, 'ReturnConditions', true);
							else
								r_sol = solve(round(X_R.'*X_R, round_to)*r == round(X_R.'*z_R, round_to), r, 'ReturnConditions', true);
							end
							r_sol_fields = struct2cell(r_sol);
							r_sol_fields = r_sol_fields(1:number_measurements*number_controls);
							r_sol_empty_approx_round = cellfun(@isempty, r_sol_fields, 'UniformOutput', true);
							r_sol_empty_approx_round = any(r_sol_empty_approx_round(:));
							if r_sol_empty_approx_round
								error('control:design:gamma:decoupling', 'Calculation of controller constraints failed due to numerical difficulties.');
							end
						end
					else
						message = 'X_R*r=z_R has no solution. Allow calculation of approximate solution, using GammaDecouplingStrategy.APPROXIMATE.';
						valid = false;
						return;
					end
				end
				r_sol_array = [r_sol_fields{:}].';
				free_params = r_sol.parameters;
				[~, idx_intersect_r] = intersect(r_sol_array, free_params);
				idx_constr_params_r = setdiff(1:number_measurements*number_controls, idx_intersect_r);
				no_constr_params_r = length(idx_constr_params_r);

				R = reshape(r_sol_array(1:number_measurements*number_controls), number_controls, number_measurements);
				if ~isempty(free_params)
					z_rij = free_params.';
					z_rij = [
						z_rij, r(idx_intersect_r)
					];
					free_params_r = z_rij(:, 2).';
					if size(free_params, 1) ~= size(free_params_r, 1)
						free_params_r = transpose(free_params_r);
					end
					R = subs(R, free_params, free_params_r);
				else
					if output_verbosity(solveroptions)
						disp('Only one controller can fulfill the decoupling conditions.');
					end
				end

				% Structural constraints of controller for gammasyn
				R_fixed_A = zeros(number_controls, number_measurements, no_constr_params_r);
				R_fixed_B = zeros(1, 1, no_constr_params_r);
				parfor cnt = 1:no_constr_params_r
					ii = idx_constr_params_r(cnt);
					[eqA, eqB] = equationsToMatrix(r(ii) == R(ii), r); %#ok<PFBNS>
					R_fixed_A(:, :, cnt) = double(reshape(eqA, number_controls, number_measurements));
					R_fixed_B(:, :, cnt) = double(eqB);
				end
				R_fixed_B = reshape(R_fixed_B, no_constr_params_r, 1);
				R_fixed = {
					R_fixed_A, R_fixed_B
				};

				if output_verbosity(solveroptions)
					disp('------------The structure of the controller is:-------');
					fprintf('\n');
					disp(vpa(R, 4));
				end
			else
				equation_system_aug = [
					X_R, z_R
				];
				equation_system_reduced = rref(equation_system_aug);
				equation_system_reduced(all(equation_system_reduced == 0, 2), :) = [];
				allzero = all(equation_system_reduced(:, 1:end - 1) == 0, 2);
				if any(allzero & equation_system_reduced(:, end) == 1)
					% try solution with parametric solution returned by \
					if rank(X_R) == rank(equation_system_aug)% TODO: use tolerance to allow for solution?
						% TODO: No, but round equation systems if approximate system also does not have solution?
						% TODO: another possibility would be to define a tube of inequalities around the equation system and solve with linprog
						r_0 = X_R\z_R;
						r_0_ker = null(X_R, 'r');
						A = null(r_0_ker.', 'r').';
						equation_system_reduced = [
							A,	A*r_0
						];
					else
						if output_verbosity(solveroptions, 'notify')
							disp('------------X_R*r=z_R has no solution.------------------');
						end
						if allowApproxSol
							if output_verbosity(solveroptions)
								disp('------------Calculate approximate solution.-----------');
							end
							equation_system_aug = [
								X_R.'*X_R, X_R.'*z_R
							];
							equation_system_reduced = rref(equation_system_aug);
							equation_system_reduced(all(equation_system_reduced == 0, 2), :) = [];
						else
							message = 'X_R*r=z_R has no solution. Allow calculation of approximate solution, using GammaDecouplingStrategy.APPROXIMATE.';
							valid = false;
							return;
						end
					end
				end
				no_constr_params_r = size(equation_system_reduced, 1);
				R_fixed_A = zeros(number_controls, number_measurements, no_constr_params_r);
				R_fixed_B = zeros(1, 1, no_constr_params_r);
				parfor cnt = 1:no_constr_params_r
					equation_system_reduced_cnt = equation_system_reduced(cnt, :);
					R_fixed_A(:, :, cnt) = double(reshape(equation_system_reduced_cnt(1, 1:end - 1), number_controls, number_measurements));
					R_fixed_B(:, :, cnt) = double(equation_system_reduced_cnt(1, end));
				end
				R_fixed_B = reshape(R_fixed_B, no_constr_params_r, 1);
				R_fixed = {
					R_fixed_A, R_fixed_B
				};
			end
		else
			R_fixed = {[], []};
			disp('------------There are no controller constraints.------');
		end
	end
	%% Prefilter
	c_f = objectiveoptions.decouplingcontrol.tolerance_prefilter;% parameter to avoid trivial solution for prefilter
	if control_design_type == GammaDecouplingStrategy.APPROXIMATE_INEQUALITY
		F_bounds = convert_vectorized2hadamard([
			X_F;
			-X_F
		], [
			z_F + objectiveoptions.decouplingcontrol.tolerance_prefilter;
			-z_F - objectiveoptions.decouplingcontrol.tolerance_prefilter
		], [number_controls, dim_F_2]);
		F_fixed_A = [];
		F_fixed_B = [];
	else
		if ~isempty(X_F)
			if solvesymbolic
				f_sol_zero = NaN(1, dim_F_2);
				already_displayed = false;
				prefilter_sym = false;
				idx_constr_params_f_cell = cell(1, dim_F_2);
				for ii = 1:dim_F_2
					X_F_temp = X_F(:, (ii - 1)*dim_F_1 + (1:dim_F_1));
					if all(X_F_temp(:) == 0)
						prefilter_sym = true;
						f_sol_zero(ii) = false;
						continue;
					end
					% Kann man das hier abbrechen, falls X_F_temp == 0 ist?
					f_temp = f((ii - 1)*dim_F_1 + (1:dim_F_1));
					f_sol = solve(X_F_temp*f_temp == z_F, f_temp, 'ReturnConditions', true);
					f_sol_fields = struct2cell(f_sol);
					f_sol_fields = f_sol_fields(1:dim_F_1);
					f_sol_zero(ii) = all(logical([f_sol_fields{:}] == 0));

					if f_sol_zero(ii)
						if output_verbosity(solveroptions, 'notify') && ~already_displayed
							disp('------------There is no non-zero prefilter.-----------');
						end
						if allowApproxSol
							if output_verbosity(solveroptions, 'notify') && ~already_displayed
								already_displayed = true;
								disp('------------Calculate approximate solution.-----------');
							end
							X_F_tilde = [
								X_F_temp;
								ones(1, dim_F_1)
							];
							z_F_tilde = [
								z_F;
								c_f
							];
							f_sol = solve(X_F_tilde.'*X_F_tilde*f_temp == X_F_tilde.'*z_F_tilde, f_temp, 'ReturnConditions', true);
							f_sol_fields = struct2cell(f_sol);
							f_sol_fields = f_sol_fields(1:dim_F_1);
							f_sol_empty_approx = cellfun(@isempty, f_sol_fields, 'UniformOutput', true);
							f_sol_empty_approx = any(f_sol_empty_approx(:));
							if f_sol_empty_approx
								if isnan(round_to)
									f_sol = solve(X_F_tilde.'*X_F_tilde*f_temp == X_F_tilde.'*z_F_tilde, f_temp, 'ReturnConditions', true);
								else
									f_sol = solve(round(X_F_tilde.'*X_F_tilde, round_to)*f_temp == round(X_F_tilde.'*z_F_tilde, round_to), f_temp, 'ReturnConditions', true);
								end
								f_sol_fields = struct2cell(f_sol);
								f_sol_fields = f_sol_fields(1:dim_F_1);
								f_sol_empty_approx_round = cellfun(@isempty, f_sol_fields, 'UniformOutput', true);
								f_sol_empty_approx_round = any(f_sol_empty_approx_round(:));
								if f_sol_empty_approx_round
									error('control:design:gamma:decoupling', 'Calculation of prefilter constraints failed due to numerical difficulties.');
								end
							end
						else
							message = 'There is no non-zero prefilter. Allow calculation of approximate solution, using GammaDecouplingStrategy.APPROXIMATE.';
							valid = false;
							return;
						end
					end
					f_sol_array = cat(1, f_sol_fields{:});
					free_params_zf = f_sol.parameters;
					[~, idx_intersect_f] = intersect(f_sol_array, free_params_zf);
					idx_constr_params_f_cell{ii} = setdiff(1:dim_F_1, idx_intersect_f) + (ii - 1)*dim_F_1;

					F(:, ii) = f_sol_array;

					if ~isempty(free_params_zf)
						f_intersect = f_temp(idx_intersect_f);
						if size(f_intersect, 1) ~= size(free_params_zf, 1)
							f_intersect = transpose(f_intersect);
						end
						F = subs(F, free_params_zf, f_intersect);
						prefilter_sym = true;
					end
				end
				idx_constr_params_f = [idx_constr_params_f_cell{:}].';
				no_constr_params_f = length(idx_constr_params_f);

				% Structural constraints of prefilter for gammasyn
				if ~all(f_sol_zero)
					cols_force_zeros = find(any(tf_structure == 0, 1) & ~f_sol_zero); % those columns are not ensured to be non-zero yet
					% TODO: in the line above, either check every element of tf_structure or only allow one element in tf_structure
					num_cols_force_zeros = length(cols_force_zeros);
					size_F_fixed = no_constr_params_f + num_cols_force_zeros;
					F_fixed_A = zeros(dim_F_1, dim_F_2, size_F_fixed);
					F_fixed_B = zeros(1, 1, size_F_fixed);
					for ii = 1:num_cols_force_zeros
						F_fixed_A_tmp = zeros(dim_F_1, dim_F_2);
						F_fixed_A_tmp(:, cols_force_zeros(ii)) = ones(dim_F_1, 1);
						F_fixed_A(:, :, size_F_fixed - ii + 1) = F_fixed_A_tmp;
						F_fixed_B(:, :, size_F_fixed - ii + 1) = c_f;
					end
				else
					size_F_fixed = no_constr_params_f;
					F_fixed_A = zeros(dim_F_1, dim_F_2, no_constr_params_f);
					F_fixed_B = zeros(1, 1, no_constr_params_f);
				end
				for cnt = 1:no_constr_params_f %#ok<FORPF> no parfor because of non linear indexing
					ii = idx_constr_params_f(cnt);
					[eqA, eqB] = equationsToMatrix(f(ii) == F(ii), f);
					F_fixed_A(:, :, cnt) = double(reshape(eqA, dim_F_1, dim_F_2));
					F_fixed_B(:, :, cnt) = double(eqB);
				end
				F_fixed_B = reshape(F_fixed_B, size_F_fixed, 1);
			else
				prefilter_sym = true;
				equation_system_aug = [
					X_F, z_F
				];
				equation_system_reduced = rref(equation_system_aug);
				equation_system_reduced(all(equation_system_reduced == 0, 2), :) = [];
				allzero = all(equation_system_reduced(:, 1:end - 1) == 0, 2);
				if any(allzero & equation_system_reduced(:, end) == 1)
					% try solution with parametric solution returned by \
					if rank(X_F) == rank(equation_system_aug)% TODO: use tolerance to allow for solution?
						% TODO: No, but round equation systems if approximate system also does not have solution?
						% TODO: another possibility would be to define a tube of inequalities around the equation system and solve with linprog
						f_0 = X_F\z_F;
						f_0_ker = null(X_F, 'r');
						A = null(f_0_ker.', 'r').';
						equation_system_reduced = [
							A,	A*f_0
						];
					else
						if output_verbosity(solveroptions, 'notify')
							disp('------------There is no non-zero prefilter.-----------');
						end
						if allowApproxSol
							if output_verbosity(solveroptions)
								disp('------------Calculate approximate solution.-----------');
							end
							X_F_tilde = [
								X_F;
								ones(1, size(X_F, 2))
							];
							z_F_tilde = [
								z_F;
								c_f
							];
							equation_system_aug = [
								X_F_tilde.'*X_F_tilde, X_F_tilde.'*z_F_tilde
							];
							equation_system_reduced = rref(equation_system_aug);
							equation_system_reduced(all(equation_system_reduced == 0, 2), :) = [];
						else
							message = 'There is no non-zero prefilter. Allow calculation of approximate solution, using GammaDecouplingStrategy.APPROXIMATE.';
							valid = false;
							return;
						end
					end
					cols_force_zeros = find(any(G_structure == 0, 1)); % TODO: either check every element of tf_structure or only allow one element in tf_structure
					num_cols_force_zeros = length(cols_force_zeros);
					size_F_fixed = size(equation_system_reduced, 1) + num_cols_force_zeros;
					F_fixed_A = NaN(dim_F_1, dim_F_2, size_F_fixed);
					F_fixed_B = NaN(1, 1, size_F_fixed);
					for ii = 1:num_cols_force_zeros
						F_fixed_A_tmp = zeros(dim_F_1, dim_F_2);
						F_fixed_A_tmp(:, cols_force_zeros(ii)) = ones(dim_F_1, 1);
						F_fixed_A(:, :, size_F_fixed - ii + 1) = F_fixed_A_tmp;
						F_fixed_B(:, :, size_F_fixed - ii + 1) = c_f;
					end
					F_fixed_B(:, :, size_F_fixed) = c_f;
				else
					size_F_fixed = size(equation_system_reduced, 1);
					F_fixed_A = NaN(dim_F_1, dim_F_2, size(equation_system_reduced, 1));
					F_fixed_B = NaN(1, 1, size(equation_system_reduced, 1));
				end
				parfor cnt = 1:size(equation_system_reduced, 1)
					equation_system_reduced_cnt = equation_system_reduced(cnt, :);
					F_fixed_A(:, :, cnt) = reshape(equation_system_reduced_cnt(1, 1:end - 1), dim_F_1, dim_F_2);
					F_fixed_B(:, :, cnt) = equation_system_reduced_cnt(1, end);
				end
				F_fixed_B = reshape(F_fixed_B, size_F_fixed, 1);
			end
		else
			F_fixed_A = [];
			F_fixed_B = [];
			prefilter_sym = true;
			disp('------------There are no prefilter constraints.-------');
		end
		if solvesymbolic
			if ~prefilter_sym
				F = double(F);
				normF1 = norm(F);
				if normF1 ~= 0
					F = F/normF1;
				end
			end
			if output_verbosity(solveroptions) && dim_F_2 > 0
				disp('------------A possible prefilter is:------------------');
				fprintf('\n');
				disp(vpa(F, 4));
			end
		end
	end
	RKF_fixed = {
		R_fixed;
		{
			[], []
		};
		{
			F_fixed_A, F_fixed_B
		}
	};
	RKF_bounds = {
		R_bounds;
		{
			[], []
		};
		F_bounds
	};
end
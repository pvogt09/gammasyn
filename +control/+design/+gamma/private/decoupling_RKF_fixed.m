function [RKF_fixed_out, RKF_bounds_out, valid, message] = decoupling_RKF_fixed(systems, R_fixed_ext, objectiveoptions, solveroptions, descriptor)
	%DECOUPLING_RKF_FIXED calculates structural constraints for proportional controller and prefilter coefficients, that are necessary for decoupling control
	%	Input:
	%		systems:					structure with dynamic systems to take into consideration
	%		R_fixed_ext:				cell array with controller constraints given externally
	%		objectiveoptions:			structure with objective options
	%		solveroptions:				options for optimization algorithm to use
	%		descriptor:					indicator if design should be performed for a DAE system
	%	Output:
	%		RKF_fixed_out:				cell array of three cell arrays providing the structural constraints of proportional, derivative and prefilter gain compatible with gammasyn
	%		RKF_bounds_out:				cell array of three cell arrays providing the structural inequality constraints of proportional, derivative and prefilter gain compatible with gammasyn
	%		valid:						indicator of failure
	%		message:					failure message
	if nargin <= 4
		descriptor = false;
	end
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
	number_measurements_xdot = size(systems(1).C_dot, 1);
	number_models = size(systems, 1);
	R_fixed = {zeros(number_controls, number_measurements, 0), zeros(0, 1)};
	K_fixed = {zeros(number_controls, number_measurements_xdot, 0), zeros(0, 1)};
	F_fixed = {zeros(number_controls, number_references, 0), zeros(0, 1)};
	RKF_fixed = {zeros(number_controls, number_measurements + number_measurements_xdot + number_references, 0), zeros(0, 1)};
	RKF_fixed_out = {
		R_fixed;
		K_fixed;
		F_fixed;
		RKF_fixed
	};
	R_bounds = {zeros(number_controls, number_measurements, 0), zeros(0, 1)};
	K_bounds = {zeros(number_controls, number_measurements_xdot, 0), zeros(0, 1)};
	F_bounds = {zeros(number_controls, number_references, 0), zeros(0, 1)};
	RKF_bounds = {zeros(number_controls, number_measurements + number_measurements_xdot + number_references, 0), zeros(0, 1)};
	RKF_bounds_out = {
		R_bounds;
		K_bounds;
		F_bounds;
		RKF_bounds
	};

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

	if solvesymbolic
		R = sym('r%d%d', [number_controls, number_measurements]);
		assume(R, 'real');
		r = reshape(R, number_controls*number_measurements, 1);
		K = sym('k%d%d', [number_controls, number_measurements_xdot]);
		assume(K, 'real');
		k = reshape(K, number_controls*number_measurements_xdot, 1);
		F = sym('f%d%d', [number_controls, number_references]);
		assume(F, 'real');
		f = reshape(F, number_controls*number_references, 1);
	else
		R = [];
		r = [];
		k = [];
		f = [];
	end
	rkf = [
		r;
		k;
		f
	];

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
		E  = systems(ii).E;
		null_C = null(C);
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
			Q = []; %#ok<NASGU> needed for parfor
			null_Cjj = null(Cjj);
			if isempty(null_Cjj)
				error('control:design:gamma:input', 'Matrix Cjj must have a null space.');
			end

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
			if size(null_C, 2) > 0 % only in this case, Q might not be conditioned invariant
				if rank([A*ints(Q, null_C), Q]) > rank(Q) % not the right condition for systems with feedthrough
					% controlled invariant subspace is not conditioned invariant
					not_con_invariant_mat(ii, jj) = true;
				end
			end
			m = size(Q, 2);
			dim_invariant_mat(ii, jj) = m;
			Q(abs(Q) < eps) = 0;
			Q_orth = null(Q.');
			Q_orth(abs(Q_orth) < eps) = 0;
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

	% include constraints given by user
	[R_fixed_X, R_fixed_z] = convert_hadamard2vectorized(R_fixed_ext{1});
	[K_fixed_X, K_fixed_z] = convert_hadamard2vectorized(R_fixed_ext{2});
	[F_fixed_X, F_fixed_z] = convert_hadamard2vectorized(R_fixed_ext{3});
	[RKF_fixed_X, RKF_fixed_z] = convert_hadamard2vectorized(R_fixed_ext{4});
	combined_constraints = ~isempty(RKF_fixed_X);

	X_R_cell_cat = cat(1, X_R_cell{:});
	z_R_cell_cat = cat(1, z_R_cell{:});
	X_F_cell_cat = cat(1, Q_orth_T_B_cell{:});
	rows_X_F = size(X_F_cell_cat, 1);

	if control_design_type == GammaDecouplingStrategy.APPROXIMATE_INEQUALITY
		X_R = X_R_cell_cat;
		z_R = z_R_cell_cat;
		X_F = X_F_cell_cat;
		z_F = zeros(rows_X_F, 1);
		X_comb = [];
		z_comb = [];
	else
		X_R = [];
		z_R = [];
		X_F = [];
		z_F = [];
		X_comb = [
			blkdiag(X_R_cell_cat, zeros(0, number_controls*number_measurements_xdot), X_F_cell_cat);
			blkdiag(R_fixed_X, K_fixed_X, F_fixed_X);
			RKF_fixed_X
		];
		z_comb = [
			z_R_cell_cat;
			zeros(0, 1);
			zeros(rows_X_F, 1);
			R_fixed_z;
			K_fixed_z;
			F_fixed_z;
			RKF_fixed_z
		];
	end

	X_R(abs(X_R) < eps) = 0; % avoid basic numerical difficulties
	z_R(abs(z_R) < eps) = 0;
	X_F(abs(X_F) < eps) = 0;
	X_comb(abs(X_comb) < eps) = 0;
	z_comb(abs(z_comb) < eps) = 0;

	if any(any((dim_invariant_mat == number_states) & ~sys_feedthrough_mat))
		error('RBABS:control:design:gamma:dimensions', 'Dimension m of controlled invariant subspace is equal to the system dimension %d, while there is no feedthrough in the decoupling condition. Specify non-trivial decoupling conditions.', number_states)
	end

	%% Calculate controller and prefilter constraints
	if control_design_type == GammaDecouplingStrategy.APPROXIMATE_INEQUALITY
		R_bounds = convert_vectorized2hadamard([
			X_R;
			-X_R
		], [
			z_R + objectiveoptions.decouplingcontrol.tolerance_decoupling;
			-z_R + objectiveoptions.decouplingcontrol.tolerance_decoupling
		], [number_controls, number_measurements]);
		F_bounds = convert_vectorized2hadamard([
			X_F;
			-X_F
		], [
			z_F + objectiveoptions.decouplingcontrol.tolerance_prefilter;
			-z_F + objectiveoptions.decouplingcontrol.tolerance_prefilter
		], [number_controls, number_references]);

		R_fixed = R_fixed_ext{1};
		K_fixed = R_fixed_ext{2};
		F_fixed = R_fixed_ext{3};
		RKF_fixed = R_fixed_ext{4};
	else
		sol = [];
		free_params_raw = [];
		size_RKF = [number_controls, number_measurements + number_measurements_xdot + number_references];
		if ~isempty(X_comb)
			alreadydisplayed = false;
			gototilde = false;
			[Xz_comb, sol, free_params_raw, sol_empty, sol_zero] = solveandcheck(X_comb, rkf, z_comb, size_RKF);
			if sol_empty
				if output_verbosity(solveroptions, 'notify')
					if combined_constraints
						disp('---------------X_comb*rkf=z_comb has no solution.--------------');
					else
						disp('------------------X_R*r=z_R has no solution.------------------');
					end
				end
				if allowApproxSol
					if output_verbosity(solveroptions)
					disp('----------------Calculate approximate solution.---------------');
						alreadydisplayed = true;
					end
					[Xz_comb, sol, free_params_raw, sol_empty_approx, sol_zero] = solveandcheck(X_comb.'*X_comb, rkf, X_comb.'*z_comb, size_RKF);
					if sol_empty_approx
						if ~isnan(round_to)
							[Xz_comb, sol, free_params_raw, sol_empty_round, sol_zero] = solveandcheck(round(X_comb.'*X_comb, round_to), rkf, round(X_comb.'*z_comb, round_to), size_RKF);
						else
							sol_empty_round = true;
						end
						if sol_empty_round
							error('control:design:gamma:decoupling', 'Calculation of combined constraints failed due to numerical difficulties.');
						elseif any(sol_zero(:, number_measurements + number_measurements_xdot + 1:end))
							gototilde = true;
						end
					elseif any(sol_zero(:, number_measurements + number_measurements_xdot + 1:end))
						gototilde = true;
					end
				else
					if combined_constraints
						message = 'X_comb*rkf=z_comb has no solution. Allow calculation of approximate solution, using GammaDecouplingStrategy.APPROXIMATE.';
					else
						message = 'X_R*r=z_R has no solution. Allow calculation of approximate solution, using GammaDecouplingStrategy.APPROXIMATE.';
					end
					valid = false;
					return;
				end
			elseif any(sol_zero(:, number_measurements + number_measurements_xdot + 1:end))
				if allowApproxSol
					gototilde = true;
				else
					message = 'At least one prefilter column is forced to zero. Allow calculation of approximate solution, using GammaDecouplingStrategy.APPROXIMATE.';
					valid = false;
					return;
				end
			end
			if gototilde
				if output_verbosity(solveroptions)
					disp('------At least one prefilter column is forced to zero.--------');
				end
				if output_verbosity(solveroptions) && ~alreadydisplayed
					disp('----------------Calculate approximate solution.---------------');
				end
				add_ones = [
					zeros(number_references, number_controls*(number_measurements + number_measurements_xdot)), kron(eye(number_references, number_references), ones(1, number_controls))
				];
				add_ones(~sol_zero(:, number_measurements + number_measurements_xdot + 1:end), :) = [];
				c_f = objectiveoptions.decouplingcontrol.tolerance_prefilter;% parameter to avoid zero columns in prefilter
				add_c_f = c_f*ones(size(add_ones, 1), 1);

				X_comb_tilde = [
					X_comb;
					add_ones
				];
				z_F_tilde = [
					z_comb;
					add_c_f
				];
				[Xz_comb, sol, free_params_raw, sol_empty_tilde, sol_zero] = solveandcheck(X_comb_tilde.'*X_comb_tilde, rkf, X_comb_tilde.'*z_F_tilde, size_RKF);
				if sol_empty_tilde
					if ~isnan(round_to)
						[Xz_comb, sol, free_params_raw, sol_empty_tilde_round, sol_zero] = solveandcheck(round(X_comb.'*X_comb, round_to), rkf, round(X_comb.'*z_comb, round_to), size_RKF);
					else
						sol_empty_tilde_round = true;
					end
					if sol_empty_tilde_round
						error('control:design:gamma:decoupling', 'Calculation of combined constraints failed due to numerical difficulties.');
					end
				end
				if any(sol_zero)
					warning('control:design:gamma:decoupling', 'Some prefilter columns are forced to zero due to external prefilter constraints.');
				end
			end
			if ~all(sol_zero(:, number_measurements + (1:number_measurements_xdot)))
				error('control:design:gamma:decoupling', 'K is not forced to zero.');
			end

			% Structural constraints of controller for gammasyn
			Xz_comb_fixed = rref([
				Xz_comb{1}, Xz_comb{2}
			]);
			Xz_comb_fixed(all(Xz_comb_fixed.' == 0), :) = [];
			RKF_fixed = convert_vectorized2hadamard(Xz_comb_fixed(:, 1:end - 1), Xz_comb_fixed(:, end), size_RKF);
		end
		% Show results
		if output_verbosity(solveroptions)
			if isempty(RKF_fixed{1}) && ~solvesymbolic
				disp('----There are no controller and prefilter constraints.--------');
			end
			if solvesymbolic
				showresults(R, K, F, sol, free_params_raw);
			end
		end
	end
	%% finalize
	RKF_fixed_out = {
		R_fixed;
		K_fixed;
		F_fixed;
		RKF_fixed
	};
	RKF_bounds_out = {
		R_bounds;
		K_bounds;
		F_bounds;
		RKF_bounds
	};
end

function [Ab, sol, parameters, sol_empty, sol_zero] = solveandcheck(A, x, b, sz)
	%SOLVEANDCHECK solves A*x = b and checks if the solution set is empty and/or zero
	%	Input:
	%		A:				coefficient matrix for A*x = b
	%		x:				symbolic vector for A*x = b or [] if no symbolic calculations should be done
	%		b:				vector for A*x = b
	%		sz:				optional: 2D-array with size in which solution vector is reshaped before performing zero check column wise
	%	Output:
	%		Ab:				cell array containing A and b
	%		sol:			vector with symbolic results for x, empty if no symbolic calculations performed
	%		parameters:		vector of free parameters, empty if no symbolic calculations performed
	%		sol_empty:		indicator if any solution field is empty. True if A*x = b has no solution.
	%		sol_zero:		indicator if all solution fields are empty. NaN if no solution exists or checkzero is false.
	if nargin <= 3
		sz = [
			size(A, 2), 1
		];
	else
		if prod(sz) ~= size(A, 2)
			error('control:design:gamma:decoupling', 'Wrong size for reshape.');
		end
	end
	if isa(x, 'sym')
		solvesymbolic = true;
	else
		solvesymbolic = false;
	end
	Ab = {
		A, b
	};
	if solvesymbolic
		sol_raw = solve(A*x == b, x, 'ReturnConditions', true);
		parameters = sol_raw.parameters.';
		sol_fields = struct2cell(sol_raw);
		sol_fields = sol_fields(1:size(x, 1));
		sol_empty = cellfun(@isempty, sol_fields, 'UniformOutput', true);
		sol_empty = any(sol_empty(:));
		sol = [sol_fields{:}].';
		if nargout >= 5
			if ~sol_empty
				sol_fields_mat = reshape(sol, sz);
				sol_zero = all(logical(sol_fields_mat == 0), 1);
			else
				sol_zero = false(1, sz(2));
			end
		end
	else
		sol = [];
		parameters = [];
		Ab_mat = [
			A, b
		];
		[A_rref, b_rref, sol_empty] = checkempty(Ab_mat);
		if sol_empty && rank(A) == rank(Ab_mat)
			% TODO: use tolerance to allow for solution?
			% TODO: No, but round equation systems if approximate system also does not have solution?
			% TODO: another possibility would be to define a tube of inequalities around the equation system and solve with linprog
			% try solution with parametric solution returned by \
			f_0 = A\b;
			f_0_ker = null(A, 'r');
			A_null = null(f_0_ker.', 'r').';
			Ab_mat = [
				A_null,	A_null*f_0
			];
			[A_rref, b_rref, sol_empty] = checkempty(Ab_mat);
		end
		pinv_Ab = pinv(A_rref)*b_rref;
		null_A = null(A_rref);
		comb = [
			pinv_Ab, null_A
		];
		zero_params = all(comb == 0, 2);
		zero_reshape = reshape(zero_params, sz);
		sol_zero = all(zero_reshape, 1);
	end

	function [A_rref, b_rref, sol_empty] = checkempty(Ab_mat)
		Ab_rref = rref(Ab_mat);
		Ab_rref(all(Ab_rref == 0, 2), :) = [];
		A_rref = Ab_rref(:, 1:end - 1);
		b_rref = Ab_rref(:, end);
		rows_zero = all(A_rref == 0, 2);
		sol_empty = any(rows_zero & b_rref == 1);
	end
end

function showresults(R, K, F, sol, free_params_raw)
	%SHOWRESULTS displays constrained controller or prefilter matrix
	%	Input:
	%		R:						symbolic controller matrix
	%		K:						symbolic differential controller matrix
	%		F:						symbolic prefilter matrix
	%		sol:					symbolic solution with constraints
	%		free_params_raw:		free parameters in sol
	msg1 = '--There are no free parameters for controller and prefilter.--';
	msg2 = '--------There are no free parameters for the controller.------';
	msg3 = '--------There are no free parameters for the prefilter.-------';
	msg4 = '----There are no controller and prefilter constraints.--------';
	msg5 = '-----Possible controller and prefilter matrices are:----------';

	if ~isempty(sol)
		[number_controls, number_measurements] = size(R);
		number_measurements_xdot = size(K, 2);
		number_references = size(F, 2);
		rkf = [
			reshape(R, [], 1);
			reshape(K, [], 1);
			reshape(F, [], 1);
		];
		[~, idx_intersect] = intersect(sol, free_params_raw);
		[~, r_idx_intersect] = intersect(sol(1:numel(R)), free_params_raw);
		[~, f_idx_intersect] = intersect(sol(numel(R) + numel(K) + 1:end), free_params_raw);
		RKF = reshape(sol, number_controls, number_measurements + number_measurements_xdot + number_references);
		if ~isempty(free_params_raw)
			z_ij = [
				free_params_raw, rkf(idx_intersect)
			];
			free_params = z_ij(:, 2);
			if size(free_params_raw, 1) ~= size(free_params, 1)
				free_params = transpose(free_params);
			end
			RKF = subs(RKF, free_params_raw, free_params);
		end

		R = RKF(:, 1:number_measurements);
		K = RKF(:, number_measurements + (1:number_measurements_xdot));
		F = RKF(:, number_measurements + number_measurements_xdot + 1:end);

		if isempty(r_idx_intersect) && isempty(f_idx_intersect)
			disp(msg1);
		elseif isempty(r_idx_intersect) && ~isempty(f_idx_intersect)
			disp(msg2);
		elseif ~isempty(r_idx_intersect) && isempty(f_idx_intersect)
			disp(msg3);
		end
		if isempty(f_idx_intersect)
			F = double(F);
			minF = min(F(:));
			if minF ~= 0
				F = F/minF;
			end
		end
	else
		disp(msg4);
	end

	disp(msg5);
	fprintf('\n');
	disp(vpa(R, 4));
	if ~isempty(K)
		fprintf('\n');
		disp(vpa(K, 4));
	end
	fprintf('\n');
	disp(vpa(F, 4));
end
function [RF_fixed, valid, message] = coupling_RF_fixed(systems, objectiveoptions, solveroptions, descriptor)
	%COUPLING_RF_FIXED calculates structural constraints for proportional controller and prefilter coefficients, that are necessary for coupling control
	%	Input:
	%		systems:					structure with dynamic systems to take into consideration
	%		objectiveoptions:			structure with objective options
	%		solveroptions:				options for optimization algorithm to use
	%		descriptor:					indicator if design should be performed for a DAE system
	%	Output:
	%		RF_fixed:					cell array of three cell arrays providing the structural constraints of proportional, derivative and prefilter gain compatible with gammasyn
	%		valid:						indicator of failure
	%		message:					failure message
	if nargin <= 3
		descriptor = false;
	end
	hassymbolic = configuration.matlab.hassymbolictoolbox();
	solvesymbolic = objectiveoptions.couplingcontrol.solvesymbolic;
	if solvesymbolic && ~hassymbolic
		warning('control:design:gamma:coupling', 'Constraints shall be calculated symbolically, but Symbolic Toolbox was not found.');
	end
	solvesymbolic = hassymbolic && solvesymbolic;
	% parameter for rounding equation systems in case no solution of normal equations (which must have a solution) is found
	round_to = objectiveoptions.couplingcontrol.round_equations_to_digits;
	RF_fixed = [];
	valid = true;
	message = '';
	control_design_type = objectiveoptions.couplingcontrol.couplingstrategy;
	if ~isa(control_design_type, 'GammaCouplingStrategy')
		error('control:design:gamma:input', 'Coupling controller design type must be of type ''GammaCouplingStrategy''.');
	end
	if control_design_type == GammaCouplingStrategy.EXACT
		allowApproxSol = false;
	elseif control_design_type == GammaCouplingStrategy.APPROXIMATE
		allowApproxSol = true;
	else
		error('control:design:gamma:input', 'Wrong GammaCouplingStrategy.');
	end

	%% Constraint equations for controller and prefilter
	%  Equations have the form X_R*r = z_R and X_F*f = z_F
	number_couplingconditions = double(objectiveoptions.couplingcontrol.couplingconditions);
	number_states = size(systems(1).A, 1);
	number_controls = size(systems(1).B, 2);
	number_references = size(systems(1).C_ref, 1);
	number_models = size(systems, 1);

	dim_F1_1 = number_controls;									% number of rows of prefilter F1
	dim_F1_2 = number_references - number_couplingconditions;	% number of columns of prefilter F1
	dim_F2_2 = number_couplingconditions;						% number of columns of prefilter F2

	if solvesymbolic
		R = sym('r%d%d', [number_controls, number_states]);
		assume(R, 'real');
		r = reshape(R, number_controls*number_states, 1);
		F1 = sym('f%d%d', [dim_F1_1, dim_F1_2]);
		assume(F1, 'real');
		f = reshape(F1, dim_F1_1*dim_F1_2, 1);
	else
		R = [];
		r = [];
		f = [];
	end

	Q_orth_T_B_cell = cell(number_models, 1);
	X_R_cell = cell(number_models, 1);
	z_R_cell = cell(number_models, 1);
	dim_invariant_vec = zeros(number_models, 1);
	sys_feedthrough_vec = false(number_models, 1);
	parfor ii = 1:number_models
		A  = systems(ii).A;
		B  = systems(ii).B;
		C2 = systems(ii).C_ref(end - number_couplingconditions + 1:end, :);
		D2 = systems(ii).D_ref(end - number_couplingconditions + 1:end, :);
		if any(D2(:) ~= 0)
			sys_feedthrough_vec(ii, 1) = true;
		end
		E  = systems(ii).E;
		Q = []; %#ok<NASGU> needed for parfor

		if descriptor
			if rank(E) ~= number_states
				error('control:design:gamma:input', 'Matrix E must be invertible for coupling controller design.');
			else
				Q = vstar(E\A, E\B, C2, D2);
			end
		else
			Q = vstar(A, B, C2, D2); % output nulling controlled invariant subspace. If D2 == 0, same as mainco(A, B, null(C2))
		end
		if all(all(Q(:) == 0))
			Q = zeros(number_states, 0);
		end
		m = size(Q, 2);
		dim_invariant_vec(ii, 1) = m;
		Q_orth = null(Q.');
		V = [
			Q, Q_orth
		];
		if sys_feedthrough_vec(ii, 1) % if D2 == 0: no new equations
			Q_orth_T_B_cell{ii, 1} = [
				Q_orth'*B;
				D2
			];
		else
			Q_orth_T_B_cell{ii, 1} = Q_orth'*B;
		end
		if solvesymbolic
			ArT = V'*(A - B*R)*V; % V' == inv(V)
			A21 = ArT(m + 1:end, 1:m);
			[X_calC_sym, z_calC_sym] = equationsToMatrix(A21 == 0, r);
			X_calC = double(X_calC_sym);
			z_calC = double(z_calC_sym);
		else
			X_calC = kron(Q', Q_orth'*B);
			z_calC = reshape(Q_orth'*A*Q, (number_states - m)*m, 1);
		end
		if sys_feedthrough_vec(ii, 1) % if D2 == 0: kron(Q', D2) = 0 and C2*Q = 0 --> no new equations
			X = [
				X_calC;
				kron(Q', D2)
			];
			z = [
				z_calC;
				reshape(C2*Q, number_couplingconditions*m, 1)
			];
		else
			X = X_calC;
			z = z_calC;
		end
		X_R_cell{ii, 1} = X;
		z_R_cell{ii, 1} = z;
	end

	if any(dim_invariant_vec(:) ~= dim_invariant_vec(1))
		error('control:design:gamma:dimensions', 'Dimension of controlled invariant subspace must be the same for every system.');
	else
		dim_invariant = dim_invariant_vec(1);
	end

	X_R = cat(1, X_R_cell{:});
	z_R = cat(1, z_R_cell{:});
	Q_orth_T_B = cat(1, Q_orth_T_B_cell{:});
	X_F = kron(eye(dim_F1_2), Q_orth_T_B);
	z_F = zeros(number_models*((number_states - dim_invariant_vec(1))*dim_F1_2) + number_couplingconditions*sum(sys_feedthrough_vec), 1);


	%% Choose constrained parameters
	if dim_invariant == number_states && ~any(sys_feedthrough_vec)
		error('control:design:gamma:dimensions', 'Dimension m of controlled invariant subspace is equal to the system dimension %d. Specify non-trivial coupling conditions.', number_states)
	end
	if ~isempty(X_R)
		if solvesymbolic
			r_sol = solve(X_R*r == z_R, r, 'ReturnConditions', true);
			r_sol_fields = struct2cell(r_sol);
			r_sol_fields = r_sol_fields(1:number_states*number_controls);
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
					r_sol_fields = r_sol_fields(1:number_states*number_controls);
					r_sol_empty_approx = cellfun(@isempty, r_sol_fields, 'UniformOutput', true);
					r_sol_empty_approx = any(r_sol_empty_approx(:));
					if r_sol_empty_approx
						if isnan(round_to)
							r_sol = solve(X_R.'*X_R*r == X_R.'*z_R, r, 'ReturnConditions', true);
						else
							r_sol = solve(round(X_R.'*X_R, round_to)*r == round(X_R.'*z_R, round_to), r, 'ReturnConditions', true);
						end
						r_sol_fields = struct2cell(r_sol);
						r_sol_fields = r_sol_fields(1:number_states*number_controls);
						r_sol_empty_approx_round = cellfun(@isempty, r_sol_fields, 'UniformOutput', true);
						r_sol_empty_approx_round = any(r_sol_empty_approx_round(:));
						if r_sol_empty_approx_round
							error('control:design:gamma:coupling', 'Calculation of controller constraints failed due to numerical difficulties.');
						end
					end
				else
					message = 'X_R*r=z_R has no solution. Allow calculation of approximate solution, using GammaCouplingStrategy.APPROXIMATE.';
					valid = false;
					return;
				end
			end
			r_sol_array = [r_sol_fields{:}].';
			free_params = r_sol.parameters;
			[~, idx_intersect_r] = intersect(r_sol_array, free_params);
			idx_constr_params_r = setdiff(1:number_states*number_controls, idx_intersect_r);
			no_constr_params_r = length(idx_constr_params_r);

			R = reshape(r_sol_array(1:number_states*number_controls), number_controls, number_states);
			if ~isempty(free_params)
				z_rij = free_params.';
				z_rij = [
					z_rij, r(idx_intersect_r)
				];
				free_params_r = z_rij(:, 2).';
				R = subs(R, free_params, free_params_r);
			else
				if output_verbosity(solveroptions)
					disp('Only one controller can fulfill the coupling conditions.');
				end
			end

			% Structural constraints of controller for gammasyn
			R_fixed_A = zeros(number_controls, number_states, no_constr_params_r);
			R_fixed_B = zeros(1, 1, no_constr_params_r);
			parfor cnt = 1:no_constr_params_r
				ii = idx_constr_params_r(cnt);
				[eqA, eqB] = equationsToMatrix(r(ii) == R(ii), r); %#ok<PFBNS>
				R_fixed_A(:, :, cnt) = double(reshape(eqA, number_controls, number_states));
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
					% TODO: another possibility would be to define a tube of inequalities around the equations system and solve with linprog
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
						message = 'X_R*r=z_R has no solution. Allow calculation of approximate solution, using GammaCouplingStrategy.APPROXIMATE.';
						valid = false;
						return;
					end
				end
			end
			no_constr_params_r = size(equation_system_reduced, 1);
			R_fixed_A = zeros(number_controls, number_states, no_constr_params_r);
			R_fixed_B = zeros(1, 1, no_constr_params_r);
			parfor cnt = 1:no_constr_params_r
				equation_system_reduced_cnt = equation_system_reduced(cnt, :);
				R_fixed_A(:, :, cnt) = double(reshape(equation_system_reduced_cnt(1, 1:end - 1), number_controls, number_states));
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
	%% Prefilter
	c_f = objectiveoptions.couplingcontrol.tolerance_prefilter;% parameter to avoid trivial solution for prefilter
	if ~isempty(X_F)
		if solvesymbolic
			f_sol = solve(X_F*f == z_F, f, 'ReturnConditions', true);
			f_sol_fields = struct2cell(f_sol);
			f_sol_fields = f_sol_fields(1:dim_F1_1*dim_F1_2);
			f_sol_zero  = all(logical([f_sol_fields{:}] == 0));

			if f_sol_zero
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
					f_sol = solve(X_F_tilde.'*X_F_tilde*f == X_F_tilde.'*z_F_tilde, f, 'ReturnConditions', true);
					f_sol_fields = struct2cell(f_sol);
					f_sol_fields = f_sol_fields(1:dim_F1_1*dim_F1_2);
					f_sol_empty_approx = cellfun(@isempty, f_sol_fields, 'UniformOutput', true);
					f_sol_empty_approx = any(f_sol_empty_approx(:));
					if f_sol_empty_approx
						if isnan(round_to)
							f_sol = solve(X_F_tilde.'*X_F_tilde*f == X_F_tilde.'*z_F_tilde, f, 'ReturnConditions', true);
						else
							f_sol = solve(round(X_F_tilde.'*X_F_tilde, round_to)*f == round(X_F_tilde.'*z_F_tilde, round_to), f, 'ReturnConditions', true);
						end
						f_sol_fields = struct2cell(f_sol);
						f_sol_fields = f_sol_fields(1:dim_F1_1*dim_F1_2);
						f_sol_empty_approx_round = cellfun(@isempty, f_sol_fields, 'UniformOutput', true);
						f_sol_empty_approx_round = any(f_sol_empty_approx_round(:));
						if f_sol_empty_approx_round
							error('control:design:gamma:coupling', 'Calculation of prefilter constraints failed due to numerical difficulties.');
						end
					end
				else
					message = 'There is no non-zero prefilter. Allow calculation of approximate solution, using GammaCouplingStrategy.APPROXIMATE.';
					valid = false;
					return;
				end
			end
			f_sol_array = cat(1, f_sol_fields{:});
			free_params_zf = f_sol.parameters;
			[~, idx_intersect_f] = intersect(f_sol_array, free_params_zf);
			idx_constr_params_f = setdiff(1:dim_F1_1*dim_F1_2, idx_intersect_f);
			no_constr_params_f = length(idx_constr_params_f);

			F1 = reshape(f_sol_array, dim_F1_1, dim_F1_2);

			if ~isempty(free_params_zf)
				F1 = subs(F1, free_params_zf, f(idx_intersect_f));
				prefilter_sym = true;
			else
				prefilter_sym = false;
			end

			% Structural constraints of prefilter for gammasyn
			if ~f_sol_zero
				size_F_fixed = no_constr_params_f + 1;
				F_fixed_A = zeros(dim_F1_1, dim_F1_2 + dim_F2_2, size_F_fixed);
				F_fixed_B = zeros(1, 1, size_F_fixed);
				F_fixed_A(:, :, size_F_fixed) = [
					ones(dim_F1_1, dim_F1_2), zeros(dim_F1_1, dim_F2_2)
				];
				F_fixed_B(:, :, size_F_fixed) = c_f;
			else
				size_F_fixed = no_constr_params_f;
				F_fixed_A = zeros(dim_F1_1, dim_F1_2 + dim_F2_2, no_constr_params_f);
				F_fixed_B = zeros(1, 1, no_constr_params_f);
			end
			for cnt = 1:no_constr_params_f %#ok<FORPF> no parfor because of non linear indexing
				ii = idx_constr_params_f(cnt);
				[eqA, eqB] = equationsToMatrix(f(ii) == F1(ii), f);
				F_fixed_A(:, :, cnt) = [
					double(reshape(eqA, dim_F1_1, dim_F1_2)), zeros(dim_F1_1, dim_F2_2)
				];
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
					% TODO: another possibility would be to define a tube of inequalities around the equations system and solve with linprog
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
						message = 'There is no non-zero prefilter. Allow calculation of approximate solution, using GammaCouplingStrategy.APPROXIMATE.';
						valid = false;
						return;
					end
				end
				size_F_fixed = size(equation_system_reduced, 1) + 1;
				F_fixed_A = NaN(dim_F1_1, dim_F1_2 + dim_F2_2, size_F_fixed);
				F_fixed_B = NaN(1, 1, size_F_fixed);
				F_fixed_A(:, :, size_F_fixed) = [
					ones(dim_F1_1, dim_F1_2), zeros(dim_F1_1, dim_F2_2)
				];
				F_fixed_B(:, :, size_F_fixed) = c_f;
			else
				size_F_fixed = size(equation_system_reduced, 1);
				F_fixed_A = NaN(dim_F1_1, dim_F1_2 + dim_F2_2, size(equation_system_reduced, 1));
				F_fixed_B = NaN(1, 1, size(equation_system_reduced, 1));
			end
			parfor cnt = 1:size(equation_system_reduced, 1)
				equation_system_reduced_cnt = equation_system_reduced(cnt, :);
				F_fixed_A(:, :, cnt) = [
					reshape(equation_system_reduced_cnt(1, 1:end - 1), dim_F1_1, dim_F1_2), zeros(dim_F1_1, dim_F2_2)
				];
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
	RF_fixed = {
		R_fixed;
		{
			[], []
		};
		{
			F_fixed_A, F_fixed_B
		}
	};
	if solvesymbolic
		if ~prefilter_sym
			F1 = double(F1);
			normF1 = norm(F1);
			if normF1 ~= 0
				F1 = F1/normF1;
			end
		end
		if output_verbosity(solveroptions) && dim_F1_2 > 0
			disp('------------A possible prefilter is:------------------');
			fprintf('\n');
			disp(vpa(F1, 4));
		end
	end
end
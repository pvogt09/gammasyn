function [Ropt, Jopt, information] = gammasyn_couplingcontrol(systems, areafun, weights, R_fixed, R_0, solveroptions, objectiveoptions, R_bounds, R_nonlin)
	%GAMMASYN_COUPLINGCONTROL robust coupling control design and pole placement for multi-models including models in DAE form
	%	Input:
	%		systems:			structure with dynamic systems to take into consideration. Systems must only have matrices E, A, B, C = eye(n).
	%		areafun:			area border functions and gradients as cell array with number of system entries, each function returns a row vector with as much elements as area border functions are defined for the corresponding system
	%		weights:			weighting matrix with number of systems columns and number of pole area border functions rows
	%		R_fixed:			cell array with indicator matrix for gain elements that should be fixed and the values the fixed gains have, empty if no fixed elements are needed. For linear dependent gain components the first element is a 3D matrix with relation in the third dimension and the second element a vector of constant constraint elements
	%		R_0:				cell array or array with initial value for state feedback controller and optionally prefilter.
	%		solveroptions:		options for optimization algorithm to use
	%		objectiveoptions:	options for problem functions
	%		R_bounds:			cell array with indicator matrix for gain elements that should be bounded and the values the gains are bounded by, empty if no bounded elements are needed. For linear dependent gain components the first element is a 3D matrix with relation in the third dimension and the second element a vector of upper bound constraint elements
	%		R_nonlin:			function pointer to a function of nonlinear inequality and equality constraints on gains with signature [c_R, ceq_R, c_K, ceq_K, c_F, ceq_F, gradc_R, gradceq_R, gradc_K, gradceq_K, gradc_F, gradceq_F] = constraint(R, K, F)
	%	Output:
	%		Ropt:				cell array containing optimal controller and prefilter
	%		Jopt:				optimal objective function value for optimal gain
	%		information:		structure with information about optimization runs

	%% initialization
	if nargin <= 5
		solveroptions = checkandtransform_solveroptions();
	else
		solveroptions = checkandtransform_solveroptions(solveroptions);
	end
	if nargin <= 6
		objectiveoptions = struct();
	end
	if isa(objectiveoptions, 'control.design.gamma.GammasynOptions')
		objectiveoptions = struct(objectiveoptions);
	end
	if ~isstruct(objectiveoptions)
		error('control:design:gamma:input', 'Objectiveoptions must be a structure, not a ''%s''.', class(objectiveoptions));
	end
	if isfield(objectiveoptions, 'couplingcontrol')
		objectiveoptions.couplingcontrol = checkobjectiveoptions_coupling(objectiveoptions.couplingcontrol);
	else
		objectiveoptions.couplingcontrol = checkobjectiveoptions_coupling();
	end

	%% check Geomteric Approach Toolbox
	if ~configuration.control.design.hasGeometricApproachToolbox()
		% VSTAR function needs "Geometric Approach Toolbox" installed
		% http://www3.deis.unibo.it/Staff/FullProf/GiovanniMarro/geometric.htm
		error('control:design:gamma:GeomtricApproachToolboxMissing', 'Coupling controller synthesis needs ''Geometric Approach Toolbox'' installed. See http://www3.deis.unibo.it/Staff/FullProf/GiovanniMarro/geometric.htm');
	end

	%% inspect coupling options
	control_design_type = objectiveoptions.couplingcontrol.couplingstrategy;
	if ~isscalar(control_design_type)
		error('control:design:gamma:input', 'Coupling controller design type must be scalar.');
	end
	if ~isa(control_design_type, 'GammaCouplingStrategy')
		error('control:design:gamma:input', 'Coupling controller design type must be of type ''GammaCouplingStrategy''.');
	end
	if control_design_type == GammaCouplingStrategy.EXACT
		numeric = false;
	elseif any(control_design_type == [
		GammaCouplingStrategy.APPROXIMATE;
		GammaCouplingStrategy.APPROXIMATE_INEQUALITY
	])
		numeric = false;
	elseif control_design_type == GammaCouplingStrategy.NUMERIC_NONLINEAR_EQUALITY
		numeric = true;
	elseif control_design_type == GammaCouplingStrategy.NUMERIC_NONLINEAR_INEQUALITY
		numeric = true;
	elseif control_design_type == GammaCouplingStrategy.NONE
		error('control:design:gamma:input', 'Coupling controller shall be designed but is turned off explicitly.');
	else
		error('control:design:gamma:input', 'Wrong coupling controller option.');
	end

	%% check and transform systems
	number_models = size(systems, 1);
	[systems, number_states, number_states_all, number_controls, number_measurements, number_measurements_xdot, number_references, descriptor] = checkandtransformsystems(systems, objectiveoptions);
	if any(number_states ~= number_states_all(:))
		error('control:design:gamma:dimensions', 'For coupling controller design, all systems must have same number of states.');
	end
	if number_references ~= number_controls
		error('control:design:gamma:dimensions', 'For coupling controller design, systems must have as many references as controls.');
	end
	if number_states ~= number_measurements
		error('control:design:gamma:dimensions', 'For coupling controller design, all states must be measured.');
	end
	C_dot = systems(1).C_dot;
	parfor ii = 1:number_models
		if any(any(systems(ii).C ~= eye(number_states)))
			error('control:design:gamma:dimensions', 'For coupling controller design, a complete state feedback must be calculated. Matrix C must be eye(%d)', number_states);
		end
		if any(any(systems(ii).D ~= zeros(number_measurements, number_controls)))
			error('control:design:gamma:dimensions', 'For coupling controller design, systems must not have measurement feedthrough.');
		end
		if any(any(systems(ii).C_dot ~= C_dot)) && descriptor
			error('control:design:gamma:input', 'For DAE design, systems must have equal differential output.');
		end
	end

	% RKF_fixed
	if nargin <= 3
		[R_fixed, K_fixed, F_fixed, RKF_fixed] = input_gain_constraint2RKF();
	else
		[R_fixed, K_fixed, F_fixed, RKF_fixed] = input_gain_constraint2RKF(R_fixed);
	end
	[R_fixed, constraint_system, constraint_border, rg, T, T_inv, hasfixed_R, onlyfixed_R, allfixed_R, constraint_system_hadamard_R, isforced2zero_R] = checkandtransform_gain_fixed(R_fixed, number_controls, number_measurements, 'proportional');
	[K_fixed, constraint_system_xdot, constraint_border_xdot, rg_xdot, T_xdot, T_inv_xdot, hasfixed_K, onlyfixed_K, allfixed_K, constraint_system_hadamard_K, isforced2zero_K] = checkandtransform_gain_fixed(K_fixed, number_controls, number_measurements_xdot, 'derivative', true(number_controls, number_measurements_xdot));
	[F_fixed, constraint_system_prefilter, constraint_border_prefilter, rg_prefilter, T_prefilter, T_inv_prefilter, hasfixed_F, onlyfixed_F, allfixed_F, constraint_system_hadamard_F, isforced2zero_F] = checkandtransform_gain_fixed(F_fixed, number_controls, number_references, 'prefilter');
	K_pattern = false(number_controls, number_measurements + number_measurements_xdot + number_references);
	K_pattern(:, number_measurements + 1:number_measurements + number_measurements_xdot) = true;
	[RKF_fixed, constraint_system_RKF, constraint_border_RKF, rg_RKF, T_RKF, T_inv_RKF, hasfixed_RKF, onlyfixed_RKF, allfixed_RKF, constraint_system_hadamard_RKF, isforced2zero_RKF_K] = checkandtransform_gain_fixed(RKF_fixed, number_controls, number_measurements + number_measurements_xdot + number_references, 'combined', K_pattern);
	[...
		~, ~, ~, ~, ~, ~, ~, ~, ~, constraint_system_hadamard_R, ~,...
		~, ~, ~, ~, ~, ~, ~, ~, ~, constraint_system_hadamard_K, isforced2zero_K,...
		~, ~, ~, ~, ~, ~, ~, ~, ~, constraint_system_hadamard_F, ~,...
		~, ~, ~, ~, ~, ~, hasfixed_RKF, ~, ~, constraint_system_hadamard_RKF...
	] = checkandtransform_gain_fixed_RKF(...
		R_fixed, constraint_system, constraint_border, rg, T, T_inv, hasfixed_R, onlyfixed_R, allfixed_R, constraint_system_hadamard_R, isforced2zero_R,...
		K_fixed, constraint_system_xdot, constraint_border_xdot, rg_xdot, T_xdot, T_inv_xdot, hasfixed_K, onlyfixed_K, allfixed_K, constraint_system_hadamard_K, isforced2zero_K,...
		F_fixed, constraint_system_prefilter, constraint_border_prefilter, rg_prefilter, T_prefilter, T_inv_prefilter, hasfixed_F, onlyfixed_F, allfixed_F, constraint_system_hadamard_F, isforced2zero_F,...
		RKF_fixed, constraint_system_RKF, constraint_border_RKF, rg_RKF, T_RKF, T_inv_RKF, hasfixed_RKF, onlyfixed_RKF, allfixed_RKF, constraint_system_hadamard_RKF,...
		number_controls, number_measurements, number_measurements_xdot, number_references, false...
	);

	% RKF_bounds
	if nargin <= 7
		[R_bounds, K_bounds, F_bounds, RKF_bounds] = input_gain_constraint2RKF();
	else
		[R_bounds, K_bounds, F_bounds, RKF_bounds] = input_gain_constraint2RKF(R_bounds);
	end
	[R_bounds, bound_system, bound_border, rg_bounds, hasbounds_R, onlybounds_R, bound_system_hadamard_R] = checkandtransform_gain_bounds(R_bounds, number_controls, number_measurements, 'proportional');
	[K_bounds, bound_system_xdot, bound_border_xdot, rg_xdot_bounds, hasbounds_K, onlybounds_K, bound_system_hadamard_K] = checkandtransform_gain_bounds(K_bounds, number_controls, number_measurements_xdot, 'proportional');
	[F_bounds, bound_system_prefilter, bound_border_prefilter, rg_prefilter_bounds, hasbounds_F, onlybounds_F, bound_system_hadamard_F] = checkandtransform_gain_bounds(F_bounds, number_controls, number_references, 'prefilter');
	[RKF_bounds, bound_system_RKF, bound_border_RKF, rg_RKF_bounds, hasbounds_RKF, onlybounds_RKF, bound_system_hadamard_RKF] = checkandtransform_gain_bounds(RKF_bounds, number_controls, number_measurements + number_measurements_xdot + number_references, 'combined');
	[...
		~, ~, ~, ~, ~, ~, bound_system_hadamard_R,...
		~, ~, ~, ~, ~, ~, bound_system_hadamard_K,...
		~, ~, ~, ~, ~, ~, bound_system_hadamard_F,...
		~, ~, ~, ~, ~, ~, bound_system_hadamard_RKF...
	] = checkandtransform_gain_bounds_RKF(...
		R_bounds, bound_system, bound_border, rg_bounds, hasbounds_R, onlybounds_R, bound_system_hadamard_R,...
		K_bounds, bound_system_xdot, bound_border_xdot, rg_xdot_bounds, hasbounds_K, onlybounds_K, bound_system_hadamard_K,...
		F_bounds, bound_system_prefilter, bound_border_prefilter, rg_prefilter_bounds, hasbounds_F, onlybounds_F, bound_system_hadamard_F,...
		RKF_bounds, bound_system_RKF, bound_border_RKF, rg_RKF_bounds, hasbounds_RKF, onlybounds_RKF, bound_system_hadamard_RKF,...
		number_controls, number_measurements, number_measurements_xdot, number_references, hasfixed_RKF...
	);
	[R_fixed_A, R_fixed_b] = convert_hadamard2vectorized(constraint_system_hadamard_R);
	[K_fixed_A, K_fixed_b] = convert_hadamard2vectorized(constraint_system_hadamard_K);
	[F_fixed_A, F_fixed_b] = convert_hadamard2vectorized(constraint_system_hadamard_F);
	[RKF_fixed_A, RKF_fixed_b] = convert_hadamard2vectorized(constraint_system_hadamard_RKF);
	[R_bounds_A, R_bounds_b] = convert_hadamard2vectorized(bound_system_hadamard_R);
	[K_bounds_A, K_bounds_b] = convert_hadamard2vectorized(bound_system_hadamard_K);
	[F_bounds_A, F_bounds_b] = convert_hadamard2vectorized(bound_system_hadamard_F);
	[RKF_bounds_A, RKF_bounds_b] = convert_hadamard2vectorized(bound_system_hadamard_RKF);
	if ~isforced2zero_K && ~isforced2zero_RKF_K
		error('control:design:gamma:input', 'Differential output must be forced to zero for coupling controller design.');
	end
	% R_nonlin
	if nargin <= 8
		R_nonlin = [];
	end

	%% transform systems to systems_tilde, that are used for gammasyn
	if descriptor
		systems_tilde = struct(...
			'E',						cell(number_models, 1),...
			'A',						cell(number_models, 1),...
			'B',						cell(number_models, 1),...
			'C',						cell(number_models, 1),...
			'C_dot',					cell(number_models, 1),...
			'D',						cell(number_models, 1),...
			'C_ref',					cell(number_models, 1),...
			'D_ref',					cell(number_models, 1)...
		);
		N_E_cell = cell(number_models, 1);

		number_states_tilde_vec =		NaN(number_models, 1);
		number_controls_tilde_vec =		NaN(number_models, 1);
		number_couplingconditions_vec =	NaN(number_models, 1);
		number_references_tilde_vec =	NaN(number_models, 1);

		% create systems_tilde
		parfor ii = 1:number_models
			% original form
			A =		systems(ii).A;
			B =		systems(ii).B;
			E =		systems(ii).E;
			C_ref =	systems(ii).C_ref;
			D_ref =	systems(ii).D_ref;

			number_states_tilde =	rank(E);
			[U_E, Sigma_E, N_E] =	svd(E);
			A_trans =				U_E'*A*N_E;
			B_trans =				U_E'*B;
			Sigma_r =				Sigma_E(1:number_states_tilde, 1:number_states_tilde);
			Sigma_r_i =				diag(1./diag(Sigma_r));

			% semi-explicit form
			A11 = Sigma_r_i*A_trans(1:number_states_tilde, 1:number_states_tilde);
			A12 = Sigma_r_i*A_trans(1:number_states_tilde, number_states_tilde + 1:end);
			A21 = A_trans(number_states_tilde + 1:end, 1:number_states_tilde);
			A22 = A_trans(number_states_tilde + 1:end, number_states_tilde + 1:end);
			B1  = Sigma_r_i*B_trans(1:number_states_tilde, :);
			B2  = B_trans(number_states_tilde + 1:end, :);
			C1	= C_ref*N_E(:, 1:number_states_tilde);
			C2	= C_ref*N_E(:, number_states_tilde + 1:end);

			% tilde form (statespace system with feedthrough)
			A_tilde = A11;
			B_tilde = [
				A12, B1
			];
			C_tilde = A21;
			D_tilde = [
				A22, B2
			];
			C_ref_tilde = C1;
			D_ref_tilde = [
				C2, D_ref
			];

			% include original references
			C_ref_comb = [
				C_ref_tilde;
				C_tilde
			];
			D_ref_comb = [
				D_ref_tilde;
				D_tilde
			];

			% store data
			number_states_tilde_vec(ii) =			number_states_tilde;
			number_controls_tilde_vec(ii) =			number_controls + number_states - number_states_tilde;
			number_couplingconditions_vec(ii) =		number_states - number_states_tilde;
			number_references_tilde_vec(ii) =		number_references + number_states - number_states_tilde;

			systems_tilde(ii).E =					eye(number_states_tilde);
			systems_tilde(ii).A =					A_tilde;
			systems_tilde(ii).B =					B_tilde;
			systems_tilde(ii).C =					eye(number_states_tilde);
			systems_tilde(ii).C_dot =				zeros(0, number_states_tilde);
			systems_tilde(ii).D =					zeros(number_states_tilde, number_controls_tilde_vec(ii));
			systems_tilde(ii).C_ref =				C_ref_comb;
			systems_tilde(ii).D_ref =				D_ref_comb;

			N_E_cell{ii, 1} = N_E;
		end

		if (any(number_states_tilde_vec(:) ~= number_states_tilde_vec(1))) || any(isnan(number_states_tilde_vec(:)))
			error('control:design:gamma:dimensions', 'For DAE controller design, all matrices E must have same rank.');
		end
		if (any(number_controls_tilde_vec(:) ~= number_controls_tilde_vec(1))) || any(isnan(number_controls_tilde_vec(:)))
			error('control:design:gamma:dimensions', 'For DAE controller design, all transformed systems must have same number of controls.');
		end
		if (any(number_couplingconditions_vec(:) ~= number_couplingconditions_vec(1))) || any(isnan(number_couplingconditions_vec(:)))
			error('control:design:gamma:dimensions', 'For DAE controller design, all transformed systems must have the same number of coupling conditions.');
		end
		if any(any([N_E_cell{:}] ~= repmat(N_E_cell{1}, 1, number_models)))
			warning('control:design:gamma:couplingcontrol', 'Backtransformation not equal for every system. Poles may vary after transformation.')
		end
		N_E = N_E_cell{1};

		number_states_tilde =				number_states_tilde_vec(1);
		number_controls_tilde =				number_controls_tilde_vec(1);
		number_references_tilde =			number_references_tilde_vec(1);
		number_measurements_xdot_tilde =	0;

		objectiveoptions.couplingcontrol.couplingconditions = uint32(number_couplingconditions_vec(1));
	else
		systems_tilde =						systems;
		number_states_tilde =				number_states;
		number_controls_tilde =				number_controls;
		number_references_tilde =			number_references;
		number_measurements_xdot_tilde =	0;
	end
	C_dot_tilde = zeros(0, number_states_tilde);

	%% transform R_fixed, F_fixed
	if descriptor
		% K is already forced to zero. K_tilde is forced to zero manually afterwards
		[R_fixed_A_tilde, R_fixed_b_tilde, ~, ~, F_fixed_A_tilde, F_fixed_b_tilde, RKF_fixed_A_tilde, RKF_fixed_b_tilde, T_R, T_K, T_F] = transform_RKF_constraints(R_fixed_A, R_fixed_b, K_fixed_A, K_fixed_b, F_fixed_A, F_fixed_b, RKF_fixed_A, RKF_fixed_b, N_E, C_dot, C_dot_tilde, number_controls, number_controls_tilde, number_states, number_states_tilde, number_references_tilde, number_measurements_xdot, number_measurements_xdot_tilde, 'fixed');
	else
		R_fixed_A_tilde = R_fixed_A;
		R_fixed_b_tilde = R_fixed_b;
		F_fixed_A_tilde = F_fixed_A;
		F_fixed_b_tilde = F_fixed_b;
		RKF_fixed_A_tilde = RKF_fixed_A;
		RKF_fixed_b_tilde = RKF_fixed_b;
	end
	K_fixed_A_tilde = eye(number_controls_tilde*number_measurements_xdot_tilde);
	K_fixed_A_tilde = reshape(K_fixed_A_tilde, number_controls_tilde, number_measurements_xdot_tilde, number_controls_tilde*number_measurements_xdot_tilde);
	K_fixed_b_tilde = zeros(number_controls_tilde*number_measurements_xdot_tilde, 1);
	R_fixed_tilde = {
		reshape(R_fixed_A_tilde.', number_controls_tilde, number_states_tilde, size(R_fixed_A_tilde, 1)), R_fixed_b_tilde
	};
	K_fixed_tilde = {
		K_fixed_A_tilde, K_fixed_b_tilde
	};
	F_fixed_tilde = {
		reshape(F_fixed_A_tilde.', number_controls_tilde, number_references, size(F_fixed_A_tilde, 1)), F_fixed_b_tilde
	};
	RKF_fixed_tilde = {
		reshape(RKF_fixed_A_tilde.', number_controls_tilde, number_states_tilde + number_measurements_xdot_tilde + number_references, size(RKF_fixed_A_tilde, 1)), RKF_fixed_b_tilde
	};

	%% transform R_bounds, F_bounds
	if descriptor
		[R_bounds_A_tilde, R_bounds_b_tilde, K_bounds_A_tilde, K_bounds_b_tilde, F_bounds_A_tilde, F_bounds_b_tilde, RKF_bounds_A_tilde, RKF_bounds_b_tilde] = transform_RKF_constraints(R_bounds_A, R_bounds_b, K_bounds_A, K_bounds_b, F_bounds_A, F_bounds_b, RKF_bounds_A, RKF_bounds_b, N_E, C_dot, C_dot_tilde, number_controls, number_controls_tilde, number_states, number_states_tilde, number_references_tilde, number_measurements_xdot, number_measurements_xdot_tilde, 'bounds');
	else
		R_bounds_A_tilde = R_bounds_A;
		R_bounds_b_tilde = R_bounds_b;
		K_bounds_A_tilde = K_bounds_A;
		K_bounds_b_tilde = K_bounds_b;
		F_bounds_A_tilde = F_bounds_A;
		F_bounds_b_tilde = F_bounds_b;
		RKF_bounds_A_tilde = RKF_bounds_A;
		RKF_bounds_b_tilde = RKF_bounds_b;
	end
	R_bounds_tilde = {
		reshape(R_bounds_A_tilde.', number_controls_tilde, number_states_tilde, size(R_bounds_A_tilde, 1)), R_bounds_b_tilde
	};
	K_bounds_tilde = { % K is fixed to 0 in any case
		reshape(K_bounds_A_tilde.', number_controls_tilde, number_measurements_xdot_tilde, size(K_bounds_A_tilde, 1)), K_bounds_b_tilde
	};
	F_bounds_tilde = {
		reshape(F_bounds_A_tilde.', number_controls_tilde, number_references_tilde, size(F_bounds_A_tilde, 1)), F_bounds_b_tilde
	};
	RKF_bounds_tilde = {
		reshape(RKF_bounds_A_tilde.', number_controls_tilde, number_states_tilde + number_measurements_xdot_tilde + number_references_tilde, size(RKF_bounds_A_tilde, 1)), RKF_bounds_b_tilde
	};

	%% transform R_nonlin
	if descriptor
		T_nonlin = struct(...% r = R_A*r_tilde + R_b, k = K_A*k_tilde + K_b, f = F_A*f_tilde + F_b
			'R_A',						T_R,...
			'R_b',						zeros(number_controls*number_states),...
			'K_A',						T_K,...
			'K_b',						zeros(number_controls*number_measurements_xdot, 1),...
			'F_A',						T_F,...
			'F_b',						zeros(number_controls*number_references, 1),...
			'number_states_original',	number_states,...
			'number_controls_original',	number_controls,...
			'transform',				true...
		);
		R_nonlin = checkandtransform_gain_nonlin(R_nonlin, number_controls_tilde, number_states_tilde, number_measurements_xdot, number_controls, T_nonlin);
	end

	%% check and transform R_0
	if nargin <= 4
		R_0 = zeros(number_controls, number_states);
	end
	if isa(R_0, 'control.design.gamma.InitialValueElement')
		R_0 = control.design.gamma.InitialValue(R_0);
	end
	if isa(R_0, 'control.design.gamma.InitialValue')
		initialoptions = getinitialoptions(struct(), objectiveoptions);
		if isempty(fieldnames(initialoptions))
			initialoptions = [];
		end
		R_0 = R_0.getall(initialoptions, systems, areafun, weights, R_fixed, R_bounds, R_nonlin);
	end
	[R_0, K_0, F_0] = input_initial_value2RKF(R_0);
	% TODO: change checks to checkinitialRKF (not possible until dimensions_strict is available in this function)
	if any([size(K_0, 3), size(F_0, 3)] ~= size(R_0, 3))
		error('control:design:gamma:dimensions', 'There must be an equal number of initial values for proportional, derivative and prefilter gains.');
	else
		number_initials = size(R_0, 3);
	end
	if size(R_0, 1) ~= number_controls || size(R_0, 2) ~= number_states
		error('control:design:gamma:dimensions', 'Initial value for proportional gain must be a %dX%d matrix. Specify third dimension for multiple initial values.', number_controls, number_states);
	end
	if ~isempty(K_0)
		if size(K_0, 1) ~= number_states || size(K_0, 2) ~= number_states
			error('control:design:gamma:dimensions', 'Initial value for differential gain must be a %dX%d matrix. Specify third dimension for multiple initial values.', number_states, number_states);
		end
		if any(K_0(:) ~= 0)
			error('control:design:gamma:input', 'Initial value for differential gain must be empty or a zero %dX%d matrix. Specify third dimension for multiple initial values.', number_states, number_states);
		end
	end
	if isempty(F_0)
		error('control:design:gamma:input', 'Initial value for prefilter gain must be provided.');
	else
		if size(F_0, 1) ~= number_controls || size(F_0, 2) ~= number_references
			error('control:design:gamma:dimensions', 'Initial value for prefiltergain  must be a %dX%d matrix. Specify third dimension for multiple initial values.', number_controls, number_references);
		end
	end

	if descriptor
		% controller and prefilter both in original form --> need to be transformed.
		R_0_tilde = NaN(number_controls_tilde, number_states_tilde, number_initials);
		F_0_tilde = NaN(number_controls_tilde, number_references_tilde, number_initials);
		parfor ii = 1:number_initials
			R_0_tmp = reshape(R_0(:, :, ii), number_controls*number_states, 1);
			F_0_tmp = reshape(F_0(:, :, ii), number_controls*number_references, 1);
			R_0_tmp_tilde = pinv(T_R)*R_0_tmp;
			F_0_tmp_tilde = pinv(T_F)*F_0_tmp;

			R_0_tilde(:, :, ii) = reshape(R_0_tmp_tilde, number_controls_tilde, number_states_tilde);
			F_0_tilde(:, :, ii) = reshape(F_0_tmp_tilde, number_controls_tilde, number_references_tilde);
		end
	else
		R_0_tilde = R_0;
		F_0_tilde = F_0;
	end
	RKF_0_tilde = {
		R_0_tilde, zeros(number_controls_tilde, number_measurements_xdot, number_initials), F_0_tilde
	};

	%% prepare for gammasyn
	if numeric
		% check numeric options
		if ~any(solveroptions.ProblemType == [
			optimization.options.ProblemType.CONSTRAINED;
			optimization.options.ProblemType.CONSTRAINEDMULTI
		])
			% TODO: allow unconstrained by transforming to objective function?
			error('control:design:gamma:input', 'For numeric coupling controller design, ''ProblemType'' must be CONSTRAINED or CONSTRAINEDMULTI.');
		end
		R_fixed_tilde = {
			R_fixed_tilde, K_fixed_tilde, F_fixed_tilde, RKF_fixed_tilde
		};
		R_bounds_tilde = {
			R_bounds_tilde, K_bounds_tilde, F_bounds_tilde, RKF_bounds_tilde
		};
	else
		% calculate structural constraints
		[RF_fixed_tilde, RF_bounds_tilde, valid, message] = coupling_RKF_fixed(systems_tilde, objectiveoptions, solveroptions, descriptor);
		if ~valid % then abort control design
			Ropt = {
				NaN(number_controls, number_states);
				NaN(number_controls, number_measurements_xdot);
				NaN(number_controls, number_controls);
			};
			Jopt = NaN;
			information = optimization.options.Options.OUTPUTPROTOTYPEMULTIPLE;
			information.message = message;
			return;
		end
		R_fixed_tilde_all = cat_RKF_fixed(R_fixed_tilde, RF_fixed_tilde{1});
		K_fixed_tilde_all = cat_RKF_fixed(K_fixed_tilde, RF_fixed_tilde{2});
		F_fixed_tilde_all = cat_RKF_fixed(F_fixed_tilde, RF_fixed_tilde{3});

		R_bounds_tilde_all = cat_RKF_fixed(R_bounds_tilde, RF_bounds_tilde{1});
		K_bounds_tilde_all = cat_RKF_fixed(K_bounds_tilde, RF_bounds_tilde{2});
		F_bounds_tilde_all = cat_RKF_fixed(F_bounds_tilde, RF_bounds_tilde{3});

		R_fixed_tilde = {
			R_fixed_tilde_all, K_fixed_tilde_all, F_fixed_tilde_all, RKF_fixed_tilde
		};
		R_bounds_tilde = {
			R_bounds_tilde_all, K_bounds_tilde_all, F_bounds_tilde_all, RKF_bounds_tilde
		};
	end

	%% gammasyn
	if nargin == 9
		[Ropt, Jopt, information] = control.design.gamma.gammasyn(systems_tilde, areafun, weights, R_fixed_tilde, RKF_0_tilde, solveroptions, objectiveoptions, R_bounds_tilde, R_nonlin);
	elseif nargin == 8
		[Ropt, Jopt, information] = control.design.gamma.gammasyn(systems_tilde, areafun, weights, R_fixed_tilde, RKF_0_tilde, solveroptions, objectiveoptions, R_bounds_tilde);
	elseif nargin >= 6
		[Ropt, Jopt, information] = control.design.gamma.gammasyn(systems_tilde, areafun, weights, R_fixed_tilde, RKF_0_tilde, solveroptions, objectiveoptions);
	else
		[Ropt, Jopt, information] = control.design.gamma.gammasyn(systems_tilde, areafun, weights, R_fixed_tilde, RKF_0_tilde, [], objectiveoptions);
	end

	%% transform result back into original coordinates
	R_tilde = Ropt{1};
	if size(Ropt, 1) == 3
		K_tilde = Ropt{2};
		if any(any(K_tilde ~= zeros(number_controls_tilde, number_measurements_xdot_tilde)))
			error('control:design:gamma:derivative_gain', 'For coupling control design, derivative gain must be zero.');
		end
		F_tilde = Ropt{3};
	else
		F_tilde = Ropt{2};
	end
	if descriptor
		F	= F_tilde(end - number_controls + 1:end, 1:number_references);
		R_r = R_tilde(end - number_controls + 1:end, :);
		R_trans = [
			R_r, zeros(number_controls, number_states - number_states_tilde)
		];
		R = R_trans*N_E';
		K = zeros(number_controls, number_measurements_xdot);

		% The following check has to be done because it has not been proven, that the backtransform is independent of the system matrices used for the transformation.
		round_to = 6;
		flag = false(number_models, 1);
		flag_unstable = false(number_models, 1);
		parfor ii = 1:number_models
			eig_trans = eig(systems_tilde(ii).A - systems_tilde(ii).B*R_tilde);
			eig_orig  = eig(systems(ii).A - systems(ii).B*R, systems(ii).E);
			eig_orig  = eig_orig(~isinf(eig_orig));
			if length(setdiff(round(eig_trans, round_to), round(eig_orig, round_to))) ~= length(eig_trans) - length(eig_orig)
				flag(ii, 1) = true;
			end
			if any(eig_orig >= 0)
				flag_unstable(ii, 1) = true;
			end
		end
		if any(flag(:))
			warning('control:design:gamma:DAE', 'Eigenvalues of transformed systems rounded to %d decimal places do not equal eigenvalues of DAE system.', round_to);
			if any(flag_unstable(:))
				warning('control:design:gamma:DAE', 'Controller cannot stabilize all DAE systems.');
			end
		else
			if output_verbosity(solveroptions)
				% TODO: change success message in dependence of exitflag of optimizer?
				fprintf('Controller design for DAE successful.\nAll eigenvalues of the transformed systems rounded to %d decimal places equal eigenvalues of DAE system.\n', round_to');
			end
		end
		Ropt = {
			R, K, F
		};
	end

end

function [R_A_tilde, R_b_tilde, K_A_tilde, K_b_tilde, F_A_tilde, F_b_tilde, RKF_A_tilde, RKF_b_tilde, T_R, T_K, T_F] = transform_RKF_constraints(R_A, R_b, K_A, K_b, F_A, F_b, RKF_A, RKF_b, N_E, C_dot, C_dot_tilde, number_controls, number_controls_tilde, number_states, number_states_tilde, number_references_tilde, number_measurements_xdot, number_measurements_xdot_tilde, type_string)
	%TRANSFORM_RKF_CONSTRAINTS transformes constraint equation systems to tilde form
	%	Input:
	% 		R_A:							constraint matrix for proportional gain (R_A*vec(R) = R_b)
	% 		R_b:							constraint vector for proportional gain (R_A*vec(R) = R_b)
	% 		K_A:							constraint matrix for derivative gain (K_A*vec(K) = K_b)
	% 		K_b:							constraint vector for derivative gain (K_A*vec(K) = K_b)
	% 		F_A:							constraint matrix for prefilter gain (F_A*vec(F) = F_b)
	% 		F_b:							constraint vector for prefilter gain (F_A*vec(F) = F_b)
	% 		RKF_A:							constraint matrix for combined gain (RKF_A*vec(RKF) = RKF_b)
	% 		RKF_b:							constraint vector for combined gain (RKF_A*vec(RKF) = RKF_b)
	% 		N_E:							part of svd(E): E = U_E*Sigma*N_E'. Same for every system.
	% 		C_dot:							Differential output. Same for every system.
	%		C_dot_tilde:					Differential output of transformed systems. Same for every system.
	% 		number_controls:				number of control signals
	% 		number_controls_tilde:			number of control signals of transformed systems
	% 		number_states:					number of states
	% 		number_states_tilde:			number of states in transformed system
	%		number_references_tilde:		number of references in transformed system
	% 		number_measurements_xdot:		number of differential measurements
	% 		number_measurements_xdot_tilde:	number of differential measurements of transformed systems
	%		type_string:					indicator 'fixed' or 'bounds' whether constraints_fixed or constraints_bounds are processed.
	%	Output:
	% 		R_A_tilde:						transformed constraint matrix for proportional gain (R_A_tilde*vec(R_tilde) = R_b_tilde)
	% 		R_b_tilde:						transformed constraint matrix for proportional gain (R_A_tilde*vec(R_tilde) = R_b_tilde)
	% 		K_A_tilde:						transformed constraint matrix for derivative gain (K_A_tilde*vec(K_tilde) = K_b_tilde)
	% 		K_b_tilde:						transformed constraint matrix for derivative gain (K_A_tilde*vec(K_tilde) = K_b_tilde)
	% 		F_A_tilde:						transformed constraint matrix for prefilter gain (F_A_tilde*vec(F_tilde) = F_b_tilde)
	% 		F_b_tilde:						transformed constraint matrix for prefilter gain (F_A_tilde*vec(F_tilde) = F_b_tilde)
	% 		RKF_A_tilde:					transformed constraint matrix for combined gain (RKF_A_tilde*vec(RKF_tilde) = RKF_b_tilde)
	% 		RKF_b_tilde:					transformed constraint matrix for combined gain (RKF_A_tilde*vec(RKF_tilde) = RKF_b_tilde)
	% 		T_R:							transformation matrix for proportional gain
	% 		T_K:							transformation matrix for derivative gain
	% 		T_F:							transformation matrix for prefilter gain

	% In original coordinates: u = -R*x + K*C_dot*x_dot+ F*w
	% Transformation of DAE system to semi-explicit form uses [x1; x2] = N_E'*x <=> x = N_E*[x1; x2]
	% Definition of N_E = [N_E1, N_E2] with N_E1 having number_states_tilde columns yields
	% u = - R*N_E1*x1 - R*N_E2*x2 + K*C_dot*N_E1*x1_dot + K*C_dot*N_E2*x2_dot + F*w
	% where R*N_E1 and K*C_dot*N_E1 are interpreted as parts of R_tilde and K_tilde (u_tilde = -R_tilde*x1 + K_tilde*C_dot_tilde*x1 + F_tilde*w_tilde with u_tilde = [x2; u]).
	% The terms R*N_E2 and K*C_dot*N_E2 are neglected for the calculations.
	% Thus:
	% R_tilde(end - number_controls + 1:end) = R*N_E1
	% K_tilde(end - number_controls + 1:end) = K*C_dot*N_E1
	% F_tilde(end - number_controls + 1:end, 1:number_references) = F
	% From this, expressions for vec(R), vec(K), vec(F) in dependence of vec(R_tilde), vec(K_tilde), vec(F_tilde) are calculated.
	% Using the notation from below yields:
	% vec(R) =  N_E1_pinv(eye(number_states_tilde))*row_choice_kron(eye(number_states_tilde))*vec(R_tilde) =: T_R*vec(R_tilde)
	% vec(K) =  N_E1_pinv(C_dot.')*row_choice_kron(C_dot_tilde.')*vec(K_tilde) =: T_K*vec(K_tilde)
	% vec(F) =: T_F*vec(F_tilde)
	N_E1 = N_E(:, 1:number_states_tilde);
	function [times] = N_E1_pinv(x)
		times = pinv(kron(N_E1.'*x, eye(number_controls)));
	end
	row_choice = [
		zeros(number_controls, number_states - number_states_tilde), eye(number_controls)
	];
	row_choice_kron = @(x) kron(x, row_choice);
	T_R = N_E1_pinv(eye(number_states))*row_choice_kron(eye(number_states_tilde));
	T_K = N_E1_pinv(C_dot.')*row_choice_kron(C_dot_tilde.');
	T_F  = kron([
			eye(number_controls), zeros(number_controls, number_references_tilde - number_controls)
		], [
			zeros(number_controls, number_states - number_states_tilde), eye(number_controls)
	]);
	R_A_tilde_non_un = R_A*T_R;
	K_A_tilde_non_un = K_A*T_K;
	F_A_tilde_non_un = F_A*T_F;

	RKF_R_A_tilde_non_un = RKF_A(:, 1:number_states*number_controls)*T_R;
	RKF_K_A_tilde_non_un = RKF_A(:, number_states*number_controls + (1:number_controls*number_measurements_xdot))*T_K;
	RKF_F_A_tilde_non_un = RKF_A(:, end - number_controls^2 + 1:end)*T_F;

	RKF_A_tilde_non_un = [
		RKF_R_A_tilde_non_un, RKF_K_A_tilde_non_un, RKF_F_A_tilde_non_un
	];

	R_b_tilde_non_un = R_b;
	K_b_tilde_non_un = K_b;
	F_b_tilde_non_un = F_b;
	RKF_b_tilde_non_un = RKF_b;

	if strcmpi(type_string, 'bounds') && false
		% add default bounds for tilde gain matrices
		R_A_tilde_non_un = [
			eye(number_controls_tilde*number_states_tilde);
			-eye(number_controls_tilde*number_states_tilde);
			R_A_tilde_non_un
		];
		K_A_tilde_non_un = [
			eye(number_controls_tilde*number_measurements_xdot_tilde);
			-eye(number_controls_tilde*number_measurements_xdot_tilde);
			K_A_tilde_non_un
		];
		F_A_tilde_non_un = [
			eye(number_controls_tilde*number_references_tilde);
			-eye(number_controls_tilde*number_references_tilde);
			F_A_tilde_non_un
		];
		RKF_A_tilde_non_un = [
			eye(number_controls_tilde*(number_states_tilde + number_measurements_xdot + number_references_tilde));
			-eye(number_controls_tilde*(number_states_tilde + number_measurements_xdot + number_references_tilde));
			RKF_A_tilde_non_un
		];
		R_b_tilde_non_un = [
			Inf(2*number_controls_tilde*number_states_tilde, 1);
			R_b_tilde_non_un
		];
		K_b_tilde_non_un = [
			Inf(2*number_controls_tilde*number_measurements_xdot_tilde, 1);
			K_b_tilde_non_un
		];
		F_b_tilde_non_un = [
			Inf(2*number_controls_tilde*number_references_tilde, 1);
			F_b_tilde_non_un
		];
		RKF_b_tilde_non_un = [
			Inf(2*number_controls_tilde*(number_states_tilde + number_measurements_xdot + number_references_tilde), 1);
			RKF_b_tilde_non_un
		];
	end

	% Due to the non-unique (*_non_un) transformation between (.) and (.)_tilde quantities, redundant constraints may occur.
	% Delete these (=> *_un). Also delete zero rows.
	% unique rows
	[~, idx_R] = unique([R_A_tilde_non_un, R_b_tilde_non_un], 'rows');
	[~, idx_K] = unique([K_A_tilde_non_un, K_b_tilde_non_un], 'rows');
	[~, idx_F] = unique([F_A_tilde_non_un, F_b_tilde_non_un], 'rows');
	[~, idx_RKF] = unique([RKF_A_tilde_non_un, RKF_b_tilde_non_un], 'rows');
	R_A_tilde_un = R_A_tilde_non_un(idx_R, :);
	K_A_tilde_un = K_A_tilde_non_un(idx_K, :);
	F_A_tilde_un = F_A_tilde_non_un(idx_F, :);
	RKF_A_tilde_un = RKF_A_tilde_non_un(idx_RKF, :);
	R_b_tilde_un = R_b_tilde_non_un(idx_R);
	K_b_tilde_un = K_b_tilde_non_un(idx_K);
	F_b_tilde_un = F_b_tilde_non_un(idx_F);
	RKF_b_tilde_un = RKF_b_tilde_non_un(idx_RKF);

	% no zero rows
	zero_rows_R =	all(R_A_tilde_un == 0, 2);
	zero_rows_K =	all(K_A_tilde_un == 0, 2);
	zero_rows_F =	all(F_A_tilde_un == 0, 2);
	zero_rows_RKF = all(RKF_A_tilde_un == 0, 2);

	if strcmpi(type_string, 'fixed')
		if all(R_b_tilde_un(zero_rows_R) == 0) % solvable
			R_A_tilde = R_A_tilde_un(~zero_rows_R, :);
			R_b_tilde = R_b_tilde_un(~zero_rows_R);
		else
			error('control:design:gamma:constraints', 'By the DAE transformation to semi-explicit form, unsolvable constraint for fixed proportional gain is formed.');
		end
		if all(K_b_tilde_un(zero_rows_K) == 0) % solvable
			K_A_tilde = K_A_tilde_un(~zero_rows_K, :);
			K_b_tilde = K_b_tilde_un(~zero_rows_K);
		else
			error('control:design:gamma:constraints', 'By the DAE transformation to semi-explicit form, unsolvable constraint for fixed derivative gain is formed.');
		end
		if all(F_b_tilde_un(zero_rows_F) == 0) % solvable
			F_A_tilde = F_A_tilde_un(~zero_rows_F, :);
			F_b_tilde = F_b_tilde_un(~zero_rows_F);
		else
			error('control:design:gamma:constraints', 'By the DAE transformation to semi-explicit form, unsolvable constraint for fixed prefilter gain is formed.');
		end
		if all(RKF_b_tilde_un(zero_rows_RKF) == 0) % solvable
			RKF_A_tilde = RKF_A_tilde_un(~zero_rows_RKF, :);
			RKF_b_tilde = RKF_b_tilde_un(~zero_rows_RKF);
		else
			error('control:design:gamma:constraints', 'By the DAE transformation to semi-explicit form, unsolvable constraint for fixed combined gain is formed.');
		end
	elseif strcmp(type_string, 'bounds')
		if all(R_b_tilde_un(zero_rows_R) > 0) % solvable
			R_A_tilde = R_A_tilde_un(~zero_rows_R, :);
			R_b_tilde = R_b_tilde_un(~zero_rows_R);
		else
			error('control:design:gamma:constraints', 'By the DAE transformation to semi-explicit form, unsolvable constraint for fixed proportional gain is formed.');
		end
		if all(K_b_tilde_un(zero_rows_K) > 0) % solvable
			K_A_tilde = K_A_tilde_un(~zero_rows_K, :);
			K_b_tilde = K_b_tilde_un(~zero_rows_K);
		else
			error('control:design:gamma:constraints', 'By the DAE transformation to semi-explicit form, unsolvable constraint for fixed derivative gain is formed.');
		end
		if all(F_b_tilde_un(zero_rows_F) > 0) % solvable
			F_A_tilde = F_A_tilde_un(~zero_rows_F, :);
			F_b_tilde = F_b_tilde_un(~zero_rows_F);
		else
			error('control:design:gamma:constraints', 'By the DAE transformation to semi-explicit form, unsolvable constraint for fixed prefilter gain is formed.');
		end
		if all(RKF_b_tilde_un(zero_rows_RKF) > 0) % solvable
			RKF_A_tilde = RKF_A_tilde_un(~zero_rows_RKF, :);
			RKF_b_tilde = RKF_b_tilde_un(~zero_rows_RKF);
		else
			error('control:design:gamma:constraints', 'By the DAE transformation to semi-explicit form, unsolvable constraint for fixed combined gain is formed.');
		end
	else
		error('control:design:gamma:input', 'Wrong type_string. Type ''fixed'' or ''bounds''.');
	end
end

function [RKF_fixed_tilde_all] = cat_RKF_fixed(RKF_fixed_tilde, RKF_fixed_tilde_coupling)
	%CAT_RKF_fixed concatenate constraint equation systems
	%	Input:
	%		RKF_fixed_tilde:			user defined equation system
	%		RKF_fixed_tilde_coupling:	coupling condition equation system
	%	Output:
	%		RKF_fixed_tilde_all:		combined equation system
	if isempty(RKF_fixed_tilde)
		RKF_fixed_tilde = {[], []};
	end
	if isempty(RKF_fixed_tilde_coupling)
		RKF_fixed_tilde_coupling = {[], []};
	end
	if ~isempty(RKF_fixed_tilde{1}) && ~isempty(RKF_fixed_tilde_coupling{1})
		RKF_fixed_tilde_all = {cat(3, RKF_fixed_tilde{1}, RKF_fixed_tilde_coupling{1}), [
			RKF_fixed_tilde{2};
			RKF_fixed_tilde_coupling{2}
		]};
	elseif isempty(RKF_fixed_tilde{1}) && ~isempty(RKF_fixed_tilde_coupling{1})
		RKF_fixed_tilde_all = RKF_fixed_tilde_coupling;
	elseif isempty(RKF_fixed_tilde_coupling{1}) && ~isempty(RKF_fixed_tilde{1})
		RKF_fixed_tilde_all = RKF_fixed_tilde;
	else
		RKF_fixed_tilde_all = RKF_fixed_tilde_coupling;
	end
end
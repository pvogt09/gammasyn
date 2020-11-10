function [system, areafun_strict, areafun_loose, weight_strict, weight_loose, dimensions_strict, dimensions_loose, number_states_all, bounds, nonlcon] = checkandtransformargs(systems, areafun, weight, systemoptions, R_fixed, K_fixed, F_fixed, RKF_fixed, allowdifferentorder, allownegativeweight, R_bounds, K_bounds, F_bounds, RKF_bounds, R_nonlin)
	%CHECKANDTRANSFORMARGS check and convert input arguments for gammasyn into faster data structures to use with the objective function
	%	Input:
	%		systems:				structure/cell array or matrix with dynamic systems to take into consideration
	%		areafun:				area border functions and gradients as cell array with number of system entries, each function returns a row vector with as much elements as area border functions are defined for the corresponding system
	%		weight:					weighting matrix with number of systems columns and number of pole area border functions rows
	%		systemoptions:			structure with options for multiple models
	%		R_fixed:				cell array with indicator matrix for proportional gain elements that should be fixed and the values the fixed gains have or 3D matrix where fixed elements are the non zero elements in the first and second dimension or (expreimental) cell array with symbolic expression for the gain coefficients in the first dimension and equation system in the second
	%		K_fixed:				cell array with indicator matrix for derivative gain elements that should be fixed and the values the fixed gains have or 3D matrix where fixed elements are the non zero elements in the first and second dimension or (expreimental) cell array with symbolic expression for the gain coefficients in the first dimension and equation system in the second
	%		F_fixed:				cell array with indicator matrix for prefilter elements that should be fixed and the values the fixed gains have or 3D matrix where fixed elements are the non zero elements in the first and second dimension or (expreimental) cell array with symbolic expression for the prefilter coefficients in the first dimension and equation system in the second
	%		RKF_fixed:				cell array with indicator matrix for all gain elements that should be fixed and the values the fixed gains have or 3D matrix where fixed elements are the non zero elements in the first and second dimension or (expreimental) cell array with symbolic expression for all gain coefficients in the first dimension and equation system in the second
	%		allowvarorder:			indicator, if systems with different numbers of states are allowed
	%		allownegativeweight:	indicator, if negative weights are allowed
	%		R_bounds:				cell array with lower and upper bounds for proportional gain elements that should be bounded or 3D matrix where lower bounds are in the first plane in the third dimension and upper bounds in the second or a cell array with a 3D matrix of linear dependent bound expressions A*x <= b and the upper bounds in the second dimension of the cell array or (experimental) cell array with symbolic expression for the gain coefficients in the first dimension and equation system in the second
	%		K_bounds:				cell array with lower and upper bounds for derivative gain elements that should be bounded or 3D matrix where lower bounds are in the first plane in the third dimension and upper bounds in the second or a cell array with a 3D matrix of linear dependent bound expressions A*x <= b and the upper bounds in the second dimension of the cell array or (experimental) cell array with symbolic expression for the gain coefficients in the first dimension and equation system in the second
	%		F_bounds:				cell array with lower and upper bounds for prefilter elements that should be bounded or 3D matrix where lower bounds are in the first plane in the third dimension and upper bounds in the second or a cell array with a 3D matrix of linear dependent bound expressions A*x <= b and the upper bounds in the second dimension of the cell array or (experimental) cell array with symbolic expression for the prefilter coefficients in the first dimension and equation system in the second
	%		RKF_bounds:				cell array with lower and upper bounds for all gain elements that should be bounded or 3D matrix where lower bounds are in the first plane in the third dimension and upper bounds in the second or a cell array with a 3D matrix of linear dependent bound expressions A*x <= b and the upper bounds in the second dimension of the cell array or (experimental) cell array with symbolic expression for all gain coefficients in the first dimension and equation system in the second
	%		R_nonlin:				function pointer with signature [c_R, ceq_R, c_K, ceq_K, c_F, ceq_F, gradc_R, gradceq_R, gradc_K, gradceq_K, gradc_F, gradceq_F] = constraint(R, K, F)
	%	Output:
	%		system:					structure with system matrices of systems to take into consideration
	%		areafun_strict:			area border functions and gradients as cell array with number of system entries, each function returns a row vector with as much elements as area border functions are defined for the corresponding system for strict optimization of areafunctions
	%		areafun_loose:			area border functions and gradients as cell array with number of system entries, each function returns a row vector with as much elements as area border functions are defined for the corresponding system for loose optimization of areafunctions
	%		weight_strict:			weighting matrix with number of systems columns and number of pole area border functions rows for strict optimization of areafunctions
	%		weight_loose:			weighting matrix with number of systems columns and number of pole area border functions rows for loose optimization of areafunctions
	%		dimensions_strict:		structure with information about dimensions of the variables and systems and fixed gain parameters for strict optimization of areafunctions
	%		dimensions_loose:		structure with information about dimensions of the variables and systems and fixed gain parameters for loose optimization of areafunctions
	%		number_states_all:		vector with orders of all systems
	%		bounds:					structure with information about bounded gain parameters for optimization
	%		nonlcon:				structure with information about nonlinear gain constraints
	if nargin <= 3
		systemoptions = struct();
	end
	if nargin <= 8
		allowdifferentorder = false;
	end
	if nargin <= 9
		allownegativeweight = false;
	end
	if ~isscalar(allowdifferentorder) || ~islogical(allowdifferentorder)
		error('control:design:gamma:dimension', 'Indicator for different order of systems must be a logical scalar, not a ''%s''.', class(allowdifferentorder));
	end
	if ~isscalar(allownegativeweight) || ~islogical(allownegativeweight)
		error('control:design:gamma:dimension', 'Indicator for negative weights must be a logical scalar, not a ''%s''.', class(allownegativeweight));
	end
	if ~iscell(systems) && ~isstruct(systems)
		systems = {systems};
	end
	if (iscell(systems) || isstruct(systems)) && isrow(systems)
		systems = systems.';
	end
	if isa(systemoptions, 'control.design.gamma.GammasynOptions')
		systemoptions = struct(systemoptions);
		systemoptions = systemoptions.system;
	end
	if ~isfield(systemoptions, 'decouplingcontrol')
		systemoptions.decouplingcontrol = false;
	end
	if ~isscalar(systemoptions.decouplingcontrol)
		error('control:design:gamma:dimension', 'Indicator for decoupling controller design must be scalar.');
	end
	if ~islogical(systemoptions.decouplingcontrol)
		error('control:design:gamma:dimension', 'Indicator for decoupling controller design must be of type ''logical'', not ''%s''.', class(systemoptions.decouplingcontrol));
	end
	[system, number_states, number_states_all, number_controls, number_measurements, number_measurements_xdot, number_references, descriptor, number_descriptors_all, ~, sample_time, expanded_models] = checkandtransformsystems(systems, systemoptions);
	if ~allowdifferentorder && any(number_states_all(1) ~= number_states_all)
		error('control:design:gamma:dimension', 'All systems must have %d states.', number_states_all(1));
	end
	if ~allowdifferentorder && any(number_descriptors_all(1) ~= number_descriptors_all)
		error('control:design:gamma:dimension', 'All systems must have %d descriptors.', number_descriptors_all(1));
	end
	if ~allowdifferentorder && any(number_states_all(1) ~= number_descriptors_all)
		error('control:design:gamma:dimension', 'All systems must have %d descriptors and states.', number_states_all(1));
	end
	number_models = size(system, 1);
	number_systems = size(systems, 1);
	% check area border functions for validity
	[...
		number_areaargs_strict, area_parts_strict, number_areas_max_strict, area_hasgrad_strict, area_hashess_strict, area_parameters_strict, areafun_strict,...
		number_areaargs_loose, area_parts_loose, number_areas_max_loose, area_hasgrad_loose, area_hashess_loose, area_parameters_loose, areafun_loose...
	] = checkandtransform_areas(areafun, number_models, number_systems, expanded_models);
	% check weights for validity
	[weight_strict, weight_loose] = checkandtransform_weight(weight, allownegativeweight, number_models, number_systems, expanded_models, areafun_strict, number_areas_max_strict, areafun_loose, number_areas_max_loose);
	if systemoptions.decouplingcontrol
		if ~configuration.control.design.hasGeometricApproachToolbox()
			% VSTAR function needs "Geometric Approach Toolbox" installed
			% http://www3.deis.unibo.it/Staff/FullProf/GiovanniMarro/geometric.htm
			error('control:design:gamma:GeomtricApproachToolboxMissing', 'Decoupling controller synthesis needs ''Geometric Approach Toolbox'' installed. See http://www3.deis.unibo.it/Staff/FullProf/GiovanniMarro/geometric.htm');
		end
		tf_structure = systemoptions.tf_structure;
		number_decouplingconditions = int32(sum(tf_structure == 0, 1)).';
		m_invariant_mat = zeros(number_models, number_references);
		hasfeedthrough_decoupling_mat = false(number_models, number_references);
		not_con_invariant_mat = zeros(number_models, number_references);
		parfor ii = 1:number_models
			E = system(ii).E;
			A = system(ii).A;
			B = system(ii).B;
			C = system(ii).C;
			C_ref = system(ii).C_ref;
			D_ref = system(ii).D_ref;
			kerC = null(C);
			for jj = 1:number_references
				Cjj = C_ref(tf_structure(:, jj) == 0, :); %#ok<PFBNS>
				Djj = D_ref(tf_structure(:, jj) == 0, :);
				hasfeedthrough_decoupling_mat(ii, jj) = any(Djj(:) ~= 0);
				if hasfeedthrough_decoupling_mat(ii, jj)
					Q = vstar(E\A, E\B, Cjj, Djj);
					m_invariant_mat(ii, jj) = size(Q, 2);
				else
					Q = mainco(E\A, E\B, null(Cjj));
					m_invariant_mat(ii, jj) = size(Q, 2);
				end
				% check if output nulling controlled invariant subspace is also input containing conditioned invariant
				if size(kerC, 2) > 0 % only in this case, Q might not be conditioned invariant
					if rank([A*ints(Q, kerC), Q]) > rank(Q) % not the right condition for systems with feedthrough
						% controlled invariant subspace is not conditioned invariant
						not_con_invariant_mat(ii, jj) = true;
					end
				end
			end
		end
		hasfeedthrough_decoupling = any(hasfeedthrough_decoupling_mat, 1).';
		if any(any(m_invariant_mat ~= repmat(m_invariant_mat(1, :), number_models, 1)))
			error('control:design:gamma:dimension', 'Controlled invariant subspace must have same dimension for every system.')
		else
			m_invariant = m_invariant_mat(1, :).';
		end
		if any(hasfeedthrough_decoupling) && systemoptions.allowoutputdecoupling && number_measurements < number_states
			warning('control:design:gamma:decoupling', 'Using feedthrough decoupling and outputfeedback is "beta" since in this case, controlled invariant subspace is not properly checked for being conditioned invariant.');
		end
		if any(any(not_con_invariant_mat))
			warning('control:design:gamma:decoupling', 'No suitable output feedback exists because at least one controlled invariant subspace is not conditioned invariant. Use different measurement configuration.');
		end
	else
		tf_structure = NaN(number_references, number_references);
		number_decouplingconditions = int32(zeros(number_references, 1));
		m_invariant = zeros(number_references, 1);
		hasfeedthrough_decoupling = false(number_references, 1);
	end
	% create uniform fixed constraints and check for validity
	if nargin <= 4 || isempty(R_fixed)
		R_fixed = {false(number_controls, number_measurements), zeros(number_controls, number_measurements)};
	end
	[R_fixed, constraint_system, constraint_border, rg, T, T_inv, hasfixed_R, onlyfixed_R, allfixed_R, constraint_system_hadamard_R, isforced2zero_R] = checkandtransform_gain_fixed(R_fixed, number_controls, number_measurements, 'proportional');
	if nargin <= 5 || isempty(K_fixed)
		K_fixed = {false(number_controls, number_measurements_xdot), zeros(number_controls, number_measurements_xdot)};
	end
	[K_fixed, constraint_system_xdot, constraint_border_xdot, rg_xdot, T_xdot, T_inv_xdot, hasfixed_K, onlyfixed_K, allfixed_K, constraint_system_hadamard_K, isforced2zero_K] = checkandtransform_gain_fixed(K_fixed, number_controls, number_measurements_xdot, 'derivative', true(number_controls, number_measurements_xdot));
	if nargin <= 6 || isempty(F_fixed)
		F_fixed = {false(number_controls, number_references), zeros(number_controls, number_references)};
	end
	[F_fixed, constraint_system_prefilter, constraint_border_prefilter, rg_prefilter, T_prefilter, T_inv_prefilter, hasfixed_F, onlyfixed_F, allfixed_F, constraint_system_hadamard_F, isforced2zero_F] = checkandtransform_gain_fixed(F_fixed, number_controls, number_references, 'prefilter');
	if nargin <= 7 || isempty(RKF_fixed)
		RKF_fixed = {false(number_controls, number_measurements + number_measurements_xdot + number_references), zeros(number_controls, number_measurements + number_measurements_xdot + number_references)};
	end
	K_pattern = false(number_controls, number_measurements + number_measurements_xdot + number_references);
	K_pattern(:, number_measurements + 1:number_measurements + number_measurements_xdot) = true;
	[RKF_fixed, constraint_system_RKF, constraint_border_RKF, rg_RKF, T_RKF, T_inv_RKF, hasfixed_RKF, onlyfixed_RKF, allfixed_RKF, constraint_system_hadamard_RKF, isforced2zero_RKF_K] = checkandtransform_gain_fixed(RKF_fixed, number_controls, number_measurements + number_measurements_xdot + number_references, 'combined', K_pattern);
	[...
		R_fixed, constraint_system, constraint_border, rg, T, T_inv, hasfixed_R, onlyfixed_R, allfixed_R, ~, ~,...
		K_fixed, constraint_system_xdot, constraint_border_xdot, rg_xdot, T_xdot, T_inv_xdot, hasfixed_K, onlyfixed_K, allfixed_K, ~, isforced2zero_K,...
		F_fixed, constraint_system_prefilter, constraint_border_prefilter, rg_prefilter, T_prefilter, T_inv_prefilter, hasfixed_F, onlyfixed_F, allfixed_F, ~, ~,...
		RKF_fixed, constraint_system_RKF, constraint_border_RKF, rg_RKF, T_RKF, T_inv_RKF, hasfixed_RKF, onlyfixed_RKF, allfixed_RKF, ~...
	] = checkandtransform_gain_fixed_RKF(...
		R_fixed, constraint_system, constraint_border, rg, T, T_inv, hasfixed_R, onlyfixed_R, allfixed_R, constraint_system_hadamard_R, isforced2zero_R,...
		K_fixed, constraint_system_xdot, constraint_border_xdot, rg_xdot, T_xdot, T_inv_xdot, hasfixed_K, onlyfixed_K, allfixed_K, constraint_system_hadamard_K, isforced2zero_K,...
		F_fixed, constraint_system_prefilter, constraint_border_prefilter, rg_prefilter, T_prefilter, T_inv_prefilter, hasfixed_F, onlyfixed_F, allfixed_F, constraint_system_hadamard_F, isforced2zero_F,...
		RKF_fixed, constraint_system_RKF, constraint_border_RKF, rg_RKF, T_RKF, T_inv_RKF, hasfixed_RKF, onlyfixed_RKF, allfixed_RKF, constraint_system_hadamard_RKF,...
		number_controls, number_measurements, number_measurements_xdot, number_references, false...
	);
	isforced2zero_R = false;
	isforced2zero_F = false;
	% create uniform bound constraints and check for validity
	if nargin <= 10 || isempty(R_bounds) || (iscell(R_bounds) && numel(R_bounds) >= 2 && isempty(R_bounds{1}) && isempty(R_bounds{2}))
		R_bounds = {-Inf(number_controls, number_measurements), Inf(number_controls, number_measurements)};
	end
	[R_bounds, bound_system, bound_border, rg_bounds, hasbounds_R, onlybounds_R, bound_system_hadamard_R] = checkandtransform_gain_bounds(R_bounds, number_controls, number_measurements, 'proportional');
	if nargin <= 11 || isempty(K_bounds) || (iscell(K_bounds) && numel(K_bounds) >= 2 && isempty(K_bounds{1}) && isempty(K_bounds{2}))
		K_bounds = {-Inf(number_controls, number_measurements_xdot), Inf(number_controls, number_measurements_xdot)};
	end
	[K_bounds, bound_system_xdot, bound_border_xdot, rg_xdot_bounds, hasbounds_K, onlybounds_K, bound_system_hadamard_K] = checkandtransform_gain_bounds(K_bounds, number_controls, number_measurements_xdot, 'proportional');
	if nargin <= 12 || isempty(F_bounds) || (iscell(F_bounds) && numel(F_bounds) >= 2 && isempty(F_bounds{1}) && isempty(F_bounds{2}))
		F_bounds = {-Inf(number_controls, number_references), Inf(number_controls, number_references)};
	end
	[F_bounds, bound_system_prefilter, bound_border_prefilter, rg_prefilter_bounds, hasbounds_F, onlybounds_F, bound_system_hadamard_F] = checkandtransform_gain_bounds(F_bounds, number_controls, number_references, 'prefilter');
	if nargin <= 13 || isempty(RKF_bounds)
		RKF_bounds = {-Inf(number_controls, number_measurements + number_measurements_xdot + number_references), Inf(number_controls, number_measurements + number_measurements_xdot + number_references)};
	end
	[RKF_bounds, bound_system_RKF, bound_border_RKF, rg_RKF_bounds, hasbounds_RKF, onlybounds_RKF, bound_system_hadamard_RKF] = checkandtransform_gain_bounds(RKF_bounds, number_controls, number_measurements + number_measurements_xdot + number_references, 'combined');
	[...
		R_bounds, bound_system, bound_border, rg_bounds, hasbounds_R, onlybounds_R, ~,...
		K_bounds, bound_system_xdot, bound_border_xdot, rg_xdot_bounds, hasbounds_K, onlybounds_K, ~,...
		F_bounds, bound_system_prefilter, bound_border_prefilter, rg_prefilter_bounds, hasbounds_F, onlybounds_F, ~,...
		RKF_bounds, bound_system_RKF, bound_border_RKF, rg_RKF_bounds, hasbounds_RKF, onlybounds_RKF, ~...
	] = checkandtransform_gain_bounds_RKF(...
		R_bounds, bound_system, bound_border, rg_bounds, hasbounds_R, onlybounds_R, bound_system_hadamard_R,...
		K_bounds, bound_system_xdot, bound_border_xdot, rg_xdot_bounds, hasbounds_K, onlybounds_K, bound_system_hadamard_K,...
		F_bounds, bound_system_prefilter, bound_border_prefilter, rg_prefilter_bounds, hasbounds_F, onlybounds_F, bound_system_hadamard_F,...
		RKF_bounds, bound_system_RKF, bound_border_RKF, rg_RKF_bounds, hasbounds_RKF, onlybounds_RKF, bound_system_hadamard_RKF,...
		number_controls, number_measurements, number_measurements_xdot, number_references, hasfixed_RKF...
	);
	if hasbounds_RKF && (hasbounds_R || hasbounds_K || hasbounds_F)
		error('control:design:gamma:dimension', 'Internal error in gain constraint handling for combined inequality constraints.');
	end
	if hasbounds_RKF && ~hasfixed_RKF
		% if any bound constraint is given in combined form, combined fixed constraints are also needed
		[...
			R_fixed, constraint_system, constraint_border, rg, T, T_inv, hasfixed_R, onlyfixed_R, allfixed_R, ~,...
			K_fixed, constraint_system_xdot, constraint_border_xdot, rg_xdot, T_xdot, T_inv_xdot, hasfixed_K, onlyfixed_K, allfixed_K, ~,...
			F_fixed, constraint_system_prefilter, constraint_border_prefilter, rg_prefilter, T_prefilter, T_inv_prefilter, hasfixed_F, onlyfixed_F, allfixed_F, ~,...
			RKF_fixed, constraint_system_RKF, constraint_border_RKF, rg_RKF, T_RKF, T_inv_RKF, hasfixed_RKF, onlyfixed_RKF, allfixed_RKF, ~...
		] = checkandtransform_gain_fixed_RKF(...
			R_fixed, constraint_system, constraint_border, rg, T, T_inv, hasfixed_R, onlyfixed_R, allfixed_R, constraint_system_hadamard_R,...
			K_fixed, constraint_system_xdot, constraint_border_xdot, rg_xdot, T_xdot, T_inv_xdot, hasfixed_K, onlyfixed_K, allfixed_K, constraint_system_hadamard_K,...
			F_fixed, constraint_system_prefilter, constraint_border_prefilter, rg_prefilter, T_prefilter, T_inv_prefilter, hasfixed_F, onlyfixed_F, allfixed_F, constraint_system_hadamard_F,...
			RKF_fixed, constraint_system_RKF, constraint_border_RKF, rg_RKF, T_RKF, T_inv_RKF, hasfixed_RKF, onlyfixed_RKF, allfixed_RKF, constraint_system_hadamard_RKF,...
			number_controls, number_measurements, number_measurements_xdot, number_references, hasbounds_RKF...
		);
	end
	% check for feasible equality constraints
	if hasfixed_RKF && (hasfixed_R || hasfixed_K || hasfixed_F)
		error('control:design:gamma:dimension', 'Internal error in gain constraint handling for combined equality constraints.');
	end
	if rg + rg_xdot + rg_prefilter > 0 && rg + rg_xdot + rg_prefilter >= number_controls*number_measurements + number_controls*number_measurements_xdot + number_controls*number_references
		error('control:design:gamma:dimension', 'At least one gain component must be unconstrained.');
	end
	if rg_RKF > 0 && rg_RKF >= number_controls*number_measurements + number_controls*number_measurements_xdot + number_controls*number_references
		error('control:design:gamma:dimension', 'At least one gain component must be unconstrained.');
	end
	if allfixed_R && allfixed_K && allfixed_F || allfixed_RKF
		error('control:design:gamma:dimension', 'At least one gain component must be unconstrained.');
	end
	index_free_R = false(number_controls*(number_measurements + number_measurements_xdot + number_references), 1);
	index_free_K = false(number_controls*(number_measurements + number_measurements_xdot + number_references), 1);
	index_free_F = false(number_controls*(number_measurements + number_measurements_xdot + number_references), 1);
	index_free_RKF = false(number_controls*(number_measurements + number_measurements_xdot + number_references), 1);
	index_free_R((size(constraint_border, 1) + 1):(number_controls*number_measurements), 1) = true;
	index_free_K((number_controls*number_measurements + size(constraint_border_xdot, 1) + 1):number_controls*(number_measurements + number_measurements_xdot), 1) = true;
	index_free_F((number_controls*(number_measurements + number_measurements_xdot) + size(constraint_border_prefilter, 1) + 1):number_controls*(number_measurements + number_measurements_xdot + number_references), 1) = true;
	index_free_RKF((size(constraint_border_RKF, 1) + 1):(number_controls*(number_measurements + number_measurements_xdot + number_references)), 1) = true;
	index_free_all = index_free_R | index_free_K | index_free_F;
	dimensions_strict = struct(...
		'models',						int32(number_models),...%number of multi-models
		'states',						int32(number_states),...% number of states of multi-models
		'controls',						int32(number_controls),...% number of controls of multi-models
		'measurements',					int32(number_measurements),...% number of measurements of multi-models
		'measurements_xdot',			int32(number_measurements_xdot),...% number of derivative measurements of multi-models
		'references',					int32(number_references),...% number of reference inputs of multi-models
		'tf_structure',					double(tf_structure),...% structure of closed loop transfer matrix for all multi-models
		'number_decouplingconditions',	number_decouplingconditions,...% number of zeros per column of tf_structure
		'm_invariant',					int32(m_invariant),...% dimension of controlled invariant subspace for decoupling control for each column of transfer matrix. Same for all multi-models
		'hasfeedthrough_decoupling',	hasfeedthrough_decoupling,...% indicator, whether feedthrough D2 ~= 0 in decoupling conditions
		'descriptor',					descriptor,...% indicator if the multi-models have a descriptor matrix ~= I
		'isdiscrete',					sample_time > 0,...% indicator if systems are discrete time
		'areas_max',					int32(number_areas_max_strict),...% maximum number of areas per multi-model
		'area_args',					int32(number_areaargs_strict),...% number of arguments of area functions
		'area_parts',					int32(area_parts_strict),...% number of areas per multi-model
		'area_hasgrad',					area_hasgrad_strict,...% indicator if areafunction has gradient
		'area_hashess',					area_hashess_strict,...% indicator if areafunction has hessian
		'area_parameters',				area_parameters_strict,...% structure with area function parameters
		'R_fixed_has',					hasfixed_R,...% indicator if proportional gain matrix has fixed elements
		'R_fixed_only',					onlyfixed_R,...% indicator if proportional gain matrix has only fixed elements
		'R_fixed_constraints',			int32(rg),...% number of fixed proportional gain coefficients
		'R_fixed',						R_fixed{1},...% indcator matrix for fixed proportional gain coefficients
		'R_fixed_values',				R_fixed{2},...% value matrix for fixed proportional gain coefficients
		'R_fixed_A',					constraint_system,...% fixed proportional gain constraint system A vec(R) = b
		'R_fixed_b',					constraint_border,...% fixed proportional gain constraint system A vec(R) = b
		'R_fixed_T',					T,...% transformation matrix from proportional gain coefficients to optimization variables and fixed coefficients
		'R_fixed_T_inv',				T_inv,...% transformation matrix from optimization variables and fixed coefficients to proportional gain coefficients
		'R_isforced2zero',				isforced2zero_R,...% indicator if all coefficients of K are forced to zero (not needed)
		'K_fixed_has',					hasfixed_K,...% indicator if derivative gain matrix has fixed elements
		'K_fixed_only',					onlyfixed_K,...% indicator if derivative gain matrix has only fixed elements
		'K_fixed_constraints',			int32(rg_xdot),...% number of fixed derivative gain coefficients
		'K_fixed',						K_fixed{1},...% indcator matrix for fixed derivative gain coefficients
		'K_fixed_values',				K_fixed{2},...% value matrix for fixed derivative gain coefficients
		'K_fixed_A',					constraint_system_xdot,...% fixed derivative gain constraint system A vec(K) = b
		'K_fixed_b',					constraint_border_xdot,...% fixed derivative gain constraint system A vec(K) = b
		'K_fixed_T',					T_xdot,...% transformation matrix from derivative gain coefficients to optimization variables and fixed coefficients
		'K_fixed_T_inv',				T_inv_xdot,...% transformation matrix from optimization variables and fixed coefficients to derivative gain coefficients
		'K_isforced2zero',				isforced2zero_K || isforced2zero_RKF_K,...% indicator if all coefficients of K are forced to zero
		'F_fixed_has',					hasfixed_F,...% indicator if prefilter matrix has fixed elements
		'F_fixed_only',					onlyfixed_F,...% indicator if prefilter matrix has only fixed elements
		'F_fixed_constraints',			int32(rg_prefilter),...% number of fixed prefilter coefficients
		'F_fixed',						F_fixed{1},...% indcator matrix for fixed prefilter coefficients
		'F_fixed_values',				F_fixed{2},...% value matrix for fixed prefilter coefficients
		'F_fixed_A',					constraint_system_prefilter,...% fixed prefilter constraint system A vec(F) = b
		'F_fixed_b',					constraint_border_prefilter,...% fixed prefilter constraint system A vec(F) = b
		'F_fixed_T',					T_prefilter,...% transformation matrix from prefilter coefficients to optimization variables and fixed coefficients
		'F_fixed_T_inv',				T_inv_prefilter,...% transformation matrix from optimization variables and fixed coefficients to prefilter coefficients
		'F_isforced2zero',				isforced2zero_F,...% indicator if all coefficients of F are forced to zero (not needed)
		'RKF_fixed_has',				hasfixed_RKF,...% indicator if combined gain matrix has fixed elements
		'RKF_fixed_only',				onlyfixed_RKF,...% indicator if combined gain matrix has only fixed elements
		'RKF_fixed_constraints',		int32(rg_RKF),...% number of fixed combined gain coefficients
		'RKF_fixed',					RKF_fixed{1},...% indcator matrix for fixed gain coefficients
		'RKF_fixed_values',				RKF_fixed{2},...% value matrix for fixed gain coefficients
		'RKF_fixed_A',					constraint_system_RKF,...% fixed gain constraint system A [vec(R);vec(K);vec(F)] = b
		'RKF_fixed_b',					constraint_border_RKF,...% fixed gain constraint system A [vec(R);vec(K);vec(F)] = b
		'RKF_fixed_T',					T_RKF,...% transformation matrix from gain coefficients to optimization variables and fixed coefficients
		'RKF_fixed_T_inv',				T_inv_RKF,...% transformation matrix from optimization variables and fixed coefficients to gain coefficients
		'index_R_free',					index_free_R,...% logical indices of free proportional gain coefficients (optimization variables) in vectorized combined gain coefficients
		'index_K_free',					index_free_K,...% logical indices of free derivative gain coefficients (optimization variables) in vectorized combined gain coefficients
		'index_F_free',					index_free_F,...% logical indices of free prefilter coefficients (optimization variables) in vectorized combined gain coefficients
		'index_RKF_free',				index_free_RKF,...% logical indices of free gain coefficients (optimization variables) in vectorized combined gain coefficients
		'index_all_free',				index_free_all...% logical indices of free proportional and derivative gain and prefilter coefficients (optimization variables) in vectorized combined gain coefficients
	);
	dimensions_loose = struct(...
		'models',						int32(number_models),...%number of multi-models
		'states',						int32(number_states),...% number of states of multi-models
		'controls',						int32(number_controls),...% number of controls of multi-models
		'measurements',					int32(number_measurements),...% number of measurements of multi-models
		'measurements_xdot',			int32(number_measurements_xdot),...% number of derivative measurements of multi-models
		'references',					int32(number_references),...% number of reference inputs of multi-models
		'tf_structure',					double(tf_structure),...% structure of closed loop transfer matrix for all multi-models
		'number_decouplingconditions',	number_decouplingconditions,...% number of zeros per column of tf_structure
		'm_invariant',					int32(m_invariant),...% dimension of controlled invariant subspace for decoupling control for each column of transfer matrix. Same for all multi-models
		'hasfeedthrough_decoupling',	hasfeedthrough_decoupling,...% indicator, whether feedthrough D2 ~= 0 in decoupling conditions
		'descriptor',					descriptor,...% indicator if the multi-models have a descriptor matrix ~= I
		'isdiscrete',					sample_time > 0,...% indicator if systems are discrete time
		'areas_max',					int32(number_areas_max_loose),...% maximum number of areas per multi-model
		'area_args',					int32(number_areaargs_loose),...% number of arguments of area functions
		'area_parts',					int32(area_parts_loose),...% number of areas per multi-model
		'area_hasgrad',					area_hasgrad_loose,...% indicator if areafunction has gradient
		'area_hashess',					area_hashess_loose,...% indicator if areafunction has hessian
		'area_parameters',				area_parameters_loose,...% structure with area function parameters
		'R_fixed_has',					hasfixed_R,...% indicator if proportional gain matrix has fixed elements
		'R_fixed_only',					onlyfixed_R,...% indicator if proportional gain matrix has only fixed elements
		'R_fixed_constraints',			int32(rg),...% number of fixed proportional gain coefficients
		'R_fixed',						R_fixed{1},...% indcator matrix for fixed proportional gain coefficients
		'R_fixed_values',				R_fixed{2},...% value matrix for fixed proportional gain coefficients
		'R_fixed_A',					constraint_system,...% fixed proportional gain constraint system A vec(R) = b
		'R_fixed_b',					constraint_border,...% fixed proportional gain constraint system A vec(R) = b
		'R_fixed_T',					T,...% transformation matrix from proportional gain coefficients to optimization variables and fixed coefficients
		'R_fixed_T_inv',				T_inv,...% transformation matrix from optimization variables and fixed coefficients to proportional gain coefficients
		'R_isforced2zero',				isforced2zero_R,...% indicator if all coefficients of R are forced to zero (not needed)
		'K_fixed_has',					hasfixed_K,...% indicator if derivative gain matrix has fixed elements
		'K_fixed_only',					onlyfixed_K,...% indicator if derivative gain matrix has only fixed elements
		'K_fixed_constraints',			int32(rg_xdot),...% number of fixed derivative gain coefficients
		'K_fixed',						K_fixed{1},...% indcator matrix for fixed derivative gain coefficients
		'K_fixed_values',				K_fixed{2},...% value matrix for fixed derivative gain coefficients
		'K_fixed_A',					constraint_system_xdot,...% fixed derivative gain constraint system A vec(K) = b
		'K_fixed_b',					constraint_border_xdot,...% fixed derivative gain constraint system A vec(K) = b
		'K_fixed_T',					T_xdot,...% transformation matrix from derivative gain coefficients to optimization variables and fixed coefficients
		'K_fixed_T_inv',				T_inv_xdot,...% transformation matrix from optimization variables and fixed coefficients to derivative gain coefficients
		'K_isforced2zero',				isforced2zero_K || isforced2zero_RKF_K,...% indicator if all coefficients of K are forced to zero
		'F_fixed_has',					hasfixed_F,...% indicator if prefilter matrix has fixed elements
		'F_fixed_only',					onlyfixed_F,...% indicator if prefilter matrix has only fixed elements
		'F_fixed_constraints',			int32(rg_prefilter),...% number of fixed prefilter coefficients
		'F_fixed',						F_fixed{1},...% indcator matrix for fixed prefilter coefficients
		'F_fixed_values',				F_fixed{2},...% value matrix for fixed prefilter coefficients
		'F_fixed_A',					constraint_system_prefilter,...% fixed prefilter constraint system A vec(F) = b
		'F_fixed_b',					constraint_border_prefilter,...% fixed prefilter constraint system A vec(F) = b
		'F_fixed_T',					T_prefilter,...% transformation matrix from prefilter coefficients to optimization variables and fixed coefficients
		'F_fixed_T_inv',				T_inv_prefilter,...% transformation matrix from optimization variables and fixed coefficients to prefilter coefficients
		'F_isforced2zero',				isforced2zero_F,...% indicator if all coefficients of F are forced to zero (not needed)
		'RKF_fixed_has',				hasfixed_RKF,...% indicator if combined gain matrix has fixed elements
		'RKF_fixed_only',				onlyfixed_RKF,...% indicator if combined gain matrix has only fixed elements
		'RKF_fixed_constraints',		int32(rg_RKF),...% number of fixed combined gain coefficients
		'RKF_fixed',					RKF_fixed{1},...% indcator matrix for fixed gain coefficients
		'RKF_fixed_values',				RKF_fixed{2},...% value matrix for fixed gain coefficients
		'RKF_fixed_A',					constraint_system_RKF,...% fixed gain constraint system A [vec(R);vec(K);vec(F)] = b
		'RKF_fixed_b',					constraint_border_RKF,...% fixed gain constraint system A [vec(R);vec(K);vec(F)] = b
		'RKF_fixed_T',					T_RKF,...% transformation matrix from gain coefficients to optimization variables and fixed coefficients
		'RKF_fixed_T_inv',				T_inv_RKF,...% transformation matrix from optimization variables and fixed coefficients to gain coefficients
		'index_R_free',					index_free_R,...% logical indices of free proportional gain coefficients (optimization variables) in vectorized combined gain coefficients
		'index_K_free',					index_free_K,...% logical indices of free derivative gain coefficients (optimization variables) in vectorized combined gain coefficients
		'index_F_free',					index_free_F,...% logical indices of free prefilter coefficients (optimization variables) in vectorized combined gain coefficients
		'index_RKF_free',				index_free_RKF,...% logical indices of free gain coefficients (optimization variables) in vectorized combined gain coefficients
		'index_all_free',				index_free_all...% logical indices of free proportional and derivative gain and prefilter coefficients (optimization variables) in vectorized combined gain coefficients
	);
	% check inequality constraints for feasibility under equality constraints
	if configuration.optimization.hasoptimization()
		if hasbounds_R || hasbounds_K || hasbounds_F
			A = bound_system*dimensions_strict.R_fixed_T_inv;
			A = A(:, 1:number_controls*number_measurements - size(dimensions_strict.R_fixed_b, 1));
			temp = bound_system*dimensions_strict.R_fixed_T_inv*[
				dimensions_strict.R_fixed_b;
				zeros(number_controls*number_measurements - size(dimensions_strict.R_fixed_b, 1), 1)
			];
			b = bound_border - temp;
			infbound = isinf(b) & b > 0;
			zeroA = all(A == 0, 2) & infbound;
			A = A(~zeroA, :);
			b = b(~zeroA, :);
			neginf = isinf(b) & b < 0;
			posA = sum(A > 0, 2);
			negA = sum(A < 0, 2);
			if any(neginf & posA & (posA + negA <= 1))
				error('control:design:gamma:dimension', 'Bounded proportional gain constraint system with fixed proportional gain constraints has no feasible solution, because it results in the upper bound -inf.');
			end
			notposinf = ~(isinf(b) & b > 0);
			if any(notposinf) && size(A, 2) > 0
				if matlab.Version.CURRENT >= matlab.Version.R2016A
					linprogoptions = optimset('Display', 'off', 'Algorithm', 'interior-point-legacy');
				else
					linprogoptions = optimset('Display', 'off');
				end
				[~, ~, exitflag] = linprog(zeros(number_controls*number_measurements - size(dimensions_strict.R_fixed_b, 1), 1), A(notposinf, :), b(notposinf, 1), [], [], [], [], ones(number_controls*number_measurements - size(dimensions_strict.R_fixed_b, 1), 1), linprogoptions);
				if exitflag == -2 || exitflag == -5
					error('control:design:gamma:dimension', 'Bounded proportional gain constraint system with fixed proportional gain constraints has no feasible solution, check the bound definition.');
				end
			end
			if number_measurements_xdot > 0
				A = bound_system_xdot*dimensions_strict.K_fixed_T_inv;
				A = A(:, 1:number_controls*number_measurements_xdot - size(dimensions_strict.K_fixed_b, 1));
				temp = bound_system_xdot*dimensions_strict.K_fixed_T_inv*[
					dimensions_strict.K_fixed_b;
					zeros(number_controls*number_measurements_xdot - size(dimensions_strict.K_fixed_b, 1), 1)
				];
				b = bound_border_xdot - temp;
				infbound = isinf(b) & b > 0;
				zeroA = all(A == 0, 2) & infbound;
				A = A(~zeroA, :);
				b = b(~zeroA, :);
				neginf = isinf(b) & b < 0;
				posA = sum(A > 0, 2);
				negA = sum(A < 0, 2);
				if any(neginf & posA & (posA + negA <= 1))
					error('control:design:gamma:dimension', 'Bounded derivative gain constraint system with fixed derivative gain constraints has no feasible solution, because it results in the upper bound -inf.');
				end
				notposinf = ~(isinf(b) & b > 0);
				if any(notposinf) && size(A, 2) > 0
					if matlab.Version.CURRENT >= matlab.Version.R2016A
						linprogoptions = optimset('Display', 'off', 'Algorithm', 'interior-point-legacy');
					else
						linprogoptions = optimset('Display', 'off');
					end
					[~, ~, exitflag] = linprog(zeros(number_controls*number_measurements_xdot - size(dimensions_strict.K_fixed_b, 1), 1), A(notposinf, :), b(notposinf, 1), [], [], [], [], ones(number_controls*number_measurements_xdot - size(dimensions_strict.K_fixed_b, 1), 1), linprogoptions);
					if exitflag == -2 || exitflag == -5
						error('control:design:gamma:dimension', 'Bounded derivative gain constraint system with fixed derivative gain constraints has no feasible solution, check the bound definition.');
					end
				end
			end
			if number_references > 0
				A = bound_system_prefilter*dimensions_strict.F_fixed_T_inv;
				A = A(:, 1:number_controls*number_references - size(dimensions_strict.F_fixed_b, 1));
				temp = bound_system_prefilter*dimensions_strict.F_fixed_T_inv*[
					dimensions_strict.F_fixed_b;
					zeros(number_controls*number_references - size(dimensions_strict.F_fixed_b, 1), 1)
				];
				b = bound_border_prefilter - temp;
				infbound = isinf(b) & b > 0;
				zeroA = all(A == 0, 2) & infbound;
				A = A(~zeroA, :);
				b = b(~zeroA, :);
				neginf = isinf(b) & b < 0;
				posA = sum(A > 0, 2);
				negA = sum(A < 0, 2);
				if any(neginf & posA & (posA + negA <= 1))
					error('control:design:gamma:dimension', 'Bounded prefilter constraint system with fixed prefilter constraints has no feasible solution, because it results in the upper bound -inf.');
				end
				notposinf = ~(isinf(b) & b > 0);
				if any(notposinf) && size(A, 2) > 0
					if matlab.Version.CURRENT >= matlab.Version.R2016A
						linprogoptions = optimset('Display', 'off', 'Algorithm', 'interior-point-legacy');
					else
						linprogoptions = optimset('Display', 'off');
					end
					[~, ~, exitflag] = linprog(zeros(number_controls*number_references - size(dimensions_strict.F_fixed_b, 1), 1), A(notposinf, :), b(notposinf, 1), [], [], [], [], ones(number_controls*number_references - size(dimensions_strict.F_fixed_b, 1), 1), linprogoptions);
					if exitflag == -2 || exitflag == -5
						error('control:design:gamma:dimension', 'Bounded prefilter constraint system with fixed prefilter constraints has no feasible solution, check the bound definition.');
					end
				end
			end
		elseif hasbounds_RKF
			A = bound_system_RKF*dimensions_strict.RKF_fixed_T_inv;
			A = A(:, 1:number_controls*(number_measurements + number_measurements_xdot + number_references) - size(dimensions_strict.RKF_fixed_b, 1));
			temp = bound_system_RKF*dimensions_strict.RKF_fixed_T_inv*[
				dimensions_strict.RKF_fixed_b;
				zeros(number_controls*(number_measurements + number_measurements_xdot + number_references) - size(dimensions_strict.RKF_fixed_b, 1), 1)
			];
			b = bound_border_RKF - temp;
			infbound = isinf(b) & b > 0;
			zeroA = all(A == 0, 2) & infbound;
			A = A(~zeroA, :);
			b = b(~zeroA, :);
			neginf = isinf(b) & b < 0;
			posA = sum(A > 0, 2);
			negA = sum(A < 0, 2);
			if any(neginf & posA & (posA + negA <= 1))
				error('control:design:gamma:dimension', 'Bounded combined gain constraint system with fixed proportional gain constraints has no feasible solution, because it results in the upper bound -inf.');
			end
			notposinf = ~(isinf(b) & b > 0);
			if any(notposinf) && size(A, 2) > 0
				if matlab.Version.CURRENT >= matlab.Version.R2016A
					linprogoptions = optimset('Display', 'off', 'Algorithm', 'interior-point-legacy');
				else
					linprogoptions = optimset('Display', 'off');
				end
				[~, ~, exitflag] = linprog(zeros(number_controls*(number_measurements + number_measurements_xdot + number_references) - size(dimensions_strict.RKF_fixed_b, 1), 1), A(notposinf, :), b(notposinf, 1), [], [], [], [], ones(number_controls*(number_measurements + number_measurements_xdot + number_references) - size(dimensions_strict.RKF_fixed_b, 1), 1), linprogoptions);
				if exitflag == -2 || exitflag == -5
					error('control:design:gamma:dimension', 'Bounded combined gain constraint system with fixed proportional gain constraints has no feasible solution, check the bound definition.');
				end
			end
		end
	end
	bounds = struct(...
		'states',					int32(number_states),...
		'controls',					int32(number_controls),...
		'measurements',				int32(number_measurements),...
		'measurements_xdot',		int32(number_measurements_xdot),...
		'references',				int32(number_references),...
		'R_bounds_has',				hasbounds_R,...
		'R_bounds_only',			onlybounds_R,...
		'R_bounds_constraints',		int32(rg_bounds),...
		'R_bounds_lower',			R_bounds{1},...
		'R_bounds_upper',			R_bounds{2},...
		'R_bounds_A',				bound_system,...
		'R_bounds_b',				bound_border,...
		'R_bounds_T',				T,...
		'R_bounds_T_inv',			T_inv,...
		'K_bounds_has',				hasbounds_K,...
		'K_bounds_only',			onlybounds_K,...
		'K_bounds_constraints',		int32(rg_xdot_bounds),...
		'K_bounds_lower',			K_bounds{1},...
		'K_bounds_upper',			K_bounds{2},...
		'K_bounds_A',				bound_system_xdot,...
		'K_bounds_b',				bound_border_xdot,...
		'K_bounds_T',				T_xdot,...
		'K_bounds_T_inv',			T_inv_xdot,...
		'F_bounds_has',				hasbounds_F,...
		'F_bounds_only',			onlybounds_F,...
		'F_bounds_constraints',		int32(rg_prefilter_bounds),...
		'F_bounds_lower',			F_bounds{1},...
		'F_bounds_upper',			F_bounds{2},...
		'F_bounds_A',				bound_system_prefilter,...
		'F_bounds_b',				bound_border_prefilter,...
		'F_bounds_T',				T_prefilter,...
		'F_bounds_T_inv',			T_inv_prefilter,...
		'RKF_bounds_has',			hasbounds_RKF,...
		'RKF_bounds_only',			onlybounds_RKF,...
		'RKF_bounds_constraints',	int32(rg_RKF_bounds),...
		'RKF_bounds_lower',			RKF_bounds{1},...
		'RKF_bounds_upper',			RKF_bounds{2},...
		'RKF_bounds_A',				bound_system_RKF,...
		'RKF_bounds_b',				bound_border_RKF,...
		'RKF_bounds_T',				T_RKF,...
		'RKF_bounds_T_inv',			T_inv_RKF...
	);
	if nargin <= 14
		R_nonlin = [];
	end
	[R_nonlin, number_c_R, number_ceq_R, number_c_K, number_ceq_K, number_c_F, number_ceq_F, hasnonlinfun, hasnonlingrad, hasnonlinhessian] = checkandtransform_gain_nonlin(R_nonlin, number_controls, number_measurements, number_measurements_xdot, number_references, []);
	nonlcon = struct(...
		'f',		R_nonlin,...
		'R_c',		number_c_R,...
		'R_ceq',	number_ceq_R,...
		'K_c',		number_c_K,...
		'K_ceq',	number_ceq_K,...
		'F_c',		number_c_F,...
		'F_ceq',	number_ceq_F,...
		'hasfun',	hasnonlinfun,...
		'hasgrad',	hasnonlingrad,...
		'hashess',	hasnonlinhessian...
	);
end
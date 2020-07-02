function [...
	R_fixed, constraint_system, constraint_border, rg, T, T_inv, hasfixed_R, onlyfixed_R, allfixed_R, constraint_system_hadamard_R, isforced2zero_R,...
	K_fixed, constraint_system_xdot, constraint_border_xdot, rg_xdot, T_xdot, T_inv_xdot, hasfixed_K, onlyfixed_K, allfixed_K, constraint_system_hadamard_K, isforced2zero_K,...
	F_fixed, constraint_system_prefilter, constraint_border_prefilter, rg_prefilter, T_prefilter, T_inv_prefilter, hasfixed_F, onlyfixed_F, allfixed_F, constraint_system_hadamard_F, isforced2zero_F,...
	RKF_fixed, constraint_system_RKF, constraint_border_RKF, rg_RKF, T_RKF, T_inv_RKF, hasfixed_RKF, onlyfixed_RKF, allfixed_RKF, constraint_system_hadamard_RKF...
] = checkandtransform_gain_fixed_RKF(...
	R_fixed, constraint_system, constraint_border, rg, T, T_inv, hasfixed_R, onlyfixed_R, allfixed_R, constraint_system_hadamard_R, isforced2zero_R,...
	K_fixed, constraint_system_xdot, constraint_border_xdot, rg_xdot, T_xdot, T_inv_xdot, hasfixed_K, onlyfixed_K, allfixed_K, constraint_system_hadamard_K, isforced2zero_K,...
	F_fixed, constraint_system_prefilter, constraint_border_prefilter, rg_prefilter, T_prefilter, T_inv_prefilter, hasfixed_F, onlyfixed_F, allfixed_F, constraint_system_hadamard_F, isforced2zero_F,...
	RKF_fixed, constraint_system_RKF, constraint_border_RKF, rg_RKF, T_RKF, T_inv_RKF, hasfixed_RKF, onlyfixed_RKF, allfixed_RKF, constraint_system_hadamard_RKF,...
	number_controls, number_measurements, number_measurements_xdot, number_references, forceRKF_form...
)
	%CHECKANDTRANSFORM_GAIN_FIXED_RKF convert fixed gain constraints for single gain matrices to combined constraints system or vice versa depending on the type of specified constraints
	%	Input:
	%		R_fixed:						uniform proportional gain constraint system
	%		constraint_system:				matrix of constraint system in the form A*vec(R) = b
	%		constraint_border:				border of constraint system in the form A*vec(R) = b
	%		rg:								rank of proportional constraint system
	%		T:								transformation from proportional gain coefficients to constrained gain coefficients
	%		T_inv:							inverse transformation from constrained gain coefficients back to proportional gain coefficients
	%		hasfixed_R:						indicator, if proportional gain matrix has fixed elements
	%		onlyfixed_R:					indicator, if proportional gain matrix has only fixed elements
	%		allfixed_R:						indicator, if all coefficients of proportional gain matrix are fixed
	%		constraint_system_hadamard_R:	proportional constraint system in the form sum(A.*R) = b
	%		isforced2zero_R:				indicator if supplied constraint force R == 0
	%		K_fixed:						uniform derivative gain constraint system
	%		constraint_system_xdot:			matrix of derivative constraint system in the form A*vec(K) = b
	%		constraint_border_xdot:			border of derivative constraint system in the form A*vec(K) = b
	%		rg_xdot:						rank of derivative constraint system
	%		T_xdot:							transformation from derivative gain coefficients to constrained gain coefficients
	%		T_inv_xdot:						inverse transformation from constrained gain coefficients back to derivative gain coefficients
	%		hasfixed_K:						indicator, if derivative gain matrix has fixed elements
	%		onlyfixed_K:					indicator, if derivative gain matrix has only fixed elements
	%		allfixed_K:						indicator, if all coefficients of derivative gain matrix are fixed
	%		constraint_system_hadamard_K:	derivative constraint system in the form sum(A.*K) = b
	%		isforced2zero_K:				indicator if supplied constraint force K == 0
	%		F_fixed:						uniform prefilter gain constraint system
	%		constraint_system_prefilter:	matrix of prefilter constraint system in the form A*vec(F) = b
	%		constraint_border_prefilter:	border of prefilter constraint system in the form A*vec(F) = b
	%		rg_prefilter:					rank of prefilter constraint system
	%		T_prefilter:					transformation from prefilter gain coefficients to constrained gain coefficients
	%		T_inv_prefilter:				inverse transformation from constrained gain coefficients back to prefilter gain coefficients
	%		hasfixed_F:						indicator, if prefilter gain matrix has fixed elements
	%		onlyfixed_F:					indicator, if prefilter gain matrix has only fixed elements
	%		allfixed_F:						indicator, if all coefficients of prefilter gain matrix are fixed
	%		constraint_system_hadamard_F:	prefilter constraint system in the form sum(A.*F) = b
	%		isforced2zero_F:				indicator if supplied constraint force F == 0
	%		RKF_fixed:						uniform combined gain constraint system
	%		constraint_system_RKF:			matrix of combined constraint system in the form A*[vec(R);vec(K);vec(F)] = b
	%		constraint_border_RKF:			border of combined constraint system in the form A*[vec(R);vec(K);vec(F)] = b
	%		rg_RKF:							rank of combined constraint system
	%		T_RKF:							transformation from combined gain coefficients to constrained gain coefficients
	%		T_inv_RKF:						inverse transformation from constrained gain coefficients back to combined gain coefficients
	%		hasfixed_RKF:					indicator, if combined gain matrix has fixed elements
	%		onlyfixed_RKF:					indicator, if combined gain matrix has only fixed elements
	%		allfixed_RKF:					indicator, if all coefficients of combined gain matrix are fixed
	%		constraint_system_hadamard_RKF:	combined constraint system in the form sum(A.*[R, K, F]) = b
	%		number_controls:				number of controls
	%		number_measurements:			number of measurements
	%		number_measurements_xdot:		number of derivative measurements
	%		number_references:				number of references
	%		forceRKF_form:					indicator, if constraints should be ouput in combined form
	%	Output:
	%		R_fixed:						uniform proportional gain constraint system
	%		constraint_system:				matrix of constraint system in the form A*vec(R) = b
	%		constraint_border:				border of constraint system in the form A*vec(R) = b
	%		rg:								rank of proportional constraint system
	%		T:								transformation from proportional gain coefficients to constrained gain coefficients
	%		T_inv:							inverse transformation from constrained gain coefficients back to proportional gain coefficients
	%		hasfixed_R:						indicator, if proportional gain matrix has fixed elements
	%		onlyfixed_R:					indicator, if proportional gain matrix has only fixed elements
	%		allfixed_R:						indicator, if all coefficients of proportional gain matrix are fixed
	%		constraint_system_hadamard_R:	proportional constraint system in the form sum(A.*R) = b
	%		isforced2zero_R:				indicator if supplied constraint force R == 0
	%		K_fixed:						uniform derivative gain constraint system
	%		constraint_system_xdot:			matrix of derivative constraint system in the form A*vec(K) = b
	%		constraint_border_xdot:			border of derivative constraint system in the form A*vec(K) = b
	%		rg_xdot:						rank of derivative constraint system
	%		T_xdot:							transformation from derivative gain coefficients to constrained gain coefficients
	%		T_inv_xdot:						inverse transformation from constrained gain coefficients back to derivative gain coefficients
	%		hasfixed_K:						indicator, if derivative gain matrix has fixed elements
	%		onlyfixed_K:					indicator, if derivative gain matrix has only fixed elements
	%		allfixed_K:						indicator, if all coefficients of derivative gain matrix are fixed
	%		constraint_system_hadamard_K:	derivative constraint system in the form sum(A.*K) = b
	%		isforced2zero_K:				indicator if supplied constraint force K == 0
	%		F_fixed:						uniform prefilter gain constraint system
	%		constraint_system_prefilter:	matrix of prefilter constraint system in the form A*vec(F) = b
	%		constraint_border_prefilter:	border of prefilter constraint system in the form A*vec(F) = b
	%		rg_prefilter:					rank of prefilter constraint system
	%		T_prefilter:					transformation from prefilter gain coefficients to constrained gain coefficients
	%		T_inv_prefilter:				inverse transformation from constrained gain coefficients back to prefilter gain coefficients
	%		hasfixed_F:						indicator, if prefilter gain matrix has fixed elements
	%		onlyfixed_F:					indicator, if prefilter gain matrix has only fixed elements
	%		allfixed_F:						indicator, if all coefficients of prefilter gain matrix are fixed
	%		constraint_system_hadamard_F:	prefilter constraint system in the form sum(A.*F) = b
	%		isforced2zero_F:				indicator if supplied constraint force F == 0
	%		RKF_fixed:						uniform combined gain constraint system
	%		constraint_system_RKF:			matrix of combined constraint system in the form A*[vec(R);vec(K);vec(F)] = b
	%		constraint_border_RKF:			border of combined constraint system in the form A*[vec(R);vec(K);vec(F)] = b
	%		rg_RKF:							rank of combined constraint system
	%		T_RKF:							transformation from combined gain coefficients to constrained gain coefficients
	%		T_inv_RKF:						inverse transformation from constrained gain coefficients back to combined gain coefficients
	%		hasfixed_RKF:					indicator, if combined gain matrix has fixed elements
	%		onlyfixed_RKF:					indicator, if combined gain matrix has only fixed elements
	%		allfixed_RKF:					indicator, if all coefficients of combined gain matrix are fixed
	%		constraint_system_hadamard_RKF:	combined constraint system in the form sum(A.*[R, K, F]) = b
	if ~isnumeric(number_controls) || ~isscalar(number_controls)
		error('control:design:gamma:dimension', 'Number of controls must be a numeric scalar.');
	end
	if ~isnumeric(number_measurements) || ~isscalar(number_measurements)
		error('control:design:gamma:dimension', 'Number of measurements must be a numeric scalar.');
	end
	if ~isnumeric(number_measurements_xdot) || ~isscalar(number_measurements_xdot)
		error('control:design:gamma:dimension', 'Number of derivative measurements must be a numeric scalar.');
	end
	if ~isnumeric(number_references) || ~isscalar(number_references)
		error('control:design:gamma:dimension', 'Number of references must be a numeric scalar.');
	end
	if ~hasfixed_RKF && ~forceRKF_form
		% no combined constraints requested
		return;
	end
	if ~(hasfixed_R || hasfixed_K || hasfixed_F) && hasfixed_RKF
		% only combined constraints requested
		return;
	end
	% mix of combined and separate constraints requested
	constraint_system_hadamard_R_elevated = {
		cat(2, constraint_system_hadamard_R{1}, zeros(number_controls, number_measurements_xdot + number_references, size(constraint_system_hadamard_R{1}, 3))), constraint_system_hadamard_R{2}
	};
	constraint_system_hadamard_K_elevated = {
		cat(2, zeros(number_controls, number_measurements, size(constraint_system_hadamard_K{1}, 3)), constraint_system_hadamard_K{1}, zeros(number_controls, number_references, size(constraint_system_hadamard_K{1}, 3))), constraint_system_hadamard_K{2}
	};
	constraint_system_hadamard_F_elevated = {
		cat(2, zeros(number_controls, number_measurements + number_measurements_xdot, size(constraint_system_hadamard_F{1}, 3)), constraint_system_hadamard_F{1}), constraint_system_hadamard_F{2}
	};
	dependsonRKF = false(size(constraint_system_hadamard_RKF{1}, 3), 4);
	parfor ii = 1:size(dependsonRKF, 1)
		dep = dependsonRKF(ii, :);
		Rpart = constraint_system_hadamard_RKF{1}(1:number_controls, 1:number_measurements, ii) ~= 0;
		dep(1, 1) = any(Rpart(:));
		Kpart = constraint_system_hadamard_RKF{1}(1:number_controls, number_measurements + 1:number_measurements + number_measurements_xdot, ii) ~= 0;
		dep(1, 2) = any(Kpart(:));
		Fpart = constraint_system_hadamard_RKF{1}(1:number_controls, number_measurements + number_measurements_xdot + 1:number_measurements + number_measurements_xdot + number_references, ii) ~= 0;
		dep(1, 3) = any(Fpart(:));
		dep(1, 4) = sum(dep(1, 1:3)) > 1;
		dependsonRKF(ii, :) = dep;
	end
	dependsonR = false(size(constraint_system_hadamard_R{1}, 3), 4);
	dependsonR(:, 1) = true;
	dependsonK = false(size(constraint_system_hadamard_K{1}, 3), 4);
	dependsonK(:, 2) = true;
	dependsonF = false(size(constraint_system_hadamard_F{1}, 3), 4);
	dependsonF(:, 3) = true;
	constraint_system_combined = {
		cat(3, constraint_system_hadamard_R_elevated{1}, constraint_system_hadamard_K_elevated{1}, constraint_system_hadamard_F_elevated{1}, constraint_system_hadamard_RKF{1}), [
			constraint_system_hadamard_R_elevated{2};
			constraint_system_hadamard_K_elevated{2};
			constraint_system_hadamard_F_elevated{2};
			constraint_system_hadamard_RKF{2}
		]
	};
	dependson = [
		dependsonR;
		dependsonK;
		dependsonF;
		dependsonRKF
	];
	if ~forceRKF_form && ~any(dependson(:, 4))
		R_fixed = {
			constraint_system_combined{1}(:, 1:number_measurements, dependson(:, 1)), constraint_system_combined{2}(dependson(:, 1), 1)
		};
		K_fixed = {
			constraint_system_combined{1}(:, number_measurements + 1:number_measurements + number_measurements_xdot, dependson(:, 2)), constraint_system_combined{2}(dependson(:, 2), 1)
		};
		F_fixed = {
			constraint_system_combined{1}(:, number_measurements + number_measurements_xdot + 1:number_measurements + number_measurements_xdot + number_references, dependson(:, 3)), constraint_system_combined{2}(dependson(:, 3), 1)
		};
		RKF_fixed = {
			constraint_system_combined{1}(:, :, dependson(:, 4)), constraint_system_combined{2}(dependson(:, 4), 1)
		};
		[R_fixed, constraint_system, constraint_border, rg, T, T_inv, hasfixed_R, onlyfixed_R, allfixed_R, constraint_system_hadamard_R] = checkandtransform_gain_fixed(R_fixed, number_controls, number_measurements, 'proportional');
		[K_fixed, constraint_system_xdot, constraint_border_xdot, rg_xdot, T_xdot, T_inv_xdot, hasfixed_K, onlyfixed_K, allfixed_K, constraint_system_hadamard_K] = checkandtransform_gain_fixed(K_fixed, number_controls, number_measurements_xdot, 'derivative');
		[F_fixed, constraint_system_prefilter, constraint_border_prefilter, rg_prefilter, T_prefilter, T_inv_prefilter, hasfixed_F, onlyfixed_F, allfixed_F, constraint_system_hadamard_F] = checkandtransform_gain_fixed(F_fixed, number_controls, number_references, 'prefilter');
		[RKF_fixed, constraint_system_RKF, constraint_border_RKF, rg_RKF, T_RKF, T_inv_RKF, hasfixed_RKF, onlyfixed_RKF, allfixed_RKF, constraint_system_hadamard_RKF] = checkandtransform_gain_fixed(RKF_fixed, number_controls, number_measurements + number_measurements_xdot + number_references, 'combined');
	else
		noselect = false(size(dependson, 1), 1);
		R_fixed = {
			constraint_system_combined{1}(:, 1:number_measurements, noselect), constraint_system_combined{2}(noselect, 1)
		};
		K_fixed = {
			constraint_system_combined{1}(:, number_measurements + 1:number_measurements + number_measurements_xdot, noselect), constraint_system_combined{2}(noselect, 1)
		};
		F_fixed = {
			constraint_system_combined{1}(:, number_measurements + number_measurements_xdot + 1:number_measurements + number_measurements_xdot + number_references, noselect), constraint_system_combined{2}(noselect, 1)
		};
		RKF_fixed = constraint_system_combined;
		[R_fixed, constraint_system, constraint_border, rg, T, T_inv, hasfixed_R, onlyfixed_R, allfixed_R, constraint_system_hadamard_R] = checkandtransform_gain_fixed(R_fixed, number_controls, number_measurements, 'proportional');
		[K_fixed, constraint_system_xdot, constraint_border_xdot, rg_xdot, T_xdot, T_inv_xdot, hasfixed_K, onlyfixed_K, allfixed_K, constraint_system_hadamard_K] = checkandtransform_gain_fixed(K_fixed, number_controls, number_measurements_xdot, 'derivative');
		[F_fixed, constraint_system_prefilter, constraint_border_prefilter, rg_prefilter, T_prefilter, T_inv_prefilter, hasfixed_F, onlyfixed_F, allfixed_F, constraint_system_hadamard_F] = checkandtransform_gain_fixed(F_fixed, number_controls, number_references, 'prefilter');
		[RKF_fixed, constraint_system_RKF, constraint_border_RKF, rg_RKF, T_RKF, T_inv_RKF, hasfixed_RKF, onlyfixed_RKF, allfixed_RKF, constraint_system_hadamard_RKF] = checkandtransform_gain_fixed(RKF_fixed, number_controls, number_measurements + number_measurements_xdot + number_references, 'combined');
		K_pattern = false(number_controls, number_measurements + number_measurements_xdot + number_references);
		K_pattern(:, number_measurements + 1:number_measurements + number_measurements_xdot) = true;
		isforced2zero_K = isforced2zero_K || checkandtransform_gain_fixed_forced2zero(constraint_system_hadamard_RKF{1}, constraint_system_hadamard_RKF{2}, number_controls, number_measurements + number_measurements_xdot + number_references, 'combined', K_pattern);
	end
end
function [...
	R_bounds, bound_system, bound_border, rg, hasbounds_R, onlybounds_R, bound_system_hadamard_R,...
	K_bounds, bound_system_xdot, bound_border_xdot, rg_xdot, hasbounds_K, onlybounds_K, bound_system_hadamard_K,...
	F_bounds, bound_system_prefilter, bound_border_prefilter, rg_prefilter, hasbounds_F, onlybounds_F, bound_system_hadamard_F,...
	RKF_bounds, bound_system_RKF, bound_border_RKF, rg_RKF, hasbounds_RKF, onlybounds_RKF, bound_system_hadamard_RKF...
] = checkandtransform_gain_bounds_RKF(...
	R_bounds, bound_system, bound_border, rg, hasbounds_R, onlybounds_R, bound_system_hadamard_R,...
	K_bounds, bound_system_xdot, bound_border_xdot, rg_xdot, hasbounds_K, onlybounds_K, bound_system_hadamard_K,...
	F_bounds, bound_system_prefilter, bound_border_prefilter, rg_prefilter, hasbounds_F, onlybounds_F, bound_system_hadamard_F,...
	RKF_bounds, bound_system_RKF, bound_border_RKF, rg_RKF, hasbounds_RKF, onlybounds_RKF, bound_system_hadamard_RKF,...
	number_controls, number_measurements, number_measurements_xdot, number_references, forceRKF_form...
)
	%CHECKANDTRANSFORM_GAIN_BOUNDS_RKF convert bound gain constraints for single gain matrices to combined constraints system or vice versa depending on the type of specified constraints
	%	Input:
	%		R_bounds:					uniform proportional gain constraint system
	%		bound_system:				matrix of constraint system in the form A*vec(R) <= b
	%		bound_border:				border of constraint system in the form A*vec(R) <= b
	%		rg:							rank of proportional constraint system
	%		hasbounds_R:				indicator, if proportional gain matrix has fixed elements
	%		onlybounds_R:				indicator, if proportional gain matrix has only fixed elements
	%		bound_system_hadamard_R:	proportional constraint system in the form sum(A.*R) <= b
	%		K_bounds:					uniform derivative gain constraint system
	%		bound_system_xdot:			matrix of derivative constraint system in the form A*vec(K) <= b
	%		bound_border_xdot:			border of derivative constraint system in the form A*vec(K) <= b
	%		rg_xdot:					rank of derivative constraint system
	%		hasbounds_K:				indicator, if derivative gain matrix has fixed elements
	%		onlybounds_K:				indicator, if derivative gain matrix has only fixed elements
	%		bound_system_hadamard_K:	derivative constraint system in the form sum(A.*K) <= b
	%		F_bounds:					uniform prefilter gain constraint system
	%		bound_system_prefilter:		matrix of prefilter constraint system in the form A*vec(F) <= b
	%		bound_border_prefilter:		border of prefilter constraint system in the form A*vec(F) <= b
	%		rg_prefilter:				rank of prefilter constraint system
	%		hasbounds_F:				indicator, if prefilter gain matrix has fixed elements
	%		onlybounds_F:				indicator, if prefilter gain matrix has only fixed elements
	%		bound_system_hadamard_F:	prefilter constraint system in the form sum(A.*F) <= b
	%		RKF_bounds:					uniform combined gain constraint system
	%		bound_system_RKF:			matrix of combined constraint system in the form A*[vec(R);vec(K);vec(F)] <= b
	%		bound_border_RKF:			border of combined constraint system in the form A*[vec(R);vec(K);vec(F)] <= b
	%		rg_RKF:						rank of combined constraint system
	%		hasbounds_RKF:				indicator, if combined gain matrix has fixed elements
	%		onlybounds_RKF:				indicator, if combined gain matrix has only fixed elements
	%		bound_system_hadamard_RKF:	combined constraint system in the form sum(A.*[R, K, F]) <= b
	%		number_controls:			number of controls
	%		number_measurements:		number of measurements
	%		number_measurements_xdot:	number of derivative measurements
	%		number_references:			number of references
	%		forceKRKF_form:				indicator, if constraints should be ouput in combined form
	%	Output:
	%		R_bounds:					uniform proportional gain constraint system
	%		bound_system:				matrix of constraint system in the form A*vec(R) <= b
	%		bound_border:				border of constraint system in the form A*vec(R) <= b
	%		rg:							rank of proportional constraint system
	%		hasbounds_R:				indicator, if proportional gain matrix has fixed elements
	%		onlybounds_R:				indicator, if proportional gain matrix has only fixed elements
	%		bound_system_hadamard_R:	proportional constraint system in the form sum(A.*R) <= b
	%		K_bounds:					uniform derivative gain constraint system
	%		bound_system_xdot:			matrix of derivative constraint system in the form A*vec(K) <= b
	%		bound_border_xdot:			border of derivative constraint system in the form A*vec(K) <= b
	%		rg_xdot:					rank of derivative constraint system
	%		hasbounds_K:				indicator, if derivative gain matrix has fixed elements
	%		onlybounds_K:				indicator, if derivative gain matrix has only fixed elements
	%		bound_system_hadamard_K:	derivative constraint system in the form sum(A.*K) <= b
	%		F_bounds:					uniform prefilter gain constraint system
	%		bound_system_prefilter:		matrix of prefilter constraint system in the form A*vec(F) <= b
	%		bound_border_prefilter:		border of prefilter constraint system in the form A*vec(F) <= b
	%		rg_prefilter:				rank of prefilter constraint system
	%		hasbounds_F:				indicator, if prefilter gain matrix has fixed elements
	%		onlybounds_F:				indicator, if prefilter gain matrix has only fixed elements
	%		bound_system_hadamard_F:	prefilter constraint system in the form sum(A.*F) <= b
	%		RKF_bounds:					uniform combined gain constraint system
	%		bound_system_RKF:			matrix of combined constraint system in the form A*[vec(R);vec(K);vec(F)] <= b
	%		bound_border_RKF:			border of combined constraint system in the form A*[vec(R);vec(K);vec(F)]< = b
	%		rg_RKF:						rank of combined constraint system
	%		hasbounds_RKF:				indicator, if combined gain matrix has fixed elements
	%		onlybounds_RKF:				indicator, if combined gain matrix has only fixed elements
	%		bound_system_hadamard_RKF:	combined constraint system in the form sum(A.*[R, K, F]) <= b
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
	if ~hasbounds_RKF && ~forceRKF_form
		% no combined constraints requested
		return;
	end
	if ~(hasbounds_R || hasbounds_K || hasbounds_F) && hasbounds_RKF
		% only combined constraints requested
		return;
	end
	% mix of combined and separate constraints requested
	constraint_system_hadamard_R_elevated = {
		cat(2, bound_system_hadamard_R{1}, zeros(number_controls, number_measurements_xdot + number_references, size(bound_system_hadamard_R{1}, 3))), bound_system_hadamard_R{2}
	};
	constraint_system_hadamard_K_elevated = {
		cat(2, zeros(number_controls, number_measurements, size(bound_system_hadamard_K{1}, 3)), bound_system_hadamard_K{1}, zeros(number_controls, number_references, size(bound_system_hadamard_K{1}, 3))), bound_system_hadamard_K{2}
	};
	constraint_system_hadamard_F_elevated = {
		cat(2, zeros(number_controls, number_measurements + number_measurements_xdot, size(bound_system_hadamard_F{1}, 3)), bound_system_hadamard_F{1}), bound_system_hadamard_F{2}
	};
	dependsonRKF = false(size(bound_system_hadamard_RKF{1}, 3), 4);
	parfor ii = 1:size(dependsonRKF, 1)
		dep = dependsonRKF(ii, :);
		Rpart = bound_system_hadamard_RKF{1}(1:number_controls, 1:number_measurements, ii) ~= 0;
		dep(1, 1) = any(Rpart(:));
		Kpart = bound_system_hadamard_RKF{1}(1:number_controls, number_measurements + 1:number_measurements + number_measurements_xdot, ii) ~= 0;
		dep(1, 2) = any(Kpart(:));
		Fpart = bound_system_hadamard_RKF{1}(1:number_controls, number_measurements + number_measurements_xdot + 1:number_measurements + number_measurements_xdot + number_references, ii) ~= 0;
		dep(1, 3) = any(Fpart(:));
		dep(1, 4) = sum(dep(1, 1:3)) > 1;
		dependsonRKF(ii, :) = dep;
	end
	dependsonR = false(size(bound_system_hadamard_R{1}, 3), 4);
	dependsonR(:, 1) = true;
	dependsonK = false(size(bound_system_hadamard_K{1}, 3), 4);
	dependsonK(:, 2) = true;
	dependsonF = false(size(bound_system_hadamard_F{1}, 3), 4);
	dependsonF(:, 3) = true;
	constraint_system_combined = {
		cat(3, constraint_system_hadamard_R_elevated{1}, constraint_system_hadamard_K_elevated{1}, constraint_system_hadamard_F_elevated{1}, bound_system_hadamard_RKF{1}), [
			constraint_system_hadamard_R_elevated{2};
			constraint_system_hadamard_K_elevated{2};
			constraint_system_hadamard_F_elevated{2};
			bound_system_hadamard_RKF{2}
		]
	};
	dependson = [
		dependsonR;
		dependsonK;
		dependsonF;
		dependsonRKF
	];
	if ~forceRKF_form && ~any(dependson(:, 4))
		R_bounds = {
			constraint_system_combined{1}(:, 1:number_measurements, dependson(:, 1)), constraint_system_combined{2}(dependson(:, 1), 1)
		};
		K_bounds = {
			constraint_system_combined{1}(:, number_measurements + 1:number_measurements + number_measurements_xdot, dependson(:, 2)), constraint_system_combined{2}(dependson(:, 2), 1)
		};
		F_bounds = {
			constraint_system_combined{1}(:, number_measurements + number_measurements_xdot + 1:number_measurements + number_measurements_xdot + number_references, dependson(:, 3)), constraint_system_combined{2}(dependson(:, 3), 1)
		};
		RKF_bounds = {
			constraint_system_combined{1}(:, :, dependson(:, 4)), constraint_system_combined{2}(dependson(:, 4), 1)
		};
		[R_bounds, bound_system, bound_border, rg, hasbounds_R, onlybounds_R, bound_system_hadamard_R] = checkandtransform_gain_bounds(R_bounds, number_controls, number_measurements, 'proportional');
		[K_bounds, bound_system_xdot, bound_border_xdot, rg_xdot, hasbounds_K, onlybounds_K, bound_system_hadamard_K] = checkandtransform_gain_bounds(K_bounds, number_controls, number_measurements_xdot, 'derivative');
		[F_bounds, bound_system_prefilter, bound_border_prefilter, rg_prefilter, hasbounds_F, onlybounds_F, bound_system_hadamard_F] = checkandtransform_gain_bounds(F_bounds, number_controls, number_references, 'prefilter');
		[RKF_bounds, bound_system_RKF, bound_border_RKF, rg_RKF, hasbounds_RKF, onlybounds_RKF, bound_system_hadamard_RKF] = checkandtransform_gain_bounds(RKF_bounds, number_controls, number_measurements + number_measurements_xdot + number_references, 'combined');
	else
		noselect = false(size(dependson, 1), 1);
		R_bounds = {
			constraint_system_combined{1}(:, 1:number_measurements, noselect), constraint_system_combined{2}(noselect, 1)
		};
		K_bounds = {
			constraint_system_combined{1}(:, number_measurements + 1:number_measurements + number_measurements_xdot, noselect), constraint_system_combined{2}(noselect, 1)
		};
		F_bounds = {
			constraint_system_combined{1}(:, number_measurements + number_measurements_xdot + 1:number_measurements + number_measurements_xdot + number_references, noselect), constraint_system_combined{2}(noselect, 1)
		};
		RKF_bounds = constraint_system_combined;
		[R_bounds, bound_system, bound_border, rg, hasbounds_R, onlybounds_R, bound_system_hadamard_R] = checkandtransform_gain_bounds(R_bounds, number_controls, number_measurements, 'proportional');
		[K_bounds, bound_system_xdot, bound_border_xdot, rg_xdot, hasbounds_K, onlybounds_K, bound_system_hadamard_K] = checkandtransform_gain_bounds(K_bounds, number_controls, number_measurements_xdot, 'derivative');
		[F_bounds, bound_system_prefilter, bound_border_prefilter, rg_prefilter, hasbounds_F, onlybounds_F, bound_system_hadamard_F] = checkandtransform_gain_bounds(F_bounds, number_controls, number_references, 'prefilter');
		[RKF_bounds, bound_system_RKF, bound_border_RKF, rg_RKF, hasbounds_RKF, onlybounds_RKF, bound_system_hadamard_RKF] = checkandtransform_gain_bounds(RKF_bounds, number_controls, number_measurements + number_measurements_xdot + number_references, 'combined');
	end
end
function [A, b, lb, ub] = R2bound(dimensions, bounds)
	%R2BOUND transform gain matrices to bound matrices and vectors
	%	Input:
	%		dimensions:	structure with information about dimensions of the different variables and systems
	%		bounds:		structure with information about bounds for gain matrices
	%	Output:
	%		A:			linear inequality constraint matrix for optimization variable
	%		b:			upper bounds for linear inequality constraints on optimization variable
	%		lb:			lower bounds for optimization variable
	%		ub:			upper bounds for optimization variable
	number_controls = dimensions.controls;
	number_measurements = dimensions.measurements;
	number_measurements_xdot = dimensions.measurements_xdot;
	number_references = dimensions.references;
	derivative_feedback = number_measurements_xdot > 0;
	T_inv = dimensions.R_fixed_T_inv;
	T_inv_xdot = dimensions.K_fixed_T_inv;
	T_inv_prefilter = dimensions.F_fixed_T_inv;
	T_inv_RKF = dimensions.RKF_fixed_T_inv;
	R_bounds_has = bounds.R_bounds_has;
	K_bounds_has = bounds.K_bounds_has;
	F_bounds_has = bounds.F_bounds_has;
	RKF_bounds_has = bounds.RKF_bounds_has;
	R_onlyfixed = bounds.R_bounds_only;
	K_onlyfixed = bounds.K_bounds_only;
	F_onlyfixed = bounds.F_bounds_only;
	RKF_onlyfixed = bounds.RKF_bounds_only;
	if dimensions.RKF_fixed_has
		if RKF_bounds_has
			A_RKF = bounds.RKF_bounds_A*T_inv_RKF;
			A_RKF = A_RKF(:, size(dimensions.RKF_fixed_b, 1) + 1:end);
			temp = bounds.RKF_bounds_A*T_inv_RKF*[
				dimensions.RKF_fixed_b;
				zeros(number_controls*(number_measurements + number_measurements_xdot + number_references) - size(dimensions.RKF_fixed_b, 1), 1)
			];
			b_RKF = bounds.RKF_bounds_b - temp;
			if RKF_onlyfixed
				lb_RKF = reshape(bounds.RKF_bounds_lower, number_controls*(number_measurements + number_measurements_xdot + number_references), 1);
				ub_RKF = reshape(bounds.RKF_bounds_upper, number_controls*(number_measurements + number_measurements_xdot + number_references), 1);
				A_RKF = zeros(0, number_controls*(number_measurements + number_measurements_xdot + number_references) - size(dimensions.RKF_fixed_b, 1));
				b_RKF = zeros(0, 1);
			else
				lb_RKF = -Inf(number_controls*(number_measurements + number_measurements_xdot + number_references), 1);
				ub_RKF = Inf(number_controls*(number_measurements + number_measurements_xdot + number_references), 1);
			end
		else
			if RKF_onlyfixed
				lb_RKF = reshape(bounds.RKF_bounds_lower, number_controls*(number_measurements + number_measurements_xdot + number_references), 1);
				ub_RKF = reshape(bounds.RKF_bounds_upper, number_controls*(number_measurements + number_measurements_xdot + number_references), 1);
				A_RKF = zeros(0, number_controls*(number_measurements + number_measurements_xdot + number_references) - size(dimensions.RKF_fixed_b, 1));
				b_RKF = zeros(0, 1);
			else
				lb_RKF = -Inf(number_controls*(number_measurements + number_measurements_xdot + number_references), 1);
				ub_RKF = Inf(number_controls*(number_measurements + number_measurements_xdot + number_references), 1);
			end
		end
		% transform simple bound constraints to general linear inequality constraints
		% (not possible to transform with T_inv*ub because sign could change and upper bound could become lower bound when fixed gains are present)
		A_RKF_lb = -eye(number_controls*(number_measurements + number_measurements_xdot + number_references));
		b_RKF_lb = -lb_RKF;
		A_RKF_lb(isinf(lb_RKF) & lb_RKF < 0, :) = [];
		b_RKF_lb(isinf(lb_RKF) & lb_RKF < 0, :) = [];
		temp = A_RKF_lb*T_inv_RKF*[
			dimensions.RKF_fixed_b;
			zeros(number_controls*(number_measurements + number_measurements_xdot + number_references) - size(dimensions.RKF_fixed_b, 1), 1)
		];
		A_RKF_lb = A_RKF_lb*T_inv_RKF;
		A_RKF_lb = A_RKF_lb(:, size(dimensions.RKF_fixed_b, 1) + 1:end);
		b_RKF_lb = b_RKF_lb - temp;
		A_RKF_ub = eye(number_controls*(number_measurements + number_measurements_xdot + number_references));
		b_RKF_ub = ub_RKF;
		A_RKF_ub(isinf(ub_RKF) & ub_RKF > 0, :) = [];
		b_RKF_ub(isinf(ub_RKF) & ub_RKF > 0, :) = [];
		temp = A_RKF_ub*T_inv_RKF*[
			dimensions.RKF_fixed_b;
			zeros(number_controls*(number_measurements + number_measurements_xdot + number_references) - size(dimensions.RKF_fixed_b, 1), 1)
		];
		A_RKF_ub = A_RKF_ub*T_inv_RKF;
		A_RKF_ub = A_RKF_ub(:, size(dimensions.RKF_fixed_b, 1) + 1:end);
		b_RKF_ub = b_RKF_ub - temp;
		A = [
			A_RKF;
			A_RKF_lb;
			A_RKF_ub
		];
		b = [
			b_RKF;
			b_RKF_lb;
			b_RKF_ub
		];
	else
		if R_bounds_has
			A_R = bounds.R_bounds_A*T_inv;
			A_R = A_R(:, size(dimensions.R_fixed_b, 1) + 1:end);
			temp = bounds.R_bounds_A*T_inv*[
				dimensions.R_fixed_b;
				zeros(number_controls*number_measurements - size(dimensions.R_fixed_b, 1), 1)
			];
			b_R = bounds.R_bounds_b - temp;
			if R_onlyfixed
				lb_R = reshape(bounds.R_bounds_lower, number_controls*number_measurements, 1);
				ub_R = reshape(bounds.R_bounds_upper, number_controls*number_measurements, 1);
				A_R = zeros(0, number_controls*number_measurements - size(dimensions.R_fixed_b, 1));
				b_R = zeros(0, 1);
			else
				lb_R = -Inf(number_controls*number_measurements, 1);
				ub_R = Inf(number_controls*number_measurements, 1);
			end
		else
			if R_onlyfixed
				lb_R = reshape(bounds.R_bounds_lower, number_controls*number_measurements, 1);
				ub_R = reshape(bounds.R_bounds_upper, number_controls*number_measurements, 1);
				A_R = zeros(0, number_controls*number_measurements - size(dimensions.R_fixed_b, 1));
				b_R = zeros(0, 1);
			else
				lb_R = -Inf(number_controls*number_measurements, 1);
				ub_R = Inf(number_controls*number_measurements, 1);
			end
		end
		if derivative_feedback
			if K_bounds_has
				A_K = bounds.K_bounds_A*T_inv_xdot;
				A_K = A_K(:, size(dimensions.K_fixed_b, 1) + 1:end);
				temp = bounds.K_bounds_A*T_inv_xdot*[
					dimensions.K_fixed_b;
					zeros(number_controls*number_measurements_xdot - size(dimensions.K_fixed_b, 1), 1)
				];
				b_K = bounds.K_bounds_b - temp;
				if K_onlyfixed
					lb_K = reshape(bounds.K_bounds_lower, number_controls*number_measurements_xdot, 1);
					ub_K = reshape(bounds.K_bounds_upper, number_controls*number_measurements_xdot, 1);
					A_K = zeros(0, number_controls*number_measurements_xdot - size(dimensions.K_fixed_b, 1));
					b_K = zeros(0, 1);
				else
					lb_K = -Inf(number_controls*number_measurements_xdot, 1);
					ub_K = Inf(number_controls*number_measurements_xdot, 1);
				end
			else
				A_K = zeros(0, number_controls*number_measurements_xdot - size(dimensions.K_fixed_b, 1));
				b_K = zeros(0, 1);
				lb_K = -Inf(number_controls*number_measurements_xdot, 1);
				ub_K = Inf(number_controls*number_measurements_xdot, 1);
			end
		else
			if K_onlyfixed
				lb_K = reshape(bounds.K_bounds_lower, number_controls*number_measurements_xdot, 1);
				ub_K = reshape(bounds.K_bounds_upper, number_controls*number_measurements_xdot, 1);
				A_K = zeros(0, number_controls*number_measurements_xdot - size(dimensions.K_fixed_b, 1));
				b_K = zeros(0, 1);
			else
				lb_K = -Inf(number_controls*number_measurements_xdot, 1);
				ub_K = Inf(number_controls*number_measurements_xdot, 1);
			end
		end
		if number_references > 0
			if F_bounds_has
				A_F = bounds.F_bounds_A*T_inv_prefilter;
				A_F = A_F(:, size(dimensions.F_fixed_b, 1) + 1:end);
				temp = bounds.F_bounds_A*T_inv_prefilter*[
					dimensions.F_fixed_b;
					zeros(number_controls*number_references - size(dimensions.F_fixed_b, 1), 1)
				];
				b_F = bounds.F_bounds_b - temp;
				if F_onlyfixed
					lb_F = reshape(bounds.F_bounds_lower, number_controls*number_references, 1);
					ub_F = reshape(bounds.F_bounds_upper, number_controls*number_references, 1);
					A_F = zeros(0, number_controls*number_references - size(dimensions.F_fixed_b, 1));
					b_F = zeros(0, 1);
				else
					lb_F = -Inf(number_controls*number_references, 1);
					ub_F = Inf(number_controls*number_references, 1);
				end
			else
				A_F = zeros(0, number_controls*number_references- size(dimensions.F_fixed_b, 1));
				b_F = zeros(0, 1);
				lb_F = -Inf(number_controls*number_references, 1);
				ub_F = Inf(number_controls*number_references, 1);
			end
		else
			if F_onlyfixed
				lb_F = reshape(bounds.F_bounds_lower, number_controls*number_references, 1);
				ub_F = reshape(bounds.F_bounds_upper, number_controls*number_references, 1);
				A_F = zeros(0, number_controls*number_references - size(dimensions.F_fixed_b, 1));
				b_F = zeros(0, 1);
			else
				lb_F = -Inf(number_controls*number_references, 1);
				ub_F = Inf(number_controls*number_references, 1);
			end
		end
		if any(lb_R > ub_R)
			error('control:design:gamma', 'Lower bound for proportional gains must not be larger than upper bound.');
		end
		if any(lb_K > ub_K)
			error('control:design:gamma', 'Lower bound for derivative gains must not be larger than upper bound.');
		end
		if any(lb_F > ub_F)
			error('control:design:gamma', 'Lower bound for prefilter gains must not be larger than upper bound.');
		end
		% transform simple bound constraints to general linear inequality constraints
		% (not possible to transform with T_inv*ub because sign could change and upper bound could become lower bound when fixed gains are present)
		A_R_lb = -eye(number_controls*number_measurements);
		b_R_lb = -lb_R;
		A_R_lb(isinf(lb_R) & lb_R < 0, :) = [];
		b_R_lb(isinf(lb_R) & lb_R < 0, :) = [];
		temp = A_R_lb*T_inv*[
			dimensions.R_fixed_b;
			zeros(number_controls*number_measurements - size(dimensions.R_fixed_b, 1), 1)
		];
		A_R_lb = A_R_lb*T_inv;
		A_R_lb = A_R_lb(:, size(dimensions.R_fixed_b, 1) + 1:end);
		b_R_lb = b_R_lb - temp;
		A_R_ub = eye(number_controls*number_measurements);
		b_R_ub = ub_R;
		A_R_ub(isinf(ub_R) & ub_R > 0, :) = [];
		b_R_ub(isinf(ub_R) & ub_R > 0, :) = [];
		temp = A_R_ub*T_inv*[
			dimensions.R_fixed_b;
			zeros(number_controls*number_measurements - size(dimensions.R_fixed_b, 1), 1)
		];
		A_R_ub = A_R_ub*T_inv;
		A_R_ub = A_R_ub(:, size(dimensions.R_fixed_b, 1) + 1:end);
		b_R_ub = b_R_ub - temp;
		A_R_bounds = [
			A_R_lb;
			A_R_ub
		];
		b_R_bounds = [
			b_R_lb;
			b_R_ub
		];
		A_K_lb = -eye(number_controls*number_measurements_xdot);
		b_K_lb = -lb_K;
		A_K_lb(isinf(lb_K) & lb_K < 0, :) = [];
		b_K_lb(isinf(lb_K) & lb_K < 0, :) = [];
		temp = A_K_lb*T_inv_xdot*[
			dimensions.K_fixed_b;
			zeros(number_controls*number_measurements_xdot - size(dimensions.K_fixed_b, 1), 1)
		];
		A_K_lb = A_K_lb*T_inv_xdot;
		A_K_lb = A_K_lb(:, size(dimensions.K_fixed_b, 1) + 1:end);
		b_K_lb = b_K_lb - temp;
		A_K_ub = eye(number_controls*number_measurements_xdot);
		b_K_ub = ub_K;
		A_K_ub(isinf(ub_K) & ub_K > 0, :) = [];
		b_K_ub(isinf(ub_K) & ub_K > 0, :) = [];
		temp = A_K_ub*T_inv_xdot*[
			dimensions.K_fixed_b;
			zeros(number_controls*number_measurements_xdot - size(dimensions.K_fixed_b, 1), 1)
		];
		A_K_ub = A_K_ub*T_inv_xdot;
		A_K_ub = A_K_ub(:, size(dimensions.K_fixed_b, 1) + 1:end);
		b_K_ub = b_K_ub - temp;
		A_K_bounds = [
			A_K_lb;
			A_K_ub
		];
		b_K_bounds = [
			b_K_lb;
			b_K_ub
		];
		A_F_lb = -eye(number_controls*number_references);
		b_F_lb = -lb_F;
		A_F_lb(isinf(lb_F) & lb_F < 0, :) = [];
		b_F_lb(isinf(lb_F) & lb_F < 0, :) = [];
		temp = A_F_lb*T_inv_prefilter*[
			dimensions.F_fixed_b;
			zeros(number_controls*number_references - size(dimensions.F_fixed_b, 1), 1)
		];
		A_F_lb = A_F_lb*T_inv_prefilter;
		A_F_lb = A_F_lb(:, size(dimensions.F_fixed_b, 1) + 1:end);
		b_F_lb = b_F_lb - temp;
		A_F_ub = eye(number_controls*number_references);
		b_F_ub = ub_F;
		A_F_ub(isinf(ub_F) & ub_F > 0, :) = [];
		b_F_ub(isinf(ub_F) & ub_F > 0, :) = [];
		temp = A_F_ub*T_inv_prefilter*[
			dimensions.F_fixed_b;
			zeros(number_controls*number_references - size(dimensions.F_fixed_b, 1), 1)
		];
		A_F_ub = A_F_ub*T_inv_prefilter;
		A_F_ub = A_F_ub(:, size(dimensions.F_fixed_b, 1) + 1:end);
		b_F_ub = b_F_ub - temp;
		A_F_bounds = [
			A_F_lb;
			A_F_ub
		];
		b_F_bounds = [
			b_F_lb;
			b_F_ub
		];
		A = [
			A_R,										zeros(size(A_R, 1), size(A_K, 2) + size(A_F, 2));
			A_R_bounds,									zeros(size(A_R_bounds, 1), size(A_K, 2) + size(A_F, 2));
			zeros(size(A_K, 1), size(A_R, 2)),			A_K,														zeros(size(A_K, 1), size(A_F, 2));
			zeros(size(A_K_bounds, 1), size(A_R, 2)),	A_K_bounds,													zeros(size(A_K_bounds, 1), size(A_F, 2));
			zeros(size(A_F, 1), size(A_R, 2) + size(A_K, 2)),														A_F;
			zeros(size(A_F_bounds, 1), size(A_R, 2) + size(A_K, 2)),												A_F_bounds
		];
		b = [
			b_R;
			b_R_bounds
			b_K;
			b_K_bounds;
			b_F;
			b_F_bounds
		];
	end
	% check for feasibility
	if configuration.optimization.hasoptimization()
		infbound = isinf(b) & b > 0;
		zeroA = all(A == 0, 2) & infbound;
		A_test = A(~zeroA, :);
		b_test = b(~zeroA, :);
		neginf = isinf(b_test) & b_test < 0;
		posA = sum(A_test > 0, 2);
		negA = sum(A_test < 0, 2);
		if any(neginf & posA & (posA + negA <= 1))
			error('control:design:gamma:dimension', 'Bounded gain constraint system with fixed gain constraints has no feasible solution, because it results in the upper bound -inf.');
		end
		notposinf = ~(isinf(b_test) & b_test > 0);
		if any(notposinf) && size(A_test, 2) > 0
			if matlab.Version.CURRENT >= matlab.Version.R2016A
				linprogoptions = optimset('Display', 'off', 'Algorithm', 'interior-point-legacy');
			else
				linprogoptions = optimset('Display', 'off');
			end
			[~, ~, exitflag] = linprog(zeros(size(A_test, 2), 1), A_test(notposinf, :), b_test(notposinf, 1), [], [], [], [], ones(size(A_test, 2), 1), linprogoptions);
			if exitflag == -2 || exitflag == -5
				error('control:design:gamma:dimension', 'Bounded gain constraint system with fixed gain constraints has no feasible solution, check the bound definition.');
			end
		end
	end
	singlevariable = sum(A ~= 0, 2);
	% remove constant inequalities
	if any(singlevariable == 0)
		if any(b(singlevariable == 0, 1) < 0)
			error('control:design:gamma:dimension', 'Bounded gain constraint system with fixed gain constraints has no feasible solution, because it results in upper bounds < 0.');
		end
		A = A(singlevariable ~= 0, :);
		b = b(singlevariable ~= 0, 1);
		singlevariable = sum(A ~= 0, 2);
	end
	% extract simple bound constraints
	if any(singlevariable == 1)
		A_single = A(singlevariable == 1, :);
		b_single = b(singlevariable == 1, 1);
		bound = NaN(size(b_single, 1), 3);
		for ii = 1:size(b_single, 1)
			idx = find(A_single(ii, :) ~= 0, 1, 'first');
			bound(ii, :) = [
				b_single(ii, 1),	A_single(ii, idx),	idx
			];
		end
		lb = -Inf(size(A, 2), 1);
		ub = Inf(size(A, 2), 1);
		for ii = 1:size(bound, 1) %#ok<FORPF> no parfor because of variable indexing
			if bound(ii, 2) > 0
				if bound(ii, 1)/bound(ii, 2) < ub(bound(ii, 3), 1)
					ub(bound(ii, 3), 1) = bound(ii, 1)/bound(ii, 2);
				end
			else
				if bound(ii, 1)/bound(ii, 2) > lb(bound(ii, 3), 1)
					lb(bound(ii, 3), 1) = bound(ii, 1)/bound(ii, 2);
				end
			end
		end
		A(singlevariable == 1, :) = [];
		b(singlevariable == 1, :) = [];
		if any(lb > ub)
			error('control:design:gamma', 'Lower bound for gains must not be larger than upper bound.');
		end
	else
		lb = [];
		ub = [];
	end
end
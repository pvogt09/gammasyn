function [c, ceq, gradc, gradceq] = calculate_gain_constraints(x, dimensions, nonlcon)
	%CALCULATE_GAIN_CONSTRAINTS calculate nonlinear constraints on gain matrices
	%	Input:
	%		x:				current optimization value
	%		dimensions:		structure with information about dimensions of the variables and systems and fixed gain parameters for optimization of areafunctions
	%		nonlcon:		structure with function pointer and information about nonlinear gain constraints
	%	Output:
	%		c:				nonlinear inequality constraints on optimization vector
	%		ceq:			nonlinear equality constraints on optimization vector
	%		gradc:			gradient of nonlinear inequalityconstraints on optimization vector
	%		gradceq:		gradient of nonlinear equality constraints on optimization vector
	if nargout >= 5
		error('control:design:gamma:hessian', 'Hessian for user defined constraint function is not yet implemented.');
	end
	number_controls = dimensions.controls;
	number_measurements = dimensions.measurements;
	number_measurements_xdot = dimensions.measurements_xdot;
	number_references = dimensions.references;
	R_fixed_T = dimensions.R_fixed_T;
	K_fixed_T = dimensions.K_fixed_T;
	F_fixed_T = dimensions.F_fixed_T;
	RKF_fixed_T = dimensions.RKF_fixed_T;
	[R, K, F] = x2R(x, dimensions);
	if nargout >= 3
		dimR = number_controls*number_measurements;
		dimK = number_controls*number_measurements_xdot;
		dimF = number_controls*number_references;
		if nonlcon.hasgrad
			[c_R, ceq_R, c_K, ceq_K, c_F, ceq_F, gradc_R, gradceq_R, gradc_K, gradceq_K, gradc_F, gradceq_F] = nonlcon.f(R, K, F);
		else
			[c_R, ceq_R, c_K, ceq_K, c_F, ceq_F] = nonlcon.f(R, K, F);
			delta = 2*sqrt(1e-12)*(1 + norm(x));
			gradc_R = zeros(number_controls, number_measurements, nonlcon.R_c);
			gradceq_R = zeros(number_controls, number_measurements, nonlcon.R_ceq);
			gradc_K = zeros(number_controls, number_measurements_xdot, nonlcon.K_c);
			gradceq_K = zeros(number_controls, number_measurements_xdot, nonlcon.K_ceq);
			gradc_F = zeros(number_controls, number_references, nonlcon.F_c);
			gradceq_F = zeros(number_controls, number_references, nonlcon.F_ceq);
			e_j = zeros(number_controls, number_measurements);
			for jj = 1:number_controls %#ok<FORPF> parfor is not possible because of indexing in e_j
				for kk = 1:number_measurements
					e_j(jj, kk) = 1;
					[gradc_R(jj, kk, :), gradceq_R(jj, kk, :)] = nonlcon.f(R + delta*e_j, K);
					e_j(jj, kk) = 0;
				end
			end
			diffc_R = gradc_R;
			parfor ii = 1:nonlcon.R_c
				diffc_R(:, :, ii) = (gradc_R(:, :, ii) - c_R(ii))/delta
			end
			gradc_R = diffc_R;
			diffceq_R = gradceq_R;
			parfor ii = 1:nonlcon.R_ceq
				diffceq_R(:, :, ii) = (gradceq_R(:, :, ii) - ceq_R(ii))/delta
			end
			gradceq_R = diffceq_R;
			e_j = zeros(number_controls, number_measurements_xdot);
			for jj = 1:number_controls %#ok<FORPF> parfor is not possible because of indexing in e_j
				for kk = 1:number_measurements_xdot
					e_j(jj, kk) = 1;
					[~, ~, gradc_K(jj, kk, :), gradceq_K(jj, kk, :)] = nonlcon.f(R, K + delta*e_j);
					e_j(jj, kk) = 0;
				end
			end
			diffc_K = gradc_K;
			parfor ii = 1:nonlcon.K_c
				diffc_K(:, :, ii) = (gradc_K(:, :, ii) - c_K(ii))/delta
			end
			gradc_K = diffc_K;
			diffceq_K = gradceq_K;
			parfor ii = 1:nonlcon.K_ceq
				diffceq_K(:, :, ii) = (gradceq_K(:, :, ii) - ceq_K(ii))/delta
			end
			gradceq_K = diffceq_K;
			e_j = zeros(number_controls, number_references);
			for jj = 1:number_controls %#ok<FORPF> parfor is not possible because of indexing in e_j
				for kk = 1:number_references
					e_j(jj, kk) = 1;
					[~, ~, ~, ~, gradc_F(jj, kk, :), gradceq_F(jj, kk, :)] = nonlcon.f(R, K, F + delta*e_j);
					e_j(jj, kk) = 0;
				end
			end
			diffc_F = gradc_F;
			parfor ii = 1:nonlcon.F_c
				diffc_F(:, :, ii) = (gradc_F(:, :, ii) - c_F(ii))/delta
			end
			gradc_F = diffc_F;
			diffceq_F = gradceq_F;
			parfor ii = 1:nonlcon.F_ceq
				diffceq_F(:, :, ii) = (gradceq_F(:, :, ii) - ceq_F(ii))/delta
			end
			gradceq_F = diffceq_F;
		end
		c = [
			c_R,	zeros(size(c_R, 1), 1 - size(c_R, 2));
			c_K,	zeros(size(c_K, 1), 1 - size(c_K, 2));
			c_F,	zeros(size(c_F, 1), 1 - size(c_F, 2));
		];
		ceq = [
			ceq_R,	zeros(size(ceq_R, 1), 1 - size(ceq_R, 2));
			ceq_K,	zeros(size(ceq_K, 1), 1 - size(ceq_K, 2));
			ceq_F,	zeros(size(ceq_F, 1), 1 - size(ceq_F, 2))
		];
		if dimensions.RKF_fixed_has
			if isempty(gradc_R)
				gradc_R_opt = zeros(dimR, 0);
			else
				gradc_R_opt = zeros(dimR, size(gradc_R, 3));
				parfor ii = 1:size(gradc_R, 3)
					gradc_R_opt(:, ii) = reshape(gradc_R(:, :, ii), dimR, 1);
				end
			end
			if isempty(gradc_K)
				gradc_K_opt = zeros(dimK, 0);
			else
				gradc_K_opt = zeros(dimK, size(gradc_K, 3));
				parfor ii = 1:size(gradc_K, 3)
					gradc_K_opt(:, ii) = reshape(gradc_K(:, :, ii), dimK, 1);
				end
			end
			if isempty(gradc_F)
				gradc_F_opt = zeros(dimF, 0);
			else
				gradc_F_opt = zeros(dimF, size(gradc_F, 3));
				parfor ii = 1:size(gradc_F, 3)
					gradc_F_opt(:, ii) = reshape(gradc_F(:, :, ii), dimF, 1);
				end
			end
			gradc = [
				gradc_R_opt,										zeros(size(gradc_R_opt, 1), size(gradc_K_opt, 2) + size(gradc_F_opt, 2));
				zeros(size(gradc_K_opt, 1), size(gradc_R_opt, 2)),	gradc_K_opt,			zeros(size(gradc_K_opt, 1), size(gradc_F_opt, 2));
				zeros(size(gradc_F_opt, 1), size(gradc_R_opt, 2) + size(gradc_K_opt, 2)),	gradc_F_opt
			];
			gradc = RKF_fixed_T*gradc;
			gradc = gradc(dimensions.index_RKF_free, :);
		else
			if isempty(gradc_R)
				gradc_R_opt = zeros(dimR - size(dimensions.R_fixed_b, 1), 0);
			else
				gradc_R_temp = zeros(dimR, size(gradc_R, 3));
				parfor ii = 1:size(gradc_R, 3)
					gradc_R_temp(:, ii) = R_fixed_T*reshape(gradc_R(:, :, ii), dimR, 1);
				end
				gradc_R_opt = gradc_R_temp(size(dimensions.R_fixed_b, 1) + 1:end, :);
			end
			if isempty(gradc_K)
				gradc_K_opt = zeros(dimK - size(dimensions.K_fixed_b, 1), 0);
			else
				gradc_K_temp = zeros(dimK, dimK - size(gradc_K, 3));
				parfor ii = 1:size(gradc_K, 3)
					gradc_K_temp(:, ii) = K_fixed_T*reshape(gradc_K(:, :, ii), dimK, 1);
				end
				gradc_K_opt = gradc_K_temp(size(dimensions.K_fixed_b, 1) + 1:end, :);
			end
			if isempty(gradc_F)
				gradc_F_opt = zeros(dimF - size(dimensions.F_fixed_b, 1), 0);
			else
				gradc_F_temp = zeros(dimF, dimF - size(gradc_F, 3));
				parfor ii = 1:size(gradc_F, 3)
					gradc_F_temp(:, ii) = F_fixed_T*reshape(gradc_F(:, :, ii), dimF, 1);
				end
				gradc_F_opt = gradc_F_temp(size(dimensions.F_fixed_b, 1) + 1:end, :);
			end
			gradc = [
				gradc_R_opt,										zeros(size(gradc_R_opt, 1), size(gradc_K_opt, 2) + size(gradc_F_opt, 2));
				zeros(size(gradc_K_opt, 1), size(gradc_R_opt, 2)),	gradc_K_opt,			zeros(size(gradc_K_opt, 1), size(gradc_F_opt, 2));
				zeros(size(gradc_F_opt, 1), size(gradc_R_opt, 2) + size(gradc_K_opt, 2)),	gradc_F_opt
			];
		end
		if dimensions.RKF_fixed_has
			if isempty(gradceq_R)
				gradceq_R_opt = zeros(dimR, 0);
			else
				gradceq_R_opt = zeros(dimR, size(gradceq_R, 3));
				parfor ii = 1:size(gradceq_R, 3)
					gradceq_R_opt(:, ii) = reshape(gradceq_R(:, :, ii), dimR, 1);
				end
			end
			if isempty(gradceq_K)
				gradceq_K_opt = zeros(dimK, 0);
			else
				gradceq_K_opt = zeros(dimK, size(gradceq_K, 3));
				parfor ii = 1:size(gradceq_K, 3)
					gradceq_K_opt(:, ii) = reshape(gradceq_K(:, :, ii), dimK, 1);
				end
			end
			if isempty(gradceq_F)
				gradceq_F_opt = zeros(dimF, 0);
			else
				gradceq_F_opt = zeros(dimF, size(gradceq_F, 3));
				parfor ii = 1:size(gradceq_F, 3)
					gradceq_F_opt(:, ii) = reshape(gradceq_F(:, :, ii), dimF, 1);
				end
			end
			gradceq = [
				gradceq_R_opt,											zeros(size(gradceq_R_opt, 1), size(gradceq_K_opt, 2) + size(gradceq_F_opt, 2));
				zeros(size(gradceq_K_opt, 1), size(gradceq_R_opt, 2)),	gradceq_K_opt,			zeros(size(gradceq_K_opt, 1), size(gradceq_F_opt, 2));
				zeros(size(gradceq_F_opt, 1), size(gradceq_R_opt, 2) + size(gradceq_K_opt, 2)),	gradceq_F_opt
			];
			gradceq = RKF_fixed_T*gradceq;
			gradceq = gradceq(dimensions.index_RKF_free, :);
		else
			if isempty(gradceq_R)
				gradceq_R_opt = zeros(dimR - size(dimensions.R_fixed_b, 1), 0);
			else
				gradceq_R_temp = zeros(dimR, size(gradceq_R, 3));
				parfor ii = 1:size(gradceq_R, 3)
					gradceq_R_temp(:, ii) = R_fixed_T*reshape(gradceq_R(:, :, ii), dimR, 1);
				end
				gradceq_R_opt = gradceq_R_temp(size(dimensions.R_fixed_b, 1) + 1:end, :);
			end
			if isempty(gradceq_K)
				gradceq_K_opt = zeros(dimK - size(dimensions.K_fixed_b, 1), 0);
			else
				gradceq_K_temp = zeros(dimK, size(gradceq_K, 3));
				parfor ii = 1:size(gradceq_K, 3)
					gradceq_K_temp(:, ii) = K_fixed_T*reshape(gradceq_K(:, :, ii), dimK, 1);
				end
				gradceq_K_opt = gradceq_K_temp(size(dimensions.K_fixed_b, 1) + 1:end, :);
			end
			if isempty(gradceq_F)
				gradceq_F_opt = zeros(dimF - size(dimensions.F_fixed_b, 1), 0);
			else
				gradceq_F_temp = zeros(dimF, size(gradceq_F, 3));
				parfor ii = 1:size(gradceq_F, 3)
					gradceq_F_temp(:, ii) = F_fixed_T*reshape(gradceq_F(:, :, ii), dimF, 1);
				end
				gradceq_F_opt = gradceq_F_temp(size(dimensions.F_fixed_b, 1) + 1:end, :);
			end
			gradceq = [
				gradceq_R_opt,												zeros(size(gradceq_R_opt, 1), size(gradceq_K_opt, 2) + size(gradceq_F_opt, 2));
				zeros(size(gradceq_K_opt, 1), size(gradceq_R_opt, 2)),		gradceq_K_opt,		zeros(size(gradceq_K_opt, 1), size(gradceq_F_opt, 2));
				zeros(size(gradceq_F_opt, 1), size(gradceq_R_opt, 2) + size(gradceq_K_opt, 2)),	gradceq_F_opt
			];
		end
	else
		[c_R, ceq_R, c_K, ceq_K, c_F, ceq_F] = nonlcon.f(R, K, F);
		c = [
			c_R,	zeros(size(c_R, 1), 1 - size(c_R, 2));
			c_K,	zeros(size(c_K, 1), 1 - size(c_K, 2));
			c_F,	zeros(size(c_F, 1), 1 - size(c_F, 2));
		];
		ceq = [
			ceq_R,	zeros(size(ceq_R, 1), 1 - size(ceq_R, 2));
			ceq_K,	zeros(size(ceq_K, 1), 1 - size(ceq_K, 2));
			ceq_F,	zeros(size(ceq_F, 1), 1 - size(ceq_F, 2))
		];
	end
end
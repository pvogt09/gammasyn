function [J, gradJ, hessianJ] = calculate_objective_gain(system, R, K, F, dimensions, options)
	%CALCULATE_OBJECTIVE_GAIN helper function for calculation of objective function, gradient and hessian dependent only on the gain matrices for gamma pole placement
	%	Input:
	%		system:						structure with systems to calculate eigenvalues for
	%		R:							proportional gain matrix
	%		K:							derivative gain matrix
	%		F:							prefilter matrix
	%		dimensions:					structure with information about dimensions of the different variables and systems
	%		options:					structure with options for objective function
	%	Output:
	%		J:							objective function value for current optimization value
	%		gradJ:						gradient of objective function value for current optimization value
	%		hessianJ:					hessian of objective function value for current optimization value
	codegen_supports_lyap = coder.const(false);
	codegen_is_generating = coder.const(~coder.target('MATLAB'));
	codegen_use_extrinsic_lyap = ~codegen_supports_lyap && codegen_is_generating;
	number_models = dimensions.models;
	%number_states = dimensions.states;
	number_controls = dimensions.controls;
	number_measurements = dimensions.measurements;
	number_measurements_xdot = dimensions.measurements_xdot;
	number_references = dimensions.references;
	isdiscrete = dimensions.isdiscrete;
	descriptor = dimensions.descriptor;
	derivative_feedback = number_measurements_xdot > 0;
	objective_type = options.type;
	objective_weight = options.weight;
	objective_Q_lyap = options.objective.lyapunov.Q;
	numthreads = options.numthreads;
	if codegen_is_generating && ~codegen_supports_lyap
		% HINT: coder.extrinsic is not working inside of parfor
		numthreads_lyap = 1;
	else
		numthreads_lyap = numthreads;
	end
	J_objective = zeros(size(objective_type, 1), 1);
	J_gradient_gain = zeros(number_controls, number_measurements, size(objective_type, 1));
	J_gradient_gain_xdot = zeros(number_controls, number_measurements_xdot, size(objective_type, 1));
	J_gradient_gain_prefilter = zeros(number_controls, number_references, size(objective_type, 1));
	needsgradient = nargout >= 2;
	needshessian = nargout >= 3;
	number_R_coefficients = number_controls*number_measurements;% number of proportional parameters
	number_K_coefficients = number_controls*number_measurements_xdot; % number of derivative parameters
	number_F_coefficients = number_controls*number_references; % number of prefilter parameters
	number_coefficients = number_R_coefficients + number_K_coefficients + number_F_coefficients; % number of parameters
	hessianJ_RKF = zeros(number_coefficients, number_coefficients, size(objective_type, 1));
	for ii = 1:size(objective_type, 1) %#ok<FORPF> number of objective functions is usually smaller than number of models, so parfor is used for models
		switch objective_type(ii, 1)
			case GammaJType.NORMGAIN
				WR = options.objective.normgain.R.*(R - options.objective.normgain.R_shift);
				WK = options.objective.normgain.K.*(K - options.objective.normgain.K_shift);
				WF = options.objective.normgain.F.*(F - options.objective.normgain.F_shift);
				J_objective(ii, 1) = objective_weight(ii, 1)*(trace(WR'*WR) + trace(WK'*WK) + trace(WF'*WF));
				if needsgradient
					J_gradient_gain(:, :, ii) = objective_weight(ii, 1)*2*options.objective.normgain.R.*WR;
					if derivative_feedback
						J_gradient_gain_xdot(:, :, ii) = objective_weight(ii, 1)*2*options.objective.normgain.K.*WK;
					end
					J_gradient_gain_prefilter(:, :, ii) = objective_weight(ii, 1)*2*options.objective.normgain.F.*WF;
					if needshessian
						hessianJ_RKF(1:number_R_coefficients, 1:number_R_coefficients, ii) = objective_weight(ii, 1)*2*reshape(options.objective.normgain.R, [], 1)*reshape(options.objective.normgain.R, 1, []);
						if derivative_feedback
							hessianJ_RKF(number_R_coefficients + 1:number_R_coefficients + number_K_coefficients, number_R_coefficients + 1:number_R_coefficients + number_K_coefficients, ii) = objective_weight(ii, 1)*2*reshape(options.objective.normgain.K, [], 1)*reshape(options.objective.normgain.K, 1, []);
						end
						if number_references > 0
							hessianJ_RKF(number_R_coefficients + number_K_coefficients + 1:number_R_coefficients + number_K_coefficients + number_F_coefficients, number_R_coefficients + number_K_coefficients + 1:number_R_coefficients + number_K_coefficients + number_F_coefficients, ii) = objective_weight(ii, 1)*2*reshape(options.objective.normgain.F, [], 1)*reshape(options.objective.normgain.F, 1, []);
						end
					end
				end
			case GammaJType.LYAPUNOV
				if needshessian
					% TODO: calculate hessian for lyapunov objective function (is always zero?)
					error('control:design:gamma:hessian', 'Hessian for Lyapunov objective function is not yet implemented.');
				end
				J_gradient_gain_temp = zeros(number_controls, number_measurements, number_models);
				J_gradient_gain_xdot_temp = zeros(number_controls, number_measurements_xdot, number_models);
				J_gradient_gain_prefilter_temp = zeros(number_controls, number_references, number_models);
				J_objective_gain_temp = zeros(number_models, 1);
				parfor (kk = 1:number_models, numthreads_lyap)
					% closed loop system matrix
					A = system(kk).A - system(kk).B*R*system(kk).C;
					system_order = size(A, 1);
					% closed loop descriptor matrix
					if descriptor
						if derivative_feedback
							E = system(kk).E - system(kk).B*K*system(kk).C_dot;
						else
							E = system(kk).E;
						end
					else
						if derivative_feedback
							E = eye(system_order) - system(kk).B*K*system(kk).C_dot;
						else
							E = eye(system_order);
						end
					end
					if descriptor || derivative_feedback
						eigenvalues = eig(A, E);
					else
						eigenvalues = eig(A);
					end
					% tolerance of lyap function
					if descriptor || derivative_feedback
						tolerance = sqrt(eps)*max(1, norm(A, 'fro'))/min(1, norm(E, 'fro'));
					else
						tolerance = sqrt(eps)*max(1, norm(A, 'fro'));
					end
					sign = 1;
					P = zeros(system_order, system_order);
					P_stable = zeros(system_order, system_order);
					P_unstable = zeros(system_order, system_order);
					% weighing matrix
					Q_all = objective_Q_lyap(:, :, kk);
					Q = Q_all(1:size(A, 1), 1:size(A, 2));
					if isdiscrete
						no_unique_solution = any(abs(abs(eigenvalues) - 1) <= tolerance) || any(any(abs(eigenvalues*eigenvalues' - ones(system_order, system_order)) <= tolerance));
						isunstable = any(abs(eigenvalues) > 1 + tolerance);
						if all(abs(eigenvalues) > 1 + tolerance)
							sign = -1;
						end
					else
						no_unique_solution = any(abs(real(eigenvalues)) <= tolerance) || any(any(abs(eigenvalues*ones(1, system_order) + ones(system_order, 1)*eigenvalues.') <= tolerance));% eigenvalue_i + eigenvalue_j ~= 0
						isunstable = any(real(eigenvalues) > tolerance);
						if all(real(eigenvalues) > tolerance)
							sign = -1;
						end
					end
					if no_unique_solution
						% J_objective_gain_temp is set to zero if unique solution does not exists
						J_objective_gain_temp(kk, 1) = 0;
						if needsgradient
							J_gradient_gain_temp(:, :, kk) = zeros(number_controls, number_measurements);
							J_gradient_gain_xdot_temp(:, :, kk) = zeros(number_controls, number_measurements_xdot);
						end
					else
						% lyapunov matrix of closed loop system
						if codegen_use_extrinsic_lyap
							if descriptor || derivative_feedback
								P = calculate_objective_gain_lyap_wrapper_extrinsic(isdiscrete, A', Q, [], E');
							else
								P = calculate_objective_gain_lyap_wrapper_extrinsic(isdiscrete, A', Q);
							end
						else
							if descriptor || derivative_feedback
								P = calculate_objective_gain_lyap_wrapper_intrinsic(isdiscrete, A', Q, [], E');
							else
								P = calculate_objective_gain_lyap_wrapper_intrinsic(isdiscrete, A', Q);
							end
						end
						% initialize variables for parfor
						A_stabilized = A;
						A_unstable_inv = A;
						A_Schur = A;
						U = A;
						dU = U;
						if ~(descriptor || derivative_feedback) && isunstable
							% TODO: handle descriptor case similarly as well and use [#ANDERSON1987] for derivative feedback too?
							% if system is not a descriptor system and has unstable eigenvalues:
							% calculation for Schur decomposed system (unstable eigenvalues treated as stable)
							[P_stable, P_unstable, P, U, A_unstable_inv, A_stabilized, A_Schur] = calculate_objective_gain_lyap_decomposed(isdiscrete, codegen_use_extrinsic_lyap, tolerance, A, Q);
							if isempty(P_stable)
								J_objective_gain_temp(kk, 1) = 1/trace(P_unstable'*P_unstable);
							else
								J_objective_gain_temp(kk, 1) = 1/trace(P_unstable'*P_unstable) - 1/trace(P_stable'*P_stable);
							end
						else
							J_objective_gain_temp(kk, 1) = -sign/trace(P'*P);
						end
						if needsgradient
							% another set of temporary variables for correct indexing/slicing in parfor
							J_gradient_gain_temp_parfor = zeros(number_controls, number_measurements);
							% no conditional assignment to avoid "The temporary variable J_gradient_gain_xdot_temp_parfor will be cleared at the beginning of each iteration of the parfor loop."
							J_gradient_gain_xdot_temp_parfor = zeros(number_controls, number_measurements_xdot);
							for ll = 1:number_controls
								for mm = 1:number_measurements
									R_derivative = zeros(number_controls, number_measurements);
									R_derivative(ll, mm) = 1;
									A_derivative = (-system(kk).B*R_derivative*system(kk).C);
									A_stabilized_derivative = A_derivative;
									if isdiscrete
										if ~(descriptor || derivative_feedback) && isunstable
											% derivative of U according to [#ANDERSON1987, A procedure for differentiating perfect-foresight-model reduced-from coefficients]
											dU_vec = (kron(A_Schur(1:size(P_unstable, 1), 1:size(P_unstable, 2))', eye(size(A, 1) - size(P_unstable, 1))) - kron(A_Schur(size(P_unstable, 1) + 1:end, size(P_unstable, 2) + 1:end), eye(size(P_unstable, 1))))\reshape(U(:, size(P_unstable, 1) + 1:end)'*A_derivative*U(:, 1:size(P_unstable, 1)), [], 1);
											dU_block = reshape(dU_vec, size(P_stable, 1), size(P_unstable, 2));
											dU = U*[
												zeros(size(dU_block, 2), size(dU_block, 2)),	-dU_block';
												dU_block,										zeros(size(dU_block, 1), size(dU_block, 1))
											];
											% derivative of corrected system matrix in original coordinates
											% A_correct = U*(U'*A*U + [I;0]*(-[I, 0]*U'*A*U*[I;0] + inv([I, 0]*U'*A*U*[I;0]))*[I, 0])*U'
											selector = blkdiag(eye(size(P_unstable, 1)), zeros(size(A, 1) - size(P_unstable, 1), size(A, 2) - size(P_unstable, 2)));
											dx = selector(1:size(P_unstable, 1), :)*(dU'*A*U + U'*A_derivative*U + U'*A*dU)*selector(:, 1:size(P_unstable, 2));
											A_stabilized_derivative = A_derivative + (...
												- dU*selector*U'*A*U*selector*U' - U*selector*dU'*A*U*selector*U' - U*selector*U'*A_derivative*U*selector*U' - U*selector*U'*A*dU*selector*U' - U*selector*U'*A*U*selector*dU'...
											) + (...
												dU*selector(:, 1:size(P_unstable, 2))*A_unstable_inv*selector(1:size(P_unstable, 1), :)*U' + U*selector(:, 1:size(P_unstable, 2))*A_unstable_inv*selector(1:size(P_unstable, 1), :)*dU'...
											) + (...
												U*selector(:, 1:size(P_unstable, 2))*(-A_unstable_inv*dx*A_unstable_inv)*selector(1:size(P_unstable, 1), :)*U'...
											);
										else
											dU = zeros(size(A, 1), size(A, 2));
										end
										Q_gradient = A_stabilized_derivative'*P*A_stabilized + A_stabilized'*P*A_stabilized_derivative;
									else
										if descriptor || derivative_feedback
											Q_gradient = E'*P*A_derivative + A_derivative'*P*E;
										else
											Q_gradient = P*A_derivative + A_derivative'*P;
										end
									end
									Q_gradient = (Q_gradient + Q_gradient')/2;
									if ~(descriptor || derivative_feedback) && isunstable
										% calculate derivative of P in Schur coordinates in dependance of derivative of P in original coordinates
										if codegen_use_extrinsic_lyap
											if descriptor || derivative_feedback
												P_derivative = calculate_objective_gain_lyap_wrapper_extrinsic(isdiscrete, A_stabilized', Q_gradient, [], E');
											else
												P_derivative = calculate_objective_gain_lyap_wrapper_extrinsic(isdiscrete, A_stabilized', Q_gradient);
											end
										else
											if descriptor || derivative_feedback
												P_derivative = calculate_objective_gain_lyap_wrapper_intrinsic(isdiscrete, A_stabilized', Q_gradient, [], E');
											else
												P_derivative = calculate_objective_gain_lyap_wrapper_intrinsic(isdiscrete, A_stabilized', Q_gradient);
											end
										end
										if isdiscrete
											P_Schur = U'*P*U;
											P_derivative = U'*P_derivative*U - U'*dU*P_Schur - P_Schur*dU'*U;
											P_derivative_stable = P_derivative(size(P_unstable, 1) + 1:end, size(P_unstable, 2) + 1:end);
											P_derivative_unstable = P_derivative(1:size(P_unstable, 1), 1:size(P_unstable, 2));
										else
											[P_derivative_stable, P_derivative_unstable] = calculate_objective_gain_lyap_decomposed(isdiscrete, codegen_use_extrinsic_lyap, tolerance, A, Q_gradient);
										end
										% calculate derivative of objective
										if isdiscrete
											dP_unstable_dA_unstable = 1;
										else
											dP_unstable_dA_unstable = -1;
										end
										if isempty(P_stable)
											J_gradient_gain_temp_parfor(ll, mm) = -dP_unstable_dA_unstable*2*trace(P_unstable'*P_derivative_unstable)/trace(P_unstable'*P_unstable)^2;
										else
											J_gradient_gain_temp_parfor(ll, mm) = -dP_unstable_dA_unstable*2*trace(P_unstable'*P_derivative_unstable)/trace(P_unstable'*P_unstable)^2 + 2*trace(P_stable'*P_derivative_stable)/trace(P_stable'*P_stable)^2;
										end
									else
										if codegen_use_extrinsic_lyap
											if descriptor || derivative_feedback
												P_derivative = calculate_objective_gain_lyap_wrapper_extrinsic(isdiscrete, A', Q_gradient, [], E');
											else
												P_derivative = calculate_objective_gain_lyap_wrapper_extrinsic(isdiscrete, A', Q_gradient);
											end
										else
											if descriptor || derivative_feedback
												P_derivative = calculate_objective_gain_lyap_wrapper_intrinsic(isdiscrete, A', Q_gradient, [], E');
											else
												P_derivative = calculate_objective_gain_lyap_wrapper_intrinsic(isdiscrete, A', Q_gradient);
											end
										end
										dtrP = trace(P'*P_derivative);
										trP = trace(P'*P);
										if isinf(dtrP) && isinf(trP)
											J_gradient_gain_temp_parfor(ll, mm) = 0;
										else
											J_gradient_gain_temp_parfor(ll, mm) = 2*sign*dtrP/trP^2;
										end
									end
								end
								if derivative_feedback
									for mm = 1:number_measurements_xdot
										K_derivative = zeros(number_controls, number_measurements_xdot);
										K_derivative(ll, mm) = 1;
										if isdiscrete
											Q_gradient = -(-system(kk).B*K_derivative*system(kk).C_dot)'*P*E - E'*P*(-system(kk).B*K_derivative*system(kk).C_dot);
										else
											Q_gradient = (-system(kk).B*K_derivative*system(kk).C_dot)'*P*A_stabilized + A_stabilized'*P*(-system(kk).B*K_derivative*system(kk).C_dot);
										end
										Q_gradient = (Q_gradient + Q_gradient')/2;
										if ~(descriptor || derivative_feedback) && isunstable
											[P_derivative_stable, P_derivative_unstable] = calculate_objective_gain_lyap_decomposed(isdiscrete, codegen_use_extrinsic_lyap, tolerance, A, Q_gradient);
											if isdiscrete
												dP_unstable_dA_unstable = 1;
											else
												dP_unstable_dA_unstable = -1;
											end
											if isempty(P_stable)
												J_gradient_gain_xdot_temp_parfor(ll, mm) = -dP_unstable_dA_unstable*2*trace(P_unstable'*P_derivative_unstable)/trace(P_unstable'*P_unstable)^2;
											else
												J_gradient_gain_xdot_temp_parfor(ll, mm) = -dP_unstable_dA_unstable*2*trace(P_unstable'*P_derivative_unstable)/trace(P_unstable'*P_unstable)^2 + 2*trace(P_stable'*P_derivative_stable)/trace(P_stable'*P_stable)^2;
											end
										else
											if codegen_use_extrinsic_lyap
												if descriptor || derivative_feedback
													P_derivative = calculate_objective_gain_lyap_wrapper_extrinsic(isdiscrete, A', Q_gradient, [], E');
												else
													P_derivative = calculate_objective_gain_lyap_wrapper_extrinsic(isdiscrete, A', Q_gradient);
												end
											else
												if descriptor || derivative_feedback
													P_derivative = calculate_objective_gain_lyap_wrapper_intrinsic(isdiscrete, A', Q_gradient, [], E');
												else
													P_derivative = calculate_objective_gain_lyap_wrapper_intrinsic(isdiscrete, A', Q_gradient);
												end
											end
											dtrP_xdot = trace(P'*P_derivative);
											trP_xdot = trace(P'*P);
											if isinf(trP_xdot) && isinf(dtrP_xdot)
												J_gradient_gain_xdot_temp_parfor(ll, mm) = 0;
											else
												J_gradient_gain_xdot_temp_parfor(ll, mm) = 2*sign*dtrP_xdot/dtrP_xdot^2;
											end
										end
									end
								end
							end
							J_gradient_gain_temp(:, :, kk) = J_gradient_gain_temp_parfor;
							if derivative_feedback
								J_gradient_gain_xdot_temp(:, :, kk) = J_gradient_gain_xdot_temp_parfor;
							end
							J_gradient_gain_prefilter_temp(:, :, kk) = zeros(number_controls, number_references);
						end
					end
				end
				J_objective(ii, 1) = objective_weight(ii, 1)*sum(J_objective_gain_temp, 1);
				if needsgradient
					J_gradient_gain(:, :, ii) = objective_weight(ii, 1)*sum(J_gradient_gain_temp, 3);
					if derivative_feedback
						J_gradient_gain_xdot(:, :, ii) = objective_weight(ii, 1)*sum(J_gradient_gain_xdot_temp, 3);
					end
					J_gradient_gain_prefilter(:, :, ii) = objective_weight(ii, 1)*sum(J_gradient_gain_prefilter_temp, 3);
					if needshessian
					end
				end
			otherwise
				continue;
		end
	end
	J = sum(J_objective);
	if needsgradient
		% codegen crashes when concatenating empty variable size matrices
		if derivative_feedback
			if isempty(J_gradient_gain_prefilter)
				if isempty(J_gradient_gain)
					gradJ = sum(J_gradient_gain_xdot, 3);
				else
					gradJ = cat(2, sum(J_gradient_gain, 3), sum(J_gradient_gain_xdot, 3));
				end
			else
				if isempty(J_gradient_gain)
					gradJ = cat(2, sum(J_gradient_gain_xdot, 3), sum(J_gradient_gain_prefilter, 3));
				else
					gradJ = cat(2, sum(J_gradient_gain, 3), sum(J_gradient_gain_xdot, 3), sum(J_gradient_gain_prefilter, 3));
				end
			end
		else
			if isempty(J_gradient_gain_prefilter)
				gradJ = sum(J_gradient_gain, 3);
			else
				if isempty(J_gradient_gain)
					gradJ = sum(J_gradient_gain_prefilter, 3);
				else
					gradJ = cat(2, sum(J_gradient_gain, 3), sum(J_gradient_gain_prefilter, 3));
				end
			end
		end
		if needshessian
			hessianJ = sum(hessianJ_RKF, 3);
		end
	end
end
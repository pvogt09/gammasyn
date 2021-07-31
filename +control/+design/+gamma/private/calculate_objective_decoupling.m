function [J, gradJ, hessJ] = calculate_objective_decoupling(R, ~, F, systems, dimensions, options)
	%CALCULATE_OBJECTIVE_DECOUPLING calculates quadratic objective function, gradient and hessian that ensures decoupling
	%	Input:
	%		R:				proportional gain matrix
	%		K:				derivative gain matrix
	%		F:				prefilter gain matrix
	%		systems:		structure with system description
	%		dimensions:		structure with dimensions information
	%		options:		options structure
	%	Output:
	%		J:				objective function
	%		gradJ:			gradient of objective function
	%		hessJ:			hessian of gradient function
	needsgradient = nargout >= 2;
	needshessian = 	nargout >= 3;
	number_models = dimensions.models;
	number_controls = dimensions.controls;
	number_states = dimensions.states;
	number_measurements = dimensions.measurements;
	number_measurements_xdot = dimensions.measurements_xdot;
	number_references = dimensions.references;
	V_invariant = dimensions.V_invariant;
	m_invariant = dimensions.m_invariant;
	hasfeedthrough = dimensions.hasfeedthrough_decoupling;
	number_decouplingconditions = dimensions.number_decouplingconditions;
	tf_structure = options.decouplingcontrol.tf_structure;
	objective_type = options.type;
	objective_weight = options.weight;
	r = reshape(R, [], 1);

	J_decoupling = zeros(size(objective_type, 1), 1);
	gradJ_decoupling = zeros(number_controls*(number_measurements + number_measurements_xdot + number_references), size(objective_type, 1));
	hessJ_decoupling = zeros(number_controls*(number_measurements + number_measurements_xdot + number_references), number_controls*(number_measurements + number_measurements_xdot + number_references), size(objective_type, 1));
	for kk = 1:size(objective_type, 1) %#ok<FORPF> number of objective functions is usually smaller than number of models, so parfor is used for models
		switch objective_type(kk, 1)
			case GammaJType.DECOUPLING
				J_decoupling_temp = zeros(number_models, 1);
				gradJ_decoupling_temp = zeros(number_models, number_controls*(number_measurements + number_measurements_xdot + number_references), 1);
				hessJ_decoupling_temp = zeros(number_models, number_controls*(number_measurements + number_measurements_xdot + number_references), number_controls*(number_measurements + number_measurements_xdot + number_references));
				parfor(ii = 1:number_models, options.numthreads)
					J_decoupling_temp_parfor = J_decoupling_temp(ii, 1);
					gradJ_decoupling_temp_parfor = gradJ_decoupling_temp(ii, :, 1);
					hessJ_decoupling_temp_parfor = hessJ_decoupling_temp(ii, :, :);
					A  = systems(ii).A;
					B  = systems(ii).B;
					C  = systems(ii).C;
					C_ref = systems(ii).C_ref;
					D_ref = systems(ii).D_ref;
					V_ii = V_invariant(ii, :, :, :);
					for jj = 1:number_references
						g_structure = tf_structure(:, jj); %#ok<PFBNS>
						Cjj = C_ref(g_structure == 0, :);
						Djj = D_ref(g_structure == 0, :);
						m = m_invariant(jj); %#ok<PFBNS>
						if m == 0
							continue;
						end
						Q = reshape(V_ii(1, :, 1:m, jj), number_states, m);
						Q_orth = reshape(V_ii(1, :, m + 1:end, jj), number_states, number_states - m);

						if hasfeedthrough(jj) %#ok<PFBNS>
							X_R = [
								kron(Q'*C', Q_orth'*B);
								kron(Q'*C', Djj)
							];
							z_R = [
								reshape(Q_orth'*A*Q, (number_states - m)*m, 1);
								reshape(Cjj*Q, number_decouplingconditions(jj)*m, 1)
							]; %#ok<PFBNS>
						else
							X_R = kron(Q'*C', Q_orth'*B);
							z_R = reshape(Q_orth'*A*Q, (number_states - m)*m, 1);
						end
						e_R = X_R*r - z_R;
						J_R = e_R.'*e_R;

						f = F(:, jj); %#ok<PFBNS>
						if hasfeedthrough(jj)
							X_F = [
								Q_orth.'*B;
								Djj
							];
						else
							X_F = Q_orth.'*B;
						end
						e_F = X_F*f;
						J_F = e_F.'*e_F;

						J_decoupling_temp_parfor = J_decoupling_temp_parfor + J_R + J_F;
						if needsgradient
							gradJ_decoupling_temp_parfor = gradJ_decoupling_temp_parfor + 2*[
								r.'*(X_R.'*X_R) - z_R.'*X_R, zeros(1, number_controls*number_measurements_xdot), zeros(1, (jj - 1)*number_controls), f.'*(X_F.'*X_F), zeros(1, (number_references - jj)*number_controls)
							].';
						end
						if needshessian
							hessJ_tmp = 2*blkdiag(...
								X_R.'*X_R,...
								zeros(number_controls*number_measurements_xdot, number_controls*number_measurements_xdot),...
								zeros((jj - 1)*number_controls, (jj - 1)*number_controls),...
								X_F.'*X_F,...
								zeros((number_references - jj)*number_controls, (number_references - jj)*number_controls)...
							);
							hessJ_decoupling_temp_parfor = hessJ_decoupling_temp_parfor + hessJ_tmp;
						end
					end
					J_decoupling_temp(ii, 1) = J_decoupling_temp_parfor;
					if needsgradient
						gradJ_decoupling_temp(ii, :, 1) = gradJ_decoupling_temp_parfor;
						if needshessian
							hessJ_decoupling_temp(ii, :, :) = hessJ_decoupling_temp_parfor;
						end
					end
				end
				J_decoupling(kk, 1) = objective_weight(kk, 1).*sum(J_decoupling_temp(:));
				gradJ_decoupling(:, kk) = objective_weight(kk, 1).*sum(gradJ_decoupling_temp, 1);
				hessJ_decoupling(:, :, kk) = objective_weight(kk, 1).*sum(hessJ_decoupling_temp, 1);
			otherwise
				continue;
		end
	end
	J = sum(J_decoupling(:));
	if needsgradient
		gradJ = sum(gradJ_decoupling, 2);
		if needshessian
			hessJ = sum(hessJ_decoupling, 3);
		end
	end
end
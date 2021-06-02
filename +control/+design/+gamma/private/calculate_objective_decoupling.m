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
	r = reshape(R, [], 1);

	J = 0;
	gradJ = zeros(number_controls*(number_measurements + number_measurements_xdot + number_references), 1);
	hessJ = zeros(number_controls*(number_measurements + number_measurements_xdot + number_references), number_controls*(number_measurements + number_measurements_xdot + number_references));
	parfor(ii = 1:number_models, options.numthreads)
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

			J = J + J_R + J_F;
			if needsgradient
				gradJ = gradJ + 2*[
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
				hessJ = hessJ + hessJ_tmp;
			end
		end
	end
end
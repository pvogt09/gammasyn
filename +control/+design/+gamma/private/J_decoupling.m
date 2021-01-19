function [J, gradJ] = J_decoupling(x, systems, ~, ~, dimensions, options, ~)

	tic;
	if nargout == 2
		error('control:design:gamma:gradient', 'Gradient for J_decoupling not implemented.');
	end
	if nargout == 3
		error('control:design:gamma:hessian', 'Hessian for J_decoupling not implemented.');
	end
	number_models = dimensions.models;
	number_controls = dimensions.controls;
	number_states = dimensions.states;
	number_measurements = dimensions.measurements;
	number_measurements_xdot = dimensions.measurements_xdot;
	number_references = dimensions.references;
	J = 0;
	gradJ = zeros(number_controls*(number_measurements + number_measurements_xdot + number_references), 1);
	[R, ~, F] = x2R(x, dimensions);
	r = reshape(R, [], 1);
	
	tf_structure = options.decouplingcontrol.tf_structure;
	
	for ii = 1:number_models
		A  = systems(ii).A;
		B  = systems(ii).B;
		C  = systems(ii).C;
		C_ref = systems(ii).C_ref;
% 		D_ref = systems(ii).D_ref;
% 		E  = systems(ii).E;
% 		null_C = null(C);
		for jj = 1:number_references
			g_structure = tf_structure(:, jj);
			Cjj = C_ref(g_structure == 0, :);
			if isempty(Cjj)
				continue;
			end
% 			Djj = D_ref(g_structure == 0, :);
			Q = mainco(A, B, null(Cjj));
			m = size(Q, 2);
			Q_orth = null(Q.');
			A11 = Q'*(A-B*R*C)*Q;
			A21 = Q_orth.'*(A-B*R*C)*Q;
			X_1 = kron(Q'*C', Q'*B);
			z_1 = reshape(Q'*A*Q, m^2, 1);
			X_2 = kron(Q'*C', Q_orth'*B);
			z_2 = reshape(Q_orth'*A*Q, (number_states - m)*m, 1);
			for kk = 1:m
				J = J + sum(A21(:, kk).^2)/sum(A11(:, kk).^2);
				X_1_kk = X_1((kk - 1)*m + (1:m), :);
				z_1_kk = z_1((kk - 1)*m + (1:m), 1);
				X_2_kk = X_2((kk - 1)*(number_states - m) + (1:number_states - m), :);
				z_2_kk = z_2((kk - 1)*(number_states - m) + (1:number_states - m), 1);
				e_1 = X_1_kk*r - z_1_kk;
				e_2 = X_2_kk*r - z_2_kk;
				gradJ = gradJ + [ 2/(e_1.'*e_1)^2*(X_2_kk.'*e_2*(e_1.'*e_1) - X_1_kk.'*e_1*(e_2.'*e_2)); zeros(number_controls*(number_measurements_xdot + number_references), 1)];
			end
			J = J + sum((Q_orth.'*B*F(:, jj)).^2);
			gradJ = gradJ + [zeros(number_controls*(number_measurements + number_measurements_xdot), 1); zeros((jj - 1)*number_controls, 1); 2*B.'*(Q_orth.'*Q_orth)*B*F(:, jj); zeros((number_references - jj)*number_controls, 1)];
		end
	end
	gradJ = gradJ.';
	toc;
end
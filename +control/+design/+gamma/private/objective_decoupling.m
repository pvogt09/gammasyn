function [J, gradJ, hessJ] = objective_decoupling(R, ~, F, systems, dimensions, options)
	%OBJECTIVE_DECOUPLING calculates quadratic objective function, gradient and hessian that ensures decoupling
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
	tf_structure = options.decouplingcontrol.tf_structure;
	r = reshape(R, [], 1);
	
	J = 0;
	gradJ = zeros(number_controls*(number_measurements + number_measurements_xdot + number_references), 1);
	hessJ = zeros(number_controls*(number_measurements + number_measurements_xdot + number_references), number_controls*(number_measurements + number_measurements_xdot + number_references));
	parfor(ii = 1:number_models, options.numthreads)
		A  = systems(ii).A;
		B  = systems(ii).B;
		C  = systems(ii).C;
		D_ref = systems(ii).D_ref;
		for jj = 1:number_references
			g_structure = tf_structure(:, jj); %#ok<PFBNS>
			Djj = D_ref(g_structure == 0, :);
			if any(any(Djj ~= 0))
				error('control:design:gamma:decoupling', 'Merit function for decoupling not implemented for feedthrough.');
			end
			m = m_invariant(jj); %#ok<PFBNS>
			if m == 0
				continue;
			end
			Q = V_invariant(:, 1:m, ii, jj); %#ok<PFBNS>
			Q_orth = V_invariant(:, m + 1:end, ii, jj);
			
			X = kron(Q'*C', Q_orth'*B);
			z = reshape(Q_orth'*A*Q, (number_states - m)*m, 1);
			e = X*r - z;
			f = F(:, jj); %#ok<PFBNS>
			J = J + e.'*e + sum((Q_orth.'*B*f).^2);
			if needsgradient
				gradJ = gradJ + 2*[
					r.'*(X.'*X) - z.'*X, zeros(1, number_controls*number_measurements_xdot), zeros(1, (jj - 1)*number_controls), f.'*B.'*(Q_orth*Q_orth.')*B, zeros(1, (number_references - jj)*number_controls)
				].';
			end
			if needshessian
				hessJ_tmp = 2*blkdiag(...
					X.'*X,...
					zeros(number_controls*number_measurements_xdot, number_controls*number_measurements_xdot),...
					zeros((jj - 1)*number_controls, (jj - 1)*number_controls),...
					B.'*(Q_orth*Q_orth.')*B,...
					zeros((number_references - jj)*number_controls, (number_references - jj)*number_controls)...
				);
				hessJ = hessJ + hessJ_tmp;
			end
		end
	end
end
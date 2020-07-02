function [J, gradJ] = calculate_objective_eigenvector(system, ~, ~, ~, dimensions, options, eigenvector_right, eigenvector_left, eigenvector_right_derivative, eigenvector_right_derivative_xdot, eigenvector_left_derivative, eigenvector_left_derivative_xdot)
	%CALCULATE_OBJECTIVE_EIGENVECTOR helper function for calculation of objective function and gradient dependent on eigenvectors for gamma pole placement
	%	Input:
	%		system:								structure with system matrices of systems to take into consideration
	%		R:									proportional gain matrix
	%		K:									derivative gain matrix
	%		F:									prefilter matrix
	%		dimensions:							structure with information about dimensions of the different variables and systems
	%		options:							structure with options for objective function
	%		eigenvector_right:					matrix of right eigenvectors of all systems
	%		eigenvector_left:					matrix of right eigenvectors of all systems
	%		eigenvector_right_derivative:		derivative of right eigenvectors with respect to proportional gain
	%		eigenvector_right_derivative_xdot:	derivative of right eigenvectors with respect to derivative gain
	%		eigenvector_left_derivative:		derivative of left eigenvectors with respect to proportional gain
	%		eigenvector_left_derivative_xdot:	derivative of left eigenvectors with respect to derivative gain
	%	Output:
	%		J:									objective function value for current optimization value
	%		gradJ:								gradient of objective function value for current optimization value
	number_models = dimensions.models;
	number_states = dimensions.states;
	number_controls = dimensions.controls;
	number_measurements = dimensions.measurements;
	number_measurements_xdot = dimensions.measurements_xdot;
	number_references = dimensions.references;
	derivative_feedback = number_measurements_xdot > 0;
	objective_type = options.type;
	objective_weight = options.weight;
	numthreads = options.numthreads;
	needsgradient = nargout >= 2;
	haseigenvector = nargin >= 7;
	haseigenvectorleft = nargin >= 8;
	haseigenvectorderivative = nargin >= 9;
	haseigenvectorderivative_xdot = nargin >= 10;
	haseigenvectorderivativeleft = nargin >= 11;
	haseigenvectorderivativeleft_xdot = nargin >= 12;
	J_objective_eigenvector = zeros(size(objective_type, 1), 1);
	J_gradient_eigenvector = zeros([number_controls, number_measurements, size(objective_type, 1)]);
	J_gradient_eigenvector_xdot = zeros([number_controls, number_measurements_xdot, size(objective_type, 1)]);
	J_gradient_eigenvector_prefilter = zeros([number_controls, number_references, size(objective_type, 1)]);
	if any(objective_type == GammaJType.EIGENVALUECONDITION)
		if ~haseigenvector
			% needed for code generation
			eigenvector_right = NaN(number_states, number_states, number_models) + 0i;
		end
		if ~haseigenvectorleft
			% needed for code generation
			eigenvector_left = NaN(number_states, number_states, number_models) + 0i;
		end
		eigenvectorderivativeavailable = false;
		eigenvectorderivativeavailable_xdot = false;
		if ~haseigenvector || isempty(eigenvector_right)
			error('control:design:gamma:gradient', 'Right eigenvectors were not supplied.');
		end
		if ~haseigenvectorleft || isempty(eigenvector_left)
			error('control:design:gamma:gradient', 'Left eigenvectors were not supplied.');
		end
		if needsgradient
			if ~haseigenvectorderivative
				eigenvector_right_derivative = NaN(number_states, number_states, number_controls, number_measurements, number_models) + 0i;
			end
			if ~haseigenvectorderivativeleft
				eigenvector_left_derivative = NaN(number_states, number_states, number_controls, number_measurements, number_models) + 0i;
			end
			if ~haseigenvectorderivative || ~haseigenvectorderivativeleft || isempty(eigenvector_left_derivative) || isempty(eigenvector_right_derivative)
				error('control:design:gamma:gradient', 'Gradient for eigenvectors was not supplied.');
			else
				eigenvectorderivativeavailable = true;
			end
			if derivative_feedback
				if ~haseigenvectorderivative_xdot || ~haseigenvectorderivativeleft_xdot || isempty(eigenvector_left_derivative_xdot) || isempty(eigenvector_right_derivative_xdot)
					error('control:design:gamma:gradient', 'Gradient for eigenvectors was not supplied.');
				else
					eigenvectorderivativeavailable_xdot = true;
				end
			else
				% needed for code generation
				eigenvector_right_derivative_xdot = NaN(number_states, number_states, number_controls, number_measurements_xdot, number_models) + 0i;
				eigenvector_left_derivative_xdot = NaN(number_states, number_states, number_controls, number_measurements_xdot, number_models) + 0i;
			end
		else
			% TODO: for some reason parfor crashes with 'not enough input arguments' if these variables are not defined, while in the generated code everything works fine
			eigenvector_right_derivative = NaN(number_states, number_states, number_controls, number_measurements, number_models) + 0i;
			eigenvector_left_derivative = NaN(number_states, number_states, number_controls, number_measurements, number_models) + 0i;
			eigenvector_right_derivative_xdot = NaN(number_states, number_states, number_controls, number_measurements_xdot, number_models) + 0i;
			eigenvector_left_derivative_xdot = NaN(number_states, number_states, number_controls, number_measurements_xdot, number_models) + 0i;
		end
		if needsgradient && ~eigenvectorderivativeavailable
			error('control:design:gamma:gradient', 'Gradient for eigenvectors was not supplied.');
		end
		if needsgradient && derivative_feedback && ~eigenvectorderivativeavailable_xdot
			error('control:design:gamma:gradient', 'Gradient for eigenvectors was not supplied.');
		end
		if size(eigenvector_right, 3) ~= number_models || size(eigenvector_left, 3) ~= number_models
			error('control:design:gamma:gradient', 'Eigenvectors were not supplied.');
		end
		for ii = 1:size(objective_type, 1) %#ok<FORPF> number of objective functions is usually smaller than number of models, so parfor is used for models
			switch objective_type(ii, 1)
				case GammaJType.EIGENVALUECONDITION
					J_gradient_eigenvector_temp = zeros(number_controls, number_measurements, number_models);
					J_gradient_eigenvector_xdot_temp = zeros(number_controls, number_measurements_xdot, number_models);
					J_gradient_eigenvector_prefilter_temp = zeros(number_controls, number_references, number_models);
					J_objective_eigenvector_temp = zeros(number_models, 1);
					% TODO: vectorize multiplication and trace
					parfor (kk = 1:number_models, numthreads)
						system_order = size(system(kk).A, 1);
						ev_left = eigenvector_left(:, :, kk);
						ev_right = eigenvector_right(:, :, kk);
						W = ev_left(1:system_order, 1:system_order);
						V = ev_right(1:system_order, 1:system_order);
						J_objective_eigenvector_temp(kk, 1) = real(trace(V'*V)*trace(W'*W)/2);
						if needsgradient && eigenvectorderivativeavailable
							% another set of temporary variables for correct indexing/slicing in parfor
							J_gradient_eigenvector_temp_parfor = zeros(number_controls, number_measurements);
							% no conditional assignment to avoid "The temporary variable J_gradient_eigenvector_xdot_temp_parfor will be cleared at the beginning of each iteration of the parfor loop."
							J_gradient_eigenvector_xdot_temp_parfor = zeros(number_controls, number_measurements_xdot);
							for ll = 1:number_controls
								for mm = 1:number_measurements
									ev_left_derivative = eigenvector_left_derivative(:, :, ll, mm, kk);
									dW = ev_left_derivative(1:system_order, 1:system_order);
									ev_right_derivative = eigenvector_right_derivative(:, :, ll, mm, kk);
									dV = ev_right_derivative(1:system_order, 1:system_order);
									J_gradient_eigenvector_temp_parfor(ll, mm) = real(trace(V'*dV)*trace(W'*W) + trace(V'*V)*trace(W'*dW));
								end
								if derivative_feedback && eigenvectorderivativeavailable_xdot
									for mm = 1:number_measurements_xdot
										ev_left_derivative_xdot = eigenvector_left_derivative_xdot(:, :, ll, mm, kk);
										dW_xdot = ev_left_derivative_xdot(1:system_order, 1:system_order);
										ev_right_derivative_xdot = eigenvector_right_derivative_xdot(:, :, ll, mm, kk);
										dV_xdot = ev_right_derivative_xdot(1:system_order, 1:system_order);
										J_gradient_eigenvector_xdot_temp_parfor(ll, mm) = real(trace(V'*dV_xdot)*trace(W'*W) + trace(V'*V)*trace(W'*dW_xdot));
									end
								end
							end
							J_gradient_eigenvector_temp(:, :, kk) = J_gradient_eigenvector_temp_parfor;
							if derivative_feedback && eigenvectorderivativeavailable_xdot
								J_gradient_eigenvector_xdot_temp(:, :, kk) = J_gradient_eigenvector_xdot_temp_parfor;
							end
						end
					end
					J_objective_eigenvector(ii, 1) = objective_weight(ii, 1)*sum(J_objective_eigenvector_temp, 1);
					if needsgradient
						J_gradient_eigenvector(:, :, ii) = objective_weight(ii, 1)*sum(J_gradient_eigenvector_temp, 3);
						if derivative_feedback
							J_gradient_eigenvector_xdot(:, :, ii) = objective_weight(ii, 1)*sum(J_gradient_eigenvector_xdot_temp, 3);
						end
						J_gradient_eigenvector_prefilter(:, :, ii) = objective_weight(ii, 1)*sum(J_gradient_eigenvector_prefilter_temp, 3);
					end
				otherwise
					continue;
			end
		end
	end
	J = sum(J_objective_eigenvector);
	if needsgradient
		% codegen crashes when concatenating empty variable size matrices
		if derivative_feedback
			if isempty(J_gradient_eigenvector_prefilter)
				if isempty(J_gradient_eigenvector)
					gradJ = sum(J_gradient_eigenvector_xdot, 3);
				else
					gradJ = cat(2, sum(J_gradient_eigenvector, 3), sum(J_gradient_eigenvector_xdot, 3));
				end
			else
				if isempty(J_gradient_eigenvector)
					gradJ = cat(2, sum(J_gradient_eigenvector_xdot, 3), sum(J_gradient_eigenvector_prefilter, 3));
				else
					gradJ = cat(2, sum(J_gradient_eigenvector, 3), sum(J_gradient_eigenvector_xdot, 3), sum(J_gradient_eigenvector_prefilter, 3));
				end
			end
		else
			if isempty(J_gradient_eigenvector_prefilter)
				gradJ = sum(J_gradient_eigenvector, 3);
			else
				if isempty(J_gradient_eigenvector)
					gradJ = sum(J_gradient_eigenvector_prefilter, 3);
				else
					gradJ = cat(2, sum(J_gradient_eigenvector, 3), sum(J_gradient_eigenvector_prefilter, 3));
				end
			end
		end
	end
end
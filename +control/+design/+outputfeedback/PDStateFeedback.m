classdef PDStateFeedback < control.design.outputfeedback.OutputFeedback
	%PDSTATEFEEDBACK class for casting a control system in PD state feedback form and specify the needed constraints on the resulting gain matrix
	%	For the continuous control system
	%		Ex' = Ax + Bu
	%		y = Cx + Du
	%		y' = C_dot x
	%	the control law u = -Rx + K C_dot x' + Fw + F_D w' results in the output feedback form
	%		(E - B K C_dot)x' = Ax - B R Ix + B Fw + B F_D w'
	%	For the discrete control system
	%		Ex_{k+1} = Ax_k + Bu_k
	%		y_k = Cx_k + Du_k
	%		y_{k+1} = C_dot x_{k+1}
	%	the control law u = -Rx_k + Kx_k - K x_D_k + Fw_k + F_D w_{k-1} with the additional dynamic x_D_{k+1} = x_k results in the output feedback form
	%		[
	%			E,	0;
	%			0,	I
	%		]x_{k+1} = [
	%			A,	0;
	%			I,	0
	%		]x_k - [
	%			B,	B;
	%			0,	0
	%		][
	%			R,	K;
	%			-K,	0
	%		][
	%			I,	0;
	%			0,	I
	%		]x_k + [
	%			B,	B;
	%			0,	0
	%		][
	%			F,	F_D;
	%			0,	0
	%		][
	%			w_k;
	%			w_{k-1}
	%		]

	methods(Static=true)
		function [name] = SimulinkVariant()
			%SIMULINKVARIANT return name of corresponding simulink variant for controller block in control_outputfeedback_lib
			%Output:
			%		name:	name of the corresponding simulink variant
			name = 'PDStateFeedback';
		end
	end

	methods
		function [this] = PDStateFeedback(varargin) %#ok<VANUS> varargin is not used but allowes to call the constructor with arguments
			%PDSTATEFEEDBACK create new PD state feedback class
			%	Input:
			%		varargin:	unused input arguments
			%	Output:
			%		this:		instance
			this@control.design.outputfeedback.OutputFeedback();
		end
	end
	methods(Access=protected)
		function [E, A, B, C, C_dot, D, C_ref, D_ref] = amend_system(~, E, A, B, C, C_dot, ~, C_ref, ~, T)
			%AMEND_SYSTEM add additional dynamics and partition matrices according to a PD state feedback
			%	Input:
			%		this:		instance
			%		E:			descriptor matrix
			%		A:			system matrix
			%		B:			control matrix
			%		C:			output matrix
			%		C_dot:		derivative output matrix
			%		D:			throughput matrix
			%		C_ref:		reference output matrix
			%		D_ref:		reference throughput matrix
			%		T:			sampling time
			%	Output:
			%		E:			descriptor matrix of extended system
			%		A:			system matrix of extended system
			%		B:			control matrix of extended system
			%		C:			output matrix of extended system
			%		C_dot:		derivative output matrix of extended system
			%		D:			throughput matrix of extended system
			%		C_ref:		reference output matrix
			%		D_ref:		reference throughput matrix
			n = size(A, 1);
			p = size(B, 2);
			q = size(C, 1);
			if control.design.outputfeedback.OutputFeedback.isranksupported(E) && rank(E) < size(A, 1)
				% TODO: allow real descriptor systems and check conditions for regular E - B K C_dot
				error('control:design:outputfeedback:input', 'Descriptor matrix must be regular for a PD state feedback controller.');
			end
			if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
				C_ref = [
					C_ref, zeros(size(C_ref, 1), n);
					C_ref, zeros(size(C_ref, 1), n)
				];
				D_ref = zeros(size(C_ref, 1), 2*q);
				E = [
					E,				zeros(n, n);
					zeros(n, n),	eye(n)
				];
				A = [
					A,		zeros(n, n);
					eye(n),	zeros(n, n)
				];
				B = [
					B,	B;
					zeros(n, 2*p)
				];
				C = eye(2*n);
				C_dot = zeros(0, 2*n);
				D = zeros(2*n, 2*p);
			else
				C = eye(size(A, 1));
				if isempty(C_dot)
					C_dot = C;
				end
				D = zeros(size(A, 1), size(B, 2));
				D_ref = zeros(size(C_ref, 1) + size(C_dot, 1), q + size(C_dot, 1));
				C_ref = [
					zeros(size(C_ref, 1), size(A, 1));
					zeros(size(C_dot, 1), size(A, 1))
				];
			end
		end

		function [R_fixed, K_fixed, F_fixed, RKF_fixed, R_bounds, K_bounds, F_bounds, RKF_bounds, R_nonlin] = gainpattern_system(~, ~, A, B, C, C_dot, ~, ~, ~, T)
			%GAINPATTERN_SYSTEM return gain pattern constraint system for a PD state feedback gain matrix
			%	Input:
			%		this:		instance
			%		E:			descriptor matrix
			%		A:			system matrix
			%		B:			control matrix
			%		C:			output matrix
			%		C_dot:		derivative output matrix
			%		D:			throughput matrix
			%		C_ref:		measurement matrix for reference outputs
			%		D_ref:		throughput matrix for reference outputs
			%		T:			sampling time
			%	Output:
			%		R_fixed:	cell array with constraint system for proportional gain matrix
			%		K_fixed:	cell array with constraint system for derivative gain matrix
			%		F_fixed:	cell array with constraint system for prefilter gain matrix
			%		RKF_fixed:	cell array with inequality constraint system for combined gain matrix
			%		R_bounds:	cell array with inequality constraint system for proportional gain matrix
			%		K_bounds:	cell array with inequality constraint system for derivative gain matrix
			%		F_bounds:	cell array with inequality constraint system for prefilter gain matrix
			%		RKF_bounds:	cell array with inequality constraint system for combined gain matrix
			%		R_nonlin:	function pointer to nonlinear constraints on proportional and derivative gain matrix
			n = size(A, 1);
			p = size(B, 2);
			q = size(C, 1);
			if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
				R = [
					ones(p, 2*n);
					ones(p, n),		zeros(p, n)
				];
				[row, col] = find(R == 0);
				number_zeros = size(row, 1);
				number_D = p*n;
				R_fixed_system = NaN([size(R), number_zeros + number_D]);
				R_fixed_border = zeros(number_zeros + number_D, 1);
				parfor ii = 1:number_zeros
					temp = zeros(size(R));
					temp(row(ii), col(ii)) = 1;
					R_fixed_system(:, :, ii) = temp;
				end
				R = [
					zeros(p, n),	ones(p, n);
					zeros(p, 2*n)
				];
				[row, col] = find(R == 1);
				parfor ii = 1:size(row, 1)
					temp = zeros(size(R));
					temp(row(ii), col(ii)) = 1;
					temp(row(ii) + p, col(ii) - n) = 1;
					R_fixed_system(:, :, ii + number_zeros) = temp;
				end
				R_fixed = {R_fixed_system, R_fixed_border};
				if nargout >= 2
					K_fixed = {true(2*p, 0), zeros(2*p, 0)};
					if nargout >= 3
						F_fixed = {[
							false(p, 2*q);
							true(p, 2*q);
						], [
							NaN(p, 2*q);
							zeros(p, 2*q)
						]};
					end
				end
			else
				if isempty(C_dot)
					C_dot = eye(n);
				end
				R_fixed = {false(p, n), NaN(p, n)};
				if nargout >= 2
					K_fixed = {false(p, size(C_dot, 1)), NaN(p, size(C_dot, 1))};
					if nargout >= 3
						F_fixed = {false(p, q + size(C_dot, 1)), NaN(p, q + size(C_dot, 1))};
					end
				end
			end
			if nargout >= 4
				RKF_fixed = [];
				if nargout >= 5
					R_bounds = [];
					if nargout >= 6
						K_bounds = [];
						if nargout >= 7
							F_bounds = [];
							if nargout >= 8
								RKF_bounds = [];
								if nargout >= 5
									R_nonlin = [];
								end
							end
						end
					end
				end
			end
		end

		function [R_gain, K_gain, F_prefilter] = gainpattern_parametric_system(~, ~, A, B, C, C_dot, ~, ~, ~, T)
			%GAINPATTERN_PARAMETRIC_SYSTEM return parametric gain matrix for a real PD state feedback gain matrix R = R, gain matrix K = K and prefilter matrix F = [
			%		F,	F_D
			%	] in continuous time and R = [
			%		R,	K;
			%		-K,	0
			%	] and F = [
			%		F,	F_D;
			%		0,	0
			%	] in discrete time
			%	Input:
			%		this:			instance
			%		system:			state space system or structure with system matrices or descriptor matrix
			%		A:				system matrix or sample time if system is given as first argument
			%		B:				control matrix
			%		C:				output matrix
			%		C_dot:			derivative output matrix
			%		D:				throughput matrix
			%		C_ref:			measurement matrix for reference outputs
			%		D_ref:			throughput matrix for reference outputs
			%		T:				sampling time
			%	Output:
			%		R_gain:			parametric proportional gain matrix
			%		K_gain:			parametric derivative gain matrix
			%		F_prefilter:	parametric prefilter matrix
			n = size(A, 1);
			p = size(B, 2);
			q = size(C, 1);
			%q_dot = size(C_dot, 1);
			if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
				R = realp('R', ones(p, n));
				K = realp('K', ones(p, n));
				R_gain = [
					R,	K;
					-K,	zeros(p, n)
				];
				if nargout >= 2
					K_gain = zeros(2*p, 0);
					if nargout >= 3
						F = realp('F', ones(p, q));
						F_D = realp('F_D', ones(p, q));
						F_prefilter = [
							F,		F_D;
							zeros(p, 2*q)
						];
					end
				end
			else
				if isempty(C_dot)
					C_dot = eye(n);
				end
				q_dot = size(C_dot, 1);
				F = realp('F', ones(p, q));
				R = realp('R', ones(p, n));
				K = realp('K', ones(p, q_dot));
				R_gain = R;
				if nargout >= 2
					K_gain = K;
					if nargout >= 3
						F_D = realp('F_D', ones(p, q_dot));
						F_prefilter = [
							F,	F_D
						];
					end
				end
			end
		end

		function [T_x, T_u, T_y, T_y_dot, T_w] = scalegain_system(~, T_x, T_u, T_y, T_y_dot, T_w, ~, A, ~, ~, C_dot, ~, ~, ~, T)
			%SCALEGAIN_SYSTEM return scaling matrices for given system
			%	Input:
			%		this:		instance
			%		T_x:		state transformation matrix for nominal system
			%		T_u:		control transformation matrix for nominal system
			%		T_Y:		measurement transformation matrix for nominal system
			%		T_y_dot:	derivative measurement transformation matrix for nominal system
			%		T_w:		reference value transformation matrix for nominal system
			%		E:			descriptor matrix
			%		A:			system matrix
			%		B:			control matrix
			%		C:			output matrix
			%		C_dot:		derivative output matrix
			%		D:			throughput matrix
			%		C_ref:		measurement matrix for reference outputs
			%		D_ref:		throughput matrix for reference outputs
			%		T:			sampling time
			%	Output:
			%		T_x:		state transformation matrix for augmented system
			%		T_u:		control transformation matrix for augmented system
			%		T_Y:		measurement transformation matrix for augmented system
			%		T_y_dot:	derivative measurement transformation matrix for augmented system
			%		T_w:		reference value transformation matrix for augmented system
			n = size(A, 1);
			%p = size(B, 2);
			%q = size(C, 1);
			%q_dot = size(C_dot, 1);
			if isempty(T_w)
				T_w = T_y;
			end
			if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
				if nargout >= 2
					T_u = blkdiag(T_u, T_u);
					if nargout >= 3
						T_y = blkdiag(T_x, eye(n));
						if nargout >= 5
							T_w = blkdiag(T_w, T_w);
						end
					end
				end
				T_x = blkdiag(T_x, eye(n));
			else
				if isempty(C_dot)
					T_y_dot = T_x;
				end
				T_y = T_x;
				if nargout >= 5
					T_w = blkdiag(T_w, T_y_dot);
				end
			end
		end

		function [F, F_fixed] = prefilterpattern_system(~, ~, ~, ~, A, B, C, C_dot, ~, ~, ~, T)
			%PREFILTERPATTERN_SYSTEM return prefilter and prefilter pattern constraint system for a PD state feedback with given gain matrices
			%	Input:
			%		this:		instance
			%		R:			gain matrix to close loop with
			%		system:		state space system or structure with system matrices or descriptor matrix
			%		A:			system matrix or sample time if system is given as first argument
			%		B:			control matrix
			%		C:			output matrix
			%		C_dot:		derivative output matrix
			%		D:			throughput matrix
			%		C_ref:		measurement matrix for reference outputs
			%		D_ref:		throughput matrix for reference outputs
			%		T:			sampling time
			%	Output:
			%		F:			prefilter matrix for extended system and supplied gain matrices
			%		F_fixed:	indicator matrix for fixed elements of prefilter matrix
			n = size(A, 1);
			p = size(B, 2);
			q = size(C, 1);
			if isempty(C_dot)
				C_dot = eye(n);
			end
			q_dot = size(C_dot, 1);
			if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
				F = [
					eye(p, q),		eye(p, q);
					zeros(p, 2*q)
				];
				if nargout >= 2
					F_fixed = [
						false(p, 2*q);
						true(p, 2*q);
					];
				end
			else
				F = [
					eye(p, q),		eye(p, q_dot)
				];
				if nargout >= 2
					F_fixed = false(p, q + q_dot);
				end
			end
		end

		function [partitionR, partitionF] = gainpartitioning_system(~, R, K, F, ~, A, B, C, C_dot, ~, ~, ~, T)
			%GAINPARTITIONING_SYSTEM return partitioning for gain matrix of extended system for a real PD state feedback with given gain matrix
			%	Input:
			%		this:		instance
			%		R:			proportional gain matrix to close loop with
			%		K:			derivative gain matrix to close loop with
			%		F:			prefilter matrix
			%		E:			descriptor matrix
			%		A:			system matrix
			%		B:			control matrix
			%		C:			output matrix
			%		C_dot:		derivative output matrix
			%		D:			throughput matrix
			%		C_ref:		measurement matrix for reference outputs
			%		D_ref:		throughput matrix for reference outputs
			%		T:			sampling time
			%	Output:
			%		partitionR:	named partition matrices of gain matrix in structure fields
			%		partitionF:	named partition matrices of prefilter matrix in structure fields
			n = size(A, 1);
			p = size(B, 2);
			q = size(C, 1);
			if isempty(C_dot)
				C_dot = eye(n);
			end
			q_dot = size(C_dot, 1);
			if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
				partitionR = struct(...
					'R',	R(1:p, 1:n),...
					'K_D',	R(1:p, n + 1:2*n)...
				);
				if nargout >= 2
					partitionF = struct(...
						'F',	F(1:p, 1:q),...
						'F_D',	F(1:p, q + 1:2*q)...
					);
				end
			else
				partitionR = struct(...
					'R',	R(1:p, 1:n),...
					'K_D',	K(1:p, 1:q_dot)...
				);
				if nargout >= 2
					partitionF = struct(...
						'F',	F(1:p, 1:q),...
						'F_D',	F(1:p, (q + 1):(q + q_dot))...
					);
				end
			end
		end

		function [E, A, B, C, C_dot, D, C_ref, D_ref, needsstate, useCasCdot] = realization_system(this, R, K, F, E, A, B, C, C_dot, D, C_ref, D_ref, T)
			%REALIZATION return controller without system for output feedback with given gain matrix
			%	Input:
			%		this:			instance
			%		R:				proportional gain matrix to close loop with
			%		K:				derivative gain matrix to close loop with
			%		F:				prefilter matrix
			%		system:			state space system or structure with system matrices or descriptor matrix
			%		A:				system matrix or sample time if system is given as first argument
			%		B:				control matrix
			%		C:				output matrix
			%		C_dot:			derivative output matrix
			%		D:				throughput matrix
			%		C_ref:			measurement matrix for reference outputs
			%		D_ref:			throughput matrix for reference outputs
			%		T:				sampling time
			%	Output:
			%		E:				descriptor matrix of controller
			%		A:				system matrix of controller
			%		B:				control matrix of controller
			%		C:				output matrix of controller
			%		C_dot:			derivative output matrix of controller
			%		D:				throughput matrix of controller
			%		C_ref:			measurement matrix for reference outputs of controller
			%		D_ref:			throughput matrix for reference outputs of controller
			%		needsstate:		indicator if state is needed as controller input instead of output
			%		usesCasCdot:	indicator if C is used as Cdot in case of derivative feedback
			n = size(A, 1);
			p = size(B, 2);
			q = size(C, 1);
			needsstate = true;
			if control.design.outputfeedback.OutputFeedback.isranksupported(D) && rank(D) > 0
				error('control:design:outputfeedback:input', 'System model must not have a throughput matrix.');
			end
			if control.design.outputfeedback.OutputFeedback.isranksupported(E) && rank(E) < size(A, 1)
				% TODO: allow real descriptor systems and check conditions for regular E - B K C_dot
				error('control:design:outputfeedback:input', 'Descriptor matrix must be regular for a PID state feedback controller.');
			end
			[partitionR, partitionF] = this.gainpartitioning_system(R, K, F, E, A, B, C, C_dot, D, C_ref, D_ref, T);
			if isempty(C_dot)
				C_dot = eye(n);
				useCasCdot = true;
			else
				useCasCdot = false;
			end
			q_dot = size(C_dot, 1);
			if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
				if control.design.outputfeedback.OutputFeedback.isranksupported(D) && rank(D) > 0
					error('control:design:outputfeedback:input', 'System must not have a throughput matrix.');
				end
				E = eye(n);
				A = zeros(n, n);
				B = [
					eye(n),		zeros(n, 2*q)
				];
				C = -partitionR.K_D;
				C_dot = zeros(0, n);
				D = [
					-partitionR.R + partitionR.K_D,	partitionF.F,	partitionF.F_D
				];
				C_ref = zeros(0, size(A, 1));
				D_ref = zeros(0, size(B, 2));
			else
				E = zeros(0, 0);
				A = zeros(0, 0);
				B = zeros(0, n + q + 2*q_dot);
				C = zeros(p, 0);
				C_dot = zeros(0, 0);
				D = [
					-partitionR.R,	partitionR.K_D,	partitionF.F,	partitionF.F_D
				];
				C_ref = zeros(0, size(A, 1));
				D_ref = zeros(0, size(B, 2));
			end
		end
	end

end
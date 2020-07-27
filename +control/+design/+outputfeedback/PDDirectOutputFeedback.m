classdef PDDirectOutputFeedback < control.design.outputfeedback.OutputFeedback
	%PDDIRECTOUTPUTFEEDBACK class for casting a control system in PD output feedback form and specify the needed constraints on the resulting gain matrix
	%	For the continuous control system
	%		Ex' = Ax + Bu
	%		y = Cx + Du
	%		y' = C_dot x
	%	the control law u = -RCx - FCx - F0u - Kw' + K C_dot x' + K0u' + Fw results in the output feedback form
	%		(E - B K C_dot)x' = Ax - B [
	%			R,	F
	%		][
	%			C;
	%			C
	%		]x + BFw - BKw'
	%	For the discrete control system
	%		Ex_{k+1} = Ax_k + Bu_k
	%		y_k = Cx_k + Du_k
	%		y_{k+1} = C_dot x_{k+1}
	%	the control law u = -RCx_k - FCx_k - KCx_k + K x_D_k + F w_k + K w_k - K w_{k-1} with the additional dynamic x_D_{k+1} = C x_k + D u_k results in the output feedback form
	%		[
	%			E,	0;
	%			0,	I
	%		]x_{k+1} = [
	%			A,	0;
	%			C,	0
	%		]x_k - [
	%			B,	B;
	%			D,	D
	%		][
	%			R,	K;
	%			F,	0
	%		][
	%			C,	0;
	%			-C,	I
	%		]x_k + [
	%			B,	B;
	%			D,	D
	%		][
	%			F,	0;
	%			K,	-K
	%		] [
	%			w_k;
	%			w_{k-1}
	%		]
	%	In contrast to the PDOutputFeedback F (w - y) is fed back directly.

	methods(Static=true)
		function [name] = SimulinkVariant()
			%SIMULINKVARIANT return name of corresponding simulink variant for controller block in control_outputfeedback_lib
			%Output:
			%		name:	name of the corresponding simulink variant
			name = 'PDDirectOutputFeedback';
		end
	end

	methods
		function [this] = PDDirectOutputFeedback(varargin) %#ok<VANUS> varargin is not used but allowes to call the constructor with arguments
			%PDDIRECTOUTPUTFEEDBACK create new PD output feedback class
			%	Input:
			%		varargin:	unused input arguments
			%	Output:
			%		this:		instance
			this@control.design.outputfeedback.OutputFeedback();
		end
	end
	methods(Access=protected)
		function [E, A, B, C, C_dot, D, C_ref, D_ref] = amend_system(~, E, A, B, C, C_dot, D, C_ref, D_ref, T)
			%AMEND_SYSTEM add additional dynamics and partition matrices according to a PD output feedback
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
			%p = size(B, 2);
			q = size(C, 1);
			if control.design.outputfeedback.OutputFeedback.isranksupported(D) && rank(D) > 0
				error('control:design:outputfeedback:input', 'System model must not have a throughput matrix.');
			end
			if control.design.outputfeedback.OutputFeedback.isranksupported(E) && rank(E) < size(A, 1)
				% TODO: allow real descriptor systems and check conditions for regular E - B K C_dot
				error('control:design:outputfeedback:input', 'Descriptor matrix must be regular for a PD state feedback controller.');
			end
			if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
				C_ref = [
					C_ref, zeros(size(C_ref, 1), q);
					C_ref, zeros(size(C_ref, 1), q)
				];
				D_ref = zeros(size(C_ref, 1), 2*q);
				E = [
					E,				zeros(n, q);
					zeros(q, n),	eye(q)
				];
				A = [
					A,		zeros(n, q);
					C,		zeros(q, q)
				];
				B = [
					B,	B;
					D,	D
				];
				C = [
					C,	zeros(q, q);
					-C,	eye(q)
				];
				C_dot = zeros(0, n + q);
				D = zeros(size(C, 1), size(B, 2));
			else
				if isempty(C_dot)
					C_dot = C;
				end
				C_ref = [
					C_ref;
					zeros(size(C_dot, 1), size(C_ref, 2))
				];
				D_ref = [
					D_ref, zeros(size(D_ref, 1), size(C_dot, 1))
					zeros(size(C_dot, 1), size(D_ref, 2) + size(C_dot, 1))
				];
				C = [
					C;
					C
				];
				D = zeros(size(C, 1), size(B, 2));
			end
		end

		function [R_fixed, K_fixed, F_fixed, RKF_fixed, R_bounds, K_bounds, F_bounds, RKF_bounds, R_nonlin] = gainpattern_system(~, ~, ~, B, C, C_dot, ~, ~, ~, T)
			%GAINPATTERN_SYSTEM return gain pattern constraint system for a PD output feedback gain matrix R = [
			%		R,	F
			%	] and gain matrix K = [
			%		K
			%	] in continuous time and R = [
			%		R,	K;
			%		F,	0
			%	] in discrete time
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
			%		R_nonlin:	function pointer to nonlinear constraints on proportional, derivative and prefilter gain matrix
			%n = size(A, 1);
			p = size(B, 2);
			q = size(C, 1);
			if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
				R = [
					NaN(p, 2*q);
					NaN(p, q),	zeros(p, q)
				];
				R_fixed = {~isnan(R), R};
				if nargout >= 2
					K_fixed = {true(2*p, 0), zeros(2*p, 0)};
					if nargout >= 3
						F_fixed = {[
							false(p, q),	true(p, q);
							false(p, 2*q)
						], [
							NaN(p, q),	zeros(p, q);
							NaN(p, 2*q)
						]};
						if nargout >= 4
							R = cat(1,...
								zeros(p, 2*q, p*q),...
								cat(2, reshape(eye(p*q), p, q, p*q), zeros(p, q, p*q))...
							);
							K = cat(1,...
								cat(2, zeros(p, q, p*q), reshape(eye(p*q), p, q, p*q)),...
								zeros(p, 2*q, p*q)...
							);
							F_1 = cat(1,...
								cat(2, reshape(-eye(p*q), p, q, p*q), zeros(p, q, p*q)),...
								zeros(p, 2*q, p*q)...
							);
							F_2 = cat(1,...
								zeros(p, 2*q, p*q),...
								cat(2, reshape(-eye(p*q), p, q, p*q), zeros(p, q, p*q))...
							);
							F_3 = cat(1,...
								zeros(p, 2*q, p*q),...
								cat(2, zeros(p, q, p*q), reshape(eye(p*q), p, q, p*q))...
							);
							RKF_fixed = {
								cat(3,...
									cat(2, R, zeros(2*p, 0, p*q), F_1),...
									cat(2, K, zeros(2*p, 0, p*q), F_2),...
									cat(2, K, zeros(2*p, 0, p*q), F_3)...
								), zeros(3*p*q, 1)
							};
						end
					end
				end
			else
				if isempty(C_dot)
					C_dot = C;
				end
				q_dot = size(C_dot, 1);
				R_fixed = {false(p, 2*q), NaN(p, 2*q)};
				if nargout >= 2
					K_fixed = {false(p, q_dot), NaN(p, q_dot)};
					if nargout >= 3
						F_fixed = {false(p, q + q_dot), NaN(p, q + q_dot)};
						if nargout >= 4
							R = cat(2, zeros(p, q, p*q), reshape(eye(p*q), p, q, p*q));
							K = reshape(eye(p*q_dot), p, q_dot, p*q_dot);
							F_1 = cat(2, reshape(-eye(p*q), p, q, p*q), zeros(p, q_dot, p*q));
							F_2 = cat(2, zeros(p, q, p*q_dot), reshape(eye(p*q_dot), p, q_dot, p*q_dot));
							RKF_fixed = {
								cat(3,...
									cat(2, R, zeros(p, q_dot, p*q), F_1),...
									cat(2, zeros(p, 2*q, p*q_dot), K, F_2)...
								), zeros(p*(q + q_dot), 1)
							};
						end
					end
				end
			end
			if nargout >= 5
				R_bounds = [];
				if nargout >= 6
					K_bounds = [];
					if nargout >= 7
						F_bounds = [];
						if nargout >= 8
							RKF_bounds = [];
							if nargout >= 9
								R_nonlin = [];
							end
						end
					end
				end
			end
		end

		function [R_gain, K_gain, F_prefilter] = gainpattern_parametric_system(~, ~, ~, B, C, C_dot, ~, ~, ~, T)
			%GAINPATTERN_PARAMETRIC_SYSTEM return parametric gain matrix for a PD output feedback gain matrix R = [
			%		R,	F
			%	], gain matrix K = K and prefilter matrix F = [
			%		F,	-K
			%	] in continuous time and R = [
			%		R,	K;
			%		F,	0
			%	] and F = [
			%		F,	0;
			%		K,	-K
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
			%n = size(A, 1);
			p = size(B, 2);
			q = size(C, 1);
			%q_dot = size(C_dot, 1);
			if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
				F = realp('F', ones(p, q));
				R = realp('R', ones(p, q));
				K = realp('K', ones(p, q));
				R_gain = [
					R,	K;
					F,	zeros(p, q)
				];
				if nargout >= 2
					K_gain = zeros(2*p, 0);
					if nargout >= 3
						F_prefilter = [
							F,		zeros(p, q);
							K,		-K
						];
					end
				end
			else
				F = realp('F', ones(p, q));
				R = realp('R', ones(p, q));
				R_gain = [
					F,	R
				];
				if nargout >= 2
					if isempty(C_dot)
						C_dot = C;
					end
					K = realp('K', ones(p, size(C_dot, 1)));
					K_gain = K;
					if nargout >= 3
						F_prefilter = [
							F,	-K
						];
					end
				end
			end
		end

		function [T_x, T_u, T_y, T_y_dot, T_w] = scalegain_system(~, T_x, T_u, T_y, T_y_dot, T_w, ~, ~, ~, C, C_dot, ~, ~, ~, T)
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
			%n = size(A, 1);
			%p = size(B, 2);
			q = size(C, 1);
			%q_dot = size(C_dot, 1);
			if isempty(T_w)
				T_w = T_y;
			end
			if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
				T_x = blkdiag(T_x, eye(q));
				if nargout >= 2
					T_u = blkdiag(T_u, T_u);
					if nargout >= 3
						T_y = blkdiag(T_y, T_y);
						if nargout >= 5
							T_w = blkdiag(T_w, T_w);
						end
					end
				end
			else
				if nargout >= 3
					if nargout >= 4
						if isempty(C_dot)
							T_y_dot = T_y;
						end
						if nargout >= 5
							T_w = blkdiag(T_w, T_y_dot);
						end
					end
					T_y = blkdiag(T_y, T_y);
				end
			end
		end

		function [F, F_fixed] = prefilterpattern_system(~, R, K, ~, ~, B, C, C_dot, ~, ~, ~, T)
			%PREFILTERPATTERN_SYSTEM return prefilter and prefilter pattern constraint system for a PD output feedback with given gain matrices
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
			%n = size(A, 1);
			p = size(B, 2);
			q = size(C, 1);
			if isempty(C_dot)
				C_dot = C;
			end
			q_dot = size(C_dot, 1);
			if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
				F = [
					R(p + 1:2*p, 1:q),	zeros(p, q);
					R(1:p, q + 1:2*q),	-R(1:p, q + 1:2*q)
				];
				if nargout >= 2
					F_fixed = true(2*p, 2*q);
				end
			else
				F = [
					R(:, q + 1:2*q),	-K
				];
				if nargout >= 2
					F_fixed = true(p, q + q_dot);
				end
			end
		end

		function [partitionR, partitionF] = gainpartitioning_system(~, R, K, F, ~, ~, B, C, C_dot, ~, ~, ~, T)
			%GAINPARTITIONING_SYSTEM return partitioning for gain matrix of extended system for a PD output feedback with given gain matrix
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
			%n = size(A, 1);
			p = size(B, 2);
			q = size(C, 1);
			if isempty(C_dot)
				C_dot = C;
			end
			q_dot = size(C_dot, 1);
			if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
				partitionR = struct(...
					'R',	R(1:p, 1:q),...
					'F',	R( p + 1:2*p, 1:q),...
					'K_D',	R(1:p, q + 1:2*q)...
				);
				if nargout >= 2
					partitionF = struct(...
						'F',	F(1:p, 1:q),...
						'K_D',	F(p + 1:2*p, 1:q)...
					);
				end
			else
				partitionR = struct(...
					'R',	R(1:p, 1:q),...
					'F',	R(1:p, q + 1:2*q),...
					'K_D',	K...
				);
				if nargout >= 2
					partitionF = struct(...
						'F',	F(1:p, 1:q),...
						'K_D',	-F(1:p, q + (1:q_dot))...
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
			needsstate = false;
			if isempty(C_dot)
				C_dot = C;
				useCasCdot = true;
			else
				useCasCdot = false;
			end
			%n = size(A, 1);
			p = size(B, 2);
			q = size(C, 1);
			q_dot = size(C_dot, 1);
			if control.design.outputfeedback.OutputFeedback.isranksupported(D) && rank(D) > 0
				error('control:design:outputfeedback:input', 'System model must not have a throughput matrix.');
			end
			if control.design.outputfeedback.OutputFeedback.isranksupported(E) && rank(E) < size(A, 1)
				% TODO: allow real descriptor systems and check conditions for regular E - B K C_dot
				error('control:design:outputfeedback:input', 'Descriptor matrix must be regular for a PID state feedback controller.');
			end
			[partitionR, partitionF] = this.gainpartitioning_system(R, K, F, E, A, B, C, C_dot, D, C_ref, D_ref, T);
			if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
				if control.design.outputfeedback.OutputFeedback.isranksupported(D) && rank(D) > 0
					error('control:design:outputfeedback:input', 'System must not have a throughput matrix.');
				end
				E = eye(q);
				A = zeros(q, q);
				B = [
					eye(q),		zeros(q, 2*q)
				];
				C = partitionR.K_D;
				C_dot = zeros(0, q);
				D = [
					-(partitionR.F + partitionR.R + partitionR.K_D),	partitionF.F + partitionF.K_D,	-partitionF.K_D
				];
				C_ref = zeros(0, size(A, 1));
				D_ref = zeros(0, size(B, 2));
			else
				E = zeros(0, 0);
				A = zeros(0, 0);
				B = zeros(0, 2*q + 2*q_dot);
				C = zeros(p, 0);
				C_dot = zeros(0, 0);
				D = [
					-partitionR.F  - partitionR.R,	partitionR.K_D,	partitionF.F,	-partitionF.K_D
				];
				C_ref = zeros(0, size(A, 1));
				D_ref = zeros(0, size(B, 2));
			end
		end
	end

end
classdef PIDRealDirectOutputFeedback < control.design.outputfeedback.AbstractDRealFeedback
	%PIDREALDIRECTOUTPUTFEEDBACK class for casting a control system in real PID output feedback form and specify the needed constraints on the resulting gain matrix
	%	For the continuous control system
	%		Ex' = Ax + Bu
	%		y = Cx + Du
	%		y' = C_dot x
	%	the control law u = -RCx + K_I x_I + K w' - K x_D' + Fw - FCx - FDu - K0u' with the additional dynamic x_I' = 0x_I + w - Cx - Du and x_D' = -T x_D + C_dot x results in the output feedback form
	%		([
	%			E,	0,	0;
	%			0,	I,	0;
	%			0,	0,	I
	%		] - [
	%			B,	0,	0;
	%			-D,	I,	0;
	%			0,	0,	I
	%		][
	%			-K;
	%			0;
	%			0
	%		][
	%			0,	0,	I
	%		])x' = ([
	%			A,		0,	0;
	%			-C,		0,	0;
	%			C_dot,	0,	0
	%		] - [
	%			B,	0,	0;
	%			-D,	I,	0;
	%			0,	0,	I
	%		][
	%			F,	R, -K_I,	0;
	%			0,	0,	0,		0;
	%			0,	0,	0,		T
	%		][
	%			C,	0,	0;
	%			C,	0,	0;
	%			0,	I,	0;
	%			0,	0,	I
	%		]x + [
	%			B,	0,	0;
	%			-D,	I,	0;
	%			0,	0,	I
	%		][
	%			F,	K;
	%			I,	0;
	%			0,	0
	%		][
	%			w;
	%			w'
	%		]
	%	For the discrete control system
	%		Ex_{k+1} = Ax_k + Bu_k
	%		y_k = Cx_k + Du_k
	%		y_{k+1} = C_dot x_{k+1}
	%	the control law u = -RCx_k + K_I x_I_k + F w_k - F C x_k - F 0 u_k + K w_k - K x_T_k + K x_D_k - K w_{k-1} with the additional dynamic x_I_{k+1} = x_I_k + w_k - C x_k - D x_k, x_T_{k+1} = -T x_T_k + Cx_k + D u_k and x_D_{k+1} = C x_k + D u_k results in the output feedback form
	%		[
	%			E,	0,	0,	0;
	%			0,	I,	0,	0;
	%			0,	0,	I,	0;
	%			0,	0,	0,	I
	%		]x_{k+1} = [
	%			A,	0,	0,	0;
	%			-C,	I,	0,	0;
	%			C,	0,	0,	0;
	%			0,	0,	I,	0
	%		]x_k - [
	%			B,	0,	B,	0;
	%			-D,	I,	-D,	0;
	%			D,	0,	D,	I;
	%			0,	0,	0,	0
	%		][
	%			F,	R,	-K_I,	K,	0;
	%			0,	0,	0,		0,	0;
	%			0,	0,	0,		0,	0;
	%			0,	0,	0,		0,	T
	%		][
	%			C,	0,	0,	0;
	%			C,	0,	0,	0;
	%			0,	I,	0,	0;
	%			C,	0,	I,	-I;
	%			0,	0,	I,	0
	%		]x_k + [
	%			B,	0,	B,	0;
	%			-D,	I,	-D,	0;
	%			D,	0,	D,	I;
	%			0,	0,	0,	0
	%		][
	%			F,	0;
	%			I,	0;
	%			K,	-K;
	%			0,	0
	%		] [
	%			w_k;
	%			w_{k-1}
	%		]
	%	In contrast to the PIDRealOutputFeedback F (w - y) is fed back directly.
	
	methods(Static=true)
		function [name] = SimulinkVariant()
			%SIMULINKVARIANT return name of corresponding simulink variant for controller block in control_outputfeedback_lib
			%Output:
			%		name:	name of the corresponding simulink variant
			name = 'PIDRealDirectOutputFeedback';
		end
	end
	
	methods
		function [this] = PIDRealDirectOutputFeedback(varargin)
			%PIDREALDIRECTOUTPUTFEEDBACK create new real PID output feedback class
			%	Input:
			%		varargin:	unused input arguments
			%	Output:
			%		this:		instance
			this@control.design.outputfeedback.AbstractDRealFeedback(varargin{:});
		end
	end
	methods(Access=protected)
		function [E, A, B, C, C_dot, D, C_ref, D_ref] = amend_system(~, E, A, B, C, C_dot, D, C_ref, ~, T)
			%AMEND_SYSTEM add additional dynamics and partition matrices according to a real PID output feedback
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
			q_dot = size(C_dot, 1);
			if control.design.outputfeedback.OutputFeedback.isranksupported(D) && rank(D) > 0
				error('control:design:outputfeedback:input', 'System model must not have a throughput matrix.');
			end
			if control.design.outputfeedback.OutputFeedback.isranksupported(E) && rank(E) < size(A, 1)
				% TODO: allow real descriptor systems and check conditions for regular E - B K C_dot
				error('control:design:outputfeedback:input', 'Descriptor matrix must be regular for a real PID state feedback controller.');
			end
			if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
				if control.design.outputfeedback.OutputFeedback.isranksupported(D) && rank(D) > 0
					error('control:design:outputfeedback:input', 'System must not have a throughput matrix.');
				end
				C_ref = [
					C_ref, zeros(size(C_ref, 1), 3*q);
					C_ref, zeros(size(C_ref, 1), 3*q)
				];
				D_ref = zeros(size(C_ref, 1), 2*q);
				E = [
					E,				zeros(n, 3*q);
					zeros(3*q, n),	eye(3*q)
				];
				A = [
					A,					zeros(n, 3*q);
					-C,					eye(q),			zeros(q, 2*q);
					C,					zeros(q, 3*q);
					zeros(q, n + q),	eye(q),			zeros(q, q)
				];
				B = [
					B,	zeros(n, q),	B,	zeros(n, q);
					-D,	eye(q),			-D,	zeros(q, q);
					D,	zeros(q, q),	D,	eye(q, q);
					zeros(q, 2*p + 2*q)
				];
				C = [
					C,					zeros(q, 3*q);
					C,					zeros(q, 3*q);
					zeros(q, n),		eye(q),	zeros(q, 2*q);
					zeros(q, n + q),	eye(q),	-eye(q);
					zeros(q, n + q),	eye(q),	zeros(q, q)
				];
				C_dot = zeros(0, n + 3*q);
				D = zeros(size(C, 1), size(B, 2));
			else
				C_ref = [
					C_ref, zeros(size(C_ref, 1), q + q_dot);
					zeros(q_dot, size(C_ref, 2) + q + q_dot)
				];
				D_ref = zeros(size(C_ref, 1), q + q_dot);
				E = [
					E,						zeros(n, q + q_dot);
					zeros(q + q_dot, n),	eye(q + q_dot)
				];
				A = [
					A,		zeros(n, q + q_dot);
					-C,		zeros(q, q + q_dot);
					C_dot,	zeros(q_dot, q + q_dot)
				];
				B = [
					B,						zeros(n, q + q_dot);
					-D,						eye(q),	zeros(q, q_dot);
					zeros(q_dot, p + q),	eye(q_dot)
				];
				C = [
					C,						zeros(q, q + q_dot);
					C,						zeros(q, q + q_dot);
					zeros(q + q_dot, n),	eye(q + q_dot)
				];
				C_dot = [
					zeros(q_dot, n + q), eye(q_dot)
				];
				D = zeros(size(C, 1), size(B, 2));
			end
		end
		
		function [R_fixed, K_fixed, F_fixed, RKF_fixed, R_bounds, K_bounds, F_bounds, RKF_bounds, R_nonlin] = gainpattern_system(this, ~, ~, B, C, C_dot, ~, ~, ~, T)
			%GAINPATTERN_SYSTEM return gain pattern constraint system for a real PID output feedback gain matrix R = [
			%		F,	R,	-K_I,	0;
			%		0,	0,	0,		0;
			%		0,	0,	0,		T
			%	] and gain matrix K = [
			%		-K;
			%		0;
			%		0
			%	] in continuous time and R = [
			%		F,	R,	-K_I,	K,	0;
			%		0,	0,	0,		0,	0;
			%		0,	0,	0,		0,	0;
			%		0,	0,	0,		0,	T
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
			%		R_fixed:	proportional gain constraint system as a cell array of a 3D constraint matrix and a constraint border vector
			%		K_fixed:	derivative gain constraint system as a cell array of a 3D constraint matrix and a constraint border vector
			%		R_bounds:	cell array with inequality constraint system for proportional gain matrix
			%		K_bounds:	cell array with inequality constraint system for derivative gain matrix
			%		R_nonlin:	function pointer to nonlinear constraints on proportional and derivative gain matrix
			%n = size(A, 1);
			p = size(B, 2);
			q = size(C, 1);
			q_dot = size(C_dot, 1);
			realK = this.realD;
			if isempty(realK)
				realK = NaN(n, n);
			end
			if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
				if size(realK, 1) < q
					error('control:design:outputfeedback:input', 'Not enough realization poles (%d) supplied for system of size %d.', size(realK, 1), q);
				end
				R = [
					NaN(p, 4*q),	zeros(p, q);
					zeros(p + q, 5*q);
					zeros(q, 4*q),	realK(1:q, 1:q)
				];
				R_fixed = {~isnan(R), R};
				if nargout >= 2
					K_fixed = {true(2*p + 2*q, 0), zeros(2*p + 2*q, 0)};
					if nargout >= 3
						F_fixed = {[
							false(p, q),	true(p, q);
							true(q, 2*q);
							false(p, 2*q);
							true(q, 2*q)
						], [
							NaN(p, q),		zeros(p, q);
							eye(q),			zeros(q, q);
							NaN(p, q),		NaN(p, q);
							zeros(q, 2*q)
						]};
						if nargout >= 4
							R_1 = cat(1,...
								cat(2, reshape(eye(p*q), p, q, p*q), zeros(p, 4*q, p*q)),...
								zeros(p + 2*q, 5*q, p*q)...
							);
							R_2 = cat(1,...
								cat(2, zeros(p, 3*q, p*q), reshape(eye(p*q), p, q, p*q), zeros(p, q, p*q)),...
								zeros(p + 2*q, 5*q, p*q)...
							);
							F_1 = cat(1,...
								cat(2, reshape(-eye(p*q), p, q, p*q), zeros(p, q, p*q)),...
								zeros(q + p + q, 2*q, p*q)...
							);
							F_2 = cat(1,...
								zeros(p + q, 2*q, p*q),...
								cat(2, zeros(p, q, p*q), reshape(-eye(p*q), p, q, p*q)),...
								zeros(q, 2*q, p*q)...
							);
							F_3 = cat(1,...
								zeros(p + q, 2*q, p*q),...
								cat(2, zeros(p, q, p*q), reshape(eye(p*q), p, q, p*q)),...
								zeros(q, 2*q, p*q)...
							);
							RKF_fixed = {
								cat(3,...
									cat(2, R_1, zeros(2*p + 2*q, 0, p*q), F_1),...
									cat(2, R_2, zeros(2*p + 2*q, 0, p*q), F_2),...
									cat(2, R_2, zeros(2*p + 2*q, 0, p*q), F_3)...
								), zeros(3*p*q, 1)
							};
						end
					end
				end
			else
				if size(realK, 1) < q_dot
					error('control:design:outputfeedback:input', 'Not enough realization poles (%d) supplied for system of size %d.', size(realK, 1), q_dot);
				end
				R = [
					NaN(p, 3*q),	zeros(p, q_dot);
					zeros(q, 3*q + q_dot);
					zeros(q_dot, 3*q),	realK(1:q_dot, 1:q_dot)
				];
				R_fixed = {~isnan(R), R};
				if nargout >= 2
					K = [
						NaN(p, q_dot);
						zeros(q + q_dot, q_dot)
					];
					K_fixed = {~isnan(K), K};
					if nargout >= 3
						F_fixed = {[
							false(p, q + q_dot);
							true(q + q_dot, q + q_dot)
						], [
							NaN(p, q + q_dot);
							eye(q),		zeros(q, q_dot);
							zeros(q_dot, q + q_dot)
						]};
						if nargout >= 4
							R = cat(1,...
								cat(2, reshape(eye(p*q), p, q, p*q), zeros(p, 2*q + q_dot, p*q)),...
								zeros(q + q_dot, 3*q + q_dot, p*q)...
							);
							K = cat(1, reshape(eye(p*q_dot), p, q_dot, p*q_dot), zeros(q + q_dot, q_dot, p*q_dot));
							F_1 = cat(1,...
								cat(2, reshape(-eye(p*q), p, q, p*q), zeros(p, q_dot, p*q)),...
								zeros(q + q_dot, q + q_dot, p*q)...
							);
							F_2 = cat(1,...
								cat(2, zeros(p, q, p*q_dot), reshape(eye(p*q_dot), p, q_dot, p*q_dot)),...
								zeros(q + q_dot, q + q_dot, p*q_dot)...
							);
							RKF_fixed = {
								cat(3,...
									cat(2, R, zeros(p + q + q_dot, q_dot, p*q), F_1),...
									cat(2, zeros(p + q + q_dot, 3*q + q_dot, p*q_dot), K, F_2)...
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
								if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
									R_nonlin = constraintfun(true, 2*p + 1:q, 4*q + 1:q);
								else
									R_nonlin = constraintfun(false, p + q + 1:q_dot, 3*q + 1:q_dot);
								end
							end
						end
					end
				end
			end
		end
		
		function [R_gain, K_gain, F_prefilter] = gainpattern_parametric_system(this, ~, ~, B, C, C_dot, ~, ~, ~, T)
			%GAINPATTERN_PARAMETRIC_SYSTEM return parametric gain matrix for a real PID output feedback gain matrix R = [
			%		F,	R, -K_I,	0;
			%		0,	0,	0,		0;
			%		0,	0,	0,		T
			%	], gain matrix K = [
			%		-K;
			%		0;
			%		0
			%	] and prefilter matrix F = [
			%		F,	K;
			%		I,	0;
			%		0,	0
			%	] in continuous time and R = [
			%		F,	R,	-K_I,	K,	0;
			%		0,	0,	0,		0,	0;
			%		0,	0,	0,		0,	0;
			%		0,	0,	0,		0,	T
			%	] and F = [
			%		F,	0;
			%		I,	0;
			%		K,	-K;
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
			%n = size(A, 1);
			p = size(B, 2);
			q = size(C, 1);
			q_dot = size(C_dot, 1);
			realK = this.realD;
			if isempty(realK)
				realK = NaN(n, n);
			end
			if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
				if size(realK, 1) < q
					error('control:design:outputfeedback:input', 'Not enough realization poles (%d) supplied for system of size %d.', size(realK, 1), q);
				end
				T = realK(1:q, 1:q);
				T_free = isnan(T);
				T(T_free) = 1;
				T = realp('T', T);
				T.Free(~T_free) = false;
				F = realp('F', ones(p, q));
				R = realp('R', ones(p, q));
				K_I = realp('K_I', ones(p, q));
				K = realp('K', ones(p, q));
				R_gain = [
					F,	R,	-K_I,	-K,	zeros(p, q);
					zeros(p + q, 5*q);
					zeros(q, 4*q),		T
				];
				if nargout >= 2
					K_gain = zeros(2*p + 2*q, 0);
					if nargout >= 3
						F_prefilter = [
							F,		zeros(p, q);
							eye(q),	zeros(q, q);
							K,		-K;
							zeros(q, 2*q)
						];
					end
				end
			else
				if size(realK, 1) < q_dot
					error('control:design:outputfeedback:input', 'Not enough realization poles (%d) supplied for system of size %d.', size(realK, 1), q_dot);
				end
				T = realK(1:q_dot, 1:q_dot);
				T_free = isnan(T);
				T(T_free) = 1;
				T = realp('T', T);
				T.Free(~T_free) = false;
				F = realp('F', ones(p, q));
				R = realp('R', ones(p, q));
				K_I = realp('K_I', ones(p, q));
				K = realp('K', ones(p, q_dot));
				R_gain = [
					F,		R,	-K_I,		zeros(p, q_dot);
					zeros(q, 3*q + q_dot);
					zeros(q_dot, 3*q),		T
				];
				if nargout >= 2
					K_gain = [
						-K;
						zeros(q + q_dot, q_dot)
					];
					if nargout >= 3
						F_prefilter = [
							F,		K;
							eye(q),	zeros(q, q_dot);
							zeros(q_dot, q + q_dot)
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
			q_dot = size(C_dot, 1);
			if isempty(T_w)
				T_w = T_y;
			end
			if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
				T_x = blkdiag(T_x, eye(3*q));
				if nargout >= 2
					T_u = blkdiag(T_u, eye(q), T_u, eye(q));
					if nargout >= 3
						T_y = blkdiag(T_y, T_y, eye(3*q));
						if nargout >= 5
							T_w = blkdiag(T_w, T_w);
						end
					end
				end
			else
				T_x = blkdiag(T_x, eye(q + q_dot));
				if nargout >= 2
					T_u = blkdiag(T_u, eye(q + q_dot));
					if nargout >= 3
						if nargout >= 5
							T_w = blkdiag(T_w, T_y_dot);
						end
						T_y = blkdiag(T_y, T_y, eye(q + q_dot));
					end
				end
			end
		end
		
		function [F, F_fixed] = prefilterpattern_system(~, R, K, ~, ~, B, C, C_dot, ~, ~, ~, T)
			%PREFILTERPATTERN_SYSTEM return prefilter and prefilter pattern constraint system for a real PID output feedback with given gain matrices
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
			q_dot = size(C_dot, 1);
			if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
				F = [
					R(1:p, 1:q),			zeros(p, q);
					eye(q),					zeros(q, q);
					R(1:p, 3*q + 1:4*q),	-R(1:p, 3*q + 1:4*q);
					zeros(q, 2*q)
				];
				if nargout >= 2
					F_fixed = true(2*p + 2*q, 2*q);
				end
			else
				F = [
					R(1:p, 1:q), -K(1:p, 1:q_dot);
					eye(q),		zeros(q, q_dot);
					zeros(q_dot, q + q_dot)
				];
				if nargout >= 2
					F_fixed = true(p + q + q_dot, q + q_dot);
				end
			end
		end
		
		function [partitionR, partitionF] = gainpartitioning_system(~, R, K, F, ~, ~, B, C, C_dot, ~, ~, ~, T)
			%GAINPARTITIONING_SYSTEM return partitioning for gain matrix of extended system for a real PID output feedback with given gain matrix
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
			q_dot = size(C_dot, 1);
			if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
				partitionR = struct(...
					'F',	R(1:p, 1:q),...
					'R',	R(1:p, q + 1:2*q),...
					'K_I',	-R(1:p, 2*q + 1:3*q),...
					'K_D',	R(1:p, 3*q + 1:4*q),...
					'T',	R(2*p + q + 1:2*p + 2*q, 4*q + 1:5*q)...
				);
				if nargout >= 2
					partitionF = struct(...
						'F',	F(1:p, 1:q),...
						'K_D',	F(1:p, q + 1:2*q)...
					);
				end
			else
				partitionR = struct(...
					'F',	R(1:p, 1:q),...
					'R',	R(1:p, q + 1:2*q),...
					'K_I',	-R(1:p, 2*q + 1:3*q),...
					'K_D',	-K(1:p, 1:q_dot),...
					'T',	R(p + q + 1:p + q + q_dot, 3*q + 1:3*q + q_dot)...
				);
				if nargout >= 2
					partitionF = struct(...
						'F',	F(1:p, 1:q),...
						'K_D',	F(1:p, q + 1:q + q_dot)...
					);
				end
			end
		end
		
		function [E, A, B, C, C_dot, D, C_ref, D_ref, needsstate, usesCasCdot] = realization_system(this, R, K, F, E, A, B, C, C_dot, D, C_ref, D_ref, T)
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
			usesCasCdot = false;
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
				E = eye(3*q);
				A = [
					eye(q),			zeros(q, 2*q);
					zeros(q, q),	-partitionR.T,	zeros(q, q);
					zeros(q, q),	eye(q),			zeros(q, q)
				];
				B = [
					-eye(q),	eye(q),	zeros(q, q);
					eye(q),		zeros(q, 2*q);
					eye(q),		zeros(q, 2*q)
				];
				C = [
					partitionR.K_I,	-partitionR.K_D,	partitionR.K_D
				];
				C_dot = zeros(0, 3*q);
				D = [
					-(partitionR.R + partitionR.F),	partitionF.F + partitionF.K_D,	-partitionF.K_D
				];
				C_ref = zeros(0, size(A, 1));
				D_ref = zeros(0, size(B, 2));
			else
				E = eye(q + q_dot);
				A = [
					zeros(q, q + q_dot);
					zeros(q_dot, q),	-partitionR.T
				];
				B = [
					-eye(q),			zeros(q, q_dot),	eye(q),	zeros(q, q_dot);
					zeros(q_dot, q),	eye(q_dot),			zeros(q_dot, q + q_dot)
				];
				C = [
					partitionR.K_I,	-partitionR.K_D
				];
				C_dot = zeros(0, q + q_dot);
				D = [
					-(partitionR.R + partitionR.F),	zeros(p, q_dot),	partitionF.F,	partitionF.K_D
				];
				C_ref = zeros(0, size(A, 1));
				D_ref = zeros(0, size(B, 2));
			end
		end
	end
end

function [fun] = constraintfun(isdiscrete, idxR1, idxR2, ~, ~)
	%CONSTRAINTFUN return constraint function for dynamical part of PIDRealDirectOutputFeedback
	%	Input:
	%		isdiscrete:	indicator, if system is discrete
	%		idxR1:		indexing vector for first dimension of proportional gain matrix
	%		idxR2:		indexing vector for second dimension of proportional gain matrix
	%		idxK1:		indexing vector for first dimension of derivative gain matrix
	%		idxK2:		indexing vector for second dimension of derivative gain matrix
	%	Output:
	%		fun:		function handle to constraint function
	if length(idxR1) ~= length(idxR2)
		error('control:design:outputfeedback:input', 'System matrix must be square.');
	end
	if ~islogical(isdiscrete)
		error('control:design:outputfeedback:input', 'Discrete system indicator must be of type ''logical''.');
	end
	function [c_R, ceq_R, c_K, ceq_K, c_F, ceq_F, gradc_R, gradceq_R, gradc_K, gradceq_K, gradc_F, gradceq_F] = f_z(R, ~, ~)
		%F_Z constraint function for stable dynamics in z domain
		%	Input:
		%		R:			proportional gain matrix
		%		K:			derivative gain matrix
		%		F:			prefilter matrix
		%	Output:
		%		c_R:		inequality constraints on proportional gain matrix
		%		ceq_R:		equality constraints on proportional gain matrix
		%		c_K:		inequality constraints on derivative gain matrix
		%		ceq_K:		equality constraints on derivative gain matrix
		%		c_F:		inequality constraints on prefilter matrix
		%		ceq_F:		equality constraints on prefilter matrix
		%		gradc_R:	gradient of inequality constraints on proportional gain matrix
		%		gradceq_R:	gradient of equality constraints on proportional gain matrix
		%		gradc_K:	gradient of inequality constraints on derivative gain matrix
		%		gradceq_K:	gradient of equality constraints on derivative gain matrix
		[c_R, gradc_R_temp] = constraint_dynamics_stable_z(-R(idxR1, idxR2));
		gradc_R = zeros([size(R), size(c_R)]);
		gradc_R(idxR1, idxR2, :) = -gradc_R_temp;
		ceq_R = [];
		gradceq_R = [];
		c_K = [];
		gradc_K = [];
		ceq_K = [];
		gradceq_K = [];
		c_F = [];
		gradc_F = [];
		ceq_F = [];
		gradceq_F = [];
	end
	function [c_R, ceq_R, c_K, ceq_K, c_F, ceq_F, gradc_R, gradceq_R, gradc_K, gradceq_K, gradc_F, gradceq_F] = f_s(R, ~, ~)
		%F_S constraint function for stable dynamics in s domain
		%	Input:
		%		R:			proportional gain matrix
		%		K:			derivative gain matrix
		%		F:			prefilter matrix
		%	Output:
		%		c_R:		inequality constraints on proportional gain matrix
		%		ceq_R:		equality constraints on proportional gain matrix
		%		c_K:		inequality constraints on derivative gain matrix
		%		ceq_K:		equality constraints on derivative gain matrix
		%		c_F:		inequality constraints on prefilter matrix
		%		ceq_F:		equality constraints on prefilter matrix
		%		gradc_R:	gradient of inequality constraints on proportional gain matrix
		%		gradceq_R:	gradient of equality constraints on proportional gain matrix
		%		gradc_K:	gradient of inequality constraints on derivative gain matrix
		%		gradceq_K:	gradient of equality constraints on derivative gain matrix
		[c_R, gradc_R_temp] = constraint_dynamics_stable_s(-R(idxR1, idxR2));
		gradc_R = zeros([size(R), size(c_R)]);
		gradc_R(idxR1, idxR2, :) = -gradc_R_temp;
		ceq_R = [];
		gradceq_R = [];
		c_K = [];
		gradc_K = [];
		ceq_K = [];
		gradceq_K = [];
		c_F = [];
		gradc_F = [];
		ceq_F = [];
		gradceq_F = [];
	end
	if isdiscrete
		fun = @f_z;
	else
		fun = @f_s;
	end
end
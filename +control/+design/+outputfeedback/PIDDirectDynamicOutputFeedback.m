classdef PIDDirectDynamicOutputFeedback < control.design.outputfeedback.AbstractDynamicOrderModelFeedback
	%PIDDIRECTDYNAMICOUTPUTFEEDBACK class for casting a control system in PID output feedback form with additional dynamics and specify the needed constraints on the resulting gain matrix
	%	For the continuous control system
	%		Ex' = Ax + Bu
	%		y = Cx + Du
	%		y' = C_dot x
	%	the control law u = -RCx + K_I x_I + K w' - K C_dot x' + Fw - FCx - FDu + C_D x_D + D_D C x + D_D D u with the additional dynamic x_I' = 0x_I + w - Cx - Du and x_D' = A_D x_D + B_D u results in the output feedback form
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
	%			C_dot,	0,	0
	%		])x' = ([
	%			A,	0,	0;
	%			-C,	0,	0;
	%			0,	0,	0
	%		] - [
	%			B,	0,	0;
	%			-D,	I,	0;
	%			0,	0,	I
	%		][
	%			F,		R,	-D_D,	-K_I,	-C_D;
	%			0,		0,	0,		0,		0;
	%			0,		0,	-B_D,	0,		-A_D
	%		][
	%			C,	0,	0;
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
	%	the control law u = -RCx_k + K_I x_I_k + F w_k - F C x_k - F 0 u_k + K w_k - K C x_k + K x_D_k - K w_{k-1} + C_D x_A_k + D_D C x_k + D_D 0 u_k with the additional dynamic x_I_{k+1} = x_I_k + w_k - C x_k - D x_k, x_D_{k+1} = C x_k + D u_k and x_A_{k+1} = A_D x_k + B_D C x_k + B_D D u_k results in the output feedback form
	%		[
	%			E,	0,	0,	0;
	%			0,	I,	0,	0;
	%			0,	0,	I,	0;
	%			0,	0,	0,	I
	%		]x_{k+1} = [
	%			A,	0,	0,	0;
	%			-C,	I,	0,	0;
	%			C,	0,	0,	0;
	%			0,	0,	0,	0
	%		]x_k - [
	%			B,	0,	B,	0;
	%			-D,	I,	-D,	0;
	%			D,	0,	D,	0;
	%			0,	0,	0,	I
	%		][
	%			F,	R,	-D_D,	-K_I,	-K,	-C_D;
	%			0,	0,	0,		0,		0,	0;
	%			0,	0,	0,		0,		0,	0;
	%			0,	0,	-B_D,	0,		0,	-A_D
	%		][
	%			C,	0,	0,	0;
	%			C,	0,	0,	0;
	%			C,	0,	0,	0;
	%			0,	I,	0,	0;
	%			-C,	0,	I,	0;
	%			0,	0,	0,	I
	%		]x_k + [
	%			B,	0,	B,	0;
	%			-D,	I,	-D,	0;
	%			D,	0,	D,	0;
	%			0,	0,	0,	I
	%		][
	%			F,	-K;
	%			I,	0;
	%			K,	0;
	%			0,	0
	%		] [
	%			w_k;
	%			w_{k-1}
	%		]
	%	In contrast to the PIDDynamicOutputFeedback F (w - y) is fed back directly.
	
	methods(Static=true)
		function [name] = SimulinkVariant()
			%SIMULINKVARIANT return name of corresponding simulink variant for controller block in control_outputfeedback_lib
			%Output:
			%		name:	name of the corresponding simulink variant
			name = 'PIDDirectDynamicOutputFeedback';
		end
	end
	
	methods
		function [this] = PIDDirectDynamicOutputFeedback(system, varargin)
			%PIDDIRECTDYNAMICOUTPUTFEEDBACK create new PID output feedback class with additional dynamics
			%	Input:
			%		system:		state space system or structure with system matrices or descriptor matrix or system order
			%		A:			system matrix or sample time if system or system order is given as first argument
			%		B:			control matrix
			%		C:			output matrix
			%		C_dot:		derivative output matrix
			%		D:			throughput matrix
			%		T:			sampling time
			%	Output:
			%		this:		instance
			this@control.design.outputfeedback.AbstractDynamicOrderModelFeedback(system, varargin{:});
		end
	end
	methods(Access=protected)
		function [E, A, B, C, C_dot, D, C_ref, D_ref] = amend_system(this, E, A, B, C, C_dot, D, C_ref, ~, T)
			%AMEND_SYSTEM add additional dynamics and partition matrices according to a PID output feedback with additional dynamics
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
				error('control:design:outputfeedback:input', 'Descriptor matrix must be regular for a PID state feedback controller.');
			end
			nnom = this.n;
			if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
				if control.design.outputfeedback.OutputFeedback.isranksupported(D) && rank(D) > 0
					error('control:design:outputfeedback:input', 'System must not have a throughput matrix.');
				end
				C_ref = [
					C_ref, zeros(size(C_ref, 1), 2*q + nnom);
					C_ref, zeros(size(C_ref, 1), 2*q + nnom)
				];
				D_ref = zeros(size(C_ref, 1), 2*q);
				E = [
					E,						zeros(n, 2*q + nnom);
					zeros(2*q + nnom, n),	eye(2*q + nnom)
				];
				A = [
					A,					zeros(n, 2*q + nnom);
					-C,					eye(q),			zeros(q, q + nnom);
					C,					zeros(q, 2*q + nnom);
					zeros(nnom, n + 2*q + nnom)
				];
				B = [
					B,	zeros(n, q),	B,	zeros(n, nnom);
					-D,	eye(q),			-D,	zeros(q, nnom);
					D,	zeros(q, q),	D,	zeros(q, nnom);
					zeros(nnom,	2*p + q),	eye(nnom)
				];
				C = [
					C,				zeros(q, 2*q + nnom);
					C,				zeros(q, 2*q + nnom);
					C,				zeros(q, 2*q + nnom);
					zeros(q, n),	eye(q),			zeros(q, q + nnom);
					-C,				zeros(q, q),	eye(q),	zeros(q, nnom);
					zeros(nnom, n + 2*q),			eye(nnom)
				];
				C_dot = zeros(0, n + 2*q + nnom);
				D = zeros(size(C, 1), size(B, 2));
			else
				C_ref = [
					C_ref, zeros(size(C_ref, 1), q + nnom);
					zeros(q_dot, size(C_ref, 2) + q + nnom)
				];
				D_ref = zeros(size(C_ref, 1), q + q_dot);
				E = [
					E,					zeros(n, q + nnom);
					zeros(q + nnom, n),	eye(q + nnom)
				];
				A = [
					A,		zeros(n, q + nnom);
					-C,		zeros(q, q + nnom);
					zeros(nnom, n + q + nnom)
				];
				B = [
					B,						zeros(n, q + nnom);
					-D,						eye(q),	zeros(q, nnom);
					zeros(nnom, p + q),				eye(nnom)
				];
				C = [
					C,					zeros(q, q + nnom);
					C,					zeros(q, q + nnom);
					C,					zeros(q, q + nnom);
					zeros(q + nnom, n),	eye(q + nnom)
				];
				C_dot = [
					C_dot,	zeros(q_dot, q + nnom);
				];
				D = zeros(size(C, 1), size(B, 2));
			end
		end
		
		function [R_fixed, K_fixed, F_fixed, RKF_fixed, R_bounds, K_bounds, F_bounds, RKF_bounds, R_nonlin] = gainpattern_system(this, ~, ~, B, C, C_dot, ~, ~, ~, T)
			%GAINPATTERN_SYSTEM return gain pattern constraint system for a PID output feedback with additional dynamics gain matrix R = [
			%		F,		R,	-D_D,	-K_I,	-C_D;
			%		0,		0,	0,		0,		0;
			%		0,		0,	-B_D,	0,		-A_D
			%	] and gain matrix K = [
			%		-K;
			%		0;
			%		0
			%	] in continuous time and R = [
			%		F,	R,	-D_D,	-K_I,	-K,	-C_D;
			%		0,	0,	0,		0,		0,	0;
			%		0,	0,	0,		0,		0,	0;
			%		0,	0,	-B_D,	0,		0,	-A_D
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
			%		R_nonlin:	function pointer to nonlinear constraints on proportional and derivative gain matrix
			%n = size(A, 1);
			p = size(B, 2);
			q = size(C, 1);
			q_dot = size(C_dot, 1);
			nnom = this.n;
			A_nom = this.A_model_ref;
			B_nom = this.B_model_ref;
			C_nom = this.C_model_ref;
			D_nom = this.D_model_ref;
			if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
				R = [
					NaN(p, 2*q),		D_nom(1:p, 1:q),	NaN(p, 2*q),		C_nom(1:p, 1:nnom);
					zeros(p + q, 5*q + nnom);
					zeros(nnom, 2*q),	B_nom(1:nnom, 1:q),	zeros(nnom, 2*q),	A_nom(1:nnom, 1:nnom)
				];
				R_fixed = {~isnan(R), R};
				if nargout >= 2
					K_fixed = {true(2*p + q + nnom, 0), zeros(2*p + q + nnom, 0)};
					if nargout >= 3
						F_fixed = {[
							false(p, 2*q);
							true(q, 2*q);
							false(p, q),	true(p, q);
							true(nnom, 2*q)
						], [
							NaN(p, 2*q);
							eye(q),			zeros(q, q);
							NaN(p, q),		zeros(p, q);
							zeros(nnom, 2*q)
						]};
						if nargout >= 4
							R = cat(1,...
								cat(2, reshape(eye(p*q), p, q, p*q), zeros(p, 4*q + nnom, p*q)),...
								zeros(p + q + nnom, 5*q + nnom, p*q)...
							);
							K = cat(1,...
								cat(2, zeros(p, 4*q, p*q), reshape(eye(p*q), p, q, p*q), zeros(p, nnom, p*q)),...
								zeros(p + q + nnom, 5*q + nnom, p*q)...
							);
							F_1 = cat(1,...
								cat(2, reshape(-eye(p*q), p, q, p*q), zeros(p, q, p*q)),...
								zeros(p + q + nnom, 2*q, p*q)...
							);
							F_2 = cat(1,...
								cat(2, zeros(p, q, p*q), reshape(-eye(p*q), p, q, p*q)),...
								zeros(p + q + nnom, 2*q, p*q)...
							);
							F_3 = cat(1,...
								zeros(p, 2*q, p*q),...
								cat(2, reshape(eye(p*q), p, q, p*q), zeros(p, q, p*q)),...
								zeros(q + nnom, 2*q, p*q)...
							);
							RKF_fixed = {
								cat(3,...
									cat(2, R, zeros(2*p + q + nnom, 0, p*q), F_1),...
									cat(2, K, zeros(2*p + q + nnom, 0, p*q), F_2),...
									cat(2, K, zeros(2*p + q + nnom, 0, p*q), F_3)...
								), zeros(3*p*q, 1)
							};
						end
					end
				end
			else
				R = [
					NaN(p, 2*q),		D_nom(1:p, 1:q),	NaN(p, q),		C_nom(1:p, 1:nnom);
					zeros(q, 4*q + nnom);
					zeros(nnom, 2*q),	B_nom(1:nnom, 1:q),	zeros(nnom, q),	A_nom(1:nnom, 1:nnom)
				];
				R_fixed = {~isnan(R), R};
				if nargout >= 2
					K = [
						NaN(p, q_dot);
						zeros(q + nnom, q_dot)
					];
					K_fixed = {~isnan(K), K};
					if nargout >= 3
						F_fixed = {[
							false(p, q + q_dot);
							true(q + nnom, q + q_dot)
						], [
							NaN(p, q + q_dot);
							eye(q),		zeros(q, q_dot);
							zeros(nnom, q + q_dot)
						]};
						if nargout >= 4
							R = cat(1,...
								cat(2, reshape(eye(p*q), p, q, p*q), zeros(p, 3*q + nnom, p*q)),...
								zeros(q + nnom, 4*q + nnom, p*q)...
							);
							K = cat(1,...
								reshape(eye(p*q_dot), p, q_dot, p*q_dot),...
								zeros(q + nnom, q_dot, p*q_dot)...
							);
							F_1 = cat(1,...
								cat(2, reshape(-eye(p*q), p, q, p*q), zeros(p, q_dot, p*q)),...
								zeros(q + nnom, q + q_dot, p*q)...
							);
							F_2 = cat(1,...
								cat(2, zeros(p, q, p*q_dot), reshape(eye(p*q_dot), p, q_dot, p*q_dot)),...
								zeros(q + nnom, q + q_dot, p*q_dot)...
							);
							RKF_fixed = {
								cat(3,...
									cat(2, R, zeros(p + q + nnom, q_dot, p*q), F_1),...
									cat(2, zeros(p + q + nnom, 4*q + nnom, p*q_dot), K, F_2)...
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
									R_nonlin = constraintfun(true, 2*p + q + (1:nnom), 5*q + (1:nnom));
								else
									R_nonlin = constraintfun(false, p + q + (1:nnom), 4*q + (1:nnom));
								end
							end
						end
					end
				end
			end
		end
		
		function [R_gain, K_gain, F_prefilter] = gainpattern_parametric_system(this, ~, ~, B, C, C_dot, ~, ~, ~, T)
			%GAINPATTERN_PARAMETRIC_SYSTEM return parametric gain matrix for a dynamic PID output feedback gain matrix R = [
			%		F,		R,	-D_D,	-K_I,	-C_D;
			%		0,		0,	0,		0,		0;
			%		0,		0,	-B_D,	0,		-A_D
			%	], gain matrix K = [
			%		-K;
			%		0;
			%		0
			%	] and prefilter matrix F = [
			%		F,	K;
			%		I,	0;
			%		0,	0
			%	] in continuous time and R = [
			%		F,	R,	-D_D,	-K_I,	-K,	-C_D;
			%		0,	0,	0,		0,		0,	0;
			%		0,	0,	0,		0,		0,	0;
			%		0,	0,	-B_D,	0,		0,	-A_D
			%	] and F = [
			%		F,	-K;
			%		I,	0;
			%		K,	0;
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
			nnom = this.n;
			A_nom = this.A_model_ref(1:nnom, 1:nnom);
			B_nom = this.B_model_ref(1:nnom, 1:q);
			C_nom = this.C_model_ref(1:p, 1:nnom);
			D_nom = this.D_model_ref(1:p, 1:q);
			A_nom_free = isnan(A_nom);
			A_nom(isnan(A_nom)) = 1;
			B_nom_free = isnan(B_nom);
			B_nom(isnan(B_nom)) = 1;
			C_nom_free = isnan(C_nom);
			C_nom(isnan(C_nom)) = 1;
			D_nom_free = isnan(D_nom);
			D_nom(isnan(D_nom)) = 1;
			A_D = realp('A_D', A_nom);
			A_D.Free(~A_nom_free) = false;
			B_D = realp('B_D', B_nom);
			B_D.Free(~B_nom_free) = false;
			C_D = realp('C_D', C_nom);
			C_D.Free(~C_nom_free) = false;
			D_D = realp('D_D', D_nom);
			D_D.Free(~D_nom_free) = false;
			if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
				F = realp('F', ones(p, q));
				R = realp('R', ones(p, q));
				K_I = realp('K_I', ones(p, q));
				K = realp('K', ones(p, q));
				R_gain = [
					F,	R,				-D_D,	-K_I,	-K,			-C_D;
					zeros(p + q, 5*q + nnom);
					zeros(nnom, 2*q),	-B_D,	zeros(nnom, 2*q),	-A_D
				];
				if nargout >= 2
					K_gain = zeros(2*p + q + nnom, 0);
					if nargout >= 3
						F_prefilter = [
							F,		K;
							eye(q),	zeros(q, q);
							K,		zeros(p, q);
							zeros(nnom, 2*q)
						];
					end
				end
			else
				F = realp('F', ones(p, q));
				R = realp('R', ones(p, q));
				K_I = realp('K_I', ones(p, q));
				K = realp('K', ones(p, q_dot));
				R_gain = [
					F,		R,			-D_D,	-K_I,			-C_D;
					zeros(q, 4*q + nnom);
					zeros(nnom, 2*q),	-B_D,	zeros(nnom, q),	-A_D
				];
				if nargout >= 2
					K_gain = [
						-K;
						zeros(q + nnom, q_dot)
					];
					if nargout >= 3
						F_prefilter = [
							F,		K;
							eye(q),	zeros(q, q_dot);
							zeros(nnom, q + q_dot)
						];
					end
				end
			end
		end
		
		function [T_x, T_u, T_y, T_y_dot, T_w] = scalegain_system(this, T_x, T_u, T_y, T_y_dot, T_w, ~, ~, ~, C, ~, ~, ~, ~, T)
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
			nnom = this.n;
			if isempty(T_w)
				T_w = T_y;
			end
			if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
				T_x = blkdiag(T_x, eye(2*q + nnom));
				if nargout >= 2
					T_u = blkdiag(T_u, eye(q), T_u, eye(nnom));
					if nargout >= 3
						T_y = blkdiag(T_y, T_y, T_y, eye(q), T_y, eye(nnom));
						if nargout >= 5
							T_w = blkdiag(T_w, T_w);
						end
					end
				end
			else
				T_x = blkdiag(T_x, eye(q + nnom));
				if nargout >= 2
					T_u = blkdiag(T_u, eye(q + nnom));
					if nargout >= 3
						T_y = blkdiag(T_y, T_y, eye(2*q + nnom));
						if nargout >= 5
							T_w = blkdiag(T_w, T_y_dot);
						end
					end
				end
			end
		end
		
		function [F, F_fixed] = prefilterpattern_system(this, R, K, ~, ~, B, C, C_dot, ~, ~, ~, T)
			%PREFILTERPATTERN_SYSTEM return prefilter and prefilter pattern constraint system for a PID output feedback with additional dynamics with given gain matrices
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
			nnom = this.n;
			if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
				F = [
					R(1:p, 1:q),			R(1:p, 4*q + 1:5*q);
					eye(q),					zeros(q, q);
					-R(1:p, 4*q + 1:5*q),	zeros(p, q);
					zeros(nnom, 2*q)
				];
				if nargout >= 2
					F_fixed = [
						false(p, q),	true(p, q);
						true(q + p + nnom, 2*q)
					];
				end
			else
				F = [
					R(1:p, 1:q), -K(1:p, 1:q_dot);
					eye(q),		zeros(q, q_dot);
					zeros(nnom, q + q_dot)
				];
				if nargout >= 2
					F_fixed = true(p + q + nnom, q + q_dot);
				end
			end
		end
		
		function [partitionR, partitionF] = gainpartitioning_system(this, R, K, F, ~, ~, B, C, C_dot, ~, ~, ~, T)
			%GAINPARTITIONING_SYSTEM return partitioning for gain matrix of extended system for a PID output feedback with additional dynamics with given gain matrix
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
			nnom = this.n;
			if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
				partitionR = struct(...
					'F',	R(1:p, 1:q),...
					'R',	R(1:p, q + 1:2*q),...
					'D_D',	-R(1:p, 2*q + 1:3*q),...
					'K_I',	-R(1:p, 3*q + 1:4*q),...
					'K_D',	-R(1:p, 4*q + 1:5*q),...
					'C_D',	-R(1:p, 5*q + 1:5*q + nnom),...
					'B_D',	-R(p + q + 1:p + q + nnom, 2*q + 1:3*q),...
					'A_D',	-R(p + q + 1:p + q + nnom, 5*q + 1:5*q + nnom)...
				);
				if nargout >= 2
					partitionF = struct(...
						'F',	F(1:p, 1:q),...
						'K_D',	-F(1:p, q + 1:2*q)...
					);
				end
			else
				partitionR = struct(...
					'F',	R(1:p, 1:q),...
					'R',	R(1:p, q + 1:2*q),...
					'D_D',	-R(1:p, 2*q + 1:3*q),...
					'K_I',	-R(1:p, 3*q + 1:4*q),...
					'K_D',	-K(1:p, 1:q_dot),...
					'C_D',	-R(1:p, 4*q + 1:4*q + nnom),...
					'B_D',	-R(p + q + 1:p + q + nnom, 2*q + 1:3*q),...
					'A_D',	-R(p + q + 1:p + q + nnom, 4*q + 1:4*q + nnom)...
				);
				if nargout >= 2
					partitionF = struct(...
						'F',	F(1:p, 1:q),...
						'K_D',	F(1:p, q + (1:q_dot))...
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
			useCasCdot = false;
			%n = size(A, 1);
			%p = size(B, 2);
			q = size(C, 1);
			q_dot = size(C_dot, 1);
			if control.design.outputfeedback.OutputFeedback.isranksupported(D) && rank(D) > 0
				error('control:design:outputfeedback:input', 'System model must not have a throughput matrix.');
			end
			if control.design.outputfeedback.OutputFeedback.isranksupported(E) && rank(E) < size(A, 1)
				% TODO: allow real descriptor systems and check conditions for regular E - B K C_dot
				error('control:design:outputfeedback:input', 'Descriptor matrix must be regular for a PID state feedback controller.');
			end
			nnom = this.n;
			[partitionR, partitionF] = this.gainpartitioning_system(R, K, F, E, A, B, C, C_dot, D, C_ref, D_ref, T);
			if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
				if control.design.outputfeedback.OutputFeedback.isranksupported(D) && rank(D) > 0
					error('control:design:outputfeedback:input', 'System must not have a throughput matrix.');
				end
				E = eye(2*q + nnom);
				A = [
					eye(q),			zeros(q, q),	zeros(q, nnom);
					zeros(q, 2*q + nnom);
					zeros(nnom, 2*q),	partitionR.A_D
				];
				B = [
					-eye(q),		eye(q),	zeros(q, q);
					eye(q),			zeros(q, 2*q);
					partitionR.B_D,	zeros(nnom, 2*q)
				];
				C = [
					partitionR.K_I,	partitionR.K_D,	partitionR.C_D
				];
				C_dot = zeros(0, 2*q + nnom);
				D = [
					-(partitionR.F + partitionR.R + partitionR.K_D - partitionR.D_D),	partitionF.F + partitionF.K_D,	-partitionF.K_D
				];
				C_ref = zeros(0, size(A, 1));
				D_ref = zeros(0, size(B, 2));
			else
				E = eye(q + nnom);
				A = [
					zeros(q, q + nnom);
					zeros(nnom, q), partitionR.A_D
				];
				B = [
					-eye(q),		zeros(q, q_dot),	eye(q),	zeros(q, q_dot);
					partitionR.B_D,	zeros(nnom, q_dot),	zeros(nnom, q + q_dot)
				];
				C = [
					partitionR.K_I,	partitionR.C_D
				];
				C_dot = zeros(0, q + nnom);
				D = [
					-(partitionR.F + partitionR.R - partitionR.D_D),	-partitionR.K_D,	partitionF.F,	partitionF.K_D
				];
				C_ref = zeros(0, size(A, 1));
				D_ref = zeros(0, size(B, 2));
			end
		end
	end
end

function [fun] = constraintfun(isdiscrete, idxR1, idxR2, ~, ~)
	%CONSTRAINTFUN return constraint function for dynamical part of PIDDirectDynamicOutputFeedback
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
classdef PDRealStateFeedback < control.design.outputfeedback.AbstractDRealFeedback
	%PDREALSTATEFEEDBACK class for casting a control system in real PD state feedback form and specify the needed constraints on the resulting gain matrix
	%	For the continuous control system
	%		Ex' = Ax + Bu
	%		y = Cx + Du
	%		y' = C_dot x
	%	the control law u = -Rx + K C_dot x'??_D? + Fw + F_D w' with the additional dynamic x_D' = -T x_D + C_dot x results in the output feedback form
	%		[
	%			E,	0;
	%			0,	I
	%		][
	%			B,	0;
	%			0,	I
	%		] [
	%			K_D;
	%			0
	%		][
	%			0,	I
	%		]x' = [
	%			A,		0;
	%			C_dot,	0
	%		]x - [
	%			B,	0;
	%			0,	I
	%		][
	%			R,	0;
	%			0,	T
	%		][
	%			I,	0;
	%			0,	I
	%		]x + [
	%			B,	0;
	%			0,	I
	%		][
	%			F,	F_D;
	%			0,	0
	%		][
	%			w;
	%			w'
	%		]
	%	For the discrete control system
	%		Ex_{k+1} = Ax_k + Bu_k
	%		y_k = Cx_k + Du_k
	%		y_{k+1} = C_dot x_{k+1}
	%	the control law u = -Rx_k + Kx_k??_T? - K x_D_k + Fw_k + F_D w_{k-1} with the additional dynamic x_T_{k+1} = -T x_T_k + Cx_k + Du_k and x_D_{k+1} = x_T_k results in the output feedback form
	%		[
	%			E,	0,	0;
	%			0,	I,	0;
	%			0,	0,	I
	%		]x_{k+1} = [
	%			A,	0,	0;
	%			C,	0,	0;
	%			0,	I,	0
	%		]x_k - [
	%			B,	0,	B;
	%			D,	I,	D;
	%			0,	0,	0
	%		][
	%			R,	0,	K;
	%			0,	T,	0;
	%			-K,	0,	0
	%		][
	%			I,	0,	0;
	%			0,	I,	0;
	%			0,	0,	I
	%		]x_k + [
	%			B,	0,	B;
	%			D,	I,	D;
	%			0,	0,	0
	%		][
	%			F,	F_D;
	%			0,	0;
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
			name = 'PDRealStateFeedback';
		end
	end

	methods
		function [this] = PDRealStateFeedback(varargin)
			%PDREALSTATEFEEDBACK create new real PD state feedback class
			%	Input:
			%		system:		state space system or structure with system matrices or descriptor matrix or vector with pole locations
			%		A:			system matrix or sample time if system is given as first argument
			%		B:			control matrix
			%		C:			output matrix
			%		C_dot:		derivative output matrix
			%		D:			throughput matrix
			%		T:			sampling time
			%		varargin:	pole locations of realization poles, if all arguments are scalar
			%	Output:
			%		this:		instance
			this@control.design.outputfeedback.AbstractDRealFeedback(varargin{:});
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
			if isempty(C_dot)
				C_dot = C;
			end
			q_dot = size(C_dot, 1);
			if control.design.outputfeedback.OutputFeedback.isranksupported(E) && rank(E) < size(A, 1)
				% TODO: allow real descriptor systems and check conditions for regular E - B K C_dot
				error('control:design:outputfeedback:input', 'Descriptor matrix must be regular for a PD state feedback controller.');
			end
			if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
				C_ref = [
					C_ref, zeros(size(C_ref, 1), 2*n);
					C_ref, zeros(size(C_ref, 1), 2*n)
				];
				D_ref = zeros(size(C_ref, 1), 2*q);
				E = [
					E,				zeros(n, 2*n);
					zeros(2*n, n),	eye(2*n)
				];
				A = [
					A,				zeros(n, 2*n);
					eye(n),			zeros(n, 2*n);
					zeros(n, n),	eye(n),	zeros(n, n)
				];
				B = [
					B,				zeros(n, n),	B;
					zeros(n, p),	eye(n),			zeros(n, p);
					zeros(n, 2*p + n)
				];
				C = eye(3*n);
				C_dot = zeros(0, 3*n);
				D = zeros(3*n, 2*p + n);
			else
				C_ref = [
					C_ref, zeros(size(C_ref, 1), q_dot);
					zeros(q_dot, size(C_ref, 2) + q_dot)
				];
				D_ref = zeros(size(C_ref, 1), q + q_dot);
				E = [
					E,					zeros(n, q_dot);
					zeros(q_dot, n),	eye(q_dot)
				];
				A = [
					A,			zeros(n, q_dot);
					C_dot,		zeros(q_dot, q_dot)
				];
				B = [
					B,					zeros(n, q_dot);
					zeros(q_dot, p),	eye(q_dot)
				];
				C = eye(n + q_dot);
				C_dot = [
					zeros(q_dot, n), eye(q_dot)
				];
				D = zeros(size(C, 1), size(B, 2));
			end
		end

		function [R_fixed, K_fixed, F_fixed, RKF_fixed, R_bounds, K_bounds, F_bounds, RKF_bounds, R_nonlin] = gainpattern_system(this, ~, A, B, C, C_dot, ~, ~, ~, T)
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
			if isempty(C_dot)
				C_dot = C;
			end
			q_dot = size(C_dot, 1);
			realK = this.realD;
			if isempty(realK)
				realK = NaN(n, n);
			end
			if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
				if size(realK, 1) < n
					error('control:design:outputfeedback:input', 'Not enough realization poles (%d) supplied for system of size %d.', size(realK, 1), n);
				end
				K = [
					NaN(p, n),		zeros(p, n),	NaN(p, n);
					zeros(n, n),	NaN(n, n),		zeros(n, n);
					NaN(p, n),		zeros(p, 2*n)
				];
				nanrealD = ~isnan(realK(1:n, 1:n));
				[nanrow, nancol] = find(nanrealD);
				constrainedrealK = size(nanrow, 1);
				[row, col] = find(K == 0);
				number_zeros = size(row, 1);
				number_K = p*n;
				R_fixed_system = NaN([size(K), number_zeros + number_K + constrainedrealK]);
				R_fixed_border = zeros(number_zeros + number_K + constrainedrealK, 1);
				parfor ii = 1:number_zeros
					temp = zeros(size(K));
					temp(row(ii), col(ii)) = 1;
					R_fixed_system(:, :, ii) = temp;
				end
				K = [
					zeros(p, 2*n),	ones(p, n);
					zeros(n, 2*n),	zeros(n, n);
					zeros(p, 3*n)
				];
				[row, col] = find(K == 1);
				parfor ii = 1:size(row, 1)
					temp = zeros(size(K));
					temp(row(ii), col(ii)) = 1;
					temp(row(ii) + p + n, col(ii) - 2*n) = 1;
					R_fixed_system(:, :, ii + number_zeros) = temp;
				end
				for ii = 1:constrainedrealK
					temp = zeros(size(K));
					temp(nanrow(ii), nancol(ii)) = 1;
					R_fixed_system(:, :, ii + number_zeros + number_K) = temp;
					R_fixed_border(ii + number_zeros + number_K, 1) = realK(nanrow(ii), nancol(ii));
				end
				R_fixed = {R_fixed_system, R_fixed_border};
				if nargout >= 2
					K_fixed = {true(2*p + n, 0), zeros(2*p + n, 0)};
					if nargout >= 3
						F_fixed = {[
							false(p, 2*q);
							true(n + p, 2*q);
						], [
							NaN(p, 2*q);
							zeros(n + p, 2*q)
						]};
					end
				end
			else
				if size(realK, 1) < q_dot
					error('control:design:outputfeedback:input', 'Not enough realization poles (%d) supplied for system of size %d.', size(realK, 1), n);
				end
				K = [
					NaN(p, n),			zeros(p, q_dot);
					zeros(q_dot, n),	realK(1:q_dot, 1:q_dot)
				];
				R_fixed = {~isnan(K), K};
				if nargout >= 2
					K = [
						NaN(p, q_dot);
						zeros(q_dot, q_dot)
					];
					K_fixed = {~isnan(K), K};
					if nargout >= 3
						F_fixed = {[
							false(p, q + q_dot);
							true(q_dot, q + q_dot)
						], [
							NaN(p, q + q_dot);
							zeros(q_dot, q + q_dot)
						]};
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
								if nargout >= 9
									if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
										R_nonlin = constraintfun(true, p + (1:n), n + (1:n));
									else
										R_nonlin = constraintfun(false, p + (1:q_dot), n + (1:q_dot));
									end
								end
							end
						end
					end
				end
			end
		end

		function [R_gain, K_gain, F_prefilter] = gainpattern_parametric_system(this, ~, A, B, C, C_dot, ~, ~, ~, T)
			%GAINPATTERN_PARAMETRIC_SYSTEM return parametric gain matrix for a real PD state feedback gain matrix R = [
			%		R,	0;
			%		0,	T
			%	], gain matrix K = [
			%		K_D;
			%		0
			%	] and prefilter matrix F = [
			%		F,	F_D;
			%		0,	0
			%	] in continuous time and R = [
			%		R,	0,	K;
			%		0,	T,	0;
			%		-K,	0,	0
			%	] and F = [
			%		F,	F_D;
			%		0,	0;
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
			if isempty(C_dot)
				C_dot = C;
			end
			q_dot = size(C_dot, 1);
			realK = this.realD;
			if isempty(realK)
				realK = NaN(n, n);
			end
			if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
				if size(realK, 1) < n
					error('control:design:outputfeedback:input', 'Not enough realization poles (%d) supplied for system of size %d.', size(realK, 1), n);
				end
				R = realp('R', ones(p, n));
				K = realp('K', ones(p, n));
				T = realK(1:n, 1:n);
				T_free = isnan(T);
				T(T_free) = 1;
				T = realp('T', T);
				T.Free(~T_free) = false;
				R_gain = [
					R,				zeros(p, n),	K;
					zeros(n, n),	T,				zeros(n, n);
					-K,				zeros(p, 2*n)
				];
				if nargout >= 2
					K_gain = zeros(2*p + n, 0);
					if nargout >= 3
						F = realp('F', ones(p, q));
						F_D = realp('F_D', ones(p, q));
						F_prefilter = [
							F,		F_D;
							zeros(p + n, 2*q)
						];
					end
				end
			else
				if size(realK, 1) < n
					error('control:design:outputfeedback:input', 'Not enough realization poles (%d) supplied for system of size %d.', size(realK, 1), n);
				end
				F = realp('F', ones(p, q));
				R = realp('R', ones(p, n));
				T = realK(1:q_dot, 1:q_dot);
				T_free = isnan(T);
				T(T_free) = 1;
				T = realp('T', T);
				T.Free(~T_free) = false;
				K_D = realp('K_D', ones(p, q_dot));
				R_gain = [
					R,					zeros(p, q_dot);
					zeros(q_dot, n),	T
				];
				if nargout >= 2
					K_gain = [
						K_D;
						zeros(q_dot, q_dot)
					];
					if nargout >= 3
						F_D = realp('F_D', ones(p, q_dot));
						F_prefilter = [
							F,	F_D;
							zeros(q_dot, q + q_dot)
						];
					end
				end
			end
		end

		function [T_x, T_u, T_y, T_y_dot, T_w] = scalegain_system(~, T_x, T_u, T_y, T_y_dot, T_w, ~, A, ~, C, C_dot, ~, ~, ~, T)
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
			q = size(C, 1);
			if isempty(C_dot)
				T_y_dot = T_y;
				C_dot = C;
			end
			q_dot = size(C_dot, 1);
			if isempty(T_w)
				T_w = T_y;
			end
			if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
				if nargout >= 2
					T_u = blkdiag(T_u, eye(n), T_u);
					if nargout >= 3
						T_y = blkdiag(T_x, eye(2*n));
						if nargout >= 5
							T_w = blkdiag(T_w, T_w);
						end
					end
				end
				T_x = blkdiag(T_x, eye(2*n));
			else
				if nargout >= 2
					T_u = blkdiag(T_u, eye(q_dot));
					if nargout >= 3
						T_y = blkdiag(T_x, eye(q_dot));
						if nargout >= 4
							if nargout >= 5
								T_y_dot = eye(q_dot);
								T_w = blkdiag(T_w, T_y_dot);
							end
						end
					end
				end
				T_x = blkdiag(T_x, eye(q_dot));
			end
		end

		function [F, F_fixed] = prefilterpattern_system(~, ~, ~, ~, A, B, C, C_dot, ~, ~, ~, T)
			%PREFILTERPATTERN_SYSTEM return prefilter and prefilter pattern constraint system for a real PD state feedback with given gain matrices
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
				C_dot = C;
			end
			q_dot = size(C_dot, 1);
			if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
				F = [
					eye(p, q),		eye(p, q);
					zeros(n + p, 2*q)
				];
				if nargout >= 2
					F_fixed = [
						false(p, 2*q);
						true(n + p, 2*q);
					];
				end
			else
				F = [
					eye(p, q),		eye(p, q_dot);
					zeros(q_dot, q + q_dot)
				];
				if nargout >= 2
					F_fixed = false(p + q_dot, q + q_dot);
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
			q_dot = size(C_dot, 1);
			if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
				partitionR = struct(...
					'R',	R(1:p, 1:n),...
					'K_D',	R(1:p, 2*n + 1:3*n),...
					'T',	R(p + 1:p + n, n + 1:2*n)...
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
					'K_D',	K(1:p, 1:q_dot),...
					'T',	R(p + 1:p + q_dot, n + 1:n + q_dot)...
				);
				if nargout >= 2
					partitionF = struct(...
						'F',	F(1:p, 1:q),...
						'F_D',	F(1:p, q + 1:q + q_dot)...
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
			needsstate = true;
			if isempty(C_dot)
				C_dot = C;
				useCasCdot = true;
			else
				useCasCdot = false;
			end
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
			[partitionR, partitionF] = this.gainpartitioning_system(R, K, F, E, A, B, C, C_dot, D, C_ref, D_ref, T);
			if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
				if control.design.outputfeedback.OutputFeedback.isranksupported(D) && rank(D) > 0
					error('control:design:outputfeedback:input', 'System must not have a throughput matrix.');
				end
				E = eye(2*n);
				A = [
					-partitionR.T,	zeros(n, n);
					eye(n),			zeros(n, n)
				];
				B = [
					eye(n),		zeros(n, 2*q);
					zeros(n, n + 2*q)
				];
				C = [
					partitionR.K_D,	-partitionR.K_D
				];
				C_dot = zeros(0, 2*n);
				D = [
					-partitionR.R,	partitionF.F,	partitionF.F_D
				];
				C_ref = zeros(0, size(A, 1));
				D_ref = zeros(0, size(B, 2));
			else
				E = eye(q_dot);
				A = -partitionR.T;
				B = [
					zeros(q_dot, n),	eye(q_dot),			zeros(q_dot, q + q_dot)
				];
				C = partitionR.K_D;
				C_dot = zeros(0, q_dot);
				D = [
					-partitionR.R,	zeros(p, q_dot),	partitionF.F,	partitionF.F_D
				];
				C_ref = zeros(0, size(A, 1));
				D_ref = zeros(0, size(B, 2));
			end
		end
	end
end

function [fun] = constraintfun(isdiscrete, idxR1, idxR2, ~, ~)
	%CONSTRAINTFUN return constraint function for dynamical part of PDRealStateFeedback
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
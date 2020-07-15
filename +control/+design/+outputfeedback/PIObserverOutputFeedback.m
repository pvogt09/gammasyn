classdef PIObserverOutputFeedback < control.design.outputfeedback.AbstractReferenceModelFeedback
	%PIOBSERVEROUTPUTFEEDBACK class for casting a control system in output feedback form and specify the needed constraints on the resulting gain matrix for a PI observer output feedback controller
	%	For the control system
	%		Ex' = Ax + Bu
	%		y = Cx + 0u
	%		y' = C_dot x
	%	the control law u = -R C_O x_O - R 0 u + K_I x_I + Fw and the additional dynamics x_I' = 0x_I - Cx - 0u + w and the observer dynamics E_O x_O' = A_O x_O + B_O u + LCx - L C_O x_O results in the output feedback form
	%		[
	%			E,	0,		0;
	%			0,	E_O,	0;
	%			0,	0,		I
	%		]x' = [
	%			A,	0,		0;
	%			0,	A_O,	0;
	%			0,	0,		0
	%		]x - [
	%			B,		0,	0;
	%			B_O,	I,	0;
	%			0,		0,	I
	%		][
	%			0,	R,	-K_I;
	%			-L,	L,	0;
	%			I,	0,	0
	%		][
	%			C,	0,		0;
	%			0,	C_O,	0;
	%			0,	0,		I
	%		]x + [B, 0, 0; B_O, I, 0; 0, 0, I] [
	%			F;
	%			0;
	%			I
	%		]w
	%	In contrast to the PIObserverDirectOutputFeedback F (w - y) is not fed back directly.

	methods(Static=true)
		function [name] = SimulinkVariant()
			%SIMULINKVARIANT return name of corresponding simulink variant for controller block in control_outputfeedback_lib
			%Output:
			%		name:	name of the corresponding simulink variant
			name = 'PIObserverOutputFeedback';
		end
	end

	methods
		function [this] = PIObserverOutputFeedback(system, varargin)
			%PIOBSERVEROUTPUTFEEDBACK create new PI observer output feedback class
			%	Input:
			%		system:		state space system or structure with system matrices or descriptor matrix to use as observer
			%		A:			system matrix of observer or sample time if system is given as first argument
			%		B:			control matrix of observer
			%		C:			output matrix of observer
			%		C_dot:		derivative output matrix of observer
			%		D:			throughput matrix of observer
			%		C_ref:		reference output matrix
			%		D_ref:		reference throughput matrix
			%		T:			sampling time of observer
			%	Output:
			%		this:	instance
			narginchk(1, 9);
			this@control.design.outputfeedback.AbstractReferenceModelFeedback(system, varargin{:});
			if control.design.outputfeedback.OutputFeedback.isranksupported(this.D_model_ref) && rank(this.D_model_ref) > 0
				error('control:design:outputfeedback:input', 'Observer model must not have a throughput matrix.');
			end
			if control.design.outputfeedback.OutputFeedback.isranksupported(this.E_model_ref) && rank(this.E_model_ref) < size(this.A_model_ref, 1)
				error('control:design:outputfeedback:input', 'Descriptor matrix of observer model must be regular.');
			end
		end
	end

	methods(Access=protected)
		function [E, A, B, C, C_dot, D, C_ref, D_ref] = amend_system(this, E, A, B, C, ~, D, C_ref, ~, T)
			%AMEND_SYSTEM add additional dynamics and partition matrices according to a PI observer output feedback
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
				error('control:design:outputfeedback:input', 'System must not have a throughput matrix.');
			end
			if control.design.outputfeedback.OutputFeedback.isranksupported(this.D_model_ref) && rank(this.D_model_ref) > 0
				error('control:design:outputfeedback:input', 'Observer model must not have a throughput matrix.');
			end
			if control.design.outputfeedback.OutputFeedback.isranksupported(E) && rank(E) < n
				error('control:design:outputfeedback:input', 'Descriptor matrix must be regular for a PI observer output feedback controller.');
			end
			Anom = this.A_model_ref;
			Enom = this.E_model_ref;
			Bnom = this.B_model_ref;
			Cnom = this.C_model_ref;
			if xor(this.isdiscreteT(this.T_model_ref), this.isdiscreteT(T))
				error('control:design:outputfeedback:input', 'Discrete and continuous systems must not be mixed.');
			end
			C_ref = [
				C_ref,	zeros(size(C_ref, 1), q + size(Anom, 1))
			];
			D_ref = zeros(size(C_ref, 1), q);
			E = [
				E,								zeros(n, size(Enom, 2) + q);
				zeros(size(Enom, 1), n),		Enom,	zeros(size(Enom, 1), q);
				zeros(q, n + size(Enom, 2)),			eye(q)
			];
			A = [
				A,														zeros(n, size(Anom, 1) + q);
				zeros(size(Anom, 1), n),	Anom,						zeros(size(Anom, 1), q);
				zeros(q, n + size(Anom, 2)),							eye(q)*double(T > 0)
			];
			B = [
				B,						zeros(n, n + q);
				Bnom,	eye(n),			zeros(n, q);
				-D,		zeros(q, n),	eye(q)
			];
			C = [
				C,										zeros(q, q + size(Cnom, 2));
				zeros(size(Cnom, 1), n),		Cnom,	zeros(size(Cnom, 1), q);
				zeros(q, n + size(Cnom, 2)),			eye(q)
			];
			C_dot = zeros(0, n + size(Enom, 2) + q);
			D = zeros(size(C, 1), size(B, 2));
		end

		function [R_fixed, K_fixed, F_fixed, RKF_fixed, R_bounds, K_bounds, F_bounds, RKF_bounds, R_nonlin] = gainpattern_system(this, ~, ~, B, C, ~, ~, ~, ~, ~)
			%GAINPATTERN_SYSTEM return gain pattern constraint system for a PI observer feedback gain matrix with gain matrix R = [
			%		0,	R,	-K_I;
			%		-L,	L,	0;
			%		I,	0,	0
			%	];
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
			nnom = size(this.A_model_ref, 1);
			R = [
				zeros(p, q),		ones(p, 2*q);
				ones(nnom, 2*q),	zeros(nnom, q);
				ones(q, q),			zeros(q, 2*q)
			];
			[row, col] = find(R == 0);
			number_zeros = size(row, 1);
			number_observer = q*nnom;
			R_fixed_system = NaN([size(R), size(row, 1) + q*q + number_observer]);
			R_fixed_border = zeros(number_zeros + q*q + number_observer, 1);
			R_fixed_border(end - q*q - number_observer + 1:end - number_observer, 1) = 1;
			parfor ii = 1:number_zeros
				temp = zeros(size(R));
				temp(row(ii), col(ii)) = 1;
				R_fixed_system(:, :, ii) = temp;
			end
			R = [
				zeros(p + nnom, 3*q);
				ones(q, q),		zeros(q, 2*q)
			];
			[row, col] = find(R == 1);
			parfor ii = 1:size(row, 1)
				temp = zeros(size(R));
				temp(row(ii), col(ii)) = 1;
				R_fixed_system(:, :, ii + number_zeros) = temp;
			end
			R = [
				zeros(p, 3*q);
				ones(nnom, q), zeros(nnom, 2*q);
				zeros(q, 3*q)
			];
			[row, col] = find(R == 1);
			for ii = 1:size(row, 1)
				temp = zeros(size(R));
				temp(row(ii), col(ii)) = 1;
				temp(row(ii), col(ii) + q) = 1;
				R_fixed_system(:, :, ii + number_zeros + q*q) = temp;
			end
			R_fixed = {R_fixed_system, R_fixed_border};
			if nargout >= 2
				K_fixed = {true(p + nnom + q, 0), zeros(p + nnom + q, 0)};
				if nargout >= 3
					F_fixed = {[
							false(p, q);
							true(nnom + q, q)
						], [
							NaN(p, q);
							zeros(nnom, q);
							eye(q)
						]
					};
					if nargout >= 4
						R = cat(2, reshape(eye(p*q), p, q, p*q), zeros(p, 2*q, p*q));
						F = reshape(-eye(p*q), p, q, p*q);
						RKF_fixed = {
							cat(1, cat(2, R, zeros(p, 0, p*q), F), zeros(nnom + q, 4*q, p*q)), zeros(p*q, 1)
						};
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
				end
			end
		end

		function [R_gain, K_gain, F_prefilter] = gainpattern_parametric_system(this, ~, ~, B, C, ~, ~, ~, ~, ~)
			%GAINPATTERN_PARAMETRIC_SYSTEM return parametric gain matrix for a PI observer output feedback gain matrix R = [
			%		F,	R,	-K_I;
			%		-L,	L,	0;
			%		I,	0,	0
			%	], gain matrix K = [] and prefilter matrix F = [
			%		F;
			%		0;
			%		I
			%	] in continuous and discrete time
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
			nnom = size(this.A_model_ref, 1);
			R = realp('R', ones(p, q));
			L = realp('L', ones(nnom, q));
			K_I = realp('K_I', ones(p, q));
			R_gain = [
				zeros(p, q),	R,	K_I;
				-L,				L,	zeros(nnom, q);
				eye(q),			zeros(q, 2*q)
			];
			if nargout >= 2
				K_gain = zeros(p + nnom + q, 0);
				if nargout >= 3
					F = realp('F', ones(p, q));
					F_prefilter = [
						F;
						zeros(nnom, q);
						eye(q)
					];
				end
			end
		end

		function [T_x, T_u, T_y, T_y_dot, T_w] = scalegain_system(this, T_x, T_u, T_y, T_y_dot, T_w, ~, ~, ~, C, ~, ~, ~, ~, ~)
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
			nnom = size(this.A_model_ref, 1);
			if isempty(T_w)
				T_w = T_y;
			end
			T_x = blkdiag(T_x, eye(nnom + q));
			if nargout >= 2
				T_u = blkdiag(T_u, eye(nnom + q));
				if nargout >= 3
					T_y = blkdiag(T_y, eye(size(this.C_model_ref, 1) + q));
				end
			end
		end

		function [F, F_fixed] = prefilterpattern_system(this, R, ~, ~, ~, B, C, ~, ~, ~, ~, ~)
			%PREFILTERPATTERN_SYSTEM return prefilter and prefilter pattern constraint system for a PI observer output feedback with given gain matrices
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
			%q_dot = size(C_dot, 1);
			F = [
				R(1:p, 1:q);
				zeros(size(this.A_model_ref, 1), q);
				eye(q)
			];
			if nargout >= 2
				F_fixed = true(p + size(this.A_model_ref, 1) + q, q);
			end
		end

		function [partitionR, partitionF] = gainpartitioning_system(this, R, ~, F, ~, ~, B, C, ~, ~, ~, ~, ~)
			%GAINPARTITIONING_SYSTEM return partitioning for gain matrix of extended system for a PI observer output feedback with given gain matrix
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
			%q_dot = size(C_dot, 1);
			partitionR = struct(...
				'R',	R(1:p, q + 1:2*q),...
				'K_I',	-R(1:p, 2*q + 1:3*q),...
				'L',	-R(p + 1:p + size(this.A_model_ref, 1), 1:q)...
			);
			if nargout >= 2
				partitionF = struct(...
					'F',	F(1:p, :)...
				);
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
			nnom = this.n;
			[partitionR, partitionF] = this.gainpartitioning_system(R, K, F, E, A, B, C, C_dot, D, C_ref, D_ref, T);
			if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
				if control.design.outputfeedback.OutputFeedback.isranksupported(D) && rank(D) > 0
					error('control:design:outputfeedback:input', 'System must not have a throughput matrix.');
				end
				E = [
					eye(q),			zeros(q, nnom);
					zeros(nnom, q), this.E_model_ref
				];
				A = [
					eye(q),								zeros(q, nnom);
					this.B_model_ref*partitionR.K_I,	this.A_model_ref - partitionR.L*this.C_model_ref - this.B_model_ref*partitionR.R*this.C_model_ref
				];
				B = [
					-eye(q),			eye(q),							zeros(q, q);
					partitionR.L,		this.B_model_ref*partitionF.F,	zeros(nnom, q)
				];
				C = [
					partitionR.K_I,	-partitionR.R*this.C_model_ref
				];
				C_dot = zeros(0, q + nnom);
				D = [
					zeros(p, q),	partitionF.F,	zeros(p, q)
				];
				C_ref = zeros(0, size(A, 1));
				D_ref = zeros(0, size(B, 2));
			else
				E = [
					eye(q),			zeros(q, nnom);
					zeros(nnom, q), this.E_model_ref
				];
				A = [
					zeros(q, q),						zeros(q, nnom);
					this.B_model_ref*partitionR.K_I,	this.A_model_ref - partitionR.L*this.C_model_ref - this.B_model_ref*partitionR.R*this.C_model_ref
				];
				B = [
					-eye(q),			zeros(q, q_dot),	eye(q),							zeros(q, q_dot);
					partitionR.L,		zeros(nnom, q_dot),	this.B_model_ref*partitionF.F,	zeros(nnom, q_dot)
				];
				C = [
					partitionR.K_I,	-partitionR.R*this.C_model_ref
				];
				C_dot = zeros(0, q + nnom);
				D = [
					zeros(p, q + q_dot),	partitionF.F,	zeros(p, q_dot)
				];
				C_ref = zeros(0, size(A, 1));
				D_ref = zeros(0, size(B, 2));
			end
		end
	end

end
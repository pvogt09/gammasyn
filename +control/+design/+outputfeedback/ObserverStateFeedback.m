classdef ObserverStateFeedback < control.design.outputfeedback.AbstractReferenceModelFeedback
	%OBSERVERSTATEFEEDBACK class for casting a control system in output feedback form and specify the needed constraints on the resulting gain matrix for a observer state feedback controller
	%	For the control system
	%		Ex' = Ax + Bu
	%		y = Cx + Du
	%		y' = C_dot x
	%	the control law u = -R I x_O + Fw and the observer dynamics E_O x_O' = A_O x_O + B_O u + LCx - L C_O x_O results in the output feedback form
	%		[
	%			E,	0;
	%			0,	E_O
	%		]x' = [
	%			A,	0;
	%			0,	A_O
	%		]x - [
	%			B,		0;
	%			B_O,	I
	%		][
	%			0,	R;
	%			L,	0
	%		][
	%			-C,	C_O;
	%			0,	I
	%		]x + [B, 0; B_O, I] [
	%			F;
	%			0
	%		]w

	methods(Static=true)
		function [name] = SimulinkVariant()
			%SIMULINKVARIANT return name of corresponding simulink variant for controller block in control_outputfeedback_lib
			%Output:
			%		name:	name of the corresponding simulink variant
			name = 'ObserverStateFeedback';
		end
	end

	methods
		function [this] = ObserverStateFeedback(system, varargin)
			%OBSERVERSTATEFEEDBACK create new observer state feedback class
			%	Input:
			%		system:		state space system or structure with system matrices or descriptor matrix to use as observer
			%		A:			system matrix of observer or sample time if system is given as first argument
			%		B:			control matrix of observer
			%		C:			output matrix of observer
			%		C_dot:		derivative output matrix of observer
			%		D:			throughput matrix of observer
			%		C_ref:		measurement matrix for reference outputs
			%		D_ref:		throughput matrix for reference outputs
			%		T:			sampling time of observer
			%	Output:
			%		this:		instance
			narginchk(1, 9);
			this@control.design.outputfeedback.AbstractReferenceModelFeedback(system, varargin{:});
			if control.design.outputfeedback.OutputFeedback.isranksupported(this.E_model_ref) && rank(this.E_model_ref) < size(this.A_model_ref, 1)
				error('control:design:outputfeedback:input', 'Descriptor matrix of observer model must be regular.');
			end
		end
	end

	methods(Access=protected)
		function [E, A, B, C, C_dot, D, C_ref, D_ref] = amend_system(this, E, A, B, C, ~, D, C_ref, ~, T)
			%AMEND_SYSTEM add additional dynamics and partition matrices according to an observer state feedback
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
			deltaD = D ~= this.D_model_ref;
			if any(deltaD(:))
				error('control:design:outputfeedback:input', 'Throughput matrix of system and observer must be equal.');
			end
			if control.design.outputfeedback.OutputFeedback.isranksupported(E) && rank(E) < n
				error('control:design:outputfeedback:input', 'Descriptor matrix must be regular for an observer feedback controller.');
			end
			Enom = this.E_model_ref;
			Anom = this.A_model_ref;
			Bnom = this.B_model_ref;
			Cnom = this.C_model_ref;
			if xor(this.isdiscreteT(this.T_model_ref), this.isdiscreteT(T))
				error('control:design:outputfeedback:input', 'Discrete and continuous systems must not be mixed.');
			end
			if size(Cnom, 2) ~= n
				error('control:design:outputfeedback:input', 'Output matrix of observer must have %d columns, not %d.', n, size(Cnom, 2));
			end
			if size(Anom, 1) ~= n
				error('control:design:outputfeedback:input', 'System matrix of observer must be a %dX%d square matrix, not %dX%d.', n, n, size(Anom, 1), size(Anom, 2));
			end
			nnom = size(Anom, 1);
			C_ref = [
				C_ref, zeros(size(C_ref, 1), nnom)
			];
			D_ref = zeros(size(C_ref, 1), q);
			E = [
				E,				zeros(n, nnom);
				zeros(nnom, n),	Enom;
			];
			A = [
				A,				zeros(n, nnom);
				zeros(nnom, n),	Anom;
			];
			B = [
				B,		zeros(n, nnom);
				Bnom,	eye(nnom)
			];
			C = [
				-C,				Cnom;
				zeros(nnom, n),	eye(nnom)
			];
			C_dot = zeros(0, n + nnom);
			D = zeros(size(C, 1), size(B, 2));
		end

		function [R_fixed, K_fixed, F_fixed, RKF_fixed, R_bounds, K_bounds, F_bounds, RKF_bounds, R_nonlin] = gainpattern_system(this, ~, A, B, C, ~, ~, ~, ~, ~)
			%GAINPATTERN_SYSTEM return gain pattern constraint system for an observer state feedback gain matrix with gain matrix K = [
			%		0,	R;
			%		L,	0
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
			n = size(A, 1);
			p = size(B, 2);
			q = size(C, 1);
			nnom = size(this.A_model_ref, 1);
% 			K = [
% 				zeros(p, q),	ones(p, n);
% 				ones(nnom, q),	zeros(nnom, n)
% 			];
% 			[row, col] = find(K == 0);
% 			number_zeros = size(row, 1);
% 			R_fixed_system = NaN([size(K), size(row, 1)]);
% 			R_fixed_border = zeros(number_zeros, 1);
% 			parfor ii = 1:number_zeros
% 				temp = zeros(size(K));
% 				temp(row(ii), col(ii)) = 1;
% 				R_fixed_system(:, :, ii) = temp;
% 			end
			R = [
				zeros(p, q),	NaN(p, nnom);
				NaN(nnom, q),	zeros(nnom, nnom)
			];
			R_fixed = {~isnan(R), R};
			if nargout >= 2
				K_fixed = {true(p + nnom, 0), zeros(p + nnom, 0)};
				if nargout >= 3
					F_fixed = {[
						false(p, q);
						true(nnom, q)
					], [
						NaN(p, q);
						zeros(nnom, q)
					]};
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
			%GAINPATTERN_PARAMETRIC_SYSTEM return parametric gain matrix for a PID output feedback gain matrix R = [
			%		0,	R;
			%		L,	0
			%	], gain matrix K = [] and prefilter matrix F = [
			%		F;
			%		0
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
			R = realp('R', ones(p, nnom));
			L = realp('L', ones(nnom, q));
			R_gain = [
				zeros(p, q),	R;
				L,				zeros(nnom, nnom)
			];
			if nargout >= 2
				K_gain = zeros(p + nnom, 0);
				if nargout >= 3
					F = realp('F', ones(p, q));
					F_prefilter = [
						F;
						zeros(nnom, q)
					];
				end
			end
		end

		function [T_x, T_u, T_y, T_y_dot, T_w] = scalegain_system(this, T_x, T_u, T_y, T_y_dot, T_w, ~, ~, ~, ~, ~, ~, ~, ~, ~)
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
			%q = size(C, 1);
			%q_dot = size(C_dot, 1);
			nnom = size(this.A_model_ref, 1);
			if isempty(T_w)
				T_w = T_y;
			end
			T_x = blkdiag(T_x, eye(nnom));
			if nargout >= 2
				T_u = blkdiag(T_u, eye(nnom));
				if nargout >= 3
					T_y = blkdiag(T_y, eye(nnom));
				end
			end
		end

		function [F, F_fixed] = prefilterpattern_system(this, ~, ~, ~, ~, B, C, ~, ~, ~, ~, ~)
			%PREFILTERPATTERN_SYSTEM return prefilter and prefilter pattern constraint system for an observer state feedback with given gain matrices
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
			F = [
				eye(p, q);
				zeros(this.n, q)
			];
			if nargout >= 2
				F_fixed = [
					false(p, q);
					true(this.n, q)
				];
			end
		end

		function [partitionR, partitionF] = gainpartitioning_system(this, R, ~, F, ~, ~, B, C, ~, ~, ~, ~, ~)
			%GAINPARTITIONING_SYSTEM return partitioning for gain matrix of extended system for an observer state feedback with given gain matrix
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
			nnom = size(this.A_model_ref, 1);
			partitionR = struct(...
				'R',	R(1:p, q + (1:nnom)),...
				'L',	R(p + 1:p + nnom, q + 1:2*q)...
			);
			if nargout >= 2
				partitionF = struct(...
					'F',	F(1:p, :)...
				);
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
			p = size(B, 2);
			q = size(C, 1);
			q_dot = size(C_dot, 1);
			nnom = size(this.A_model_ref, 1);
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
				E = this.E_model_ref;
				A = this.A_model_ref - this.B_model_ref*partitionR.R - partitionR.L*this.C_model_ref;
				B = [
					partitionR.L,	this.B_model_ref*partitionF.F,	zeros(nnom, q)
				];
				C = -partitionR.R;
				C_dot = zeros(0, nnom);
				D = [
					zeros(p, q),	partitionF.F,	zeros(p, q)
				];
				C_ref = zeros(0, size(A, 1));
				D_ref = zeros(0, size(B, 2));
			else
				E = this.E_model_ref;
				A = this.A_model_ref - this.B_model_ref*partitionR.R - partitionR.L*this.C_model_ref;
				B = [
					partitionR.L, zeros(nnom, q_dot),	this.B_model_ref*partitionF.F,	zeros(nnom, q_dot)
				];
				C = -partitionR.R;
				C_dot = zeros(0, nnom);
				D = [
					zeros(p, q + q_dot),	partitionF.F,	zeros(p, q_dot)
				];
				C_ref = zeros(0, size(A, 1));
				D_ref = zeros(0, size(B, 2));
			end
		end
	end

end
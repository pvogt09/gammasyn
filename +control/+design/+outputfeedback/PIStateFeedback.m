classdef PIStateFeedback < control.design.outputfeedback.OutputFeedback
	%PISTATEFEEDBACK class for casting a control system in output feedback form and specify the needed constraints on the resulting gain matrix for a PI state controller
	%	For the control system
	%		Ex' = Ax + Bu
	%		y = Cx + Du
	%		y' = C_dot x
	%	the control law u = -Rx + K_I x_I + Fw and the additional dynamics x_I' = 0x_I - Cx - Du + w results in the output feedback form
	%		[
	%			E,	0;
	%			0,	I
	%		]x' = [
	%			A,	0;
	%			0,	0
	%		]x - [
	%			B,	-B,	0;
	%			-D,	D,	I
	%		][
	%			R,	0,	0;
	%			0,	0,	K_I;
	%			0,	I,	0
	%		][
	%			I,	0;
	%			C,	0;
	%			0,	I
	%		]x + [B, -B, 0; -D, D, I] [
	%			F;
	%			0;
	%			I
	%		]w

	methods(Static=true)
		function [name] = SimulinkVariant()
			%SIMULINKVARIANT return name of corresponding simulink variant for controller block in control_outputfeedback_lib
			%Output:
			%		name:	name of the corresponding simulink variant
			name = 'PIStateFeedback';
		end
	end

	methods
		function [this] = PIStateFeedback(varargin) %#ok<VANUS> varargin is not used but allowes to call the constructor with arguments
			%PISTATEFEEDBACK create new PI state feedback class
			%	Input:
			%		varargin:	unused input arguments
			%	Output:
			%		this:		instance
			this@control.design.outputfeedback.OutputFeedback();
		end
	end
	methods(Access=protected)
		function [E, A, B, C, C_dot, D, C_ref, D_ref] = amend_system(~, E, A, B, C, ~, D, C_ref, ~, T)
			%AMEND_SYSTEM add additional dynamics and partition matrices according to a PI state feedback
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
			if control.design.outputfeedback.OutputFeedback.isranksupported(E) && rank(E) < n
				error('control:design:outputfeedback:input', 'Descriptor matrix must be regular for a PI state feedback controller.');
			end
			C_ref = [
				C_ref, zeros(size(C_ref, 1), q)
			];
			D_ref = zeros(size(C_ref, 1), q);
			E = [
				E,				zeros(n, q);
				zeros(q, n),	eye(q)
			];
			A = [
				A,				zeros(n, q);
				zeros(q, n),	eye(q)*double(T > 0)
			];
			B = [
				B,	-B,	zeros(n, q);
				-D,	D,	eye(q)
			];
			C = [
				eye(n),			zeros(n, q);
				C,				zeros(q, q);
				zeros(q, n),	eye(q)
			];
			C_dot = zeros(0, n + q);
			D = zeros(size(C, 1), size(B, 2));
		end

		function [R_fixed, K_fixed, F_fixed, RKF_fixed, R_bounds, K_bounds, F_bounds, RKF_bounds, R_nonlin] = gainpattern_system(~, ~, A, B, C, ~, ~, ~, ~, ~)
			%GAINPATTERN_SYSTEM return gain pattern constraint system for a PI state feedback gain matrix with gain matrix K = [
			%		R,	0,	0;
			%		0,	0,	K_I;
			%		0,	I,	0
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
			%		R_fixed:	proportional gain constraint system as a cell array of a 3D constraint matrix and a constraint border vector
			%		K_fixed:	derivative gain constraint system as a cell array of a 3D constraint matrix and a constraint border vector
			%		F_fixed:	prefilter gain constraint system as a cell array of a 3D constraint matrix and a constraint border vector
			%		RKF_fixed:	combined gain constraint system as a cell array of a 3D constraint matrix and a constraint border vector
			%		R_bounds:	cell array with inequality constraint system for proportional gain matrix
			%		K_bounds:	cell array with inequality constraint system for derivative gain matrix
			%		F_bounds:	cell array with inequality constraint system for prefilter gain matrix
			%		RKF_bounds:	cell array with inequality constraint system for combined gain matrix
			%		R_nonlin:	function pointer to nonlinear constraints on proportional, derivative and prefilter gain matrix
			n = size(A, 1);
			p = size(B, 2);
			q = size(C, 1);
% 			R = [
% 				ones(p, n),		zeros(p, n + q);
% 				zeros(p, 2*n),	ones(p, q);
% 				zeros(q, n),	ones(q, n),	zeros(q, q)
% 			];
% 			[row, col] = find(R == 0);
% 			number_zeros = size(row, 1);
% 			R_fixed_system = NaN([size(R), size(row, 1) + q*n]);
% 			R_fixed_border = zeros(number_zeros + q*n, 1);
% 			R_fixed_border(end - q*n:end, 1) = 1;
% 			parfor ii = 1:number_zeros
% 				temp = zeros(size(K));
% 				temp(row(ii), col(ii)) = 1;
% 				R_fixed_system(:, :, ii) = temp;
% 			end
% 			R = [
% 				zeros(2*p, 2*n + q);
% 				zeros(q, n),	ones(q, n),	zeros(q, q)
% 			];
% 			[row, col] = find(R == 1);
% 			parfor ii = 1:size(row, 1)
% 				temp = zeros(size(R));
% 				temp(row(ii), col(ii)) = 1;
% 				R_fixed_system(:, :, ii + number_zeros) = temp;
% 			end
% 			R_fixed = {R_fixed_system, R_fixed_border};
			R = [
				NaN(p, n),					zeros(p, 2*q);
				zeros(p, n + q),			NaN(p, q);
				zeros(q, n),	ones(q, q),	zeros(q, q)
			];
			R_fixed = {~isnan(R), R};
			if nargout >= 2
				K_fixed = {true(2*p + q, 0), zeros(2*p + q, 0)};
				if nargout >= 3
					F_fixed = {[
						false(p, q);
						true(p + q, q);
					], [
						NaN(p, q);
						zeros(p, q);
						eye(q)
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

		function [R_gain, K_gain, F_prefilter] = gainpattern_parametric_system(~, ~, A, B, C, ~, ~, ~, ~, ~)
			%GAINPATTERN_PARAMETRIC_SYSTEM return parametric gain matrix for a state feedback gain matrix R = [
			%		R,	0,	0;
			%		0,	0,	K_I;
			%		0,	I,	0
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
			n = size(A, 1);
			p = size(B, 2);
			q = size(C, 1);
			%q_dot = size(C_dot, 1);
			F = realp('F', ones(p, q));
			R = realp('R', ones(p, n));
			K_I = realp('K_I', ones(p, q));
			R_gain = [
				R,				zeros(p, 2*q);
				zeros(p, n + q),		K_I;
				zeros(q, n),	eye(q),	zeros(q, q)
			];
			if nargout >= 2
				K_gain = zeros(2*p + q, 0);
				if nargout >= 3
					F_prefilter = [
						F;
						zeros(p, q);
						eye(q)
					];
				end
			end
		end

		function [T_x, T_u, T_y, T_y_dot, T_w] = scalegain_system(~, T_x, T_u, T_y, T_y_dot, T_w, ~, ~, ~, C, ~, ~, ~, ~, ~)
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
			if nargout >= 2
				T_u = blkdiag(T_u, T_u, T_y);
				if nargout >= 3
					T_y = blkdiag(T_x, T_y, eye(q));
				end
			end
			T_x = blkdiag(T_x, eye(q));
		end

		function [F, F_fixed] = prefilterpattern_system(~, ~, ~, ~, ~, B, C, ~, ~, ~, ~, ~)
			%PREFILTERPATTERN_SYSTEM return prefilter and prefilter pattern constraint system for a PI state feedback with given gain matrices
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
				eye(p, q);
				zeros(p, q);
				eye(q)
			];
			if nargout >= 2
				F_fixed = [
					false(p, q);
					true(p + q, q);
				];
			end
		end

		function [partitionR, partitionF] = gainpartitioning_system(~, R, ~, F, ~, A, B, C, ~, ~, ~, ~, ~)
			%GAINPARTITIONING_SYSTEM return partitioning for gain matrix of extended system for a PI state feedback with given gain matrix
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
			%q_dot = size(C_dot, 1);
			partitionR = struct(...
				'R',	R(1:p, 1:n),...
				'K_I',	R(1:p, n + q + 1:n + 2*q)...
			);
			if nargout >= 2
				partitionF = struct(...
					'F',	F(1:p, :)...
				);
			end
		end

		function [E, A, B, C, C_dot, D, C_ref, D_ref, needsstate, usesCasCdot] = realization_system(this, R, K, F, E, A, B, C, C_dot, C_ref, D_ref, D, T)
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
			%		C_ref:			measurement matrix for reference outputs
			%		D_ref:			throughput matrix for reference outputs
			%		needsstate:		indicator if state is needed as controller input instead of output
			%		usesCasCdot:	indicator if C is used as Cdot in case of derivative feedback
			needsstate = true;
			usesCasCdot = false;
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
				E = eye(q);
				A = eye(q);
				B = [
					-C,	eye(q),	zeros(q, q)
				];
				C = partitionR.K_I;
				C_dot = zeros(0, q);
				D = [
					-partitionR.R,	partitionF.F,	zeros(p, q)
				];
				C_ref = zeros(0, q);
				D_ref = zeros(0, size(B, 2));
			else
				E = eye(q);
				A = zeros(q, q);
				B = [
					-C,	zeros(q, q_dot),	eye(q),	zeros(q, q_dot)
				];
				C = partitionR.K_I;
				C_dot = zeros(0, q);
				D = [
					-partitionR.R,	zeros(p, q_dot),	partitionF.F,	zeros(p, q_dot)
				];
				C_ref = zeros(0, q);
				D_ref = zeros(0, size(B, 2));
			end
		end
	end

end
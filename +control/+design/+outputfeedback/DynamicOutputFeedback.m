classdef DynamicOutputFeedback < control.design.outputfeedback.AbstractDynamicFeedback
	%DYNAMICOUTPUTFEEDBACK class for casting a control system in output feedback form and specify the needed constraints on the resulting gain matrix, when a dynamic feedback of specified order is needed
	%	For the control system
	%		Ex' = Ax + Bu
	%		y = Cx + 0u
	%		y' = C_dot x
	%	the control law u = -D_D C x - C_D x_D + F_1 w and the additional dynamics x_D' = A_D x_D + B_D C x + F_2 w results in the output feedback form
	%		[
	%			E,	0;
	%			0,	I
	%		]x' = [
	%			A,	0;
	%			0,	0
	%		]x - [
	%			B,	0;
	%			0,	I
	%		][
	%			D_D,	C_D;
	%			-B_D,	-A_D
	%		][
	%			C,	0;
	%			0,	I
	%		]x + [B, 0; 0, I] [
	%			F_1;
	%			F_2
	%		]w

	methods(Static=true)
		function [name] = SimulinkVariant()
			%SIMULINKVARIANT return name of corresponding simulink variant for controller block in control_outputfeedback_lib
			%Output:
			%		name:	name of the corresponding simulink variant
			name = 'DynamicOutputFeedback';
		end
	end

	methods
		function [this] = DynamicOutputFeedback(order, varargin) %#ok<VANUS> varargin is not used but allowes to call the constructor with arguments
			%DYNAMICOUTPUTFEEDBACK create new dynamic feedback
			%	Input:
			%		order:		order of dynamic feedback
			%		varargin:	unused input arguments
			%	Output:
			%		this:		instance
			if ~isnumeric(order)
				if isstruct(order)
					if isfield(order, 'A')
						order = size(order.A, 1);
					elseif isfield(order, 'a')
						order = size(order.a, 1);
					end
				elseif isa(order, 'ss')
					order = size(order.a, 1);
				elseif isa(order, 'tf')
					order = size(order.den, 2);
				end
			end
			this@control.design.outputfeedback.AbstractDynamicFeedback(order);
		end
	end

	methods(Access=protected)
		function [E, A, B, C, C_dot, D, C_ref, D_ref] = amend_system(this, E, A, B, C, ~, ~, C_ref, ~, ~)
			%AMEND_SYSTEM add additional dynamics and partition matrices according to a dynamic output feedback
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
			if control.design.outputfeedback.OutputFeedback.isranksupported(E) && rank(E) < n
				error('control:design:outputfeedback:input', 'Descriptor matrix must be regular for a dynamic output feedback controller.');
			end
			nnom = this.n;
			C_ref = [
				C_ref,							zeros(size(C_ref, 1), nnom);
				zeros(nnom, size(C_ref, 2)),	eye(nnom)
			];
			D_ref = zeros(size(C_ref, 1), q + nnom);
			E = [
				E,						zeros(n, nnom);
				zeros(nnom, n),			eye(nnom)
			];
			A = [
				A,						zeros(n, nnom);
				zeros(nnom, n + nnom);
			];
			B = [
				B,				zeros(n, nnom);
				zeros(nnom,	p),	eye(nnom)
			];
			C = [
				C,					zeros(q, nnom);
				zeros(nnom, n),		eye(nnom)
			];
			C_dot = zeros(0, n + nnom);
			D = zeros(size(C, 1), size(B, 2));
		end

		function [R_fixed, K_fixed, F_fixed, RKF_fixed, R_bounds, K_bounds, F_bounds, RKF_bounds, R_nonlin] = gainpattern_system(this, ~, ~, B, C, ~, ~, ~, ~, T)
			%GAINPATTERN_SYSTEM return gain pattern constraint system for a dynamic output feedback gain matrix with gain matrix K = [
			%		D_D,	C_D;
			%		-B_D,	-A_D
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
			nnom = this.n;
			R_fixed = {false(p + nnom, q + nnom), NaN(p + nnom, q + nnom)};
			if nargout >= 2
				K_fixed = {true(p + nnom, 0), zeros(p + nnom, 0)};
				if nargout >= 3
					F_fixed = {false(size(B, 2) + nnom, size(C, 1) + nnom), NaN(size(B, 2) + nnom, size(C, 1) + nnom)};
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
											R_nonlin = constraintfun(control.design.outputfeedback.OutputFeedback.isdiscreteT(T), p + 1:p + nnom, q + 1:q + nnom);
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
			%		D_D,	C_D;
			%		-B_D,	-A_D
			%	], gain matrix K = [] and prefilter matrix F = [
			%		F_1;
			%		F_2
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
			nnom = this.n;
			F_1 = realp('F_1', ones(p, q + nnom));
			F_2 = realp('F_2', ones(nnom, q + nnom));
			D_D = realp('D_D', ones(p, q));
			C_D = realp('C_D', ones(p, nnom));
			B_D = realp('B_D', ones(nnom, q));
			A_D = realp('A_D', ones(nnom, nnom));
			R_gain = [
				D_D,	C_D;
				-B_D,	-A_D
			];
			if nargout >= 2
				K_gain = zeros(p + nnom, 0);
				if nargout >= 3
					F_prefilter = [
						F_1;
						F_2
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
			nnom = this.n;
			if isempty(T_w)
				T_w = T_y;
			end
			T_x = blkdiag(T_x, eye(nnom));
			if nargout >= 2
				T_u = blkdiag(T_u, eye(nnom));
				if nargout >= 3
					T_y = blkdiag(T_y, eye(nnom));
					if nargout >= 5
						T_w = blkdiag(T_w, eye(nnom));
					end
				end
			end
		end

		function [F, F_fixed] = prefilterpattern_system(this, ~, ~, ~, ~, B, C, ~, ~, ~, ~, ~)
			%PREFILTERPATTERN_SYSTEM return prefilter and prefilter pattern constraint system for a dynamic output feedback with given gain matrices
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
			nnom =  this.n;
			F = eye(size(B, 2) + nnom, size(C, 1) + nnom);
			if nargout >= 2
				F_fixed = false(size(B, 2) + nnom, size(C, 1) + nnom);
			end
		end

		function [partitionR, partitionF] = gainpartitioning_system(this, R, ~, F, ~, ~, B, C, ~, ~, ~, ~, ~)
			%GAINPARTITIONING_SYSTEM return partitioning for gain matrix of extended system for a dynamic output feedback with given gain matrix
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
			nnom =  this.n;
			partitionR = struct(...
				'A_D',	-R(p + 1:p + nnom, q + 1:q + nnom),...
				'B_D',	-R(p + 1:p + nnom, 1:q),...
				'C_D',	R(1:p, q + 1:q + nnom),...
				'D_D',	R(1:p, 1:q)...
			);
			if nargout >= 2
				partitionF = struct(...
					'F_1',	F(1:p, :),...
					'F_2',	F(p + 1:end, :)...
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
			if control.design.outputfeedback.OutputFeedback.isranksupported(D) && rank(D) > 0
				error('control:design:outputfeedback:input', 'System model must not have a throughput matrix.');
			end
			if control.design.outputfeedback.OutputFeedback.isranksupported(E) && rank(E) < size(A, 1)
				% TODO: allow real descriptor systems and check conditions for regular E - B K C_dot
				error('control:design:outputfeedback:input', 'Descriptor matrix must be regular for a PID state feedback controller.');
			end
			nnom =  this.n;
			[partitionR, partitionF] = this.gainpartitioning_system(R, K, F, E, A, B, C, C_dot, D, C_ref, D_ref, T);
			if control.design.outputfeedback.OutputFeedback.isdiscreteT(T)
				if control.design.outputfeedback.OutputFeedback.isranksupported(D) && rank(D) > 0
					error('control:design:outputfeedback:input', 'System must not have a throughput matrix.');
				end
				E = eye(nnom);
				A = partitionR.A_D;
				B = [
					partitionR.B_D,	partitionF.F_2,	zeros(nnom, q)
				];
				C = partitionR.C_D;
				C_dot = zeros(0, nnom);
				D = [
					-partitionR.D_D,	partitionF.F_1,	zeros(p, q)
				];
				C_ref = zeros(0, size(A, 1));
				D_ref = zeros(0, size(B, 2));
			else
				E = eye(nnom);
				A = partitionR.A_D;
				B = [
					partitionR.B_D,	zeros(nnom, q_dot),	partitionF.F_2,	zeros(nnom, q_dot)
				];
				C = partitionR.C_D;
				C_dot = zeros(0, nnom);
				D = [
					-partitionR.D_D,	zeros(p, q_dot),	partitionF.F_1,	zeros(p, q_dot)
				];
				C_ref = zeros(0, size(A, 1));
				D_ref = zeros(0, size(B, 2));
			end
		end
	end
end

function [fun] = constraintfun(isdiscrete, idxR1, idxR2, ~, ~)
	%CONSTRAINTFUN return constraint function for dynamical part of DynamicOutputFeedback
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
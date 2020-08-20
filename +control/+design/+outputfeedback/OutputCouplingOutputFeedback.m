classdef OutputCouplingOutputFeedback < control.design.outputfeedback.AbstractCouplingFeedback
	%OUTPUTCOUPLINGOUTPUTFEEDBACK class for casting a control system in output feedback form ready for gammasyn_couplingcontrol to perform a state coupling control design and specify the needed constraints on the resulting gain matrix
	%	For the control system
	%		Ex' = Ax + Bu
	%		y = Cx + Du
	%		y' = C_dot x
	%	the control law u = -Rx + Fw results in the output feedback form
	%		Ex' = Ax - B R Cx + B Fw
	%		y_ref = C_ref x + D_ref u

	properties(SetAccess=protected)
		% output transformation: y_tilde = T*y
		transformation
	end

	methods(Static=true)
		function [name] = SimulinkVariant()
			%SIMULINKVARIANT return name of corresponding simulink variant for controller block in control_outputfeedback_lib
			%	Output:
			%		name:	name of the corresponding simulink variant
			name = 'OutputCouplingOutputFeedback';
		end
	end

	methods
		function [this] = OutputCouplingOutputFeedback(number_couplingconditions, transformation, varargin) %#ok<VANUS> varargin is not used but allowes to call the constructor with arguments
			%OUTPUTCOUPLINGOUTPUTFEEDBACK create new output feedback coupling class
			%	Input:
			%		number_couplingconditions:	number of coupling conditions in C_ref
			%		transformation:				transformation matrix containing parameters of outputs that shall be coupled.
			%		varargin:					unused input arguments
			%	Output:
			%		this:						instance
			narginchk(2, Inf);
			this@control.design.outputfeedback.AbstractCouplingFeedback(number_couplingconditions);
			this.transformation = transformation;
		end

		function [this] = set.transformation(this, transformation)
			%TRANSFORMATION set output transformation matrix to choose outputs, that shall be coupled
			%	Input:
			%		this:			instance
			%		transformation:	transformation matrix containing parameters of outputs that shall be coupled.
			%	Output:
			%		this:			instance
			if ~isnumeric(transformation)
				error('control:design:outputfeedback:input', 'Transformation matrix must be numeric.');
			end
			if any(any(isinf(transformation))) || any(any(isnan(transformation)))
				error('control:design:outputfeedback:input', 'Transformation matrix must have finite elements.');
			end
			if size(transformation, 1) ~= size(transformation, 2)
				error('control:design:outputfeedback:input', 'Transformation matrix must be square matrix.');
			end
			if rank(transformation) ~= size(transformation, 2)
				error('control:design:outputfeedback:input', 'Transformation matrix must be regular.');
			end
			this.transformation = transformation;
		end
	end

	methods(Access=protected)
		function [E, A, B, C, C_dot, D, C_ref, D_ref] = amend_system(this, E, A, B, C, ~, D, ~, ~, ~)
			%AMEND_SYSTEM add additional dynamics and partition matrices according to a state feedback
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
			if isempty(this.transformation)
				error('control:design:outputfeedback:input', 'No transformation matrix specified.');
			end
			if control.design.outputfeedback.OutputFeedback.isranksupported(E) && rank(E) < size(A, 1)
				error('control:design:outputfeedback:input', 'Descriptor matrix must be regular for a state feedback controller.');
			end
			q = size(C, 1);
			if size(this.transformation, 2) ~= q
				error('control:design:outputfeedback:input', 'Transformation matrix must have %d columns.', q);
			end
			nc = this.number_couplingconditions;
			if q <= nc
				error('control:design:outputfeedback:input', 'Number of coupling conditions (%d) must be smaller than number of measurements (%d).', nc, q);
			end
			C_ref = this.transformation*C;
			D_ref = this.transformation*D;
			C_dot = zeros(0, size(A, 1));
			D = zeros(q, size(B, 2));
		end

		function [R_fixed, K_fixed, F_fixed, RKF_fixed, R_bounds, K_bounds, F_bounds, RKF_bounds, R_nonlin] = gainpattern_system(~, ~, ~, B, C, ~, ~, ~, ~, ~)
			%GAINPATTERN_SYSTEM return gain pattern constraint system for a state feedback gain matrix
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
			R_fixed = {false(size(B, 2), size(C, 1)), NaN(size(B, 2), size(C, 1))};
			if nargout >= 2
				K_fixed = {true(size(B, 2), 0), zeros(size(B, 2), 0)};
				if nargout >= 3
					F_fixed = {false(size(B, 2), size(C, 1)), NaN(size(B, 2), size(C, 1))};
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

		function [R_gain, K_gain, F_prefilter] = gainpattern_parametric_system(~, ~, ~, B, C, ~, ~, ~, ~, ~)
			%GAINPATTERN_PARAMETRIC_SYSTEM return parametric gain matrix for a state feedback gain matrix R = R, gain matrix K = [] and prefilter matrix F = F in continuous and discrete time
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
			F = realp('F', ones(p, q));
			R = realp('R', ones(p, q));
			R_gain = R;
			if nargout >= 2
				K_gain = zeros(p, 0);
				if nargout >= 3
					F_prefilter = F;
				end
			end
		end

		function [T_x, T_u, T_y, T_y_dot, T_w] = scalegain_system(~, T_x, T_u, T_y, T_y_dot, T_w, ~, ~, ~, ~, ~, ~, ~, ~, ~)
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
			if isempty(T_w)
				T_w = T_y;
			end
		end

		function [F, F_fixed] = prefilterpattern_system(~, ~, ~, ~, ~, B, C, ~, ~, ~, ~, ~)
			%PREFILTERPATTERN_SYSTEM return prefilter and prefilter pattern constraint system for a state feedback with given gain matrices
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
			F = eye(p, q);
			if nargout >= 2
				F_fixed = false(p, q);
			end
		end

		function [partitionR, partitionF] = gainpartitioning_system(~, R, ~, F, ~, ~, ~, ~, ~, ~, ~, ~, ~)
			%GAINPARTITIONING_SYSTEM return partitioning for gain matrix of extended system for a state feedback with given gain matrix
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
			%p = size(B, 2);
			%q = size(C, 1);
			%q_dot = size(C_dot, 1);
			partitionR = struct(...
				'R',	R...
			);
			if nargout >= 2
				partitionF = struct(...
					'F',	F...
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
				E = zeros(0, 0);
				A = zeros(0, 0);
				B = zeros(0, 3*q);
				C = zeros(p, 0);
				C_dot = zeros(0, 0);
				D = [
					-partitionR.R,	partitionF.F,	zeros(p, q)
				];
				C_ref = zeros(0, 0);
				D_ref = zeros(0, 3*q);
			else
				E = zeros(0, 0);
				A = zeros(0, 0);
				B = zeros(0, 2*q + 2*q_dot);
				C = zeros(p, 0);
				C_dot = zeros(0, 0);
				D = [
					-partitionR.R,	zeros(p, q_dot),	partitionF.F,	zeros(p, q_dot)
				];
				C_ref = zeros(0, 0);
				D_ref = zeros(0, 2*q + 2*q_dot);
			end
		end

		function [couplingoptions] = get_couplingoptions_system(this, couplingoptions)
			%GET_COUPLINGOPTIONS_SYSTEM return structure with options for coupling controller design
			%	Input:
			%		this:				instance
			%		options:			structure with options to append to
			%	Output:
			%		couplingoptions:	structure with appended coupling options
			couplingoptions.couplingconditions = this.number_couplingconditions;% number of coupling conditions
			couplingoptions.couplingstrategy = GammaCouplingStrategy.EXACT;% method chosen to solve coupling design
		end
	end

end
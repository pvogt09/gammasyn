classdef(Abstract) AbstractDynamicModelFeedback < control.design.outputfeedback.AbstractDynamicFeedback
	%ABSTRACTDYNAMICMODELFEEDBACK abstract class for casting a control system in output feedback form and specify the needed constraints on the resulting gain matrix, when a not fully specified dynamic model of specified order is needed
	
	properties(SetAccess=protected)
		% reference descriptor matrix
		E_model_ref;
		% reference system matrix
		A_model_ref;
		% reference control matrix
		B_model_ref;
		% reference output matrix
		C_model_ref;
		% reference derivative output matrix
		C_dot_model_ref;
		% reference throughput matrix
		D_model_ref;
		% reference output matrix of references
		C_ref_model_ref;
		% reference throughput matrix of references
		D_ref_model_ref;
		% reference model sampling time
		T_model_ref
	end
	
	methods
		function [this] = AbstractDynamicModelFeedback(system, A, B, C, C_dot, D, C_ref, D_ref, T)
			%ABSTRACTDYNAMICMODELFEEDBACK create new feedback with not fully specified dynamic model of specified order
			%	Input:
			%		this:		instance
			%		system:		state space system or structure with system matrices or descriptor matrix or system order
			%		A:			system matrix or sample time if system or system order is given as first argument
			%		B:			control matrix
			%		C:			output matrix
			%		C_dot:		derivative output matrix
			%		D:			throughput matrix
			%		C_ref:		measurement matrix for reference outputs
			%		D_ref:		throughput matrix for reference outputs
			%		T:			sampling time
			%	Output:
			%		this:		instance
			narginchk(1, 9);
			if isstruct(system)
				if nargin >= 2
					T = A;
				else
					T = -1;
				end
				if isfield(system, 'A')
					A = system.A;
				elseif isfield(system, 'a')
					A = system.a;
				else
					error('control:design:outputfeedback:input', 'System must must have field A.');
				end
				if isfield(system, 'E')
					E = system.E;
				elseif isfield(system, 'e')
					E = system.e;
				else
					E = eye(size(A, 1));
				end
				if all(size(E) == 0)
					E = eye(size(A, 1));
				end
				if isfield(system, 'B')
					B = system.B;
				elseif isfield(system, 'b')
					B = system.b;
				else
					error('control:design:outputfeedback:input', 'System must must have field B.');
				end
				if isfield(system, 'C')
					C = system.C;
				elseif isfield(system, 'c')
					C = system.c;
				else
					error('control:design:outputfeedback:input', 'System must must have field C.');
				end
				if isfield(system, 'C_dot')
					C_dot = system.C_dot;
				elseif isfield(system, 'c_dot')
					C_dot = system.c_dot;
				else
					C_dot = zeros(0, size(A, 1));
				end
				if isfield(system, 'D')
					D = system.D;
				elseif isfield(system, 'd')
					D = system.d;
				else
					error('control:design:outputfeedback:input', 'System must must have field D.');
				end
				if isfield(system, 'C_ref')
					C_ref = system.C_ref;
				elseif isfield(system, 'c_ref')
					C_ref = system.c_ref;
				else
					C_ref = zeros(0, size(A, 1));
				end
				if isfield(system, 'D_ref')
					D_ref = system.D_ref;
				elseif isfield(system, 'd_ref')
					D_ref = system.d_ref;
				else
					D_ref = zeros(size(C_ref, 1), size(D, 2));
				end
				if isfield(system, 'T')
					T = system.T;
				elseif isfield(system, 't')
					T = system.t;
				elseif isfield(system, 'T_s')
					T = system.T_s;
				end
			elseif isnumeric(system)
				if nargin <= 2
					if isscalar(system)
						if nargin <= 1
							T = -1;
						end
						n = system;
						if floor(n) ~= ceil(n) || n <= 0
							error('control:design:outputfeedback:input', 'System order must be positive integer.');
						end
						A = NaN(n, n);
						% TODO: allow variable E
						E = eye(n);
						B = NaN(n, n);
						C = NaN(n, n);
						C_dot = NaN(n, n);
						D = NaN(n, n);
						C_ref = zeros(0, n);
						D_ref = zeros(0, n);
					else
						error('control:design:outputfeedback:input', 'System order must be scalar.');
					end
				else
					if nargin <= 8
						T = -1;
					end
					E = system;
				end
			elseif isa(system, 'tf') || isa(system, 'ss')
				if nargin >= 2
					T = A;
				else
					T = -1;
				end
				[A, B, C, D] = ssdata(system);
				C_dot = C;
				C_ref = C;
				D_ref = D;
				if isa(system, 'tf') || isempty(system.e)
					E = eye(size(A, 1));
				else
					E = system.e;
				end
				if system.Ts > 0
					T = system.Ts;
				end
			else
				error('control:design:outputfeedback:input', 'System must be some kind of control system, not a %s.', class(system));
			end
			this@control.design.outputfeedback.AbstractDynamicFeedback(size(A, 1));
			[valid, errid, errmsg] = this.check(E, A, B, C, C_dot, D, C_ref, D_ref);
			if ~valid
				error(errid, errmsg);
			end
			this.E_model_ref = E;
			this.A_model_ref = A;
			this.B_model_ref = B;
			this.C_model_ref = C;
			this.C_dot_model_ref = C_dot;
			this.D_model_ref = D;
			this.C_ref_model_ref = C_ref;
			this.D_ref_model_ref = D_ref;
			this.T_model_ref = T;
		end
	end
end
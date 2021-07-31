classdef OutputFeedback < handle
	%OUTPUTFEEDBACK abstract class for casting a control system in output feedback form and specify the needed constraints on the resulting gain matrix

	methods(Static=true, Sealed=true)
		function [isdiscrete] = isdiscreteT(T)
			%ISDISCRETET return if sample time T corresponds to a discrete system
			%	Input:
			%		T:				sample time to check
			%	Output:
			%		isdiscrete:		true, if sample time corresponds to a discrete time system, else false
			if isnumeric(T)
				isdiscrete = T > 0;
			elseif isa(T, 'sym') || isa(T, 'sdpvar')
				isdiscrete = true;
			else
				isdiscrete = T > 0;
			end
		end
	end

	methods(Static=true, Abstract=true)
		%SIMULINKVARIANT return name of corresponding simulink variant for controller block in control_outputfeedback_lib
		%Output:
		%		name:	name of the corresponding simulink variant
		[name] = SimulinkVariant();
	end

	methods(Access=protected, Static=true)
		function [valid, errid, errmsg] = check(E, A, B, C, C_dot, D, C_ref, D_ref, R_fixed)
			%CHECK check a system for proper dimensions
			%	Input:
			%		E:			descriptor matrix
			%		A:			system matrix
			%		B:			control matrix
			%		C:			output matrix
			%		C_dot:		derivative output matrix
			%		D:			throughput matrix
			%		C_ref:		measurement matrix for reference outputs
			%		D_ref:		throughput matrix for reference outputs
			%		R_fixed:	constraint matrix for fixed gain matrices
			%	Output:
			%		valid:		indicator, if all arguments are valid
			%		errid:		error identifier if any argument is invalid
			%		errmsg:		error message if any argument is invalid
			n = size(A, 1);
			p = size(B, 2);
			q = size(C, 1);
			valid = true;
			errmsg = '';
			if size(A, 2) ~= n
				valid = false;
				errmsg = 'System matrix must be square.';
			end
			if size(E, 1) ~= n
				valid = false;
				errmsg = sprintf('Descriptor matrix must have %d columns, not %d.', n, size(E, 1));
			end
			if size(E, 2) ~= n
				valid = false;
				errmsg = sprintf('Descriptor matrix must have %d rows, not %d.', n, size(E, 2));
			end
			if size(B, 1) ~= n
				valid = false;
				errmsg = sprintf('Control matrix must have %d columns, not %d.', n, size(B, 1));
			end
			if size(C, 2) ~= n
				valid = false;
				errmsg = sprintf('Output matrix must have %d rows, not %d.', n, size(C, 2));
			end
			if size(C_dot, 2) ~= n
				valid = false;
				errmsg = sprintf('Derivative output matrix must have %d rows, not %d.', n, size(C_dot, 2));
			end
			if size(D, 2) ~= p
				valid = false;
				errmsg = sprintf('Throughput matrix must have %d columns, not %d.', p, size(D, 2));
			end
			if size(D, 1) ~= q
				valid = false;
				errmsg = sprintf('Throughput matrix must have %d rows, not %d.', q, size(D, 1));
			end
			if size(C_ref, 2) ~= n
				valid = false;
				errmsg = sprintf('Output matrix for reference outputs must have %d columns, not %d.', n, size(C_ref, 2));
			end
			% size(D_ref, 2) does not matter
			%if size(D_ref, 2) ~= p
			%	valid = false;
			%	errmsg = sprintf('Throughput matrix for reference outputs must have %d columns, not %d.', p, size(D_ref, 2));
			%end
			if size(C_ref, 1) ~= size(D_ref, 1)
				valid = false;
				errmsg = sprintf('Throughput matrix for reference outputs must have %d columns, not %d.', size(C_ref, 1), size(D_ref, 1));
			end
			if ~valid
				errid = 'control:design:outputfeedback:input';
			else
				errid = '';
			end
			if nargin >= 9
				if iscell(R_fixed)
					if numel(R_fixed) >= 2
						log = [
							islogical(R_fixed{1});
							islogical(R_fixed{2})
						];
						num = [
							isnumeric(R_fixed{1});
							isnumeric(R_fixed{2})
						];
						if any(log) && any(num)
							if size(R_fixed{1}, 1) ~= p
								valid = false;
								errmsg = sprintf('Fixed gain positions must be a %dX%d matrix.', p, q);
							end
							if size(R_fixed{1}, 2) ~= q
								valid = false;
								errmsg = sprintf('Fixed gain positions must be a %dX%d matrix.', p, q);
							end
							if size(R_fixed{2}, 1) ~= p
								valid = false;
								errmsg = sprintf('Fixed gain values must be a %dX%d matrix.', p, q);
							end
							if size(R_fixed{2}, 2) ~= q
								valid = false;
								errmsg = sprintf('Fixed gain values must be a %dX%d matrix.', p, q);
							end
						elseif all(num)
							if size(R_fixed{1}, 3) > 1
								if size(R_fixed{1}, 1) ~= p
									valid = false;
									errmsg = sprintf('Fixed gain positions must be a %dX%d matrix.', p, q);
								end
								if size(R_fixed{1}, 2) ~= q
									valid = false;
									errmsg = sprintf('Fixed gain positions must be a %dX%d matrix.', p, q);
								end
								if size(R_fixed{1}, 3) ~= size(R_fixed{2}, 1)
									valid = false;
									errmsg = sprintf('Fixed gain constraint system must be a %d vector of bounds.', size(R_fixed{1}, 3));
								end
								if size(R_fixed{2}, 2) ~= 1
									valid = false;
									errmsg = sprintf('Fixed gain constraint system must be a %d vector of bounds.', size(R_fixed{1}, 3));
								end
							elseif size(R_fixed{2}, 3) > 1
								if size(R_fixed{2}, 1) ~= p
									valid = false;
									errmsg = sprintf('Fixed gain positions must be a %dX%d matrix.', p, q);
								end
								if size(R_fixed{2}, 2) ~= q
									valid = false;
									errmsg = sprintf('Fixed gain positions must be a %dX%d matrix.', p, q);
								end
								if size(R_fixed{2}, 3) ~= size(R_fixed{1}, 1)
									valid = false;
									errmsg = sprintf('Fixed gain constraint system must be a %d vector of bounds.', size(R_fixed{2}, 3));
								end
								if size(R_fixed{1}, 2) ~= 1
									valid = false;
									errmsg = sprintf('Fixed gain constraint system must be a %d vector of bounds.', size(R_fixed{2}, 3));
								end
							else
								if size(R_fixed{1}, 1) ~= p
									valid = false;
									errmsg = sprintf('Fixed gain positions must be a %dX%d matrix.', p, q);
								end
								if size(R_fixed{1}, 2) ~= q
									valid = false;
									errmsg = sprintf('Fixed gain positions must be a %dX%d matrix.', p, q);
								end
								if size(R_fixed{2}, 1) ~= p
									valid = false;
									errmsg = sprintf('Fixed gain values must be a %dX%d matrix.', p, q);
								end
								if size(R_fixed{2}, 2) ~= q
									valid = false;
									errmsg = sprintf('Fixed gain values must be a %dX%d matrix.', p, q);
								end
							end
						else
							valid = false;
							errmsg = sprintf('Fixed gain must contain a logical and a numeric matrix or a numerical constraint system, not a ''%s''.', class(R_fixed{1}));
						end
					else
						valid = false;
						errmsg = 'Fixed gain must not contain more than 2 matrices.';
					end
				elseif ~isnumeric(R_fixed) && ~islogical(R_fixed)
					valid = false;
					errmsg = sprintf('Fixed gain positions must be two %dX%d matrices with positions and fixed values.', p, q);
				end
			end
		end

		function [is] = isranksupported(var)
			%ISRANKSUPPORTED return if th supplied variable supports the rank function
			%	Input:
			%		var:	variable to return rank functionality for
			%	Output:
			%		is:		true, if the variable can be the input argument of the rank function else false
			is = isnumeric(var);
			if ~is
				is = ~isobject(var);
				if ~is
					is = ~isa(var, 'rolmipvar');
				end
			end
		end
	end

	methods(Access=protected)
		function [this] = OutputFeedback()
			%OUTPUTFEEDBACK create new OtuputFeedback object
			%	Output:
			%		this:	instance
		end
	end

	methods(Sealed=true)
		function [R_fixed, K_fixed, F_fixed, RKF_fixed, R_bounds, K_bounds, F_bounds, RKF_bounds, R_nonlin] = gainpattern(this, system, A, B, C, C_dot, D, C_ref, D_ref, T)
			%GAINPATTERN return constraint system for resulting gain matrix
			%	Input:
			%		this:		instance
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
			%		R_fixed:	cell array with constraint system for proportional gain matrix
			%		K_fixed:	cell array with constraint system for derivative gain matrix
			%		F_fixed:	cell array with constraint system for prefilter gain matrix
			%		RKF_fixed:	cell array with inequality constraint system for combined gain matrix
			%		R_bounds:	cell array with inequality constraint system for proportional gain matrix
			%		K_bounds:	cell array with inequality constraint system for derivative gain matrix
			%		F_bounds:	cell array with inequality constraint system for prefilter gain matrix
			%		RKF_bounds:	cell array with inequality constraint system for combined gain matrix
			%		R_nonlin:	function pointer to nonlinear constraints on proportional, derivative and prefilter gain matrix
			if isstruct(system)
				if nargin >= 3
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
					D = zeros(size(C, 1), size(B, 2));
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
					D_ref = zeros(0, size(D, 2));
				end
				if isfield(system, 'T')
					T = system.T;
				elseif isfield(system, 't')
					T = system.t;
				elseif isfield(system, 'T_s')
					T = system.T_s;
				end
			elseif isnumeric(system)
				if nargin <= 9
					T = -1;
				end
				E = system;
			elseif isa(system, 'tf') || isa(system, 'ss')
				if nargin >= 3
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
			[valid, errid, errmsg] = this.check(E, A, B, C, C_dot, D, C_ref, D_ref);
			if ~valid
				error(errid, errmsg);
			end
			if nargout >= 9
				[R_fixed, K_fixed, F_fixed, RKF_fixed, R_bounds, K_bounds, F_bounds, RKF_bounds, R_nonlin] = this.gainpattern_system(E, A, B, C, C_dot, D, C_ref, D_ref, T);
			elseif nargout >= 8
				[R_fixed, K_fixed, F_fixed, RKF_fixed, R_bounds, K_bounds, F_bounds, RKF_bounds] = this.gainpattern_system(E, A, B, C, C_dot, D, C_ref, D_ref, T);
			elseif nargout >= 7
				[R_fixed, K_fixed, F_fixed, RKF_fixed, R_bounds, K_bounds, F_bounds] = this.gainpattern_system(E, A, B, C, C_dot, D, C_ref, D_ref, T);
			elseif nargout >= 6
				[R_fixed, K_fixed, F_fixed, RKF_fixed, R_bounds, K_bounds] = this.gainpattern_system(E, A, B, C, C_dot, D, C_ref, D_ref, T);
			elseif nargout >= 5
				[R_fixed, K_fixed, F_fixed, RKF_fixed, R_bounds] = this.gainpattern_system(E, A, B, C, C_dot, D, C_ref, D_ref, T);
			elseif nargout >= 4
				[R_fixed, K_fixed, F_fixed, RKF_fixed] = this.gainpattern_system(E, A, B, C, C_dot, D, C_ref, D_ref, T);
			elseif nargout >= 3
				[R_fixed, K_fixed, F_fixed] = this.gainpattern_system(E, A, B, C, C_dot, D, C_ref, D_ref, T);
			elseif nargout >= 2
				[R_fixed, K_fixed] = this.gainpattern_system(E, A, B, C, C_dot, D, C_ref, D_ref, T);
			else
				R_fixed = this.gainpattern_system(E, A, B, C, C_dot, D, C_ref, D_ref, T);
			end
		end

		function [R_gain, K_gain, F_prefilter] = gainpattern_parametric(this, system, A, B, C, C_dot, D, C_ref, D_ref, T)
			%GAINPATTERN_PARAMETRIC return parametric gain matrix
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
			if isstruct(system)
				if nargin >= 3
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
					D = zeros(size(C, 1), size(B, 2));
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
					D_ref = zeros(0, size(D, 2));
				end
				if isfield(system, 'T')
					T = system.T;
				elseif isfield(system, 't')
					T = system.t;
				elseif isfield(system, 'T_s')
					T = system.T_s;
				end
			elseif isnumeric(system)
				if nargin <= 9
					T = -1;
				end
				E = system;
			elseif isa(system, 'tf') || isa(system, 'ss')
				if nargin >= 3
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
			[valid, errid, errmsg] = this.check(E, A, B, C, C_dot, D, C_ref, D_ref);
			if ~valid
				error(errid, errmsg);
			end
			if nargout >= 3
				[R_gain, K_gain, F_prefilter] = this.gainpattern_parametric_system(E, A, B, C, C_dot, D, C_ref, D_ref, T);
			elseif nargout >= 2
				[R_gain, K_gain] = this.gainpattern_parametric_system(E, A, B, C, C_dot, D, C_ref, D_ref, T);
			else
				R_gain = this.gainpattern_parametric_system(E, A, B, C, C_dot, D, C_ref, D_ref, T);
			end
		end

		function [system, A, B, C, C_dot, D, C_ref, D_ref] = amend(this, system, A, B, C, C_dot, D, C_ref, D_ref, T, noreference)
			%AMEND return extended system for output feedback
			%	Input:
			%		this:			instance
			%		system:			state space system or structure with system matrices or descriptor matrix
			%		A:				system matrix or sample time if system is given as first argument
			%		B:				control matrix
			%		C:				output matrix
			%		C_dot:			derivative output matrix
			%		D:				throughput matrix
			%		C_ref:			reference output matrix
			%		D_ref:			reference throughput matrix
			%		T:				sampling time
			%		noreference:	indicator if reference outputs should be ignored
			%	Output:
			%		system:			state space system or structure with system matrices or descriptor matrix depending on what input arguments are supplied
			%		A:				system matrix
			%		B:				control matrix
			%		C:				output matrix
			%		C_dot:			derivative output matrix
			%		D:				throughput matrix
			%		C_ref:			reference output matrix
			%		D_ref:			reference throughput matrix
			asstruct = false;
			astf = false;
			asss = true;
			asuss = false;
			asstructwithT = false;
			if isstruct(system)
				if nargin >= 3
					if islogical(A)
						noreference = A;
						if nargin >= 4
							T = B;
						else
							T = -1;
						end
					else
						T = A;
						if nargin >= 4
							noreference = B;
						else
							noreference = false;
						end
					end
				else
					T = -1;
					noreference = false;
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
					D = zeros(size(C, 1), size(B, 2));
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
					D_ref = zeros(0, size(D, 2));
				end
				if isfield(system, 'T')
					T = system.T;
					asstructwithT = true;
				elseif isfield(system, 't')
					T = system.t;
					asstructwithT = true;
				elseif isfield(system, 'T_s')
					T = system.T_s;
					asstructwithT = true;
				end
				asstruct = true;
			elseif isnumeric(system)
				if nargin >= 10
					if islogical(T)
						if nargin >= 11
							temp = noreference;
							noreference = T;
							T = temp;
						else
							noreference = T;
							T = -1;
						end
					else
						if nargin <= 10
							noreference = false;
						end
					end
				else
					T = -1;
					noreference = false;
				end
				E = system;
			elseif isa(system, 'tf') || isa(system, 'ss') || isa(system, 'uss')
				if nargin >= 3
					if islogical(A)
						noreference = A;
						if nargin >= 4
							T = B;
						else
							T = -1;
						end
					else
						T = A;
						if nargin >= 4
							noreference = B;
						else
							noreference = false;
						end
					end
				else
					T = -1;
					noreference = false;
				end
				if isa(system, 'uss')
					A = system.A;
					B = system.B;
					C = system.C;
					D = system.D;
					asuss = true;
				else
					[A, B, C, D] = ssdata(system);
					asuss = false;
				end
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
				asss = isa(system, 'ss');
				astf = ~asss && ~asuss;
			else
				error('control:design:outputfeedback:input', 'System must be some kind of control system, not a %s.', class(system));
			end
			if ~islogical(noreference) || ~isscalar(noreference)
				error('control:design:outputfeedback:input', 'Reference indicator must be a logical scalar.');
			end
			[valid, errid, errmsg] = this.check(E, A, B, C, C_dot, D, C_ref, D_ref);
			if ~valid
				error(errid, errmsg);
			end
			[E_g, A_g, B_g, C_g, C_dot_g, D_g, C_ref, D_ref] = this.amend_system(E, A, B, C, C_dot, D, C_ref, D_ref, T);
			if isempty(C_ref) && isempty(D_ref)
				if noreference
					C_ref = zeros(0, size(A_g, 2));
					D_ref = zeros(0, size(B_g, 2));
				else
					C_ref = [
						C,	zeros(size(C, 1), size(A_g, 2) - size(C, 2))
					];
					D_ref = [
						D,	zeros(size(D, 1), size(B_g, 2) - size(D, 2))
					];
				end
			end
			if asstruct
				system = struct(...
					'E',		E_g,...
					'A',		A_g,...
					'B',		B_g,...
					'C',		C_g,...
					'C_dot',	C_dot_g,...
					'D',		D_g,...
					'C_ref',	C_ref,...
					'D_ref',	D_ref...
				);
				if asstructwithT && T > 0
					system.T = T;
				end
			elseif astf
				if this.isdiscreteT(T)
					if iseye(E_g)
						system = tf(ss(A_g, B_g, C_g, D_g, T));
					else
						system = tf(dss(A_g, B_g, C_g, D_g, E_g, T));
					end
				else
					if iseye(E_g)
						system = tf(ss(A_g, B_g, C_g, D_g));
					else
						system = tf(dss(A_g, B_g, C_g, D_g, E_g));
					end
				end
			elseif asuss
				if this.isdiscreteT(T)
					if iseye(E_g)
						system = uss(A_g, B_g, C_g, D_g, T);
					else
						error('control:design:outputfeedback:input', 'Descriptor systems can not be represented as uss.');
					end
				else
					if iseye(E_g)
						system = uss(A_g, B_g, C_g, D_g);
					else
						error('control:design:outputfeedback:input', 'Descriptor systems can not be represented as uss.');
					end
				end
			elseif asss
				if this.isdiscreteT(T)
					if iseye(E_g)
						system = ss(A_g, B_g, C_g, D_g, T);
					else
						system = dss(A_g, B_g, C_g, D_g, E_g, T);
					end
				else
					if iseye(E_g)
						system = ss(A_g, B_g, C_g, D_g);
					else
						system = dss(A_g, B_g, C_g, D_g, E_g);
					end
				end
			else
				if isempty(C_ref) && isempty(D_ref)
					if noreference
						C_ref = zeros(0, size(A_g, 2));
						D_ref = zeros(0, size(B_g, 2));
					else
						C_ref = [
							C,	zeros(size(C, 1), size(A_g, 2) - size(C, 2))
						];
						D_ref = [
							D,	zeros(size(D, 1), size(B_g, 2) - size(D, 2))
						];
					end
				end
				system = E_g;
				A = A_g;
				B = B_g;
				C = C_g;
				C_dot = C_dot_g;
				D = D_g;
			end
		end

		function [system, A, B, C, C_dot, D, C_ref, D_ref] = amend_parametric(this, system, A, B, C, C_dot, D, C_ref, D_ref, T, noreference)
			%AMEND_PARAMETRIC return extended system for output feedback with parameteric gain matrices
			%	Input:
			%		this:			instance
			%		system:			state space system or structure with system matrices or descriptor matrix
			%		A:				system matrix or sample time if system is given as first argument
			%		B:				control matrix
			%		C:				output matrix
			%		C_dot:			derivative output matrix
			%		D:				throughput matrix
			%		C_ref:			reference output matrix
			%		D_ref:			reference throughput matrix
			%		T:				sampling time
			%		noreference:	indicator if reference outputs should be ignored
			%	Output:
			%		system:			state space system or structure with system matrices or descriptor matrix depending on what input arguments are supplied
			%		A:				system matrix
			%		B:				control matrix
			%		C:				output matrix
			%		C_dot:			derivative output matrix
			%		D:				throughput matrix
			%		C_ref:			reference output matrix
			%		D_ref:			reference throughput matrix
			asstruct = false;
			astf = false;
			asss = true;
			asuss = false;
			asstructwithT = false;
			if isstruct(system)
				if nargin >= 3
					if islogical(A)
						noreference = A;
						if nargin >= 4
							T = B;
						else
							T = -1;
						end
					else
						T = A;
						if nargin >= 4
							noreference = B;
						else
							noreference = false;
						end
					end
				else
					T = -1;
					noreference = false;
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
					D = zeros(size(C, 1), size(B, 2));
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
					D_ref = zeros(0, size(D, 2));
				end
				if isfield(system, 'T')
					T = system.T;
					asstructwithT = true;
				elseif isfield(system, 't')
					T = system.t;
					asstructwithT = true;
				elseif isfield(system, 'T_s')
					T = system.T_s;
					asstructwithT = true;
				end
				asstruct = true;
			elseif isnumeric(system)
				if nargin >= 10
					if islogical(T)
						if nargin >= 11
							temp = noreference;
							noreference = T;
							T = temp;
						else
							noreference = T;
							T = -1;
						end
					else
						if nargin <= 10
							noreference = false;
						end
					end
				else
					T = -1;
					noreference = false;
				end
				E = system;
			elseif isa(system, 'tf') || isa(system, 'ss') || isa(system, 'uss')
				if nargin >= 3
					if islogical(A)
						noreference = A;
						if nargin >= 4
							T = B;
						else
							T = -1;
						end
					else
						T = A;
						if nargin >= 4
							noreference = B;
						else
							noreference = false;
						end
					end
				else
					T = -1;
					noreference = false;
				end
				if isa(system, 'uss')
					A = system.A;
					B = system.B;
					C = system.C;
					D = system.D;
					asuss = true;
				else
					[A, B, C, D] = ssdata(system);
					asuss = false;
				end
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
				asss = isa(system, 'ss');
				astf = ~asss && ~asuss;
			else
				if isa(system, 'genss') || isa(system, 'genmat')
					error('control:design:outputfeedback:input', 'System must not be of type ''genss''.');
				else
					error('control:design:outputfeedback:input', 'System must be some kind of control system, not a %s.', class(system));
				end
			end
			if ~islogical(noreference) || ~isscalar(noreference)
				error('control:design:outputfeedback:input', 'Reference indicator must be a logical scalar.');
			end
			[valid, errid, errmsg] = this.check(E, A, B, C, C_dot, D, C_ref, D_ref);
			if ~valid
				error(errid, errmsg);
			end
			[E_g, A_g, B_g, C_g, C_dot_g, D_g, C_ref_g, D_ref_g] = this.amend_system(E, A, B, C, C_dot, D, C_ref, D_ref, T);
			if control.design.outputfeedback.OutputFeedback.isranksupported(D_g) && rank(D_g) > 0
				error('control:design:outputfeedback:input', 'System must not have a throughput matrix.');
			end
			[R_gain, K_gain, F_prefilter] = this.gainpattern_parametric_system(E, A, B, C, C_dot, D, C_ref, D_ref, T);
			if isempty(C_ref_g) && isempty(D_ref_g)
				if noreference
					C_ref = zeros(0, size(A_g, 2));
					D_ref = zeros(0, size(B_g, 2));
				else
					C_ref = [
						C,	zeros(size(C, 1), size(A_g, 2) - size(C, 2))
					];
					D_ref = [
						D,	zeros(size(D, 1), size(B_g, 2) - size(D, 2))
					];
				end
			else
				C_ref = C_ref_g;
				D_ref = D_ref_g;
			end
			if islogical(E_g)
				E_g = double(E_g);
			end
			if islogical(A_g)
				A_g = double(A_g);
			end
			if islogical(B_g)
				B_g = double(B_g);
			end
			if islogical(C_g)
				C_g = double(C_g);
			end
			if islogical(C_dot_g)
				C_dot_g = double(C_dot_g);
			end
			%if islogical(D_g)
			%	D_g = double(D_g);
			%end
			if islogical(C_ref)
				C_ref = double(C_ref);
			end
			if islogical(D_ref)
				D_ref = double(D_ref);
			end
			E_g = E_g - B_g*K_gain*C_dot_g;
			A_g = A_g - B_g*R_gain*C_g;
			B_g = B_g*F_prefilter;
			if isnumeric(D_ref) && all(D_ref(:) == 0)
				C_g = C_ref;
				D_g = zeros(size(D_ref, 1), size(F_prefilter, 2));
			else
				C_g = C_ref - D_ref*R_gain*C_g + D_ref*K_gain*C_dot_g;
				D_g = D_ref*F_prefilter;
			end
			%C_ref_g = eye(size(C_ref, 1));
			%D_ref_g = zeros(size(D_ref, 1), size(D_ref, 1));
			if asstruct
				system = struct(...
					'E',		E_g,...
					'A',		A_g,...
					'B',		B_g,...
					'C',		C_g,...
					'C_dot',	C_dot_g,...
					'D',		D_g,...
					'C_ref',	C_ref_g,...
					'D_ref',	D_ref_g...
				);
				if asstructwithT && T > 0
					system.T = T;
				end
			elseif astf
				if this.isdiscreteT(T)
					if isnumeric(E_g) && iseye(E_g)
						system = tf(ss(A_g, B_g, C_g, D_g, T));
					else
						system = tf(dss(A_g, B_g, C_g, D_g, E_g, T));
					end
				else
					if isnumeric(E_g) && iseye(E_g)
						system = tf(ss(A_g, B_g, C_g, D_g));
					else
						system = tf(dss(A_g, B_g, C_g, D_g, E_g));
					end
				end
			elseif asuss
				if this.isdiscreteT(T)
					if isnumeric(E_g) && iseye(E_g)
						system = uss(A_g, B_g, C_g, D_g, T);
					else
						error('control:design:outputfeedback:input', 'Descriptor systems can not be represented as uss.');
					end
				else
					if isnumeric(E_g) && iseye(E_g)
						system = uss(A_g, B_g, C_g, D_g);
					else
						error('control:design:outputfeedback:input', 'Descriptor systems can not be represented as uss.');
					end
				end
			elseif asss
				if this.isdiscreteT(T)
					if isnumeric(E_g) && iseye(E_g)
						system = ss(A_g, B_g, C_g, D_g, T);
					else
						system = dss(A_g, B_g, C_g, D_g, E_g, T);
					end
				else
					if isnumeric(E_g) && iseye(E_g)
						system = ss(A_g, B_g, C_g, D_g);
					else
						system = dss(A_g, B_g, C_g, D_g, E_g);
					end
				end
			else
				if noreference
					C_ref = zeros(0, size(A_g, 2));
					D_ref = zeros(0, size(B_g, 2));
				else
					C_ref = [
						C,	zeros(size(C, 1), size(A_g, 2) - size(C, 2))
					];
					D_ref = [
						D,	zeros(size(D, 1), size(B_g, 2) - size(D, 2))
					];
				end
				system = E_g;
				A = A_g;
				B = B_g;
				C = C_g;
				C_dot = C_dot_g;
				D = D_g;
			end
		end

		function [E] = E(this, system, A, B, C, C_dot, D, C_ref, D_ref, T, noreference)
			%E return descriptor matrix of extended system for output feedback
			%	Input:
			%		this:			instance
			%		system:			state space system or structure with system matrices or descriptor matrix
			%		A:				system matrix or sample time if system is given as first argument
			%		B:				control matrix
			%		C:				output matrix
			%		C_dot:			derivative output matrix
			%		D:				throughput matrix
			%		C_ref:			reference output matrix
			%		D_ref:			reference throughput matrix
			%		T:				sampling time
			%		noreference:	indicator if reference outputs should be ignored
			%	Output:
			%		E:				descriptor matrix of extended system
			if isstruct(system)
				if nargin >= 3
					if islogical(A)
						if nargin >= 4
							T = B;
						else
							T = -1;
						end
					else
						T = A;
					end
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
					D = zeros(size(C, 1), size(B, 2));
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
					D_ref = zeros(0, size(D, 2));
				end
				if isfield(system, 'T')
					T = system.T;
				elseif isfield(system, 't')
					T = system.t;
				elseif isfield(system, 'T_s')
					T = system.T_s;
				end
			elseif isnumeric(system)
				if nargin >= 10
					if islogical(T)
						if nargin >= 11
							T = noreference;
						else
							T = -1;
						end
					end
				else
					T = -1;
				end
				E = system;
			elseif isa(system, 'tf') || isa(system, 'ss')
				if nargin >= 3
					if islogical(A)
						if nargin >= 4
							T = B;
						else
							T = -1;
						end
					else
						T = A;
					end
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
			[valid, errid, errmsg] = this.check(E, A, B, C, C_dot, D, C_ref, D_ref);
			if ~valid
				error(errid, errmsg);
			end
			E = this.amend_system(E, A, B, C, C_dot, D, C_ref, D_ref, T);
		end

		function [A] = A(this, system, A, B, C, C_dot, D, C_ref, D_ref, T, noreference)
			%A return system matrix of extended system for output feedback
			%	Input:
			%		this:			instance
			%		system:			state space system or structure with system matrices or descriptor matrix
			%		A:				system matrix or sample time if system is given as first argument
			%		B:				control matrix
			%		C:				output matrix
			%		C_dot:			derivative output matrix
			%		D:				throughput matrix
			%		C_ref:			reference output matrix
			%		D_ref:			reference throughput matrix
			%		T:				sampling time
			%		noreference:	indicator if reference outputs should be ignored
			%	Output:
			%		A:				system matrix of extended system
			if isstruct(system)
				if nargin >= 3
					if islogical(A)
						if nargin >= 4
							T = B;
						else
							T = -1;
						end
					else
						T = A;
					end
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
					D = zeros(size(C, 1), size(B, 2));
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
					D_ref = zeros(0, size(D, 2));
				end
				if isfield(system, 'T')
					T = system.T;
				elseif isfield(system, 't')
					T = system.t;
				elseif isfield(system, 'T_s')
					T = system.T_s;
				end
			elseif isnumeric(system)
				if nargin >= 10
					if islogical(T)
						if nargin >= 11
							T = noreference;
						else
							T = -1;
						end
					end
				else
					T = -1;
				end
				E = system;
			elseif isa(system, 'tf') || isa(system, 'ss')
				if nargin >= 3
					if islogical(A)
						if nargin >= 4
							T = B;
						else
							T = -1;
						end
					else
						T = A;
					end
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
			[valid, errid, errmsg] = this.check(E, A, B, C, C_dot, D, C_ref, D_ref);
			if ~valid
				error(errid, errmsg);
			end
			[~, A] = this.amend_system(E, A, B, C, C_dot, D, C_ref, D_ref, T);
		end

		function [B] = B(this, system, A, B, C, C_dot, D, C_ref, D_ref, T, noreference)
			%B return control matrix of extended system for output feedback
			%	Input:
			%		this:			instance
			%		system:			state space system or structure with system matrices or descriptor matrix
			%		A:				system matrix or sample time if system is given as first argument
			%		B:				control matrix
			%		C:				output matrix
			%		C_dot:			derivative output matrix
			%		D:				throughput matrix
			%		C_ref:			reference output matrix
			%		D_ref:			reference throughput matrix
			%		T:				sampling time
			%		noreference:	indicator if reference outputs should be ignored
			%	Output:
			%		B:				control matrix of extended system
			if isstruct(system)
				if nargin >= 3
					if islogical(A)
						if nargin >= 4
							T = B;
						else
							T = -1;
						end
					else
						T = A;
					end
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
					D = zeros(size(C, 1), size(B, 2));
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
					D_ref = zeros(0, size(D, 2));
				end
				if isfield(system, 'T')
					T = system.T;
				elseif isfield(system, 't')
					T = system.t;
				elseif isfield(system, 'T_s')
					T = system.T_s;
				end
			elseif isnumeric(system)
				if nargin >= 10
					if islogical(T)
						if nargin >= 11
							T = noreference;
						else
							T = -1;
						end
					end
				else
					T = -1;
				end
				E = system;
			elseif isa(system, 'tf') || isa(system, 'ss')
				if nargin >= 3
					if islogical(A)
						if nargin >= 4
							T = B;
						else
							T = -1;
						end
					else
						T = A;
					end
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
			[valid, errid, errmsg] = this.check(E, A, B, C, C_dot, D, C_ref, D_ref);
			if ~valid
				error(errid, errmsg);
			end
			[~, ~, B] = this.amend_system(E, A, B, C, C_dot, D, C_ref, D_ref, T);
		end

		function [C] = C(this, system, A, B, C, C_dot, D, C_ref, D_ref, T, noreference)
			%C return output matrix of extended system for output feedback
			%	Input:
			%		this:			instance
			%		system:			state space system or structure with system matrices or descriptor matrix
			%		A:				system matrix or sample time if system is given as first argument
			%		B:				control matrix
			%		C:				output matrix
			%		C_dot:			derivative output matrix
			%		D:				throughput matrix
			%		C_ref:			reference output matrix
			%		D_ref:			reference throughput matrix
			%		T:				sampling time
			%		noreference:	indicator if reference outputs should be ignored
			%	Output:
			%		C:				output matrix of extended system
			if isstruct(system)
				if nargin >= 3
					if islogical(A)
						if nargin >= 4
							T = B;
						else
							T = -1;
						end
					else
						T = A;
					end
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
					D = zeros(size(C, 1), size(B, 2));
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
					D_ref = zeros(0, size(D, 2));
				end
				if isfield(system, 'T')
					T = system.T;
				elseif isfield(system, 't')
					T = system.t;
				elseif isfield(system, 'T_s')
					T = system.T_s;
				end
			elseif isnumeric(system)
				if nargin >= 10
					if islogical(T)
						if nargin >= 11
							T = noreference;
						else
							T = -1;
						end
					end
				else
					T = -1;
				end
				E = system;
			elseif isa(system, 'tf') || isa(system, 'ss')
				if nargin >= 3
					if islogical(A)
						if nargin >= 4
							T = B;
						else
							T = -1;
						end
					else
						T = A;
					end
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
			[valid, errid, errmsg] = this.check(E, A, B, C, C_dot, D, C_ref, D_ref);
			if ~valid
				error(errid, errmsg);
			end
			[~, ~, ~, C] = this.amend_system(E, A, B, C, C_dot, D, C_ref, D_ref, T);
		end

		function [C_dot] = C_dot(this, system, A, B, C, C_dot, D, C_ref, D_ref, T, noreference)
			%C_dot return derivative output matrix of extended system for output feedback
			%	Input:
			%		this:			instance
			%		system:			state space system or structure with system matrices or descriptor matrix
			%		A:				system matrix or sample time if system is given as first argument
			%		B:				control matrix
			%		C:				output matrix
			%		C_dot:			derivative output matrix
			%		D:				throughput matrix
			%		C_ref:			reference output matrix
			%		D_ref:			reference throughput matrix
			%		T:				sampling time
			%		noreference:	indicator if reference outputs should be ignored
			%	Output:
			%		C_dot:			derivative output matrix of extended system
			if isstruct(system)
				if nargin >= 3
					if islogical(A)
						if nargin >= 4
							T = B;
						else
							T = -1;
						end
					else
						T = A;
					end
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
					D = zeros(size(C, 1), size(B, 2));
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
					D_ref = zeros(0, size(D, 2));
				end
				if isfield(system, 'T')
					T = system.T;
				elseif isfield(system, 't')
					T = system.t;
				elseif isfield(system, 'T_s')
					T = system.T_s;
				end
			elseif isnumeric(system)
				if nargin >= 10
					if islogical(T)
						if nargin >= 11
							T = noreference;
						else
							T = -1;
						end
					end
				else
					T = -1;
				end
				E = system;
			elseif isa(system, 'tf') || isa(system, 'ss')
				if nargin >= 3
					if islogical(A)
						if nargin >= 4
							T = B;
						else
							T = -1;
						end
					else
						T = A;
					end
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
			[valid, errid, errmsg] = this.check(E, A, B, C, C_dot, D, C_ref, D_ref);
			if ~valid
				error(errid, errmsg);
			end
			[~, ~, ~, ~, C_dot] = this.amend_system(E, A, B, C, C_dot, D, C_ref, D_ref, T);
		end

		function [D] = D(this, system, A, B, C, C_dot, D, C_ref, D_ref, T, noreference)
			%D return throughput matrix of extended system for output feedback
			%	Input:
			%		this:			instance
			%		system:			state space system or structure with system matrices or descriptor matrix
			%		A:				system matrix or sample time if system is given as first argument
			%		B:				control matrix
			%		C:				output matrix
			%		C_dot:			derivative output matrix
			%		D:				throughput matrix
			%		C_ref:			reference output matrix
			%		D_ref:			reference throughput matrix
			%		T:				sampling time
			%		noreference:	indicator if reference outputs should be ignored
			%	Output:
			%		D:				throughput matrix of extended system
			if isstruct(system)
				if nargin >= 3
					if islogical(A)
						if nargin >= 4
							T = B;
						else
							T = -1;
						end
					else
						T = A;
					end
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
					D = zeros(size(C, 1), size(B, 2));
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
					D_ref = zeros(0, size(D, 2));
				end
				if isfield(system, 'T')
					T = system.T;
				elseif isfield(system, 't')
					T = system.t;
				elseif isfield(system, 'T_s')
					T = system.T_s;
				end
			elseif isnumeric(system)
				if nargin >= 10
					if islogical(T)
						if nargin >= 11
							T = noreference;
						else
							T = -1;
						end
					end
				else
					T = -1;
				end
				E = system;
			elseif isa(system, 'tf') || isa(system, 'ss')
				if nargin >= 3
					if islogical(A)
						if nargin >= 4
							T = B;
						else
							T = -1;
						end
					else
						T = A;
					end
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
			[valid, errid, errmsg] = this.check(E, A, B, C, C_dot, D, C_ref, D_ref);
			if ~valid
				error(errid, errmsg);
			end
			[~, ~, ~, ~, ~, D] = this.amend_system(E, A, B, C, C_dot, D, C_ref, D_ref, T);
		end

		function [C_ref] = C_ref(this, system, A, B, C, C_dot, D, C_ref, D_ref, T, noreference)
			%C_REF return reference output matrix of extended system for output feedback
			%	Input:
			%		this:			instance
			%		system:			state space system or structure with system matrices or descriptor matrix
			%		A:				system matrix or sample time if system is given as first argument
			%		B:				control matrix
			%		C:				output matrix
			%		C_dot:			derivative output matrix
			%		D:				throughput matrix
			%		C_ref:			reference output matrix
			%		D_ref:			reference throughput matrix
			%		T:				sampling time
			%		noreference:	indicator if reference outputs should be ignored
			%	Output:
			%		C_ref:			reference output matrix of extended system
			if isstruct(system)
				if nargin >= 3
					if islogical(A)
						noreference = A;
						if nargin >= 4
							T = B;
						else
							T = -1;
						end
					else
						T = A;
						if nargin >= 4
							noreference = B;
						else
							noreference = false;
						end
					end
				else
					T = -1;
					noreference = false;
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
					D = zeros(size(C, 1), size(B, 2));
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
					D_ref = zeros(0, size(D, 2));
				end
				if isfield(system, 'T')
					T = system.T;
				elseif isfield(system, 't')
					T = system.t;
				elseif isfield(system, 'T_s')
					T = system.T_s;
				end
			elseif isnumeric(system)
				if nargin >= 10
					if islogical(T)
						if nargin >= 11
							temp = noreference;
							noreference = T;
							T = temp;
						else
							noreference = T;
							T = -1;
						end
					else
						if nargin <= 10
							noreference = false;
						end
					end
				else
					T = -1;
					noreference = false;
				end
				E = system;
			elseif isa(system, 'tf') || isa(system, 'ss')
				if nargin >= 3
					if islogical(A)
						noreference = A;
						if nargin >= 4
							T = B;
						else
							T = -1;
						end
					else
						T = A;
						if nargin >= 4
							noreference = B;
						else
							noreference = false;
						end
					end
				else
					T = -1;
					noreference = false;
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
			if ~islogical(noreference) || ~isscalar(noreference)
				error('control:design:outputfeedback:input', 'Reference indicator must be a logical scalar.');
			end
			[valid, errid, errmsg] = this.check(E, A, B, C, C_dot, D, C_ref, D_ref, T);
			if ~valid
				error(errid, errmsg);
			end
			[~, A_g, ~, ~, ~, ~, C_ref, D_ref] = this.amend_system(E, A, B, C, C_dot, D, C_ref, D_ref, T);
			if isempty(C_ref) && isempty(D_ref)
				if noreference
					C_ref = zeros(0, size(A_g, 2));
				else
					C_ref = [
						C,	zeros(size(C, 1), size(A_g, 2) - size(C, 2))
					];
				end
			end
		end

		function [D_ref] = D_ref(this, system, A, B, C, C_dot, D, C_ref, D_ref, T, noreference)
			%D_REF return reference throughput matrix of extended system for output feedback
			%	Input:
			%		this:			instance
			%		system:			state space system or structure with system matrices or descriptor matrix
			%		A:				system matrix or sample time if system is given as first argument
			%		B:				control matrix
			%		C:				output matrix
			%		C_dot:			derivative output matrix
			%		D:				throughput matrix
			%		C_ref:			reference output matrix
			%		D_ref:			reference throughput matrix
			%		T:				sampling time
			%		noreference:	indicator if reference outputs should be ignored
			%	Output:
			%		D_ref:			reference output matrix of extended system
			if isstruct(system)
				if nargin >= 3
					if islogical(A)
						noreference = A;
						if nargin >= 4
							T = B;
						else
							T = -1;
						end
					else
						T = A;
						if nargin >= 4
							noreference = B;
						else
							noreference = false;
						end
					end
				else
					T = -1;
					noreference = false;
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
					D = zeros(size(C, 1), size(B, 2));
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
					D_ref = zeros(0, size(D, 2));
				end
				if isfield(system, 'T')
					T = system.T;
				elseif isfield(system, 't')
					T = system.t;
				elseif isfield(system, 'T_s')
					T = system.T_s;
				end
			elseif isnumeric(system)
				if nargin >= 10
					if islogical(T)
						if nargin >= 11
							temp = noreference;
							noreference = T;
							T = temp;
						else
							noreference = T;
							T = -1;
						end
					else
						if nargin <= 10
							noreference = false;
						end
					end
				else
					T = -1;
					noreference = false;
				end
				E = system;
			elseif isa(system, 'tf') || isa(system, 'ss')
				if nargin >= 3
					if islogical(A)
						noreference = A;
						if nargin >= 4
							T = B;
						else
							T = -1;
						end
					else
						T = A;
						if nargin >= 4
							noreference = B;
						else
							noreference = false;
						end
					end
				else
					T = -1;
					noreference = false;
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
			if ~islogical(noreference) || ~isscalar(noreference)
				error('control:design:outputfeedback:input', 'Reference indicator must be a logical scalar.');
			end
			[valid, errid, errmsg] = this.check(E, A, B, C, C_dot, D, C_ref, D_ref, T);
			if ~valid
				error(errid, errmsg);
			end
			[~, ~, B_g, ~, ~, ~, C_ref, D_ref] = this.amend_system(E, A, B, C, C_dot, D, C_ref, D_ref, T);
			if isempty(C_ref) && isempty(D_ref)
				if noreference
					D_ref = zeros(0, size(B_g, 2));
				else
					D_ref = [
						D,	zeros(size(D, 1), size(B_g, 2) - size(D, 2))
					];
				end
			end
		end

		function [R_scaled] = scalegain(this, R_unscaled, T_x, T_u, T_y, T_y_dot, T_w, system, A, B, C, C_dot, D, C_ref, D_ref, T)
			%SCALEGAIN return scaled gain matrix for given unscaled gain matrix
			%	Input:
			%		this:		instance
			%		R_unscaled:	gain matrix to scale
			%		T_x:		state transformation matrix
			%		T_u:		control transformation matrix
			%		T_Y:		measurement transformation matrix
			%		T_y_dot:	derivative measurement transformation matrix
			%		T_w:		reference value transformation matrix
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
			%		R_scaled:	scaled gain matrix
			if isstruct(system)
				if nargin >= 9
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
					D = zeros(size(C, 1), size(B, 2));
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
					D_ref = zeros(0, size(D, 2));
				end
				if isfield(system, 'T')
					T = system.T;
				elseif isfield(system, 't')
					T = system.t;
				elseif isfield(system, 'T_s')
					T = system.T_s;
				end
			elseif isnumeric(system)
				if nargin <= 15
					T = -1;
				end
				E = system;
			elseif isa(system, 'tf') || isa(system, 'ss')
				if nargin >= 9
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
			[valid, errid, errmsg] = this.check(E, A, B, C, C_dot, D, C_ref, D_ref);
			if ~valid
				error(errid, errmsg);
			end
			if ndims(T_x) >= 3 %#ok<ISMAT> used for compatibility with octave
				error('control:design:outputfeedback:input', 'State transformation matrix must be a matrix.');
			end
			if size(T_x, 1) ~= size(T_x, 2)
				error('control:design:outputfeedback:input', 'State transformation matrix must be square.');
			end
			if size(T_x, 1) ~= size(A, 1)
				error('control:design:outputfeedback:input', 'State transformation matrix must have %d rows not %d.', size(A, 1), size(T_x, 1));
			end
			if rank(T_x) ~= size(T_x, 1)
				error('control:design:outputfeedback:input', 'State transformation matrix must be regular.');
			end
			if ndims(T_u) >= 3 %#ok<ISMAT> used for compatibility with octave
				error('control:design:outputfeedback:input', 'Control transformation matrix must be a matrix.');
			end
			if size(T_u, 1) ~= size(T_u, 2)
				error('control:design:outputfeedback:input', 'Control transformation matrix must be square.');
			end
			if size(T_u, 1) ~= size(B, 2)
				error('control:design:outputfeedback:input', 'Control transformation matrix must have %d rows not %d.', size(B, 2), size(T_u, 1));
			end
			if rank(T_u) ~= size(T_u, 1)
				error('control:design:outputfeedback:input', 'Control transformation matrix must be regular.');
			end
			if ndims(T_y) >= 3 %#ok<ISMAT> used for compatibility with octave
				error('control:design:outputfeedback:input', 'Measurement transformation matrix must be a matrix.');
			end
			if size(T_y, 1) ~= size(T_y, 2)
				error('control:design:outputfeedback:input', 'Measurement transformation matrix must be square.');
			end
			if size(T_y, 1) ~= size(C, 1)
				error('control:design:outputfeedback:input', 'Measurement transformation matrix must have %d rows not %d.', size(C, 1), size(T_y, 1));
			end
			if rank(T_y) ~= size(T_y, 1)
				error('control:design:outputfeedback:input', 'Measurement transformation matrix must be regular.');
			end
			if ndims(T_y_dot) >= 3 %#ok<ISMAT> used for compatibility with octave
				error('control:design:outputfeedback:input', 'Derivative measurement transformation matrix must be a matrix.');
			end
			if size(T_y_dot, 1) ~= size(T_y_dot, 2)
				error('control:design:outputfeedback:input', 'Derivative measurement transformation matrix must be square.');
			end
			if size(T_y_dot, 1) ~= size(C_dot, 1)
				error('control:design:outputfeedback:input', 'Derivative measurement transformation matrix must have %d rows not %d.', size(C_dot, 1), size(T_y_dot, 1));
			end
			if ~isempty(T_y_dot) && rank(T_y_dot) ~= size(T_y_dot, 1)
				error('control:design:outputfeedback:input', 'Derivative measurement transformation matrix must be regular.');
			end
			if isempty(T_w)
				T_w = T_y;
			end
			if ndims(T_w) >= 3 %#ok<ISMAT> used for compatibility with octave
				error('control:design:outputfeedback:input', 'Reference value transformation matrix must be a matrix.');
			end
			if size(T_w, 1) ~= size(T_w, 2)
				error('control:design:outputfeedback:input', 'Reference value transformation matrix must be square.');
			end
			if ~isempty(T_w) && rank(T_w) ~= size(T_w, 1)
				error('control:design:outputfeedback:input', 'Reference value transformation matrix must be regular.');
			end
			[R_fixed, K_fixed, F_fixed] = gainpattern_system(this, E, A, B, C, C_dot, D, C_ref, D_ref, T);
			numelR = numel(R_unscaled);
			if isnumeric(R_unscaled)
				numelR = -1;
				K_unscaled = zeros(size(K_fixed{1}, 1), size(K_fixed{1}, 2));
				F_unscaled = zeros(size(F_fixed{1}, 1), size(F_fixed{1}, 2));
				if size(R_unscaled, 1) ~= size(R_fixed{1}, 1)
					error('control:design:outputfeedback:input', 'Proportional gain matrix must have %d columns, not %d.', size(R_fixed{1}, 1), size(R_unscaled, 1));
				end
				if size(R_unscaled, 2) ~= size(R_fixed{1}, 2)
					error('control:design:outputfeedback:input', 'Proportional gain matrix must have %d rows, not %d.', size(R_fixed{1}, 2), size(R_unscaled, 2));
				end
			else
				if iscell(R_unscaled)
					if numel(R_unscaled) >= 3
						F_unscaled = R_unscaled{3};
					else
						F_unscaled = zeros(size(F_fixed{1}, 1), size(F_fixed{1}, 2));
					end
					K_unscaled = R_unscaled{2};
					R_unscaled = R_unscaled{1};
					if size(R_unscaled, 1) ~= size(R_fixed{1}, 1)
						error('control:design:outputfeedback:input', 'Proportional gain matrix must have %d columns, not %d.', size(R_fixed{1}, 1), size(R_unscaled, 1));
					end
					if size(R_unscaled, 2) ~= size(R_fixed{1}, 2)
						error('control:design:outputfeedback:input', 'Proportional gain matrix must have %d rows, not %d.', size(R_fixed{1}, 2), size(R_unscaled, 2));
					end
					if size(K_unscaled, 1) ~= size(K_fixed{1}, 1)
						error('control:design:outputfeedback:input', 'Derivative gain matrix must have %d columns, not %d.', size(K_fixed{1}, 1), size(K_unscaled, 1));
					end
					if size(K_unscaled, 2) ~= size(K_fixed{1}, 2)
						error('control:design:outputfeedback:input', 'Derivative gain matrix must have %d rows, not %d.', size(K_fixed{1}, 2), size(K_unscaled, 2));
					end
					if size(F_unscaled, 1) ~= size(F_fixed{1}, 1)
						error('control:design:outputfeedback:input', 'Prefilter gain matrix must have %d columns, not %d.', size(F_fixed{1}, 1), size(F_unscaled, 1));
					end
					if size(F_unscaled, 2) ~= size(F_fixed{1}, 2)
						error('control:design:outputfeedback:input', 'Prefilter gain matrix must have %d rows, not %d.', size(F_fixed{1}, 2), size(F_unscaled, 2));
					end
				else
					error('control:design:outputfeedback:input', 'Gain matrix must be the numeric proportional gain or a cell array with proportional, derivative and prefilter gain.');
				end
			end
			[~, T_u, T_y, T_y_dot, T_w] = this.scalegain_system(T_x, T_u, T_y, T_y_dot, T_w, E, A, B, C, C_dot, D, C_ref, D_ref, T);
			if size(T_w, 1) ~= size(F_unscaled, 2)
				error('control:design:outputfeedback:input', 'Reference value transformation matrix must have %d rows not %d.', size(F_unscaled, 2), size(T_w, 1));
			end
			R_scaled = T_u\R_unscaled*T_y;
			if numelR > 0
				K_scaled = T_u\K_unscaled*T_y_dot;
				F_scaled = T_u\F_unscaled*T_w;
				if numelR <= 1
					R_scaled = {
						R_scaled
					};
				elseif numelR <= 2
					R_scaled = {
						R_scaled, K_scaled
					};
				else
					R_scaled = {
						R_scaled, K_scaled, F_scaled
					};
				end
			end
		end

		function [F_scaled] = scaleprefilter(this, F_unscaled, T_x, T_u, T_y, T_y_dot, T_w, system, A, B, C, C_dot, D, C_ref, D_ref, T)
			%SCALEPREFILTER return scaled prefilter matrix for given unscaled prefilter matrix
			%	Input:
			%		this:		instance
			%		F_unscaled:	prefilter matrix to scale
			%		T_x:		state transformation matrix
			%		T_u:		control transformation matrix
			%		T_Y:		measurement transformation matrix
			%		T_y_dot:	derivative measurement transformation matrix
			%		T_w:		reference value transformation matrix
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
			%		F_scaled:	scaled prefilter matrix
			if isstruct(system)
				if nargin >= 9
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
					D = zeros(size(C, 1), size(B, 2));
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
					D_ref = zeros(0, size(D, 2));
				end
				if isfield(system, 'T')
					T = system.T;
				elseif isfield(system, 't')
					T = system.t;
				elseif isfield(system, 'T_s')
					T = system.T_s;
				end
			elseif isnumeric(system)
				if nargin <= 15
					T = -1;
				end
				E = system;
			elseif isa(system, 'tf') || isa(system, 'ss')
				if nargin >= 9
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
			[valid, errid, errmsg] = this.check(E, A, B, C, C_dot, D, C_ref, D_ref);
			if ~valid
				error(errid, errmsg);
			end
			if ndims(T_x) >= 3 %#ok<ISMAT> used for compatibility with octave
				error('control:design:outputfeedback:input', 'State transformation matrix must be a matrix.');
			end
			if size(T_x, 1) ~= size(T_x, 2)
				error('control:design:outputfeedback:input', 'State transformation matrix must be square.');
			end
			if size(T_x, 1) ~= size(A, 1)
				error('control:design:outputfeedback:input', 'State transformation matrix must have %d rows not %d.', size(A, 1), size(T_x, 1));
			end
			if rank(T_x) ~= size(T_x, 1)
				error('control:design:outputfeedback:input', 'State transformation matrix must be regular.');
			end
			if ndims(T_u) >= 3 %#ok<ISMAT> used for compatibility with octave
				error('control:design:outputfeedback:input', 'Control transformation matrix must be a matrix.');
			end
			if size(T_u, 1) ~= size(T_u, 2)
				error('control:design:outputfeedback:input', 'Control transformation matrix must be square.');
			end
			if size(T_u, 1) ~= size(B, 2)
				error('control:design:outputfeedback:input', 'Control transformation matrix must have %d rows not %d.', size(B, 2), size(T_u, 1));
			end
			if rank(T_u) ~= size(T_u, 1)
				error('control:design:outputfeedback:input', 'Control transformation matrix must be regular.');
			end
			if ndims(T_y) >= 3 %#ok<ISMAT> used for compatibility with octave
				error('control:design:outputfeedback:input', 'Measurement transformation matrix must be a matrix.');
			end
			if size(T_y, 1) ~= size(T_y, 2)
				error('control:design:outputfeedback:input', 'Measurement transformation matrix must be square.');
			end
			if size(T_y, 1) ~= size(C, 1)
				error('control:design:outputfeedback:input', 'Measurement transformation matrix must have %d rows not %d.', size(C, 1), size(T_y, 1));
			end
			if rank(T_y) ~= size(T_y, 1)
				error('control:design:outputfeedback:input', 'Measurement transformation matrix must be regular.');
			end
			if ndims(T_y_dot) >= 3 %#ok<ISMAT> used for compatibility with octave
				error('control:design:outputfeedback:input', 'Derivative measurement transformation matrix must be a matrix.');
			end
			if size(T_y_dot, 1) ~= size(T_y_dot, 2)
				error('control:design:outputfeedback:input', 'Derivative measurement transformation matrix must be square.');
			end
			if size(T_y_dot, 1) ~= size(C_dot, 1)
				error('control:design:outputfeedback:input', 'Derivative measurement transformation matrix must have %d rows not %d.', size(C_dot, 1), size(T_y_dot, 1));
			end
			if ~isempty(T_y_dot) && rank(T_y_dot) ~= size(T_y_dot, 1)
				error('control:design:outputfeedback:input', 'Derivative measurement transformation matrix must be regular.');
			end
			if isempty(T_w)
				T_w = T_y;
			end
			if ndims(T_w) >= 3 %#ok<ISMAT> used for compatibility with octave
				error('control:design:outputfeedback:input', 'Reference value transformation matrix must be a matrix.');
			end
			if size(T_w, 1) ~= size(T_w, 2)
				error('control:design:outputfeedback:input', 'Reference value transformation matrix must be square.');
			end
			if ~isempty(T_w) && rank(T_w) ~= size(T_w, 1)
				error('control:design:outputfeedback:input', 'Reference value transformation matrix must be regular.');
			end
			[R_fixed, K_fixed] = gainpattern_system(this, E, A, B, C, C_dot, D, C_ref, D_ref, T);
			R = zeros(size(R_fixed{1}, 1), size(R_fixed{1}, 2));
			K = zeros(size(K_fixed{1}, 1), size(K_fixed{1}, 2));
			F = this.prefilterpattern_system(R, K, E, A, B, C, C_dot, D, C_ref, D_ref, T);
			if size(F_unscaled, 1) ~= size(F, 1)
				error('control:design:outputfeedback:input', 'Proportional gain matrix must have %d columns, not %d.', size(F, 1), size(F_unscaled, 1));
			end
			if size(F_unscaled, 2) ~= size(F, 2)
				error('control:design:outputfeedback:input', 'Proportional gain matrix must have %d rows, not %d.', size(F, 2), size(F_unscaled, 2));
			end
			[~, T_u, ~, ~, T_w] = this.scalegain_system(T_x, T_u, T_y, T_y_dot, T_w, E, A, B, C, C_dot, D, C_ref, D_ref, T);
			if size(T_w, 1) ~= size(F, 2)
				error('control:design:outputfeedback:input', 'Reference value transformation matrix must have %d rows not %d.', size(F, 2), size(T_w, 1));
			end
			F_scaled = T_u\F_unscaled*T_w;
		end

		function [F, F_fixed] = prefilterpattern(this, R, system, A, B, C, C_dot, D, C_ref, D_ref, T)
			%F return prefilter matrix of extended system for output feedback with given gain matrix
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
			if isstruct(system)
				if nargin >= 4
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
					D = zeros(size(C, 1), size(B, 2));
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
					D_ref = zeros(0, size(D, 2));
				end
				if isfield(system, 'T')
					T = system.T;
				elseif isfield(system, 't')
					T = system.t;
				elseif isfield(system, 'T_s')
					T = system.T_s;
				end
			elseif isnumeric(system)
				if nargin <= 10
					T = -1;
				end
				E = system;
			elseif isa(system, 'tf') || isa(system, 'ss')
				if nargin >= 4
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
			[valid, errid, errmsg] = this.check(E, A, B, C, C_dot, D, C_ref, D_ref);
			if ~valid
				error(errid, errmsg);
			end
			[R_fixed, K_fixed] = gainpattern_system(this, E, A, B, C, C_dot, D, C_ref, D_ref, T);
			if isnumeric(R)
				K = zeros(size(K_fixed{1}, 1), size(K_fixed{1}, 2));
				if size(R, 1) ~= size(R_fixed{1}, 1)
					error('control:design:outputfeedback:input', 'Proportional gain matrix must have %d columns, not %d.', size(R_fixed{1}, 1), size(R, 1));
				end
				if size(R, 2) ~= size(R_fixed{1}, 2)
					error('control:design:outputfeedback:input', 'Proportional gain matrix must have %d rows, not %d.', size(R_fixed{1}, 2), size(R, 2));
				end
			else
				if iscell(R)
					K = R{2};
					R = R{1};
					if size(R, 1) ~= size(R_fixed{1}, 1)
						error('control:design:outputfeedback:input', 'Proportional gain matrix must have %d columns, not %d.', size(R_fixed{1}, 1), size(R, 1));
					end
					if size(R, 2) ~= size(R_fixed{1}, 2)
						error('control:design:outputfeedback:input', 'Proportional gain matrix must have %d rows, not %d.', size(R_fixed{1}, 2), size(R, 2));
					end
					if size(K, 1) ~= size(K_fixed{1}, 1)
						error('control:design:outputfeedback:input', 'Derivative gain matrix must have %d columns, not %d.', size(K_fixed{1}, 1), size(K, 1));
					end
					if size(K, 2) ~= size(K_fixed{1}, 2)
						error('control:design:outputfeedback:input', 'Derivative gain matrix must have %d rows, not %d.', size(K_fixed{1}, 2), size(K, 2));
					end
				else
					error('control:design:outputfeedback:input', 'Gain matrix must be the numeric proportional gain or a cell array with proportional and derivative gain.');
				end
			end
			[F, F_fixed] = this.prefilterpattern_system(R, K, E, A, B, C, C_dot, D, C_ref, D_ref, T);
		end

		function [partitionR, partitionF] = gainpartitioning(this, R, F, system, A, B, C, C_dot, D, C_ref, D_ref, T)
			%GAINPARTITIONING return partitioning for gain matrix of extended system for output feedback with given gain matrix
			%	Input:
			%		this:		instance
			%		R:			gain matrix to close loop with
			%		F:			prefilter matrix
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
			%		partitionR:	named partition matrices of gain matrix in structure fields
			%		partitionF:	named partition matrices of prefilter matrix in structure fields
			if isstruct(system)
				if nargin >= 5
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
					D = zeros(size(C, 1), size(B, 2));
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
					D_ref = zeros(0, size(D, 2));
				end
				if isfield(system, 'T')
					T = system.T;
				elseif isfield(system, 't')
					T = system.t;
				elseif isfield(system, 'T_s')
					T = system.T_s;
				end
			elseif isnumeric(system)
				if nargin <= 11
					T = -1;
				end
				E = system;
			elseif isa(system, 'tf') || isa(system, 'ss')
				if nargin >= 5
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
			[valid, errid, errmsg] = this.check(E, A, B, C, C_dot, D, C_ref, D_ref);
			if ~valid
				error(errid, errmsg);
			end
			[R_fixed, K_fixed] = gainpattern_system(this, E, A, B, C, C_dot, D, C_ref, D_ref, T);
			if isnumeric(R)
				K = zeros(size(K_fixed{1}, 1), size(K_fixed{1}, 2));
				if size(R, 1) ~= size(R_fixed{1}, 1)
					error('control:design:outputfeedback:input', 'Proportional gain matrix must have %d columns, not %d.', size(R_fixed{1}, 1), size(R, 1));
				end
				if size(R, 2) ~= size(R_fixed{1}, 2)
					error('control:design:outputfeedback:input', 'Proportional gain matrix must have %d rows, not %d.', size(R_fixed{1}, 2), size(R, 2));
				end
			else
				if iscell(R)
					K = R{2};
					R = R{1};
					if size(R, 1) ~= size(R_fixed{1}, 1)
						error('control:design:outputfeedback:input', 'Proportional gain matrix must have %d columns, not %d.', size(R_fixed{1}, 1), size(R, 1));
					end
					if size(R, 2) ~= size(R_fixed{1}, 2)
						error('control:design:outputfeedback:input', 'Proportional gain matrix must have %d rows, not %d.', size(R_fixed{1}, 2), size(R, 2));
					end
					if size(K, 1) ~= size(K_fixed{1}, 1)
						error('control:design:outputfeedback:input', 'Derivative gain matrix must have %d columns, not %d.', size(K_fixed{1}, 1), size(K, 1));
					end
					if size(K, 2) ~= size(K_fixed{1}, 2)
						error('control:design:outputfeedback:input', 'Derivative gain matrix must have %d rows, not %d.', size(K_fixed{1}, 2), size(K, 2));
					end
				else
					error('control:design:outputfeedback:input', 'Gain matrix must be the numeric proportional gain or a cell array with proportional and derivative gain.');
				end
			end
			if nargout >= 2
				if isempty(F)
					F = this.prefilterpattern_system(R, K, E, A, B, C, C_dot, D, C_ref, D_ref, T);
				end
				[partitionR, partitionF] = this.gainpartitioning_system(R, K, F, E, A, B, C, C_dot, D, C_ref, D_ref, T);
			else
				partitionR = this.gainpartitioning_system(R, K, F, E, A, B, C, C_dot, D, C_ref, D_ref, T);
			end
		end

		function [system_c, A_c, B_c, C_c, C_dot_c, K_c, C_ref_c, D_ref_c, needsstate, usesCasCdot] = realization(this, R, F, system, A, B, C, C_dot, D, C_ref, D_ref, T)
			%REALIZATION return controller without system for output feedback with given gain matrix and inputs w and y and output u
			%	Input:
			%		this:			instance
			%		R:				gain matrix to close loop with
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
			%		system_c:		state space system or structure with system matrices or descriptor matrix of controller depending on what input arguments are supplied
			%		A_c:			system matrix of controller
			%		B_c:			control matrix of controller
			%		C_c:			output matrix of controller
			%		C_dot_c:		derivative output matrix of controller
			%		K_c:			throughput matrix of controller
			%		C_ref_c:		measurement matrix for reference outputs of controller
			%		D_ref_c:		throughput matrix for reference outputs of controller
			%		needsstate:		indicator if state is needed as controller input instead of output
			%		usesCasCdot:	indicator if C is used as Cdot in case of derivative feedback
			asstruct = false;
			astf = false;
			asss = true;
			asstructwithT = false;
			if isstruct(system)
				if nargin >= 5
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
					D = zeros(size(C, 1), size(B, 2));
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
					D_ref = zeros(0, size(D, 2));
				end
				if isfield(system, 'T')
					T = system.T;
					asstructwithT = true;
				elseif isfield(system, 't')
					T = system.t;
					asstructwithT = true;
				elseif isfield(system, 'T_s')
					T = system.T_s;
					asstructwithT = true;
				end
				asstruct = true;
			elseif isnumeric(system)
				if nargin <= 11
					T = -1;
				end
				E = system;
			elseif isa(system, 'tf') || isa(system, 'ss')
				if nargin >= 5
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
				asss = isa(system, 'ss');
				astf = ~asss;
			else
				error('control:design:outputfeedback:input', 'System must be some kind of control system, not a %s.', class(system));
			end
			[valid, errid, errmsg] = this.check(E, A, B, C, C_dot, D, C_ref, D_ref);
			if ~valid
				error(errid, errmsg);
			end
			[R_fixed, K_fixed] = gainpattern_system(this, E, A, B, C, C_dot, D, C_ref, D_ref, T);
			if isnumeric(R)
				K = zeros(size(K_fixed{1}, 1), size(K_fixed{1}, 2));
				if size(R, 1) ~= size(R_fixed{1}, 1)
					error('control:design:outputfeedback:input', 'Proportional gain matrix must have %d columns, not %d.', size(R_fixed{1}, 1), size(R, 1));
				end
				if size(R, 2) ~= size(R_fixed{1}, 2)
					error('control:design:outputfeedback:input', 'Proportional gain matrix must have %d rows, not %d.', size(R_fixed{1}, 2), size(R, 2));
				end
			else
				if iscell(R)
					K = R{2};
					R = R{1};
					if size(R, 1) ~= size(R_fixed{1}, 1)
						error('control:design:outputfeedback:input', 'Proportional gain matrix must have %d columns, not %d.', size(R_fixed{1}, 1), size(R, 1));
					end
					if size(R, 2) ~= size(R_fixed{1}, 2)
						error('control:design:outputfeedback:input', 'Proportional gain matrix must have %d rows, not %d.', size(R_fixed{1}, 2), size(R, 2));
					end
					if size(K, 1) ~= size(K_fixed{1}, 1)
						error('control:design:outputfeedback:input', 'Derivative gain matrix must have %d columns, not %d.', size(K_fixed{1}, 1), size(K, 1));
					end
					if size(K, 2) ~= size(K_fixed{1}, 2)
						error('control:design:outputfeedback:input', 'Derivative gain matrix must have %d rows, not %d.', size(K_fixed{1}, 2), size(K, 2));
					end
				else
					error('control:design:outputfeedback:input', 'Gain matrix must be the numeric proportional gain or a cell array with proportional and derivative gain.');
				end
			end
			F_fixed = prefilterpattern_system(this, R, K, E, A, B, C, C_dot, D, C_ref, D_ref, T);
			if size(F, 1) ~= size(F_fixed, 1)
				error('control:design:outputfeedback:input', 'Prefilter matrix must have %d columns, not %d.', size(F_fixed, 1), size(F, 1));
			end
			if size(F, 2) ~= size(F_fixed, 2)
				error('control:design:outputfeedback:input', 'Prefilter matrix must have %d rows, not %d.', size(F_fixed, 2), size(F, 2));
			end
			[E_g, A_g, B_g, C_g, C_dot_g, D_g, C_ref_g, D_ref_g, needsstate, usesCasCdot] = this.realization_system(R, K, F, E, A, B, C, C_dot, D, C_ref, D_ref, T);
			if nargout >= 2
				A_c = A_g;
				if nargout >= 3
					B_c = B_g;
					if nargout >= 4
						C_c = C_g;
						if nargout >= 5
							C_dot_c = C_dot;
							if nargout >= 6
								K_c = D_g;
								if nargout >= 7
									C_ref_c = C_ref_g;
									if nargout >= 8
										D_ref_c = D_ref_g;
									end
								end
							end
						end
					end
				end
			end
			if asstruct
				system_c = struct(...
					'E',		E_g,...
					'A',		A_g,...
					'B',		B_g,...
					'C',		C_g,...
					'C_dot',	C_dot_g,...
					'D',		D_g,...
					'C_ref',	C_ref_g,...
					'D_ref',	D_ref_g...
				);
				if asstructwithT && T > 0
					system_c.T = T;
				end
			elseif astf
				if this.isdiscreteT(T)
					if iseye(E_g)
						system_c = tf(ss(A_g, B_g, C_g, D_g, T));
					else
						system_c = tf(dss(A_g, B_g, C_g, D_g, E_g, T));
					end
				else
					if iseye(E_g)
						system_c = tf(ss(A_g, B_g, C_g, D_g));
					else
						system_c = tf(dss(A_g, B_g, C_g, D_g, E_g));
					end
				end
			elseif asss
				if this.isdiscreteT(T)
					if iseye(E_g)
						system_c = ss(A_g, B_g, C_g, D_g, T);
					else
						system_c = dss(A_g, B_g, C_g, D_g, E_g, T);
					end
				else
					if iseye(E_g)
						system_c = ss(A_g, B_g, C_g, D_g);
					else
						system_c = dss(A_g, B_g, C_g, D_g, E_g);
					end
				end
			else
				system_c = E_g;
				A_c = A_g;
				B_c = B_g;
				C_c = C_g;
				C_dot_c = C_dot_g;
				K_c = D_g;
				C_ref_c = C_ref_g;
				D_ref_c = D_ref_g;
			end
		end
	end

	methods(Abstract=true, Access=protected)
		%AMEND_SYSTEM add additional dynamics and partition matrices according to the desired output feedback
		%	Input:
		%		this:		instance
		%		E:			descriptor matrix
		%		A:			system matrix
		%		B:			control matrix
		%		C:			output matrix
		%		C_dot:		derivative output matrix
		%		D:			throughput matrix
		%		T:			sampling time
		%	Output:
		%		E:			descriptor matrix of extended system
		%		A:			system matrix of extended system
		%		B:			control matrix of extended system
		%		C:			output matrix of extended system
		%		C_dot:		derivative output matrix of extended system
		%		D:			throughput matrix of extended system
		[E, A, B, C, C_dot, D] = amend_system(this, E, A, B, C, C_dot, D, T);

		%GAINPATTERN_SYSTEM return gain pattern constraint system for desired output feedback gain matrix
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
		[R_fixed, K_fixed, F_fixed, RKF_fixed, R_bounds, K_bounds, F_bounds, RKF_bounds, R_nonlin] = gainpattern_system(this, E, A, B, C, C_dot, D, C_ref, D_ref, T);

		%GAINPATTERN_PARAMETRIC_SYSTEM return parametric gain matrix for given system
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
		[R_gain, K_gain, F_prefilter] = gainpattern_parametric_system(this, system, A, B, C, C_dot, D, C_ref, D_ref, T);

		%SCALEGAIN_SYSTEM return scaling matrices for given system
		%	Input:
		%		this:		instance
		%		T_x:		state transformation matrix for nominal system
		%		T_u:		control transformation matrix for nominal system
		%		T_Y:		measurement transformation matrix for nominal system
		%		T_y_dot:	derivative measurement transformation matrix for nominal system
		%		T_w:		reference transformation matrix for nominal system
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
		%		T_w:		reference transformation matrix for augmented system
		[T_x, T_u, T_y, T_y_dot, T_w] = scalegain_system(this, T_x, T_u, T_y, T_y_dot, T_w, E, A, B, C, C_dot, D, C_ref, D_ref, T);

		%PREFILTERPATTERN_SYSTEM return prefilter and prefilter pattern constraint system for desired output feedback with given gain matrices
		%	Input:
		%		this:		instance
		%		R:			proportional gain matrix to close loop with
		%		K:			derivative gain matrix to close loop with
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
		%		F:			prefilter matrix for extended system and supplied gain matrices
		%		F_fixed:	indicator matrix for fixed elements of prefilter matrix
		[F, F_fixed] = prefilterpattern_system(this, R, K, E, A, B, C, C_dot, D, C_ref, D_ref, T);

		%GAINPARTITIONING_SYSTEM return partitioning for gain matrix of extended system for output feedback with given gain matrix
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
		[partitionR, partitionF] = gainpartitioning_system(this, R, K, F, E, A, B, C, C_dot, D, C_ref, D_ref, T);

		%REALIZATION_SYSTEM return controller without system for output feedback with given gain matrix and inputs w and y and output u
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
		[E, A, B, C, C_dot, D, C_ref, D_ref, needsstate, usesCasCdot] = realization_system(this, R, K, F, E, A, B, C, C_dot, D, C_ref, D_ref, T);
	end

end
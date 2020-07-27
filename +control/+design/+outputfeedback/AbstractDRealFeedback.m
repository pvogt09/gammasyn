classdef(Abstract) AbstractDRealFeedback < control.design.outputfeedback.OutputFeedback
	%ABSTRACTDREALFEEDBACK abstract class for casting a control system in output feedback form and specify the needed constraints on the resulting gain matrix, when a real D feedback with specified realization poles is needed

	properties(SetAccess=protected)
		% system matrix for realization poles of real D feedback
		realD
	end

	methods
		function [this] = AbstractDRealFeedback(varargin)
			%ABSTRACTDREALFEEDBACK create new real D feedback
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
			this@control.design.outputfeedback.OutputFeedback();
			T = -1;
			if nargin == 1
				if isnumeric(varargin{1})
					if isvector(varargin{1})
						A = diag(varargin{1});
					else
						A = varargin{1};
					end
					T = -1;
				elseif isstruct(varargin{1})
					if isfield(varargin{1}, 'A')
						A = varargin{1}.A;
					elseif isfield(varargin{1}, 'a')
						A = varargin{1}.a;
					else
						if isfield(varargin{1}, 'realD')
							A = varargin{1}.realD;
						else
							error('control:design:outputfeedback:input', 'System must must have field A.');
						end
					end
					if isfield(varargin{1}, 'T')
						T = varargin{1}.T;
					elseif isfield(varargin{1}, 't')
						T = varargin{1}.t;
					elseif isfield(varargin{1}, 'T_s')
						T = varargin{1}.T_s;
					end
				elseif isa(varargin{1}, 'tf') || isa(varargin{1}, 'ss')
					A = ssdata(varargin{1});
					if varargin{1}.Ts > 0
						T = varargin{1}.Ts;
					end
				else
					error('control:design:outputfeedback:input', 'System must be some kind of control system, not a %s.', class(system));
				end
				if size(A, 1) ~= size(A, 2)
					error('control:design:outputfeedback:input', 'System matrix must be square.');
				end
				this.realD = A;
			elseif nargin > 1
				scalar = cellfun(@isscalar, varargin, 'UniformOutput', true);
				numeric = cellfun(@isnumeric, varargin, 'UniformOutput', true);
				if all(scalar & numeric)
					this.realD = diag(cell2mat(varargin));
				elseif all(numeric)
					if nargin > 2
						if size(varargin{2}, 1) ~= size(varargin{2}, 2)
							error('control:design:outputfeedback:input', 'System matrix must be square.');
						end
						this.realD = varargin{2};
						if nargin >= 7
							T = varargin{7};
						end
					elseif nargin == 2
						if size(varargin{1}, 1) ~= size(varargin{1}, 2)
							this.realD = diag(varargin{1});
						else
							this.realD = varargin{1};
						end
						if ~isscalar(varargin{2})
							error('control:design:outputfeedback:input', 'Sampling time must be scalar.');
						end
						T = varargin{2};
					else
						if size(varargin{1}, 1) ~= size(varargin{1}, 2)
							error('control:design:outputfeedback:input', 'System matrix must be square.');
						end
						this.realD = varargin{1};
					end
				else
					if isstruct(varargin{1})
						if nargin >= 2
							T = varargin{2};
						end
						if isfield(varargin{1}, 'A')
							A = varargin{1}.A;
						elseif isfield(varargin{1}, 'a')
							A = varargin{1}.a;
						else
							error('control:design:outputfeedback:input', 'System must must have field A.');
						end
						if isfield(varargin{1}, 'T')
							T = varargin{1}.T;
						elseif isfield(varargin{1}, 't')
							T = varargin{1}.t;
						elseif isfield(varargin{1}, 'T_s')
							T = varargin{1}.T_s;
						end
					elseif isa(varargin{1}, 'tf') || isa(varargin{1}, 'ss')
						if nargin >= 2
							T = varargin{2};
						end
						A = ssdata(varargin{1});
						if varargin{1}.Ts > 0
							T = varargin{1}.Ts;
						end
					else
						error('control:design:outputfeedback:input', 'System must be some kind of control system, not a %s.', class(system));
					end
					if size(A, 1) ~= size(A, 2)
						error('control:design:outputfeedback:input', 'System matrix must be square.');
					end
					this.realD = A;
				end
			else
				this.realD = [];
			end
			if any(isinf(this.realD(:)))
				error('control:design:outputfeedback:input', 'System matrix must be infinite.');
			end
			if ~any(isnan(this.realD(:)))
				if this.isdiscreteT(T)
					ev = eigs(this.realD, [], 1, 'lm');
					if abs(ev) > 1
						error('control:design:outputfeedback:input', 'Realization poles must be stable (inside unit circle).');
					end
				else
					if issymmetric(this.realD)
						ev = eigs(this.realD, [], 1, 'la');
					else
						ev = eigs(this.realD, [], 1, 'lr');
					end
					if real(ev) > 0
						error('control:design:outputfeedback:input', 'Realization poles must be stable (inside the left half plane).');
					end
				end
			end
		end
	end
end
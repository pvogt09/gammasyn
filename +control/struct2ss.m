function [sys] = struct2ss(system, T)
	%STRUCT2SS convert a structure of dynamical systems to ss objects
	%	Input:
	%		system:	systems to convert
	%		T:		sampling time
	%	Output:
	%		sys:	converted systems
	if nargin <= 1
		T = -1;
	end
	if ~isstruct(system)
		if isa(system, 'ss')
			sys = system;
			return;
		elseif isa(system, 'tf')
			sys = system;
			return;
		else
			error('control:struct2ss', 'System must must be of type ''struct''.');
		end
	end
	tempsystem = reshape(system, [], 1);
	s = cell(size(tempsystem, 1), 1);
	parfor ii = 1:size(tempsystem, 1)
		A = [];
		B = [];
		C = [];
		if isfield(tempsystem(ii, 1), 'A')
			A = tempsystem(ii, 1).A;
		elseif isfield(tempsystem(ii, 1), 'a')
			A = tempsystem(ii, 1).a;
		else
			'control:design:gamma:input', 'System must must have field A.'
		end
		if isfield(tempsystem(ii, 1), 'B')
			B = tempsystem(ii, 1).B;
		elseif isfield(tempsystem(ii, 1), 'b')
			B = tempsystem(ii, 1).b;
		else
			'control:design:gamma:input', 'System must must have field B.'
		end
		if isfield(tempsystem(ii, 1), 'C')
			C = tempsystem(ii, 1).C;
		elseif isfield(tempsystem(ii, 1), 'c')
			C = tempsystem(ii, 1).c;
		else
			'control:design:gamma:input', 'System must must have fields C.'
		end
		if isfield(tempsystem(ii, 1), 'D')
			D = tempsystem(ii, 1).D;
		elseif isfield(tempsystem(ii, 1), 'd')
			D = tempsystem(ii, 1).d;
		else
			D = zeros(size(C, 1), size(B, 2));
		end
		if isfield(tempsystem(ii, 1), 'E')
			E = tempsystem(ii, 1).E;
			isdescriptor = true;
		elseif isfield(tempsystem(ii, 1), 'e')
			E = tempsystem(ii, 1).e;
			isdescriptor = true;
		else
			E = eye(size(A));
			isdescriptor = false;
		end
		if isdescriptor
			s{ii, 1} = dss(A, B, C, D, E, T);
		else
			s{ii, 1} = ss(A, B, C, D, T);
		end
	end
	sys = reshape(s, size(system));
	if numel(sys) == 1
		sys = sys{1};
	end
end
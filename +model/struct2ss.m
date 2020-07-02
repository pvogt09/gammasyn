function [system] = struct2ss(s, T)
	%STRUCT2SS convert structure of systems to cell array of ss objects
	%	Input:
	%		s:		structure to convert
	%		T:		sampling time to use, if the system does not specify a sampling time
	%	Output:
	%		system:	cell array of systems
	if nargin <= 1
		T = -1;
	end
	if ~isstruct(s)
		error('model:input', 'System must be supplied as structure, not as ''%s''.', class(s));
	end
	if ~isscalar(T) || ~isnumeric(T)
		error('model:input', 'Sampling time must be a numeric scalar.');
	end
	temp = reshape(s, [], 1);
	systems = cell(size(temp));
	parfor ii = 1:size(temp, 1)
		A = []; %#ok<NASGU> prevent parfor warning about reusing variables
		B = []; %#ok<NASGU> prevent parfor warning about reusing variables
		C = []; %#ok<NASGU> prevent parfor warning about reusing variables
		D = []; %#ok<NASGU> prevent parfor warning about reusing variables
		E = []; %#ok<NASGU> prevent parfor warning about reusing variables
		if isfield(temp(ii, 1), 'A')
			A = temp(ii, 1).A;
		elseif isfield(temp(ii, 1), 'a')
			A = temp(ii, 1).a;
		else
			error('model:input', 'System must must have field A.');
		end
		if isfield(temp(ii, 1), 'E')
			E = temp(ii, 1).E;
		elseif isfield(temp(ii, 1), 'e')
			E = temp(ii, 1).e;
		else
			E = eye(size(A, 1));
		end
		if all(size(E) == 0)
			E = eye(size(A, 1));
		end
		if isfield(temp(ii, 1), 'B')
			B = temp(ii, 1).B;
		elseif isfield(temp(ii, 1), 'b')
			B = temp(ii, 1).b;
		else
			error('model:input', 'System must must have field B.');
		end
		if isfield(temp(ii, 1), 'C')
			C = temp(ii, 1).C;
		elseif isfield(temp(ii, 1), 'c')
			C = temp(ii, 1).c;
		else
			error('model:input', 'System must must have field C.');
		end
		if isfield(temp(ii, 1), 'D')
			D = temp(ii, 1).D;
		elseif isfield(temp(ii, 1), 'd')
			D = temp(ii, 1).d;
		else
			error('model:input', 'System must must have field D.');
		end
		if isfield(temp(ii, 1), 'T')
			T_s = temp(ii, 1).T;
		elseif isfield(temp(ii, 1), 't')
			T_s = temp(ii, 1).t;
		elseif isfield(temp(ii, 1), 'T_s')
			T_s = temp(ii, 1).T_s;
		else
			T_s = T;
		end
		if T_s > 0
			systems{ii, 1} = dss(A, B, C, D, E, T_s);
		else
			systems{ii, 1} = dss(A, B, C, D, E);
		end
	end
	system = reshape(systems, size(s));
	if numel(system) == 1
		system = system{1};
	end
end
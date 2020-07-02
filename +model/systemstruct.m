function [system] = systemstruct(E, A, B, C, C_dot, D)
	%SYSTEMSTRUCT convert system matrices to system structure
	%	Input:
	%		E:		descriptor matrix of system
	%		A:		system matrix of system
	%		B:		control matrix of system
	%		C:		output matrix of system
	%		C_dot:	derivative output matrix of system
	%		D:		throughput matrix of system
	%	Output:
	%		system:	structure with system matrices
	n = size(A, 1);
	p = size(B, 2);
	q = size(C, 1);
	if size(A, 2) ~= n
		error('model:systemstruct', 'System matrix must be square.');
	end
	szA = size(A);
	if all(size(E) == 0)% allow [] but not other types of empty matrices
		E = repmat(eye(n), [1, 1, szA(3:end)]);
	end
	if size(E, 1) ~= n
		error('model:systemstruct', 'Descriptor matrix must have %d columns, not %d.', n, size(E, 1));
	end
	if size(E, 2) ~= n
		error('model:systemstruct', 'Descriptor matrix must have %d rows, not %d.', n, size(E, 2));
	end
	if size(B, 1) ~= n
		error('model:systemstruct', 'Control matrix must have %d columns, not %d.', n, size(B, 1));
	end
	if size(C, 2) ~= n
		error('model:systemstruct', 'Output matrix must have %d rows, not %d.', n, size(C, 2));
	end
	if nargin <= 4
		C_dot = zeros([0, n, szA(3:end)]);
	end
	if nargin <= 4
		D = zeros([q, p, szA(3:end)]);
	end
	if all(size(C_dot) == 0)% allow [] but not other types of empty matrices
		C_dot = zeros([0, n, szA(3:end)]);
	end
	if all(size(D) == 0)% allow [] but not other types of empty matrices
		D = zeros([q, p, szA(3:end)]);
	end
	if size(C_dot, 2) ~= n
		error('model:systemstruct', 'Derivative output matrix must have %d rows, not %d.', n, size(C_dot, 2));
	end
	if size(D, 2) ~= p
		error('model:systemstruct', 'Throughput matrix must have %d columns, not %d.', p, size(D, 2));
	end
	if size(D, 1) ~= q
		error('model:systemstruct', 'Throughput matrix must have %d rows, not %d.', q, size(D, 1));
	end
	dimA = ndims(A);
	if dimA ~= ndims(B) || dimA ~= ndims(E) || dimA ~= ndims(C) || dimA ~= ndims(C_dot) || dimA ~= ndims(D)
		error('model:systemstruct', 'All matrices must have same number of dimensions.');
	end
	szE = size(E);
	szB = size(B);
	szC = size(C);
	szC_dot = size(C_dot);
	szD = size(D);
	if any(szA(3:end) ~= szB(3:end)) || any(szA(3:end) ~= szE(3:end)) || any(szA(3:end) ~= szC(3:end)) || any(szA(3:end) ~= szC_dot(3:end)) || any(szA(3:end) ~= szD(3:end))
		error('model:systemstruct', 'Sizes of all matrices in higher dimensions must be equal.');
	end
	if dimA > 2
		system = struct(...
			'E',		{},...
			'A',		{},...
			'B',		{},...
			'C',		{},...
			'C_dot',	{},...
			'D',		{}...
		);
		system(prod(szA(3:end)), 1) = struct(...
			'E',		NaN,...
			'A',		NaN,...
			'B',		NaN,...
			'C',		NaN,...
			'C_dot',	NaN,...
			'D',		NaN...
		);
		rE = reshape(E, [szE(1), szE(2), prod(szE(3:end))]);
		rA = reshape(A, [szA(1), szA(2), prod(szA(3:end))]);
		rB = reshape(B, [szB(1), szB(2), prod(szB(3:end))]);
		rC = reshape(C, [szC(1), szC(2), prod(szC(3:end))]);
		rC_dot = reshape(C_dot, [szC_dot(1), szC_dot(2), prod(szC_dot(3:end))]);
		rD = reshape(D, [szD(1), szD(2), prod(szD(3:end))]);
		parfor ii = 1:size(rE, 3)
			system(ii, 1) = struct(...
				'E',		rE(:, :, ii),...
				'A',		rA(:, :, ii),...
				'B',		rB(:, :, ii),...
				'C',		rC(:, :, ii),...
				'C_dot',	rC_dot(:, :, ii),...
				'D',		rD(:, :, ii)...
			);
		end
		if length(szA) == 3
			system = reshape(system, [szA(3:end), 1]);
		else
			system = reshape(system, szA(3:end));
		end
	else
		system = struct(...
			'E',		E,...
			'A',		A,...
			'B',		B,...
			'C',		C,...
			'C_dot',	C_dot,...
			'D',		D...
		);
	end
end
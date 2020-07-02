function [is] = isequalstruct(structa, structb, tol)
	%ISEQUALSTRUCT return if two strutures are equal
	%	Input:
	%		structa:	first structure to compare
	%		structb:	second structure to compare
	%		tol:		tolerance for equality
	%	Output:
	%		is:			true, if the structures are equal within the specified tolerance, else false
	if nargin <= 2
		tol = 0;
	end
	if ~isstruct(structa) || ~isstruct(structb)
		error('struct:equal', 'Undefined function or variable for arguments of type ''%s'' and ''%s''.', class(structa), class(structb));
	end
	if ndims(structa) ~= ndims(structb)
		is = false;
		return;
	end
	szA = size(structa);
	szB = size(structb);
	if ~all(szA == szB)
		is = false;
		return;
	end
	fieldnamesa = fieldnames(structa);
	fieldnamesb = fieldnames(structb);
	if length(fieldnamesa) ~= length(fieldnamesb)
		is = false;
		return;
	end
	if ~all(ismember(fieldnamesa, fieldnamesb)) || ~all(ismember(fieldnamesb, fieldnamesa))
		is = false;
		return;
	end
	if ~isscalar(structa)
		a = reshape(structa, prod(szA), 1);
	else
		a = structa;
	end
	if ~isscalar(structb)
		b = reshape(structb, prod(szB), 1);
	else
		b = structb;
	end
	is = false(size(a));
	parfor ii = 1:size(a, 1)
		currstructa = a(ii);
		currstructb = b(ii);
		equal = false(size(fieldnamesa, 1), 1);
		for jj = 1:size(fieldnamesa, 1)
			valuea = currstructa.(fieldnamesa{jj, 1});
			valueb = currstructb.(fieldnamesa{jj, 1});
			isstructa = isstruct(valuea);
			isstructb = isstruct(valueb);
			if isstructa && isstructb
				temp = isequalstruct(valuea, valueb, tol);
				equal(jj, 1) = all(temp(:));
			else
				if ~isstructa && ~isstructb
					if tol ~= 0
						equal(jj, 1) = isequalntol(valuea, valueb, true, tol);
					else
						equal(jj, 1) = isequaln(valuea, valueb);
					end
				else
					continue;
				end
			end
		end
		is(ii, 1) = all(equal);
	end
end
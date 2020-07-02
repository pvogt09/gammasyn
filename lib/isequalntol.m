function [is] = isequalntol(a, b, nanequal, tol)
	%ISEQUALNTOL return if two variables are equal within a certain tolerance
	%	Input:
	%		a:			first variable to compare
	%		b:			second variable to compare
	%		nanequal:	indicator if NaN is equal
	%		tol:		tolerance for comparison
	%	Output:
	%		is:			true, if the variables are equal within the wolerance else false
	if nargin <= 2
		nanequal = false;
	end
	if nargin <= 3
		tol = 0;
	end
	if isfunctionhandle(a) || isfunctionhandle(b)
		is = isfunctionhandle(a) && isfunctionhandle(s2);
		if is
			is = nargin(a) == nargin(b) && nargout(a) == nargout(b);
		end
	elseif ischarstring(a) || ischarstring(b)
		res = strcmp(a, b);
		is = all(res(:));
	elseif isa(a, 'Diffable') || isa(b, 'Diffable')
		if isa(a, 'Diffable')
			[diff] = a.diffstruct(b, nanequal, ignoremissing);
		else
			[diff] = b.diffstruct(a, nanequal, ignoremissing);
		end
		is = isempty(diff) && isempty(fieldnames(diff));
	elseif iscell(a) || iscell(b)
		if nanequal
			is = isequalcell(a, b, nanequal, tol);
		else
			is = isequalcell(a, b, nanequal, tol);
		end
	elseif isobject(a) || isobject(b)
		if nanequal
			is = isequaln(a, b);
		else
			is = isequal(a, b);
		end
	else
		if isequal(size(a), size(b))
			if tol ~= 0
				res = abs(a - b) < tol;
			else
				res = a == b;
			end
			if nanequal
				res(isnan(a) & isnan(b)) = true;
			end
			is = all(res(:));
		else
			is = false;
		end
	end
end

function [is] = isequalcell(a, b, nanequal, tol)
	%ISEQUALCELL return if two cell variables are equal within a certain tolerance
	%	Input:
	%		a:			first variable to compare
	%		b:			second variable to compare
	%		nanequal:	indicator if NaN is equal
	%		tol:		tolerance for comparison
	%	Output:
	%		is:			true, if the variables are equal within the wolerance else false
	is = iscell(a) && iscell(b) && ndims(a) == ndims(b) && all(size(a) == size(b));
	if is
		for ii = 1:length(a) %#ok<FORPF> no parfor because of early exit
			if ~isequalntol(a{ii}, b{ii}, nanequal, tol)
				is = false;
				return;
			end
		end
	end
end
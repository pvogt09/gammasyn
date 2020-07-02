function [is] = isequation(eq, type)
	%ISEQUATION return whether a symbolic expression is an equation of the specified type
	%	Input:
	%		eq:		expression to check
	%		type:	type of equation to check for, can be '==', '<', '<=', '>', '>=' and 'any'
	%	Output:
	%		is:		true, when the element of the expression is an eaution of the specified type, else false
	if ~isa(eq, 'sym')
		error('sym:isequation:input', 'Expression must be of type ''sym'', not ''%s''.', class(eq));
	end
	if nargin <= 1
		type = 'any';
	end
	if ~ischar(type) && ~iscellstr(type)
		error('sym:isequation:input', 'Type must be of type ''char'', not ''%s''.', class(type));
	end
	if iscellstr(type)
		type = reshape(type, [], 1);
		checkeq = false(5, 1);
		for ii = 1:size(type, 1)
			if ~any(strcmpi(type{ii, 1}, {'==', '<', '<=', '>', '>=', 'any'}))
				error('sym:isequation:input', 'Type must be one of ''=='', ''<'', ''<='', ''>'', ''>='' or ''any''.');
			end
			if strcmpi(type{ii, 1}, 'any')
				checkeq = true(5, 1);
			else
				switch lower(type{ii, 1})
					case '=='
						checkeq(1, 1) = true;
					case '<'
						checkeq(2, 1) = true;
					case '<='
						checkeq(3, 1) = true;
					case '>'
						checkeq(4, 1) = true;
					case '>='
						checkeq(5, 1) = true;
					otherwise
						error('sym:isequation:input', 'Type must be one of ''=='', ''<'', ''<='', ''>'', ''>='' or ''any''.');
				end
			end
		end
	else
		if ~any(strcmpi(type, {'==', '<', '<=', '>', '>=', 'any'}))
			error('sym:isequation:input', 'Type must be one of ''=='', ''<'', ''<='', ''>'', ''>='' or ''any''.');
		end
		if strcmpi(type, 'any')
			checkeq = true(5, 1);
		else
			checkeq = false(5, 1);
			switch lower(type)
				case '=='
					checkeq(1, 1) = true;
				case '<'
					checkeq(2, 1) = true;
				case '<='
					checkeq(3, 1) = true;
				case '>'
					checkeq(4, 1) = true;
				case '>='
					checkeq(5, 1) = true;
				otherwise
					error('sym:isequation:input', 'Type must be one of ''=='', ''<'', ''<='', ''>'', ''>='' or ''any''.');
			end
		end
	end
	equation = reshape(eq, [], 1);
	is = false(size(equation));
	for ii = 1:size(equation, 1)
		eqstr = char(equation(ii, 1));
		eqfound = strfind(eqstr, '==');
		leqfound = strfind(eqstr, '<=');
		lefound = strfind(eqstr, '<');
		geqfound = strfind(eqstr, '>=');
		gefound = strfind(eqstr, '>');
		if checkeq(1, 1) && ~isempty(eqfound)
			is(ii, 1) = true;
		end
		if checkeq(3, 1) && ~isempty(leqfound)
			is(ii, 1) = true;
		end
		if checkeq(2, 1) && ~isempty(lefound)
			is(ii, 1) = true;
		end
		if checkeq(5, 1) && ~isempty(geqfound)
			is(ii, 1) = true;
		end
		if checkeq(4, 1) && ~isempty(gefound)
			is(ii, 1) = true;
		end
	end
	is = reshape(is, size(eq));
end
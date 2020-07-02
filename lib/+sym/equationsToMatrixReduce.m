function [A, b, Aeq, beq] = equationsToMatrixReduce(eqns, vars)
	%EQUATIONSTOMATRIXREDUCE reutrn A x <= b and A x == b form for a symbolic system of equations
	%	Input:
	%		eqns:	system of equations
	%		vars:	variables to use as independent variables, if not supplied, symvar(eqns) is used
	%	Output:
	%		A:		matrix of inequalities
	%		b:		upper bound for inequalities
	%		Aeq:	matrix of equalities
	%		beq:	upper bound for equalities
	if ~isa(eqns, 'sym')
		error('equationsToMatrixReduce:input', 'Equations must be symbolic expressions, not ''%s''.', class(eqns));
	end
	if nargin <= 1
		vars = reshape(symvar(eqns), [], 1);
	else
		temp = reshape(vars, [], 1);
		for ii = 1:size(temp, 1) %#ok<FORPF> no parfor because execution is stopped anyway, if an error occurs
			eq = char(temp(ii, 1));
			eqfound = strfind(eq, '=');
			leqfound = strfind(eq, '<=');
			lefound = strfind(eq, '<');
			geqfound = strfind(eq, '>=');
			gefound = strfind(eq, '>');
			if ~isempty(eqfound)
				error('equationsToMatrixReduce:input', 'Symbolic variables must not be equations.');
			end
			if ~isempty(leqfound)
				error('equationsToMatrixReduce:input', 'Symbolic variables must not be equations.');
			end
			if ~isempty(lefound)
				error('equationsToMatrixReduce:input', 'Symbolic variables must not be equations.');
			end
			if ~isempty(geqfound)
				error('equationsToMatrixReduce:input', 'Symbolic variables must not be equations.');
			end
			if ~isempty(gefound)
				error('equationsToMatrixReduce:input', 'Symbolic variables must not be equations.');
			end
		end
		vars = reshape(symvar(vars), [], 1);
	end
	if ~isa(vars, 'sym')
		error('equationsToMatrixReduce:input', 'Variables must be symbolic variables, not ''%s''.', class(vars));
	end
	allvars = [
		reshape(symvar(eqns), [], 1);
		vars
	];
	equations = reshape(eqns, [], 1);
	leq = false(size(equations, 1), 1);
	geq = false(size(equations, 1), 1);
	eeq = false(size(equations, 1), 1);
	for ii = 1:size(equations)
		eq = char(equations(ii, 1));
		leqfound = strfind(eq, '<=');
		lefound = strfind(eq, '<');
		geqfound = strfind(eq, '>=');
		gefound = strfind(eq, '>');
		eqfound = strfind(eq, '==');
		if ~isempty(leqfound)
			if numel(leqfound) > 1
				error('equationsToMatrixReduce:input', 'Symbolic eqation %d contains multiple ''<=''.', ii);
			end
			if numel(lefound) > 1
				error('equationsToMatrixReduce:input', 'Symbolic eqation %d contains ''<='' and ''<''.', ii);
			end
			if ~isempty(geqfound)
				error('equationsToMatrixReduce:input', 'Symbolic eqation %d contains ''<='' and ''>=''.', ii);
			end
			if ~isempty(gefound)
				error('equationsToMatrixReduce:input', 'Symbolic eqation %d contains ''<='' and ''>''.', ii);
			end
			if ~isempty(eqfound)
				error('equationsToMatrixReduce:input', 'Symbolic eqation %d contains ''<='' and ''==''.', ii);
			end
			leq(ii, 1) = true;
		end
		if ~isempty(lefound)
			if numel(lefound) > 1
				error('equationsToMatrixReduce:input', 'Symbolic eqation %d contains multiple ''<''.', ii);
			end
			if numel(leqfound) > 1
				error('equationsToMatrixReduce:input', 'Symbolic eqation %d contains ''<'' and ''<=''.', ii);
			end
			if ~isempty(geqfound)
				error('equationsToMatrixReduce:input', 'Symbolic eqation %d contains ''<'' and ''>=''.', ii);
			end
			if ~isempty(gefound)
				error('equationsToMatrixReduce:input', 'Symbolic eqation %d contains ''<'' and ''>''.', ii);
			end
			if numel(eqfound) > 1
				error('equationsToMatrixReduce:input', 'Symbolic eqation %d contains ''<'' and ''==''.', ii);
			end
			leq(ii, 1) = true;
		end
		if ~isempty(geqfound)
			if numel(geqfound) > 1
				error('equationsToMatrixReduce:input', 'Symbolic eqation %d contains multiple ''>=''.', ii);
			end
			if ~isempty(lefound)
				error('equationsToMatrixReduce:input', 'Symbolic eqation %d contains ''>='' and ''<''.', ii);
			end
			if ~isempty(leqfound)
				error('equationsToMatrixReduce:input', 'Symbolic eqation %d contains ''>='' and ''<=''.', ii);
			end
			if numel(gefound) > 1
				error('equationsToMatrixReduce:input', 'Symbolic eqation %d contains ''>='' and ''>''.', ii);
			end
			if numel(eqfound) > 1
				error('equationsToMatrixReduce:input', 'Symbolic eqation %d contains ''<'' and ''==''.', ii);
			end
			geq(ii, 1) = true;
		end
		if ~isempty(gefound)
			if numel(gefound) > 1
				error('equationsToMatrixReduce:input', 'Symbolic eqation %d contains multiple ''>''.', ii);
			end
			if ~isempty(lefound)
				error('equationsToMatrixReduce:input', 'Symbolic eqation %d contains ''>'' and ''<''.', ii);
			end
			if numel(geqfound) > 1
				error('equationsToMatrixReduce:input', 'Symbolic eqation %d contains ''>'' and ''>=''.', ii);
			end
			if ~isempty(leqfound)
				error('equationsToMatrixReduce:input', 'Symbolic eqation %d contains ''>'' and ''<=''.', ii);
			end
			if numel(eqfound) > 1
				error('equationsToMatrixReduce:input', 'Symbolic eqation %d contains ''<'' and ''==''.', ii);
			end
			geq(ii, 1) = true;
		end
		if ~isempty(eqfound)
			if numel(eqfound) > 1
				error('equationsToMatrixReduce:input', 'Symbolic eqation %d contains multiple ''==''.', ii);
			end
			if numel(leqfound) > 1
				error('equationsToMatrixReduce:input', 'Symbolic eqation %d contains ''=='' and ''<=''.', ii);
			end
			if numel(lefound) > 1
				error('equationsToMatrixReduce:input', 'Symbolic eqation %d contains ''=='' and ''<''.', ii);
			end
			if ~isempty(geqfound)
				error('equationsToMatrixReduce:input', 'Symbolic eqation %d contains ''=='' and ''>=''.', ii);
			end
			if ~isempty(gefound)
				error('equationsToMatrixReduce:input', 'Symbolic eqation %d contains ''=='' and ''>''.', ii);
			end
			eeq(ii, 1) = true;
		end
		if isempty(leqfound) && isempty(lefound) && isempty(geqfound) && isempty(gefound) && isempty(eqfound)
			% TODO: is '=' the same as '==' in symbolic equations?
			error();
		end
		temp = children(equations(ii, 1));
		if numel(temp) ~= 2
			error('equationsToMatrixReduce:input', 'Symbolic expression must be an equation.');
		end
		coefficient = jacobian(temp(1), allvars);
		if ~isempty(symvar(coefficient))
			error('equationsToMatrixReduce:input', 'Symbolic expression must be linear, but left hand side of element %d is nonlinear.', ii);
		end
		coefficient = gradient(temp(2), allvars);
		if ~isempty(symvar(coefficient))
			error('equationsToMatrixReduce:input', 'Symbolic expression must be linear, but right hand side of element %d is nonlinear.', ii);
		end
	end
	if ~(eeq | leq | geq)
		error('equationsToMatrixReduce:input', 'All equations must be equality or inequality equations.');
	end
	equation_system = eqns(eeq, 1);
	inequalities = eqns(leq | geq, 1);
	[A, b] = equationsToMatrix(equation_system, vars);
	currentsymvars = symvar(b);
	lastsymvars = currentsymvars;
	while (~isempty(currentsymvars))
		replaceidx = [];
		replacevar = [];
		for ii = 1:size(b, 1)
			if ~isempty(symvar(b(ii, 1)))
				replaceidx = ii;
				replacevar = b(ii, 1);
				break;
			end
		end
		if ~isempty(replaceidx)
			equation_system = subs(equation_system, replacevar, A(replaceidx, :)*reshape(vars, [], 1));
			[A, b] = equationsToMatrix(equation_system, vars);
			inequalities = subs(inequalities, replacevar, A(replaceidx, :)*vars);
			currentsymvars = symvar(b);
			if ~isequal(lastsymvars, currentsymvars)
				lastsymvars = currentsymvars;
			else
				error('equationsToMatrixReduce:input', 'Symbolic linear proportional gain matrix constraints can not be reduced further to conain not symbolic expressions.');
			end
		else
			error('equationsToMatrixReduce:input', 'Symbolic linear proportional gain matrix constraints can not be reduced further to conain not symbolic expressions.');
		end
	end
	Aeq = A;
	beq = b;
	[A, b] = equationsToMatrixIneq(inequalities, vars);
end
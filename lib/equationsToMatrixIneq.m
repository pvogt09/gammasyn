function [A, b] = equationsToMatrixIneq(eqns, vars)
	%EQUATIONSTOMATRIXINEQ reutrn A x <= b form for a symbolic system of equations
	%	Input:
	%		eqns:	system of equations, if no relation symbol is used <= 0 is assumed
	%		vars:	variables to use as independent variables, if not supplied, symvar(eqns) is used
	%	Output:
	%		A:		matrix of inequalities
	%		b:		upper bound for inequalities
	if ~isa(eqns, 'sym')
		error('equationsToMatrixIneq:input', 'Equations must be symbolic expressions, not ''%s''.', class(eqns));
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
				error('equationsToMatrixIneq:input', 'Symbolic variables must not be equations.');
			end
			if ~isempty(leqfound)
				error('equationsToMatrixIneq:input', 'Symbolic variables must not be equations.');
			end
			if ~isempty(lefound)
				error('equationsToMatrixIneq:input', 'Symbolic variables must not be equations.');
			end
			if ~isempty(geqfound)
				error('equationsToMatrixIneq:input', 'Symbolic variables must not be equations.');
			end
			if ~isempty(gefound)
				error('equationsToMatrixIneq:input', 'Symbolic variables must not be equations.');
			end
		end
		vars = reshape(symvar(vars), [], 1);
	end
	if ~isa(vars, 'sym')
		error('equationsToMatrixIneq:input', 'Variables must be symbolic variables, not ''%s''.', class(vars));
	end
	allvars = [
		reshape(symvar(eqns), [], 1);
		vars
	];
	equations = reshape(eqns, [], 1);
	A = zeros(size(equations, 1), size(vars, 1));
	b = sym(zeros(size(equations, 1), 1));
	leq = true(size(equations, 1), 1);
	for ii = 1:size(equations)
		eq = char(equations(ii, 1));
		if ~isempty(strfind(eq, '=='))
			error('equationsToMatrixIneq:input', 'Symbolic inequality is invalid because element %d contains ''==''.', ii);
		end
		leqfound = strfind(eq, '<=');
		lefound = strfind(eq, '<');
		geqfound = strfind(eq, '>=');
		gefound = strfind(eq, '>');
		if ~isempty(leqfound)
			if numel(leqfound) > 1
				error('equationsToMatrixIneq:input', 'Symbolic eqation %d contains multiple ''<=''.', ii);
			end
			if numel(lefound) > 1
				error('equationsToMatrixIneq:input', 'Symbolic eqation %d contains ''<='' and ''<''.', ii);
			end
			if ~isempty(geqfound)
				error('equationsToMatrixIneq:input', 'Symbolic eqation %d contains ''<='' and ''>=''.', ii);
			end
			if ~isempty(gefound)
				error('equationsToMatrixIneq:input', 'Symbolic eqation %d contains ''<='' and ''>''.', ii);
			end
			leq(ii, 1) = true;
		end
		if ~isempty(lefound)
			if numel(lefound) > 1
				error('equationsToMatrixIneq:input', 'Symbolic eqation %d contains multiple ''<''.', ii);
			end
			if numel(leqfound) > 1
				error('equationsToMatrixIneq:input', 'Symbolic eqation %d contains ''<'' and ''<=''.', ii);
			end
			if ~isempty(geqfound)
				error('equationsToMatrixIneq:input', 'Symbolic eqation %d contains ''<'' and ''>=''.', ii);
			end
			if ~isempty(gefound)
				error('equationsToMatrixIneq:input', 'Symbolic eqation %d contains ''<'' and ''>''.', ii);
			end
			leq(ii, 1) = true;
		end
		if ~isempty(geqfound)
			if numel(geqfound) > 1
				error('equationsToMatrixIneq:input', 'Symbolic eqation %d contains multiple ''>=''.', ii);
			end
			if ~isempty(lefound)
				error('equationsToMatrixIneq:input', 'Symbolic eqation %d contains ''>='' and ''<''.', ii);
			end
			if ~isempty(leqfound)
				error('equationsToMatrixIneq:input', 'Symbolic eqation %d contains ''>='' and ''<=''.', ii);
			end
			if numel(gefound) > 1
				error('equationsToMatrixIneq:input', 'Symbolic eqation %d contains ''>='' and ''>''.', ii);
			end
			leq(ii, 1) = false;
		end
		if ~isempty(gefound)
			if numel(gefound) > 1
				error('equationsToMatrixIneq:input', 'Symbolic eqation %d contains multiple ''>''.', ii);
			end
			if ~isempty(lefound)
				error('equationsToMatrixIneq:input', 'Symbolic eqation %d contains ''>'' and ''<''.', ii);
			end
			if numel(geqfound) > 1
				error('equationsToMatrixIneq:input', 'Symbolic eqation %d contains ''>'' and ''>=''.', ii);
			end
			if ~isempty(leqfound)
				error('equationsToMatrixIneq:input', 'Symbolic eqation %d contains ''>'' and ''<=''.', ii);
			end
			leq(ii, 1) = false;
		end
		if isempty(leqfound) && isempty(lefound) && isempty(geqfound) && isempty(gefound)
			equations(ii, 1) = equation(ii, 1) <= 0;
			leq(ii, 1) = true;
		end
		temp = children(equations(ii, 1));
		if numel(temp) ~= 2
			error('equationsToMatrixIneq:input', 'Symbolic expression must be an equation.');
		end
		coefficient = jacobian(temp(1), allvars);
		if ~isempty(symvar(coefficient))
			error('control:design:gamma:dimension', 'Symbolic expression must be linear, but left hand side of element %d is nonlinear.', ii);
		end
		coefficient = gradient(temp(2), allvars);
		if ~isempty(symvar(coefficient))
			error('control:design:gamma:dimension', 'Symbolic expression must be linear, but right hand side of element %d is nonlinear.', ii);
		end
		A1 = double(jacobian(temp(1), vars));
		A2 = double(jacobian(temp(2), vars));
		A(ii, :) = A1 - A2;
		b(ii, 1) = -temp(1) + A1*vars + temp(2) - A2*vars;
		if ~leq(ii, 1)
			A(ii, 1) = -A(ii, 1);
			b(ii, 1) = -b(ii, 1);
		end
	end
end
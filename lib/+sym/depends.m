function [dep] = depends(expression, var)
	%DEPENDS return if a symboic expression depends on the spedified variables
	%	Input:
	%		expression:	symbolic expression to test
	%		var:		variables to test dependence for
	%	Output:
	%		dep:		indicator, which element of the expression depends on which variable
	if ~isa(expression, 'sym')
		error('sym:depends', 'Expression must be of type ''sym'', not ''%s''.', class(expression));
	end
	if ~isa(var, 'sym')
		error('sym:depends', 'Variables must be of type ''sym'', not ''%s''.', class(var));
	end
	allvars = reshape(var, [], 1);
	vars = reshape(symvar(var), [], 1);
	if size(allvars, 1) ~= size(vars, 1)
		error('sym:depends', 'Variables must be variables, not expressions.');
	end
	jac = jacobian(allvars, vars);
	graddepends = false(size(jac));
	parfor ii = 1:size(jac, 1)
		grad = false(1, size(jac, 2));
		for jj = 1:size(jac, 2)
			grad(1, jj) = isempty(symvar(jac(ii, jj))) && double(jac(ii, jj)) == 1;
		end
		graddepends(ii, :) = grad;
	end
	if any(sum(graddepends, 2)) > 1
		error('sym:depends', 'Variables must be variables, not expressions.');
	end
	expr = reshape(expression, [], 1);
	dependsstrvars = cell(size(vars));
	parfor ii = 1:size(vars, 1)
		dependsstrvars{ii, 1} = char(vars(ii, 1));
	end
	dep = false(size(expr, 1), size(vars, 1));
	parfor ii = 1:size(expr, 1)
		symvars = reshape(symvar(expr(ii, 1)), [], 1);
		strvars = cell(size(symvars));
		for jj = 1:size(symvars, 1)
			strvars{jj, 1} = char(symvars(jj, 1));
		end
		dep(ii, :) = ismember(dependsstrvars, strvars);
	end
	if ~isscalar(expression)
		dep = reshape(dep, [size(expression), size(vars, 1)]);
	end
end
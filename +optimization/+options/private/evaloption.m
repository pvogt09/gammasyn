function [validvalue, valid, errmsg, errid] = evaloption(field, value, numvar, numineq, numeq, numbounds)
	%EVALOPTION convert a char property to a numerical value
	%	Input:
	%		this:			instance
	%		field:			name of the option to convert
	%		value:			value of the option to convert
	%		numvar:			number of optimization variables
	%		numineq:		number of inequality variables
	%		numeq:			number of equality variables
	%		numbounds:		number of bounds
	%	Output:
	%		validvalue:		interpreted value
	%		valid:			indicator, if value is valid
	%		errmsg:			error message, if value is not valid
	%		errid:			error identifier, if value is not valid
	if nargin <= 5
		numberOfBounds = 1; %#ok<NASGU> used in eval
		numberofbounds = 1; %#ok<NASGU> used in eval
	else
		numberOfBounds = numbounds; %#ok<NASGU> used in eval
		numberofbounds = numbounds; %#ok<NASGU> used in eval
	end
	if nargin <= 4
		numberOfEqualities = 3; %#ok<NASGU> used in eval
		numberofequalities = 3; %#ok<NASGU> used in eval
	else
		numberOfEqualities = numeq; %#ok<NASGU> used in eval
		numberofequalities = numeq; %#ok<NASGU> used in eval
	end
	if nargin <= 3
		numberOfInequalities = 3; %#ok<NASGU> used in eval
		numberofinequalities = 3; %#ok<NASGU> used in eval
	else
		numberOfInequalities = numineq; %#ok<NASGU> used in eval
		numberofinequalities = numineq; %#ok<NASGU> used in eval
	end
	if nargin <= 2
		numberOfVariables = 8; %#ok<NASGU> used in eval
		numberofvariables = 8; %#ok<NASGU> used in eval
	else
		numberOfVariables = numvar; %#ok<NASGU> used in eval
		numberofvariables = numvar; %#ok<NASGU> used in eval
	end
	if ischar(value)
		out = regexpi(value, 'sparse\((?<argument>.*)\)', 'names');
		if ~isempty(out)
			value = out.argument;
		end
		out = regexpi(value, '(max|min)\(\d+,\s*(?<argument>.*)\)', 'names');
		if ~isempty(out)
			value = out.argument;
		end
		out = regexpi(value, '(max|min)\(\s*(?<argument>.*),\s*\d+\)', 'names');
		if ~isempty(out)
			value = out.argument;
		end
		regexineq = '(\s*(\+|-)?\s*(\d+\.?\d*(e(\+|-)\d+)?\.?(\*|/))*\s*numberOfInequalities\s*(\.?(\*|/)(\d+\.?\d*(e(\+|-)\d+)?))*)*';
		regexeq = '(\s*(\+|-)?\s*(\d+\.?\d*(e(\+|-)\d+)?\.?(\*|/))*\s*numberOfEqualities\s*(\.?(\*|/)(\d+\.?\d*(e(\+|-)\d+)?))*)*';
		regexvar = '(\s*(\+|-)?\s*(\d+\.?\d*(e(\+|-)\d+)?\.?(\*|/))*\s*numberOfVariables\s*(\.?(\*|/)(\d+\.?\d*(e(\+|-)\d+)?))*)*';
		vartimes1 = regexpi(deblank(value), [
			'^\s*(\d+\.?\d*(e(\+|-)\d+)?\.?\*)*((\((',...
			'(', regexineq, '\s*(\+|-)\s*', regexeq, '\s*(\+|-)\s*', regexvar, ')|',...
			'(', regexvar, '\s*(\+|-)\s*', regexineq, '\s*(\+|-)\s*', regexeq, ')|',...
			'(', regexeq, '\s*(\+|-)\s*', regexvar, '\s*(\+|-)\s*', regexineq, ')|',...
			'(', regexineq, '\s*(\+|-)\s*', regexvar, '\s*(\+|-)\s*', regexeq, ')|',...
			'(', regexeq, '\s*(\+|-)\s*', regexineq, '\s*(\+|-)\s*', regexvar, ')|',...
			'(', regexvar, '\s*(\+|-)\s*', regexeq, '\s*(\+|-)\s*', regexineq, ')',...
			')\))|((',...
			'(', regexineq, '\s*(\+|-)\s*', regexeq, '\s*(\+|-)\s*', regexvar, ')|',...
			'(', regexvar, '\s*(\+|-)\s*', regexineq, '\s*(\+|-)\s*', regexeq, ')|',...
			'(', regexeq, '\s*(\+|-)\s*', regexvar, '\s*(\+|-)\s*', regexineq, ')|',...
			'(', regexineq, '\s*(\+|-)\s*', regexvar, '\s*(\+|-)\s*', regexeq, ')|',...
			'(', regexeq, '\s*(\+|-)\s*', regexineq, '\s*(\+|-)\s*', regexvar, ')|',...
			'(', regexvar, '\s*(\+|-)\s*', regexeq, '\s*(\+|-)\s*', regexineq, ')',...
			')))\s*$'
		]);
		onestimes = regexpi(deblank(value), '^\s*((\d+.?\d*(e(\+|-)\d+)?\.?\*)*(ones|zeros|eye|sqrt|exp|log|floor|ceil|)\((numberOfVariables|numberOfInequalities|numberOfEqualities|numberOfBounds)(\.?(\*|/)(\d+.?\d*(e(\+|-)\d+)?))*(\s*(\+|-)\s*(numberOfVariables|numberOfInequalities|numberOfEqualities|numberOfBounds)(\.?(\*|/)(\d+.?\d*(e(\+|-)\d+)?))*)*(\s*(\+|-)\s*(numberOfVariables|numberOfInequalities|numberOfEqualities|numberOfBounds)(\.?(\*|/)(\d+.?\d*(e(\+|-)\d+)?))*)*(,\s*\d+)?\))\s*$');
		vartimes2 = regexpi(deblank(value), '^\s*((\d+.?\d*(e(\+|-)\d+)?\.?(\*|/))*(numberOfVariables|numberOfInequalities|numberOfEqualities|numberOfBounds)(\.?(\*|/)(\d+.?\d*(e(\+|-)\d+)?))*(\s*(\+|-)\s*(numberOfVariables|numberOfInequalities|numberOfEqualities|numberOfBounds)(\.?(\*|/)(\d+.?\d*(e(\+|-)\d+)?))*)*(\s*(\+|-)\s*(numberOfVariables|numberOfInequalities|numberOfEqualities|numberOfBounds)(\.?(\*|/)(\d+.?\d*(e(\+|-)\d+)?))*)*)\s*$');
		matrix = regexpi(deblank(value), '^\s*\[(((\d+.?\d*(e(\+|-)\d+)?)|((\d+.?\d*(e(\+|-)\d+)?\.?\*)*(ones|zeros)\((numberOfVariables|numberOfInequalities|numberOfEqualities|numberOfBounds)(\s*-\s*\d+)\))),?;?\s*)+\]\s*$');
		number = regexpi(deblank(value), '^\s*(\d+.?\d*(e(\+|-)\d+)?\s*$');
		epsregex = regexpi(deblank(value), '^\s*(\d+.?\d*(e(\+|-)\d+)\.?\*)*(sqrt|exp|log|floor|ceil)\(eps\)\s*$');
		% TODO: allow this value for MaxSQPIter
		if ~isempty(onestimes) || ~isempty(vartimes1) || ~isempty(vartimes2) || ~isempty(matrix) || ~isempty(number) || ~isempty(epsregex) || strcmpi(value, '10*max(numberofvariables,numberofinequalities+numberofbounds)')
			clear regexineq regexeq regexvar onestimes vartimes matrix number epsregex out;
			try
				validvalue = eval(value);
				valid = true;
				errmsg = '';
				errid = '';
			catch e
				validvalue = [];
				valid = false;
				errmsg = e.message;
				errid = e.identifier;
			end
		else
			validvalue = [];
			valid = false;
			errmsg = sprintf('Option %s must contain ''numberOfVariables''.', field);
			errid = 'optimlib:options:checkfield:eval';
		end
	else
		validvalue = value;
		valid = true;
		errmsg = '';
		errid = '';
	end
end
classdef ProblemType < uint8
	%PROBLEMTYPE type of optimization problem
	
	enumeration
		% unconstrained optimization problem
		UNCONSTRAINED(1);
		% constrained optimization problem
		CONSTRAINED(2);
		% unconstrained multiobjective optimization problem
		UNCONSTRAINEDMULTI(3);
		% constrained multiobjective optimization problem
		CONSTRAINEDMULTI(4)
	end
	
	methods(Static=true)
		function [default] = getDefaultValue()
			%GETDEFAULTVALUE return default value for problem types
			%	Output:
			%		default:	default problem type
			default = optimization.options.ProblemType.UNCONSTRAINED;
		end
		
		function [description] = getDescription()
			%GETDESCRIPTION return description of the problem type class
			%	Output:
			%		description:	description of the class
			description = 'type of optimization problem';
		end
		
		function [addname] = addClassNameToEnumNames()
			%ADDCLASSNAMETOENUMNAMES Control whether class name is added as a prefix to enumerated names in the generated code.
			%	Output:
			%		addname:	true, to add the class name to generated code to avoid naming conflicts
			addname = true;
		end
		
		function [problemtype] = frominteger(ord)
			%FROMINTEGER create ProblemType identifier from integer
			%	Input:
			%		ord:		ordinal of ProblemType
			%	Output:
			%		losstype:	ord as ProblemType
			if ~isinteger(ord)
				if ischar(ord)
					problemtype = optimization.options.ProblemType.fromchar(ord);
					return;
				elseif isnumeric(ord) && floor(ord) == ceil(ord)
					ord = int32(ord);
				else
					error('optimization:options:ProblemType:name', 'undefined function for input arguments of type ''%s''.', class(ord));
				end
			end
			problemtype = repmat(optimization.options.ProblemType.getDefaultValue(), length(ord), 1);
			for ii = 1:length(ord)
				switch ord(ii)
					case 1
						problemtype(ii, 1) = optimization.options.ProblemType.UNCONSTRAINED;
					case 2
						problemtype(ii, 1) = optimization.options.ProblemType.CONSTRAINED;
					case 3
						problemtype(ii, 1) = optimization.options.ProblemType.UNCONSTRAINEDMULTI;
					case 4
						problemtype(ii, 1) = optimization.options.ProblemType.CONSTRAINEDMULTI;
					otherwise;
						error('optimization:options:ProblemType:name', 'ProblemType is undefined.');
				end
			end
			problemtype = reshape(problemtype, size(ord));
		end
		
		function [problemtype] = fromchar(name)
			%FROMCHAR create ProblemType from char
			%	Input:
			%		name:			name of problem type
			%	Output:
			%		problemtype:	name as ProblemType
			if isa(name, 'optimization.options.ProblemType')
				problemtype = name;
				return;
			end
			if ~ischar(name)
				if ~isinteger(name)
					if isnumeric(name) && floor(name) == ceil(name)
						name = int32(name);
					else
						error('optimization:options:ProblemType:name', 'undefined function for input arguments of type ''%s''.', class(name));
					end
				end
				problemtype = optimization.options.ProblemType.frominteger(name);
				if ~isa(problemtype, 'optimization.options.ProblemType')
					error('optimization:options:ProblemType:name', 'undefined function for input arguments of type ''%s''.', class(name));
				end
				return;
			end
			switch lower(name)
				case {'unconstrained', 'free'}
					problemtype = optimization.options.ProblemType.UNCONSTRAINED;
				case {'constrained', 'bounded'}
					problemtype = optimization.options.ProblemType.CONSTRAINED;
				case {'unconstrainedmulti', 'freemulti'}
					problemtype = optimization.options.ProblemType.UNCONSTRAINEDMULTI;
				case {'constrainedmulti', 'boundedmulti'}
					problemtype = optimization.options.ProblemType.CONSTRAINEDMULTI;
				otherwise
					enum = enumeration('optimization.options.ProblemType');
					maxvaluechar = length(sprintf('%d', max(enum)));
					if ischar(name) && length(name) <= maxvaluechar
						problemtype = optimization.options.ProblemType.frominteger(str2double(name));
						return;
					end
					problemtype = optimization.options.ProblemType.frominteger(name);
					if ~isa(problemtype, 'optimization.options.ProblemType')
						error('optimization:options:ProblemType:name', 'ProblemType is undefined.');
					end
			end
		end
	end
	
	methods
		function [problemnname] = char(this)
			%CHAR return ProblemType as char
			%	Input:
			%		this:			instance
			%	Output:
			%		frictionname:	char representation of the friction type
			persistent name;
			if isempty(name)
				name = [
					'unconstrained     ';
					'constrained       ';
					'unconstrainedmulti';
					'constrainedmulti  '
				];
			end
			problemnname = strtrim(name(this, :));
		end
		
		function [frictionname] = lower(this)
			%LOWER return char representation of the friction type in lower case letters
			%	Input:
			%		this:			instance
			%	Output:
			%		frictionname:	char representation of the friction type in lower case letters
			frictionname = lower(char(this));
		end
		
		function [frictionname] = upper(this)
			%UPPER return char representation of the friction type in upper case letters
			%	Input:
			%		this:			instance
			%	Output:
			%		frictionname:	char representation of the friction type in upper case letters
			frictionname = upper(char(this));
		end
	end
end
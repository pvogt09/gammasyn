classdef(Enumeration) FinDiffType < Simulink.IntEnumType
	%FINDIFFTYPE class to represent a finite difference type
	%#codegen
	
	enumeration
		% forward difference
		FORWARD(1);
		% backward difference
		BACKWARD(2);
		% central difference
		CENTRAL(3);
	end
	
	methods(Static=true)
		function [default] = getDefaultValue()
			%GETDEFAULTVALUE return default value for finite difference types
			%	Output:
			%		default:	default finite difference type
			default = optimization.general.FinDiffType.FORWARD;
		end
		
		function [description] = getDescription()
			%GETDESCRIPTION return description of the finite difference type class
			%	Output:
			%		description:	description of the class
			description = 'class to represent a finite difference type';
		end
		
		function [addname] = addClassNameToEnumNames()
			%ADDCLASSNAMETOENUMNAMES Control whether class name is added as a prefix to enumerated names in the generated code.
			%	Output:
			%		addname:	true, to add the class name to generated code to avoid naming conflicts
			addname = true;
		end
		
		function [findifftype] = frominteger(ord)
			%FROMINTEGER create FinDiffType identifier from integer
			%	Input:
			%		ord:			ordinal of FinDiffType
			%	Output:
			%		findifftype:	ord as FinDiffType
			if ~isinteger(ord)
				if ischar(ord)
					findifftype = optimization.general.FinDiffType.fromname(ord);
					return;
				elseif isnumeric(ord) && floor(ord) == ceil(ord)
					ord = int32(ord);
				else
					error('optimization:general:FinDiffType:name', 'undefined function for input arguments of type ''%s''.', class(ord));
				end
			end
			findifftype = repmat(optimization.general.FinDiffType.getDefaultValue(), length(ord), 1);
			for ii = 1:length(ord)
				switch ord(ii)
					case 1
						findifftype(ii, 1) = optimization.general.FinDiffType.FORWARD;
					case 2
						findifftype(ii, 1) = optimization.general.FinDiffType.BACKWARD;
					case 3
						findifftype(ii, 1) = optimization.general.FinDiffType.CENTRAL;
					otherwise;
						error('optimization:general:FinDiffType:name', 'finite difference type is undefined.');
				end
			end
			findifftype = reshape(findifftype, size(ord));
		end
		
		function [findifftype] = fromname(name)
			%FROMNAME create FrictionType from char
			%	Input:
			%		name:			name of finite difference type
			%	Output:
			%		findifftype:	name as FinDiffType
			if isa(name, 'optimization.general.FinDiffType')
				findifftype = name;
				return;
			end
			if ~ischar(name)
				if ~isinteger(name)
					if isnumeric(name) && floor(name) == ceil(name)
						name = int32(name);
					else
						error('optimization:general:FinDiffType:name', 'undefined function for input arguments of type ''%s''.', class(name));
					end
				end
				findifftype = optimization.general.FinDiffType.frominteger(name);
				if ~isa(findifftype, 'optimization.general.FinDiffType')
					error('optimization:general:FinDiffType:name', 'undefined function for input arguments of type ''%s''.', class(name));
				end
				return;
			end
			switch lower(name)
				case {'forward'}
					findifftype = optimization.general.FinDiffType.FORWARD;
				case 'backward'
					findifftype = optimization.general.FinDiffType.BACKWARD;
				case 'central'
					findifftype = optimization.general.FinDiffType.CENTRAL;
				otherwise
					enum = enumeration('optimization.general.FinDiffType');
					maxvaluechar = length(sprintf('%d', max(enum)));
					if ischar(name) && length(name) <= maxvaluechar
						findifftype = optimization.general.FinDiffType.frominteger(str2double(name));
						return;
					end
					findifftype = optimization.general.FinDiffType.frominteger(name);
					if ~isa(findifftype, 'optimization.general.FinDiffType')
						error('optimization:general:FinDiffType:name', 'finite difference type is undefined.');
					end
			end
		end
		
		function [fromDCM] = fromDCM(DCMstring, ~)
			%FROMDCM convert string from DCM file to object of FinDiffType class
			%	Input:
			%		DCMstring:	string to convert from
			%		name:		optional name of parameter
			%	Output:
			%		fromDCM:	FinDiffType, if one of the specified name exists
			fromDCM = optimization.general.FinDiffType.fromname(DCMstring);
		end
	end
	
% must not be private to allow for type cast to FinDiffType, types are automatically restricted to the defined ones internally
% 	methods(Access=private)
% 		function [this] = FinDiffType(ord)
% 			%FINDIFFTYPE create new FinDiffType object for representation of a finite difference type
% 			%	Input:
% 			%		ord:			number of finite difference type
% 			%	Output:
% 			%		this:			instance
% 			this = this@Simulink.IntEnumType(ord);
% 		end
% 	end
	
	methods
		function [findiffname] = char(this)
			%CHAR return FinDiffType as char
			%	Input:
			%		this:			instance
			%	Output:
			%		findiffname:	char representation of the finite difference type
			persistent name;
			if isempty(name)
				name = [
					'forward ';
					'backward';
					'central '
				];
			end
			findiffname = strtrim(name(this, :));
		end
		
		function [findiffname] = lower(this)
			%LOWER return char representation of the finite difference type in lower case letters
			%	Input:
			%		this:			instance
			%	Output:
			%		findiffname:	char representation of the finite difference type in lower case letters
			findiffname = lower(char(this));
		end
		
		function [findiffname] = upper(this)
			%UPPER return char representation of the finite difference type in upper case letters
			%	Input:
			%		this:			instance
			%	Output:
			%		frictionname:	char representation of the finite difference type in upper case letters
			findiffname = upper(char(this));
		end
		
		function [asDCM] = toDCM(this)
			%TODCM convert instance to string for use in an DCM file
			%	Input:
			%		this:	instance
			%	Output:
			%		asDCM:	string representation of the instance for use in an DCM file
			asDCM = sprintf('%d', int32(this));
		end
	end
	
end
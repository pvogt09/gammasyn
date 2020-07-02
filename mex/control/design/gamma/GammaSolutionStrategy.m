classdef(Enumeration) GammaSolutionStrategy < Simulink.IntEnumType
	%GAMMASOLUTIONSTRATEGY enumeration for characterization of different solution strategies for gamma pole placement for use with codegen
	%#codegen
	
	enumeration
		% simply run optimization problem once
		SINGLESHOT(0);
		% solve feasibility problem first (if supported) and then solve optimization problem with found solution
		FEASIBILITYITERATION(1)
	end
	
% must not be private to allow for type cast to GammaSolutionStrategy, types are automatically restricted to the defined ones internally
% 	methods(Access=private)
% 		function [this] = GammaSolutionStrategy(type)
% 			%GAMMASOLUTIONSTRATEGY create new gamma solution strategy object
% 			%	Input:
% 			%		type:	number for solution strategy
% 			%	Output:
% 			%		this:	instance of class
% 			this@Simulink.IntEnumType(type);
% 		end
% 	end
	
	methods(Static=true)
		function [default] = getDefaultValue()
			%GETDEFAULTVALUE return default gamma solution strategy type
			%	Output:
			%		default:	default solution strategy type
			default = GammaSolutionStrategy.SINGLESHOT;
		end
		
		function [description] = getDescription() 
			%GETDESCRIPTION	String to describe the class in Simulink Coder
			%	Output:
			%		description:	description of the class
			description = 'enumeration for characterization of different solution strategies for gamma pole placement for use with codegen';
		end

		function [addname] = addClassNameToEnumNames()
			%ADDCLASSNAMETOENUMNAMES Control whether class name is added as a prefix to enumerated names in the generated code.
			%	Output:
			%		addname:	true, to add the class name to generated code to avoid naming conflicts
			addname = true;
		end
		
		function [strategy] = fromname(name)
			%FROMNAME create GammaSolutionStrategy from name
			%	Input:
			%		name:		name of a defined GammaSolutionStrategy
			%	Output:
			%		strategy:	GammaErrorHandler, if one of the specified name exists
			if isa(name, 'GammaSolutionStrategy')
				strategy = name;
				return;
			end
			enum = enumeration('GammaSolutionStrategy');
			maxvaluechar = length(sprintf('%d', max(enum)));
			if ischar(name)
				strategy = [];
				for ii = 1:length(enum)
					if isinteger(name) && name == int32(enum(ii))
						strategy = enum(ii);
						return;
					end
					if isnumeric(name) && floor(name) == ceil(name) && floor(name) == int32(enum(ii))
						strategy = enum(ii);
						return;
					end
					if ischar(name) && strcmpi(name, char(enum(ii)))
						strategy = enum(ii);
						return;
					end
					if ischar(name) && length(name) <= maxvaluechar
						strategy = GammaSolutionStrategy.fromDCM(str2double(name));
						return;
					end
				end
			else
				strategy = repmat(GammaSolutionStrategy.getDefaultValue(), length(name), 1);
				hasJ = false(length(name), 1);
				for jj = 1:length(name)
					for ii = 1:length(enum)
						if isinteger(name(jj)) && name(jj) == int32(enum(ii))
							strategy(jj, 1) = enum(ii);
							hasJ(jj, 1) = true;
							break;
						end
						if isnumeric(name(jj)) && floor(name(jj)) == ceil(name(jj)) && floor(name(jj)) == int32(enum(ii))
							strategy(jj, 1) = enum(ii);
							hasJ(jj, 1) = true;
							break;
						end
					end
				end
				if ~all(hasJ)
					strategy = [];
				else
					strategy = reshape(strategy, size(name));
				end
			end
			if isempty(strategy)
				error('control:design:gamma:type:name', 'No GammaSolutionStrategy of specified name exists.');
			end
		end
		
		function [fromDCM] = fromDCM(DCMstring)
			%FROMDCM convert string from DCM file to object of GammaSolutionStrategy class
			%	Input:
			%		DCMstring:	string to convert from
			%	Output:
			%		fromDCM:	GammaSolutionStrategy, if one of the specified name exists
			fromDCM = GammaSolutionStrategy.fromname(DCMstring);
		end
	end
	
	methods
		function [asDCM] = toDCM(this)
			%TODCM convert instance to string for use in an DCM file
			%	Input:
			%		this:	instance
			%	Output:
			%		asDCM:	string representation of the instance for use in an DCM file
			asDCM = sprintf('%d', int32(this));
		end
		
		function [hash] = hashCode(this)
			%HASHCODE create hash code for object
			%	Input:
			%		this:	instance
			%	Output:
			%		hash:	hashcode for object
			hash = DataHash(int32(this));
		end
	end
end

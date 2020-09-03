classdef(Enumeration) GammaDecouplingconditionSortingStrategy < Simulink.IntEnumType
	%GAMMACOUPLINGCONDITIONSORTINGSTRATEGY enumeration for characterization of different decoupling condition sorting strategies in gamma pole placement for use with codegen
	%#codegen


	enumeration
		% no specific sorting strategy
		DEFAULT(0);
		% sort decoupling conditions according to minimum norm for every system
		MINIMUMNORM(1);
		% sort decoupling conditions by tracking eigenvalues over optimization iterations
		EIGENVALUETRACKING(2);
	end

% must not be private to allow for type cast to GammaDecouplingconditionSortingStrategy, types are automatically restricted to the defined ones internally
% 	methods(Access=private)
% 		function [this] = GammaDecouplingconditionSortingStrategy(type)
% 			%GAMMACOUPLINGSTRATEGY create new Gamma decouplingcondition sorting strategy
% 			%	Input:
% 			%		type:	number for decoupling condition sorting strategy
% 			%	Output:
% 			%		this:	instance of class
% 			this@Simulink.IntEnumType(type);
% 		end
% 	end

	methods(Static=true)
		function [default] = getDefaultValue()
			%GETDEFAULTVALUE return default decoupling condition sorting strategy
			%	Output:
			%		default:	default default decoupling condition sorting strategy
			default = GammaDecouplingconditionSortingStrategy.MINIMUMNORM;
		end

		function [description] = getDescription()
			%GETDESCRIPTION	String to describe the class in Simulink Coder
			%	Output:
			%		description:	description of the class
			description = 'enumeration for characterization of different decoupling condition sorting strategies';
		end

		function [addname] = addClassNameToEnumNames()
			%ADDCLASSNAMETOENUMNAMES Control whether class name is added as a prefix to enumerated names in the generated code.
			%	Output:
			%		addname:	true, to add the class name to generated code to avoid naming conflicts
			addname = true;
		end

		function [strategy] = fromname(name)
			%FROMNAME create GammaDecouplingconditionSortingStrategy from name
			%	Input:
			%		name:		name of a defined GammaDecouplingconditionSortingStrategy
			%	Output:
			%		strategy:	GammaDecouplingconditionSortingStrategy, if one of the specified name exists
			if isa(name, 'GammaDecouplingconditionSortingStrategy')
				strategy = name;
				return;
			end
			enum = enumeration('GammaDecouplingconditionSortingStrategy');
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
						strategy = GammaDecouplingconditionSortingStrategy.fromDCM(str2double(name));
						return;
					end
				end
			else
				strategy = repmat(GammaDecouplingconditionSortingStrategy.getDefaultValue(), length(name), 1);
				hasStrategy = false(length(name), 1);
				for jj = 1:length(name)
					for ii = 1:length(enum)
						if isinteger(name(jj)) && name(jj) == int32(enum(ii))
							strategy(jj, 1) = enum(ii);
							hasStrategy(jj, 1) = true;
							break;
						end
						if isnumeric(name(jj)) && floor(name(jj)) == ceil(name(jj)) && floor(name(jj)) == int32(enum(ii))
							strategy(jj, 1) = enum(ii);
							hasStrategy(jj, 1) = true;
							break;
						end
					end
				end
				if ~all(hasStrategy)
					strategy = [];
				else
					strategy = reshape(strategy, size(name));
				end
			end
			if isempty(strategy)
				error('control:design:gamma:type:name', 'No GammaDecouplingconditionSortingStrategy of specified name exists.');
			end
		end

		function [fromDCM] = fromDCM(DCMstring, ~)
			%FROMDCM convert string from DCM file to object of GammaDecouplingconditionSortingStrategy class
			%	Input:
			%		DCMstring:	string to convert from
			%		name:		optional name of parameter
			%	Output:
			%		fromDCM:	GammaDecouplingconditionSortingStrategy, if one of the specified name exists
			fromDCM = GammaDecouplingconditionSortingStrategy.fromname(DCMstring);
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
classdef(Enumeration) GammaEigenvalueMultiplicityHandlingType < Simulink.IntEnumType
	%GAMMAEIGENVALUEMULTIPLICITYHANDLINGTYPE enumeration for characterization of different methods to handle numerically equal eigenvalues in gamma pole placement for use with codegen
	%#codegen

	enumeration
		% dummy for getDefaultValue() method to use default value in generated code
		DEFAULT(1);
		% keep values of eigenvalues unchanged
		KEEP(0);
		% round numerically equal eigenvalues to tolerance
		ROUND(1);
		% use minimum of numerically equal eigenvalues
		MIN(2);
		% use mean of numerically equal eigenvalues
		MEAN(3);
		% use maximum of numerically equal eigenvalues
		MAX(4)
	end

% must not be private to allow for type cast to GammaEigenvalueMultiplicityHandlingType, types are automatically restricted to the defined ones internally
% 	methods(Access=private)
% 		function [this] = GammaEigenvalueMultiplicityHandlingType(type)
% 			%GAMMAEIGENVALUEMULTIPLICITYHANDLINGTYPE create new gamma multiplicity handling type object
% 			%	Input:
% 			%		type:	number for multiplicity handling type
% 			%	Output:
% 			%		this:	instance of class
% 			this@Simulink.IntEnumType(type);
% 		end
% 	end

	methods(Static=true)
		function [default] = getDefaultValue()
			%GETDEFAULTVALUE return default gamma eigenvalue multiplicity handling type
			%	Output:
			%		default:	default multiplicity handling type
			default = GammaEigenvalueMultiplicityHandlingType.ROUND;
		end

		function [description] = getDescription()
			%GETDESCRIPTION	String to describe the class in Simulink Coder
			%	Output:
			%		description:	description of the class
			description = 'enumeration for characterization of different methods to handle numerically equal eigenvalues in gamma pole placement for use with codegen';
		end

		function [addname] = addClassNameToEnumNames()
			%ADDCLASSNAMETOENUMNAMES Control whether class name is added as a prefix to enumerated names in the generated code.
			%	Output:
			%		addname:	true, to add the class name to generated code to avoid naming conflicts
			addname = true;
		end

		function [multiplicityhandling] = fromname(name)
			%FROMNAME create GammaEigenvalueMultiplicityHandlingType from name
			%	Input:
			%		name:					name of a defined GammaEigenvalueMultiplicityHandlingType
			%	Output:
			%		multiplicityhandling:	GammaEigenvalueMultiplicityHandlingType, if one of the specified name exists
			if isa(name, 'GammaEigenvalueMultiplicityHandlingType')
				multiplicityhandling = name;
				return;
			end
			enum = enumeration('GammaEigenvalueMultiplicityHandlingType');
			maxvaluechar = length(sprintf('%d', max(enum)));
			if ischar(name)
				multiplicityhandling = [];
				for ii = 1:length(enum)
					if isinteger(name) && name == int32(enum(ii))
						multiplicityhandling = enum(ii);
						return;
					end
					if isnumeric(name) && floor(name) == ceil(name) && floor(name) == int32(enum(ii))
						multiplicityhandling = enum(ii);
						return;
					end
					if ischar(name) && strcmpi(name, char(enum(ii)))
						multiplicityhandling = enum(ii);
						return;
					end
					if ischar(name) && length(name) <= maxvaluechar
						multiplicityhandling = GammaEigenvalueMultiplicityHandlingType.fromDCM(str2double(name));
						return;
					end
				end
			else
				multiplicityhandling = repmat(GammaEigenvalueMultiplicityHandlingType.getDefaultValue(), length(name), 1);
				hasderivative = false(length(name), 1);
				for jj = 1:length(name)
					for ii = 1:length(enum)
						if isinteger(name(jj)) && name(jj) == int32(enum(ii))
							multiplicityhandling(jj, 1) = enum(ii);
							hasderivative(jj, 1) = true;
							break;
						end
						if isnumeric(name(jj)) && floor(name(jj)) == ceil(name(jj)) && floor(name(jj)) == int32(enum(ii))
							multiplicityhandling(jj, 1) = enum(ii);
							hasderivative(jj, 1) = true;
							break;
						end
					end
				end
				if ~all(hasderivative)
					multiplicityhandling = [];
				else
					multiplicityhandling = reshape(multiplicityhandling, size(name));
				end
			end
			if isempty(multiplicityhandling)
				error('control:design:gamma:multiplicitytype:name', 'No GammaEigenvalueMultiplicityHandlingType of specified name exists.');
			end
		end

		function [fromDCM] = fromDCM(DCMstring, ~)
			%FROMDCM convert string from DCM file to object of GammaEigenvalueMultiplicityHandlingType class
			%	Input:
			%		DCMstring:	string to convert from
			%		name:		optional name of parameter
			%	Output:
			%		fromDCM:	GammaEigenvalueMultiplicityHandlingType, if one of the specified name exists
			fromDCM = GammaEigenvalueMultiplicityHandlingType.fromname(DCMstring);
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
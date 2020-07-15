classdef(Enumeration) GammaEigenvalueDerivativeType < Simulink.IntEnumType
	%GAMMAEIGENVALUEDERIVATIVETYPE enumeration for characterization of different methods to calculate the derivatives of eigenvalues and eigenvectors in gamma pole placement for use with codegen
	%#codegen

	enumeration
		% default calculation of eigenvalue derivatives like in [#FOELLINGER1994]
		DEFAULT(0);
		% method of [#VANDERAA2007] for calculation of eigenvalue and eigenvector derivatives
		VANDERAA(1);
		% method of [#RUDISILL1975] for calculation of eigenvalue and eigenvector derivatives
		RUDISILLCHU(2);
	end

% must not be private to allow for type cast to GammaEigenvalueDerivativeType, types are automatically restricted to the defined ones internally
% 	methods(Access=private)
% 		function [this] = GammaEigenvalueDerivativeType(type)
% 			%GAMMAEIGENVALUEDERIVATIVETYPE create new gamma eigenvalue derivative type object
% 			%	Input:
% 			%		type:	number for derivative method type
% 			%	Output:
% 			%		this:	instance of class
% 			this@Simulink.IntEnumType(type);
% 		end
% 	end

	methods(Static=true)
		function [default] = getDefaultValue()
			%GETDEFAULTVALUE return default gamma eigenvalue derivative type
			%	Output:
			%		default:	default derivative method type
			default = GammaEigenvalueDerivativeType.DEFAULT;
		end

		function [description] = getDescription()
			%GETDESCRIPTION	String to describe the class in Simulink Coder
			%	Output:
			%		description:	description of the class
			description = 'enumeration for characterization of different methods to calculate the derivatives of eigenvalues and eigenvectors in gamma pole placement for use with codegen';
		end

		function [addname] = addClassNameToEnumNames()
			%ADDCLASSNAMETOENUMNAMES Control whether class name is added as a prefix to enumerated names in the generated code.
			%	Output:
			%		addname:	true, to add the class name to generated code to avoid naming conflicts
			addname = true;
		end

		function [derivative] = fromname(name)
			%FROMNAME create GammaEigenvalueDerivativeType from name
			%	Input:
			%		name:		name of a defined GammaEigenvalueDerivativeType
			%	Output:
			%		derivative:	GammaEigenvalueDerivativeType, if one of the specified name exists
			if isa(name, 'GammaEigenvalueDerivativeType')
				derivative = name;
				return;
			end
			enum = enumeration('GammaEigenvalueDerivativeType');
			maxvaluechar = length(sprintf('%d', max(enum)));
			if ischar(name)
				derivative = [];
				for ii = 1:length(enum)
					if isinteger(name) && name == int32(enum(ii))
						derivative = enum(ii);
						return;
					end
					if isnumeric(name) && floor(name) == ceil(name) && floor(name) == int32(enum(ii))
						derivative = enum(ii);
						return;
					end
					if ischar(name) && strcmpi(name, char(enum(ii)))
						derivative = enum(ii);
						return;
					end
					if ischar(name) && length(name) <= maxvaluechar
						derivative = GammaEigenvalueDerivativeType.fromDCM(str2double(name));
						return;
					end
				end
			else
				derivative = repmat(GammaEigenvalueDerivativeType.getDefaultValue(), length(name), 1);
				hasderivative = false(length(name), 1);
				for jj = 1:length(name)
					for ii = 1:length(enum)
						if isinteger(name(jj)) && name(jj) == int32(enum(ii))
							derivative(jj, 1) = enum(ii);
							hasderivative(jj, 1) = true;
							break;
						end
						if isnumeric(name(jj)) && floor(name(jj)) == ceil(name(jj)) && floor(name(jj)) == int32(enum(ii))
							derivative(jj, 1) = enum(ii);
							hasderivative(jj, 1) = true;
							break;
						end
					end
				end
				if ~all(hasderivative)
					derivative = [];
				else
					derivative = reshape(derivative, size(name));
				end
			end
			if isempty(derivative)
				error('control:design:gamma:derivativetype:name', 'No GammaEigenvalueDerivativeType of specified name exists.');
			end
		end

		function [fromDCM] = fromDCM(DCMstring, ~)
			%FROMDCM convert string from DCM file to object of GammaEigenvalueDerivativeType class
			%	Input:
			%		DCMstring:	string to convert from
			%		name:		optional name of parameter
			%	Output:
			%		fromDCM:	GammaEigenvalueDerivativeType, if one of the specified name exists
			fromDCM = GammaEigenvalueDerivativeType.fromname(DCMstring);
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
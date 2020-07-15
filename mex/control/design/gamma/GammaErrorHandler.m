classdef(Enumeration) GammaErrorHandler < Simulink.IntEnumType
	%GAMMAERRORHANDLER enumeration for characterization of different error handling variants in gamma pole placement for use with codegen
	%#codegen

	enumeration
		% convert errors to warnings
		WARNING(0);
		% leave errors as is
		ERROR(1);
		% use user defined error handler
		USER(2)
	end

% must not be private to allow for type cast to GammaErrorHandler, types are automatically restricted to the defined ones internally
% 	methods(Access=private)
% 		function [this] = GammaErrorHandler(type)
% 			%GAMMAERRORHANDLER create new gamma error handler object
% 			%	Input:
% 			%		type:	number for error handler
% 			%	Output:
% 			%		this:	instance of class
% 			this@Simulink.IntEnumType(type);
% 		end
% 	end

	methods(Static=true)
		function [default] = getDefaultValue()
			%GETDEFAULTVALUE return default gamma error handler type
			%	Output:
			%		default:	default error handler type
			default = GammaErrorHandler.WARNING;
		end

		function [description] = getDescription()
			%GETDESCRIPTION	String to describe the class in Simulink Coder
			%	Output:
			%		description:	description of the class
			description = 'enumeration for characterization of different error handling variants in gamma pole placement for use with codegen';
		end

		function [addname] = addClassNameToEnumNames()
			%ADDCLASSNAMETOENUMNAMES Control whether class name is added as a prefix to enumerated names in the generated code.
			%	Output:
			%		addname:	true, to add the class name to generated code to avoid naming conflicts
			addname = true;
		end

		function [err] = fromname(name)
			%FROMNAME create GammaErrorHandler from name
			%	Input:
			%		name:		name of a defined GammaErrorHandler
			%	Output:
			%		err:		GammaErrorHandler, if one of the specified name exists
			if isa(name, 'GammaErrorHandler')
				err = name;
				return;
			end
			enum = enumeration('GammaErrorHandler');
			maxvaluechar = length(sprintf('%d', max(enum)));
			if ischar(name)
				err = [];
				for ii = 1:length(enum)
					if isinteger(name) && name == int32(enum(ii))
						err = enum(ii);
						return;
					end
					if isnumeric(name) && floor(name) == ceil(name) && floor(name) == int32(enum(ii))
						err = enum(ii);
						return;
					end
					if ischar(name) && strcmpi(name, char(enum(ii)))
						err = enum(ii);
						return;
					end
					if ischar(name) && length(name) <= maxvaluechar
						err = GammaErrroHandler.fromDCM(str2double(name));
						return;
					end
				end
			else
				err = repmat(GammaErrorHandler.getDefaultValue(), length(name), 1);
				hasJ = false(length(name), 1);
				for jj = 1:length(name)
					for ii = 1:length(enum)
						if isinteger(name(jj)) && name(jj) == int32(enum(ii))
							err(jj, 1) = enum(ii);
							hasJ(jj, 1) = true;
							break;
						end
						if isnumeric(name(jj)) && floor(name(jj)) == ceil(name(jj)) && floor(name(jj)) == int32(enum(ii))
							err(jj, 1) = enum(ii);
							hasJ(jj, 1) = true;
							break;
						end
					end
				end
				if ~all(hasJ)
					err = [];
				else
					err = reshape(err, size(name));
				end
			end
			if isempty(err)
				error('control:design:gamma:type:name', 'No GammaErrorHandler of specified name exists.');
			end
		end

		function [fromDCM] = fromDCM(DCMstring)
			%FROMDCM convert string from DCM file to object of GammaErrorHandler class
			%	Input:
			%		DCMstring:	string to convert from
			%	Output:
			%		fromDCM:	GammaErrorHandler, if one of the specified name exists
			fromDCM = GammaErrorHandler.fromname(DCMstring);
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
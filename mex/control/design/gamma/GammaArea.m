classdef(Enumeration) GammaArea < Simulink.IntEnumType
	%GAMMAAREA enumeration for characterization of different areas for poles in gamma pole placement for use with codegen
	%#codegen

	enumeration
		% no pole area
		NONE(1);
		% circle function
		CIRCLE(2);
		% circle function without square-roots
		CIRCLESQUARE(3);
		% ellipse function
		ELLIPSE(4);
		% ellipse function without square roots
		ELLIPSESQUARE(5)
		% hyperbola function
		HYPERBOLA(6);
		% hyperbola function without quare-roots
		HYPERBOLASQUARE(7);
		% imaginary axis
		IMAG(8);
		% line
		LINE(9);
		% logarithmic spiral
		LOGSPIRAL(10);
		% unit circle without square roots
		CIRCLEDISCRETE(11);
		% poly ellipse
		POLYELLIPSE(12);
		% poly ellipse without square roots
		POLYELLIPSESQUARE(13);
		% custom polearea with user defined function handle (not supported in generated code)
		CUSTOM(14)
	end

% must not be private to allow for type cast to GammaArea, types are automatically restricted to the defined ones internally
% 	methods(Access=private)
% 		function [this] = GammaArea(type)
% 			%GAMMAAREA create new gamma area object
% 			%	Input:
% 			%		type:	number for area
% 			%	Output:
% 			%		this:	instance of class
% 			this@Simulink.IntEnumType(type);
% 		end
% 	end

	methods(Static=true)
		function [default] = getDefaultValue()
			%GETDEFAULTVALUE return default gamma ploe area
			%	Output:
			%		default:	default pole area
			default = GammaArea.IMAG;
		end

		function [description] = getDescription()
			%GETDESCRIPTION	String to describe the class in Simulink Coder
			%	Output:
			%		description:	description of the class
			description = 'enumeration for characterization of different areas for poles in gamma pole placement for use with codegen';
		end

		function [addname] = addClassNameToEnumNames()
			%ADDCLASSNAMETOENUMNAMES Control whether class name is added as a prefix to enumerated names in the generated code.
			%	Output:
			%		addname:	true, to add the class name to generated code to avoid naming conflicts
			addname = true;
		end

		function [area] = fromname(name)
			%FROMNAME create GammaArea from name
			%	Input:
			%		name:		name of a defined GammaArea
			%	Output:
			%		area:		GammaArea, if one of the specified name exists
			if isa(name, 'GammaArea')
				area = name;
				return;
			end
			enum = enumeration('GammaArea');
			maxvaluechar = length(sprintf('%d', max(enum)));
			if ischar(name)
				area = [];
				for ii = 1:length(enum)
					if isinteger(name) && name == int32(enum(ii))
						area = enum(ii);
						return;
					end
					if isnumeric(name) && floor(name) == ceil(name) && floor(name) == int32(enum(ii))
						area = enum(ii);
						return;
					end
					if ischar(name) && strcmpi(name, char(enum(ii)))
						area = enum(ii);
						return;
					end
					if ischar(name) && length(name) <= maxvaluechar
						area = GammaArea.fromDCM(str2double(name));
						return;
					end
				end
			else
				area = repmat(GammaArea.getDefaultValue(), length(name), 1);
				hasarea = false(length(name), 1);
				for jj = 1:length(name)
					for ii = 1:length(enum)
						if isinteger(name(jj)) && name(jj) == int32(enum(ii))
							area(jj, 1) = enum(ii);
							hasarea(jj, 1) = true;
							break;
						end
						if isnumeric(name(jj)) && floor(name(jj)) == ceil(name(jj)) && floor(name(jj)) == int32(enum(ii))
							area(jj, 1) = enum(ii);
							hasarea(jj, 1) = true;
							break;
						end
					end
				end
				if ~all(hasarea)
					area = [];
				else
					area = reshape(area, size(name));
				end
			end
			if isempty(area)
				error('control:design:gamma:area:type:name', 'No GammaArea of specified name exists.');
			end
		end

		function [fromDCM] = fromDCM(DCMstring, ~)
			%FROMDCM convert string from DCM file to object of GammaArea class
			%	Input:
			%		DCMstring:	string to convert from
			%		name:		optional name of parameter
			%	Output:
			%		fromDCM:	GammaArea, if one of the specified name exists
			fromDCM = GammaArea.fromname(DCMstring);
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
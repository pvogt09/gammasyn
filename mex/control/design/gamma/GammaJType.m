classdef(Enumeration) GammaJType < Simulink.IntEnumType
	%GAMMAJTYPE enumeration for characterization of different objective variants in gamma pole placement for use with codegen
	%#codegen

	enumeration
		% no weighting of pole areas in objective function
		ZERO(0);
		% linear weighting of pole areas in objective function
		LINEAR(1);
		% quadratic weighting of pole areas in objective function
		SQUARE(2);
		% cubic weighting of pole areas in objective function
		CUBIC(3);
		% exponential weighting of pole areas in objective function
		EXP(4);
		% logarithmic weighting of pole areas in objective function
		LOG(5);
		% max(0, x) weighting of pole areas in objective function
		MAX(6);
		% vector performance index weighting according to Kreisselmeier
		KREISSELMEIER(7);
		% quadratic penalty function
		SQUAREPENALTY(8);
		% eigenvector matrix condition objective function
		EIGENVALUECONDITION(9);
		% norm of gain matrix
		NORMGAIN(10);
		% Lyapunov matrix objective function
		LYAPUNOV(11);
	end

% must not be private to allow for type cast to GammaJType, types are automatically restricted to the defined ones internally
% 	methods(Access=private)
% 		function [this] = GammaJType(type)
% 			%GAMMAJTYPE create new gamma objective function object
% 			%	Input:
% 			%		type:	number for objective type
% 			%	Output:
% 			%		this:	instance of class
% 			this@Simulink.IntEnumType(type);
% 		end
% 	end

	methods(Static=true)
		function [default] = getDefaultValue()
			%GETDEFAULTVALUE return default gamma objective function type
			%	Output:
			%		default:	default objective function type
			default = GammaJType.EXP;
		end

		function [description] = getDescription()
			%GETDESCRIPTION	String to describe the class in Simulink Coder
			%	Output:
			%		description:	description of the class
			description = 'enumeration for characterization of different objective variants in gamma pole placement for use with codegen';
		end

		function [addname] = addClassNameToEnumNames()
			%ADDCLASSNAMETOENUMNAMES Control whether class name is added as a prefix to enumerated names in the generated code.
			%	Output:
			%		addname:	true, to add the class name to generated code to avoid naming conflicts
			addname = true;
		end

		function [J] = fromname(name)
			%FROMNAME create GammaJType from name
			%	Input:
			%		name:		name of a defined GammaJType
			%	Output:
			%		J:			GammaJType, if one of the specified name exists
			if isa(name, 'GammaJType')
				J = name;
				return;
			end
			enum = enumeration('GammaJType');
			maxvaluechar = length(sprintf('%d', max(enum)));
			if ischar(name)
				J = [];
				for ii = 1:length(enum)
					if isinteger(name) && name == int32(enum(ii))
						J = enum(ii);
						return;
					end
					if isnumeric(name) && floor(name) == ceil(name) && floor(name) == int32(enum(ii))
						J = enum(ii);
						return;
					end
					if ischar(name) && strcmpi(name, char(enum(ii)))
						J = enum(ii);
						return;
					end
					if ischar(name) && length(name) <= maxvaluechar
						J = GammaJType.fromDCM(str2double(name));
						return;
					end
				end
			else
				J = repmat(GammaJType.getDefaultValue(), length(name), 1);
				hasJ = false(length(name), 1);
				for jj = 1:length(name)
					for ii = 1:length(enum)
						if isinteger(name(jj)) && name(jj) == int32(enum(ii))
							J(jj, 1) = enum(ii);
							hasJ(jj, 1) = true;
							break;
						end
						if isnumeric(name(jj)) && floor(name(jj)) == ceil(name(jj)) && floor(name(jj)) == int32(enum(ii))
							J(jj, 1) = enum(ii);
							hasJ(jj, 1) = true;
							break;
						end
					end
				end
				if ~all(hasJ)
					J = [];
				else
					J = reshape(J, size(name));
				end
			end
			if isempty(J)
				error('control:design:gamma:type:name', 'No GammaJType of specified name exists.');
			end
		end

		function [fromDCM] = fromDCM(DCMstring, ~)
			%FROMDCM convert string from DCM file to object of GammaJType class
			%	Input:
			%		DCMstring:	string to convert from
			%		name:		optional name of parameter
			%	Output:
			%		fromDCM:	GammaJType, if one of the specified name exists
			fromDCM = GammaJType.fromname(DCMstring);
		end
	end

	%methods(Static=true)
	%	TODO: see GammaJType_isgainobjective because this method does not work in code generation
	%	function [isgain] = isgainobjective(this)
	%		%ISGAINOBJECTIVE return if a GammaJType is an objective type that only depends on gain coeffcients
	%		%	Input:
	%		%		this:	instance (must be static to work with code generation)
	%		%	Output:
	%		%		isgain:	true, if the supplied type only depends on gain coefficients
	%		if isscalar(this)
	%			isgain = this == GammaJType.NORMGAIN;
	%		else
	%			sz = size(this);
	%			t = reshape(this, [], 1);
	%			gain = false(size(t, 1), 1);
	%			for ii = 1:size(t, 1)
	%				gain(ii, 1) = t(ii) == GammaJType.NORMGAIN;
	%			end
	%			isgain = reshape(gain, sz);
	%		end
	%	end
	%end

	methods(Static=true)
		function [hashessian] = hashessian(this)
			%HASHESSIAN return if GammaJType has hessian inforamtion implemented
			%	Input:
			%		this:		instance (must be static to work with code generation)
			%	Output:
			%		hashessian:	true, if the supplied type supplies hessian information
			if isscalar(this)
				hashessian = ~any(this == [
					GammaJType.KREISSELMEIER;
					GammaJType.EIGENVALUECONDITION;
					GammaJType.LYAPUNOV
				]);
			else
				sz = size(this);
				t = reshape(this, [], 1);
				gain = false(size(t, 1), 1);
				for ii = 1:size(t, 1)
					gain(ii, 1) = ~any(t(ii) == [
						GammaJType.KREISSELMEIER;
						GammaJType.EIGENVALUECONDITION;
						GammaJType.LYAPUNOV
					]);
				end
				hashessian = reshape(gain, sz);
			end
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
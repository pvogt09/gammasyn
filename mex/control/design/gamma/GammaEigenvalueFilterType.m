classdef(Enumeration) GammaEigenvalueFilterType < Simulink.IntEnumType
	%GAMMAEIGENVALUEFILTERTYPE enumeration for characterization of different methods to filter eigenvalues in gamma pole placement for use with codegen
	%#codegen

	enumeration
		% no filtering
		NONE(0);
		% filter out eigenvalues with negative imaginary part and only keep eigenvalues with nonnegative imaginary part
		NEGATIVEIMAG(1);
		% filter out eigenvalues with positive imaginary part and only keep eigenvalues with nonpositive imaginary part
		POSITIVEIMAG(2);
		% filter out positive infinite eigenvalues to -Inf
		INFTOMINUSINF(4);
		% filter out negative infinite eigenvalues to Inf
		MINUSINFTOINF(8);
		% filter out positive infinite eigenvalues to 0
		INFTOZERO(16);
		% filter out negative infinite eigenvalues to 0
		MINUSINFTOZERO(32);
	end

% must not be private to allow for type cast to GammaEigenvalueFilterType, types are automatically restricted to the defined ones internally
% 	methods(Access=private)
% 		function [this] = GammaEigenvalueFilterType(type)
% 			%GAMMAEIGENVALUEFILTERTYPE create new gamma eigenvalue filter type object
% 			%	Input:
% 			%		type:	number for eigenvalue filter type
% 			%	Output:
% 			%		this:	instance of class
% 			this@Simulink.IntEnumType(type);
% 		end
% 	end

	methods(Static=true)
		function [default] = getDefaultValue()
			%GETDEFAULTVALUE return default gamma eigenvalue filter type
			%	Output:
			%		default:	default eigenvalue filter type
			default = GammaEigenvalueFilterType.NONE;
		end

		function [description] = getDescription()
			%GETDESCRIPTION	String to describe the class in Simulink Coder
			%	Output:
			%		description:	description of the class
			description = 'enumeration for characterization of different methods to filter eigenvalues in gamma pole placement for use with codegen';
		end

		function [addname] = addClassNameToEnumNames()
			%ADDCLASSNAMETOENUMNAMES Control whether class name is added as a prefix to enumerated names in the generated code.
			%	Output:
			%		addname:	true, to add the class name to generated code to avoid naming conflicts
			addname = true;
		end

		function [filter] = fromname(name)
			%FROMNAME create GammaEigenvalueFilterType from name
			%	Input:
			%		name:					name of a defined GammaEigenvalueFilterType
			%	Output:
			%		multiplicityhandling:	GammaEigenvalueFilterType, if one of the specified name exists
			if isa(name, 'GammaEigenvalueFilterType')
				filter = name;
				return;
			end
			enum = enumeration('GammaEigenvalueFilterType');
			maxvaluechar = length(sprintf('%d', max(enum)));
			if ischar(name)
				filter = [];
				for ii = 1:length(enum)
					if isinteger(name) && name == int32(enum(ii))
						filter = enum(ii);
						return;
					end
					if isnumeric(name) && floor(name) == ceil(name) && floor(name) == int32(enum(ii))
						filter = enum(ii);
						return;
					end
					if ischar(name) && strcmpi(name, char(enum(ii)))
						filter = enum(ii);
						return;
					end
					if ischar(name) && length(name) <= maxvaluechar
						filter = GammaEigenvalueFilterType.fromDCM(str2double(name));
						return;
					end
				end
			else
				filter = repmat(GammaEigenvalueFilterType.getDefaultValue(), length(name), 1);
				hasfilter = false(length(name), 1);
				for jj = 1:length(name)
					for ii = 1:length(enum)
						if isinteger(name(jj)) && name(jj) == int32(enum(ii))
							filter(jj, 1) = enum(ii);
							hasfilter(jj, 1) = true;
							break;
						end
						if isnumeric(name(jj)) && floor(name(jj)) == ceil(name(jj)) && floor(name(jj)) == int32(enum(ii))
							filter(jj, 1) = enum(ii);
							hasfilter(jj, 1) = true;
							break;
						end
					end
				end
				if ~all(hasfilter)
					filter = [];
				else
					filter = reshape(filter, size(name));
				end
			end
			if isempty(filter)
				error('control:design:gamma:filtertype:name', 'No GammaEigenvalueFilterType of specified name exists.');
			end
		end

		function [filter] = extract(name)
			%EXTRACT create GammaEigenvalueFilterType from name
			%	Input:
			%		name:					name of a defined GammaEigenvalueFilterType
			%	Output:
			%		multiplicityhandling:	GammaEigenvalueFilterType, if one of the specified name exists
			if isa(name, 'GammaEigenvalueFilterType')
				filter = name;
				return;
			end
			if ischar(name)
				filter = GammaEigenvalueFilterType.fromname(name);
				return;
			else
				pow2 = nextpow2(name) == name;
				if all(pow2(:))
					filter = GammaEigenvalueFilterType.fromname(name);
					return;
				end
				enum = enumeration('GammaEigenvalueFilterType');
				filter = cell(length(name), length(enum));
				hasfilter = false(length(name), length(enum));
				for jj = 1:length(name)
					for ii = 1:length(enum)
						if isinteger(name(jj)) && name(jj) == int32(enum(ii))
							filter{jj, ii} = enum(ii);
							hasfilter(jj, ii) = true;
						end
						if isinteger(name(jj)) && bitand(name(jj), int32(enum(ii)))
							filter{jj, ii} = enum(ii);
							hasfilter(jj, ii) = true;
						end
						if isnumeric(name(jj)) && floor(name(jj)) == ceil(name(jj)) && floor(name(jj)) == int32(enum(ii))
							filter{jj, ii} = enum(ii);
							hasfilter(jj, ii) = true;
						end
						if isnumeric(name(jj)) && floor(name(jj)) == ceil(name(jj)) && bitand(floor(name(jj)), int32(enum(ii)))
							filter{jj, ii} = enum(ii);
							hasfilter(jj, ii) = true;
						end
					end
				end
				% TODO: check for remainder after extraction of valid types?
				[filter{~hasfilter}] = deal(GammaEigenvalueFilterType.NONE);
				if ~all(any(hasfilter, 2))
					filter = [];
				else
					enums = max(sum(hasfilter, 2));
					filter_reduced = repmat(GammaEigenvalueFilterType.getDefaultValue(), [size(name), enums]);
					for ii = 1:size(filter, 1)
						[idx1, idx2] = ind2sub(size(name), ii);
						filter_reduced(idx1, idx2, :) = [filter{ii, hasfilter(ii, :)}, repmat(GammaEigenvalueFilterType.NONE, 1, enums - sum(hasfilter(ii, :)))];
					end
					filter = filter_reduced;
				end
			end
			if isempty(filter)
				error('control:design:gamma:filtertype:name', 'No GammaEigenvalueFilterType of specified name exists.');
			end
		end

		function [fromDCM] = fromDCM(DCMstring, ~)
			%FROMDCM convert string from DCM file to object of GammaEigenvalueFilterType class
			%	Input:
			%		DCMstring:	string to convert from
			%		name:		optional name of parameter
			%	Output:
			%		fromDCM:	GammaEigenvalueFilterType, if one of the specified name exists
			fromDCM = GammaEigenvalueFilterType.fromname(DCMstring);
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
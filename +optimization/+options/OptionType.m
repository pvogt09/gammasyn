classdef OptionType < uint8
	%OPTIONTYPE type of optimization options
	
	enumeration
		% optiomoptions optimization options
		OPTIMOPTIONS(1);
		% optimset optimization options
		OPTIMSET(2);
		% structure optimization options
		STRUCT(3);
		% gaoptimset optimization options
		GAOPTIMSET(4);
		% saoptimset optimization options
		SAOPTIMSET(5);
		% psoptimset optimization options
		PSOPTIMSET(6);
	end
end
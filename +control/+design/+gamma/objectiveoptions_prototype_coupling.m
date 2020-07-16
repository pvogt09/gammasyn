function [objectiveoption_prototype] = objectiveoptions_prototype_coupling()
	%OBJECTIVEOPTIONS_PROTOTYPE_COUPLING return prototype structure for coupling control objective options
	%	Output:
	%		objectiveoption_prototype:	prototype for coupling control objective options
	objectiveoption_prototype = struct(...
		'couplingconditions',			uint32(0),...
		'couplingstrategy',				GammaCouplingStrategy.getDefaultValue(),...
		'tolerance_coupling',			NaN,...
		'tolerance_prefilter',			NaN,...
		'solvesymbolic',				true,...
		'round_equations_to_digits',	NaN...
	);
end
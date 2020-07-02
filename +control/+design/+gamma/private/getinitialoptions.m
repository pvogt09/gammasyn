function [initialoptions] = getinitialoptions(initialoptions, objectiveoptions)
	%GETINITIALOPTIONS combine objectiveoptions and initial options, if objectiveoptions contain a field 'initial'
	%	Input:
	%		initialoptions:		structure with options for initial value calculation
	%		objectiveoptions:	structure with options for objective functions
	%	Output:
	%		initialoptions:		structure with options for initial value calculation
	if nargin <= 0
		initialoptions = struct();
	end
	if nargin <= 1
		objectiveoptions = struct();
	end
	if isa(objectiveoptions, 'control.design.gamma.GammasynOptions')
		objectiveoptions = struct(objectiveoptions);
	end
	if isfield(objectiveoptions, 'initial')
		if ~isfield(initialoptions, 'usesystem') && isfield(objectiveoptions.initial, 'usesystem')
			initialoptions.usesystem = objectiveoptions.initial.usesystem;
		end
		if ~isfield(initialoptions, 'eigenvalues') && isfield(objectiveoptions.initial, 'eigenvalues')
			initialoptions.eigenvalues = objectiveoptions.initial.eigenvalues;
		end
		if ~isfield(initialoptions, 'eigenvalueweight') && isfield(objectiveoptions.initial, 'eigenvalueweight')
			initialoptions.eigenvalueweight = objectiveoptions.initial.eigenvalueweight;
		end
		if ~isfield(initialoptions, 'optimization') && isfield(objectiveoptions.initial, 'optimization')
			if isfield(objectiveoptions.initial.optimization, 'algorithm')
				initialoptions.optimization.algorithm = objectiveoptions.initial.optimization.algorithm;
			end
			if isfield(objectiveoptions.initial.optimization, 'options')
				initialoptions.optimization.options = objectiveoptions.initial.optimization.options;
			else
				initialoptions.optimization.options = solveroptions;
			end
			if isfield(objectiveoptions.initial.optimization, 'x_0')
				initialoptions.optimization.x_0 = objectiveoptions.initial.optimization.x_0;
			end
		end
	end
end
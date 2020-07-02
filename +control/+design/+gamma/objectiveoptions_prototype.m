function [objectiveoption_prototype] = objectiveoptions_prototype()
	%OBJECTIVEOPTIONS_PROTOTYPE return prototype structure for objective options
	%	Output:
	%		objectiveoption_prototype:	prototype for objective options
	% TODO: add deprecation warning
	objectiveoption_prototype = struct(...
		'usecompiled',			false,...
		'numthreads',			uint32(configuration.matlab.numthreads()),...
		'type',					GammaJType.ZERO,...
		'weight',				0,...
		'allowvarorder',		false,...
		'eigenvaluederivative',	GammaEigenvalueDerivativeType.getDefaultValue(),...
		'eigenvaluefilter',		GammaEigenvalueFilterType.getDefaultValue(),...
		'objective',			struct(...
			'preventNaN',		false,...
			'kreisselmeier',	struct(...
				'rho',				20,...
				'max',				(0)...
			),...
			'lyapunov',			struct(...
				'Q',				[]...
			),...
			'normgain',			struct(...
				'R',	[],...
				'K',	[],...
				'F',	[]...
			)...
		)...
	);
end
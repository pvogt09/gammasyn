function [config] = constant(config)
	%CONSTANT Einstellungen für die Codegenerierung
	%	Input:
	%		config:	Konfigurationseinstellung, die verändert werden soll
	%	Output:
	%		config:	Konfigurationseinstellung für die Codegenerierung
	if nargin < 1
		config = coder.config('mex');
	else
		if ~isa(config, 'coder.MexCodeConfig')
			error('TPFSW:codegen', 'Coder Einstellungen sind müssen vom Typ ''coder.MexCodeConfig'' sein.');
		end
	end
	config.IntegrityChecks			= ~verLessThan('matlab', '7.14');
	config.ResponsivenessChecks		= false;
	config.EchoExpressions			= false;
	config.GenerateComments			= true;
	config.MATLABSourceComments		= true;
	config.EnableDebugging			= false;
	config.GenerateReport			= true;
	config.TargetLang				= 'C++';
	%config.GenCodeOnly				= true;
	config.EnableAutoExtrinsicCalls	= false;
	if verLessThan('matlab', '7.14')
		config.DynamicMemoryAllocation	= 'AllVariableSizeArrays';
	else
		config.DynamicMemoryAllocation	= 'Threshold';
	end
end
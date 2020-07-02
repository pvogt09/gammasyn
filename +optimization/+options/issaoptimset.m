function [is] = issaoptimset(set)
	%ISSAOPTIMSET return if a structure is a saoptimset
	%	Input:
	%		set:	structure to check
	%	Output:
	%		is:		true, if the supplied structure is a saoptimset, else false
	is = isstruct(set);
	if is
		is = all(isfield(set, {
			'AnnealingFcn';
			'TemperatureFcn';
			'AcceptanceFcn';
			'TolFun';
			'StallIterLimit';
			'MaxFunEvals';
			'TimeLimit';
			'MaxIter';
			'ObjectiveLimit';
			'Display';
			'DisplayInterval';
			'HybridFcn';
			'HybridInterval';
			'PlotFcns';
			'PlotInterval';
			'OutputFcns';
			'InitialTemperature';
			'ReannealInterval';
			'DataType';
		}));
	end
end
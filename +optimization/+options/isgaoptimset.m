function [is] = isgaoptimset(set)
	%ISGAOPTIMSET return if a structure is a gaoptimset
	%	Input:
	%		set:	structure to check
	%	Output:
	%		is:		true, if the supplied structure is a gaoptimset, else false
	is = isstruct(set);
	if is
		is = all(isfield(set, {
			'PopulationType';
			'PopInitRange';
			'PopulationSize';
			'EliteCount';
			'CrossoverFraction';
			'ParetoFraction';
			'MigrationDirection';
			'MigrationInterval';
			'MigrationFraction';
			'Generations';
			'TimeLimit';
			'FitnessLimit';
			'StallGenLimit';
			'StallTest';
			'StallTimeLimit';
			'TolFun';
			'TolCon';
			'InitialPopulation';
			'InitialScores';
			'NonlinConAlgorithm';
			'InitialPenalty';
			'PenaltyFactor';
			'PlotInterval';
			'CreationFcn';
			'FitnessScalingFcn';
			'SelectionFcn';
			'CrossoverFcn';
			'MutationFcn';
			'DistanceMeasureFcn';
			'HybridFcn';
			'Display';
			'PlotFcns';
			'OutputFcns';
			'Vectorized';
			'UseParallel';
		}));
	end
end
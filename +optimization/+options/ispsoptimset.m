function [is] = ispsoptimset(set)
	%ISPSOPTIMSET return if a structure is a psoptimset
	%	Input:
	%		set:	structure to check
	%	Output:
	%		is:		true, if the supplied structure is a psoptimset, else false
	is = isstruct(set);
	if is
		is = all(isfield(set, {
			'TolMesh';
			'TolCon';
			'TolX';
			'TolFun';
			'TolBind';
			'MaxIter';
			'MaxFunEvals';
			'TimeLimit';
			'MeshContraction';
			'MeshExpansion';
			'MeshAccelerator';
			'MeshRotate';
			'InitialMeshSize';
			'ScaleMesh';
			'MaxMeshSize';
			'InitialPenalty';
			'PenaltyFactor';
			'PollMethod';
			'CompletePoll';
			'PollingOrder';
			'SearchMethod';
			'CompleteSearch';
			'Display';
			'OutputFcns';
			'PlotFcns';
			'PlotInterval';
			'Cache';
			'CacheSize';
			'CacheTol';
			'Vectorized';
			'UseParallel';
		}));
	end
end
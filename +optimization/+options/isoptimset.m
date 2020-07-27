function [is] = isoptimset(set)
	%ISOPTIMSET return if a structure is an optimset
	%	Input:
	%		set:	structure to check
	%	Output:
	%		is:		true, if the supplied structure is an optimset, else false
	is = isstruct(set);
	if is
		is = all(isfield(set, {
			'Display';
			'MaxFunEvals';
			'MaxIter';
			'TolFun';
			'TolX';
			'FunValCheck';
			'OutputFcn';
			'PlotFcns';
			'ActiveConstrTol';
			'Algorithm';
			'AlwaysHonorConstraints';
			'DerivativeCheck';
			'Diagnostics';
			'DiffMaxChange';
			'DiffMinChange';
			'FinDiffRelStep';
			'FinDiffType';
			'GoalsExactAchieve';
			'GradConstr';
			'GradObj';
			'HessFcn';
			'Hessian';
			'HessMult';
			'HessPattern';
			'HessUpdate';
			'InitBarrierParam';
			'InitTrustRegionRadius';
			'Jacobian';
			'JacobMult';
			'JacobPattern';
			'LargeScale';
			'MaxNodes';
			'MaxPCGIter';
			'MaxProjCGIter';
			'MaxSQPIter';
			'MaxTime';
			'MeritFunction';
			'MinAbsMax';
			'NoStopIfFlatInfeas';
			'ObjectiveLimit';
			'PhaseOneTotalScaling';
			'Preconditioner';
			'PrecondBandWidth';
			'RelLineSrchBnd';
			'RelLineSrchBndDuration';
			'ScaleProblem';
			'SubproblemAlgorithm';
			'TolCon';
			'TolConSQP';
			'TolGradCon';
			'TolPCG';
			'TolProjCG';
			'TolProjCGAbs';
			'TypicalX';
			'UseParallel';
		}));
		if is
			if matlab.Version.CURRENT <= matlab.Version.R2015B
				is = all(isfield(set, {
					'InitialHessType';
					'InitialHessMatrix';
					'Simplex';
				}));
			end
		end
	end
end
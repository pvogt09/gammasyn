function [value] = convertfield(solver, field, value, optiontype, replacechar)
	%CONVERTPROPERTY convert a property to the specified option type
	%	Input:
	%		solver:			instance of solver options
	%		name:			name of the option to convert
	%		value:			value of the option to convert
	%		optionstype:	type of option set, the value is returned for
	%		replacechar:	indicator, if char options should be replaced by corresponding numerical values
	%	Output:
	%		value:			value to set
	if nargin <= 4
		replacechar = false;
	end
	if replacechar
		if ~isempty(solver.NumberVariables) && ~isnan(solver.NumberVariables)
			numvar = solver.NumberVariables;
		else
			error('optimization:options:replace', 'Problem dimensions have to be specified, when variable size options are used.');
		end
		if ~isempty(solver.NumberConstraintsInequality) && ~isnan(solver.NumberConstraintsInequality)
			numineq = solver.NumberConstraintsInequality;
		else
			error('optimization:options:replace', 'Problem dimensions have to be specified, when variable size options are used.');
		end
		if ~isempty(solver.NumberConstraintsEquality) && ~isnan(solver.NumberConstraintsEquality)
			numeq = solver.NumberConstraintsEquality;
		else
			error('optimization:options:replace', 'Problem dimensions have to be specified, when variable size options are used.');
		end
		if ~isempty(solver.NumberConstraintsBounds) && ~isnan(solver.NumberConstraintsBounds)
			numbounds = solver.NumberConstraintsBounds;
		else
			error('optimization:options:replace', 'Problem dimensions have to be specified, when variable size options are used.');
		end
	else
		numvar = 8;
		numineq = 3;
		numeq = 3;
		numbounds = 1;
	end
	switch field
		case {'OutputFcn','OutputFcns','PlotFcn','PlotFcns','CreationFcn','HybridFcn'}% function or empty
		case {'Display'}
			if any(optiontype == [
				optimization.options.OptionType.GAOPTIMSET;
				optimization.options.OptionType.PSOPTIMSET;
				optimization.options.OptionType.SAOPTIMSET;
				optimization.options.OptionType.STRUCT
			])
				displaymapping = solver.displaymapping();
				idxval = strcmpi(value, displaymapping(:, 2));
				if ~any(idxval)
					idxval = strcmpi(value, displaymapping(:, 1));
					if ~any(idxval)
					else
						idxkey = find(idxval, 1, 'first');
						value = displaymapping{idxkey, 2};
					end
				else
					idxkey = find(idxval, 1, 'first');
					value = displaymapping{idxkey, 2};
				end
			end
		case {'FunValCheck'}
			if optiontype ~= optimization.options.OptionType.STRUCT
				if ~isempty(value)
					value = logicaltoonoff(value);
				end
			end
		case{'Vectorized'} % off,on
		case {'RelLineSrchBnd', 'RelLineSearchBound'}
		case {'TolFun','FunctionTolerance','OptimalityTolerance','TolX','StepTolerance','TolCon','ConstraintTolerance','TolPCG','ActiveConstrTol',...
				'DiffMaxChange','DiffMinChange','MaxTime', ...
				'TolProjCGAbs', 'TolProjCG','TolGradCon','TolConSQP',...
				'TolGapAbs', 'InitDamping'}
			if ischar(value)
				if replacechar
					value = evaloption(field, value, numvar, numineq, numeq, numbounds);
				end
			end
		case {'TolFunLP'}
		case {'TolGapRel', 'RelObjThreshold'}
		case {'TolInteger'}
		case {'ObjectiveLimit'}
		case {'SwarmSize'}
		case {'MinFractionNeighbors'}
		case {'LargeScale','Simplex','NoStopIfFlatInfeas','PhaseOneTotalScaling'}
		case {'DerivativeCheck','CheckGradients','Diagnostics','GradConstr','SpecifyConstraintGradient','GradObj','SpecifyObjectiveGradient','Jacobian'}
			if optiontype ~= optimization.options.OptionType.STRUCT
				if ~isempty(value)
					value = logicaltoonoff(value);
				end
			end
		case {'PrecondBandWidth','MinAbsMax','GoalsExactAchieve', ...
				'RelLineSrchBndDuration', 'RelLineSearchBoundDuration', 'DisplayInterval', ...
				'RootLPMaxIter', 'MaxFunEvals','MaxFunctionEvaluations', 'MaxProjCGIter', ...
				'MaxSQPIter', 'MaxPCGIter', 'MaxNodes', 'MaxIter','MaxIterations','Retries'}
			if ischar(value)
				if replacechar
					value = evaloption(field, value, numvar, numineq, numeq, numbounds);
				end
			end
		case {'StallIterLimit'}
		case {'InitialSwarm'}
		case {'JacobPattern', 'JacobianPattern', 'HessPattern', 'HessianPattern'}
			if ischar(value)
				if replacechar
					value = evaloption(field, value, numvar, numineq, numeq, numbounds);
				end
			end
		case {'TypicalX'}
			if ischar(value)
				if replacechar
					value = evaloption(field, value, numvar, numineq, numeq, numbounds);
				end
			end
		case {'HessMult', 'HessianMult', 'HessFcn', 'HessianFcn', 'JacobMult', 'JacobianMult'}
		case {'HessUpdate', 'HessianUpdate'}
		case {'MeritFunction'}
		case {'InitialHessType'}
		case {'UseParallel'}
			switch optiontype
				case optimization.options.OptionType.STRUCT
				case optimization.options.OptionType.OPTIMOPTIONS
				case optimization.options.OptionType.GAOPTIMSET
				case optimization.options.OptionType.PSOPTIMSET
				case optimization.options.OptionType.SAOPTIMSET
				case optimization.options.OptionType.OPTIMSET
					if value
						value = 'always';
					else
						value = 'never';
					end
				otherwise
					error('optimization:options:convert', 'Undefined option type ''%s''.', class(optiontype));
			end
		case {'Algorithm'}
		case {'AlwaysHonorConstraints'}
			if optiontype ~= optimization.options.OptionType.STRUCT
				if strcmpi(value, 'none')
					%value = 'off';
				elseif strcmpi(value, 'bounds')
					%value = 'on';
				end
			end
		case {'HonorBounds'}
			if optiontype ~= optimization.options.OptionType.STRUCT
				if islogical(value)
					%value = logicaltoonoff(value);
					if value
						value = 'bounds';
					else
						value = 'none';
					end
				else
					if strcmpi(value, 'none')
						%value = 'off';
					elseif strcmpi(value, 'bounds')
						%value = 'on';
					end
				end
			end
		case {'ScaleProblem'}
		case {'FinDiffType', 'FiniteDifferenceType'}
		case {'FinDiffRelStep', 'FiniteDifferenceStepSize'}
			if ischar(value)
				if replacechar
					value = evaloption(field, value, numvar, numineq, numeq, numbounds);
				end
			end
		case {'Hessian','HessianApproximation'}
		case {'SubproblemAlgorithm'}
		case {'InitialHessMatrix', 'InitialSwarmSpan'}
		case {'BranchingRule', 'Heuristics', 'NodeSelection', 'CutGeneration', ...
				'IPPreprocess', 'LPPreprocess', 'Preprocess', 'RootLPAlgorithm'}
		case {'InitBarrierParam', 'InitTrustRegionRadius', 'StallTimeLimit'}
			if ischar(value)
				if replacechar
					value = evaloption(field, value, numvar, numineq, numeq, numbounds);
				end
			end
		case {'SelfAdjustment', 'SocialAdjustment'}
		case {'InertiaRange'}
		case {'PresolveOps'}
		case {'CutGenMaxIter'}
		case {'MaxNumFeasPoints', 'LPMaxIter', 'HeuristicsMaxNodes'}
		case {'ObjectiveCutOff'}
		otherwise
	end
end
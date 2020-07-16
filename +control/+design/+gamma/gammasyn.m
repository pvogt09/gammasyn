function [Ropt, Jopt, information] = gammasyn(systems, areafun, weights, R_fixed, R_0, solveroptions, objectiveoptions, R_bounds, R_nonlin)
	%GAMMASYN robust pole placement for multi-models
	%	Input:
	%		systems:			structure/cell array or matrix with dynamic systems to take into consideration
	%		areafun:			area border functions and gradients as cell array with number of system entries, each function returns a row vector with as much elements as area border functions are defined for the corresponding system
	%		weights:			weighting matrix with number of systems columns and number of pole area border functions rows
	%		R_fixed:			cell array with indicator matrix for gain elements that should be fixed and the values the fixed gains have, empty if no fixed elements are needed. For linear dependent gain components the first element is a 3D matrix with relation in the third dimension and the second element a vector of constant constraint elements
	%		R_0:				initial gain matrix
	%		solveroptions:		options for optimization algorithm to use
	%		objectiveoptions:	options for problem functions and problem formulation
	%		R_bounds:			cell array with indicator matrix for gain elements that should be bounded and the values the gains are bounded by, empty if no bounded elements are needed. For linear dependent gain components the first element is a 3D matrix with relation in the third dimension and the second element a vector of upper bound constraint elements
	%		R_nonlin:			function pointer to a function of nonlinear inequality and equality constraints on gains with signature [c_R, ceq_R, c_K, ceq_K, c_F, ceq_F, gradc_R, gradceq_R, gradc_K, gradceq_K, gradc_F, gradceq_F] = constraint(K, D, F)
	%	Output:
	%		Ropt:				optimal value for gain
	%		Jopt:				optimal objective function value for optimal gain
	%		information:		srtucture with information about optimization runs
	if nargin <= 3
		[R_fixed, K_fixed, F_fixed, RKF_fixed] = input_gain_constraint2RKF();
	else
		[R_fixed, K_fixed, F_fixed, RKF_fixed] = input_gain_constraint2RKF(R_fixed);
	end
	if nargin <= 7
		[R_bounds, K_bounds, F_bounds, RKF_bounds] = input_gain_constraint2RKF();
	else
		[R_bounds, K_bounds, F_bounds, RKF_bounds] = input_gain_constraint2RKF(R_bounds);
	end
	if nargin <= 8
		R_nonlin = [];
	end
	allowvarorder = false;
	allownegativeweight = false;
	systemoptions = struct();
	if nargin >= 7
		if isa(objectiveoptions, 'control.design.gamma.GammasynOptions')
			objectiveoptions = objectiveoptions.userstruct();
		end
		if isstruct(objectiveoptions)
			if isfield(objectiveoptions, 'allowvarorder')
				allowvarorder = objectiveoptions.allowvarorder;
			end
			if isfield(objectiveoptions, 'allownegativeweight')
				allownegativeweight = objectiveoptions.allownegativeweight;
			end
			if isfield(objectiveoptions, 'system')
				systemoptions = objectiveoptions.system;
			end
			if isfield(objectiveoptions, 'couplingcontrol')
				% Coupling design is to be done, if objectiveoptions.couplingcontrol exists (except couplingstrategy is NONE)
				% couplingcontrol options have to come as a struct
				% A valid couplingstrategy of type GammaCouplingStrategy must always be specified
				% Only the strategies NONE, NUMERIC_NONLINEAR_EQUALITY, NUMERIC_NONLINEAR_INEQUALITY play a role here:
				%	NONE:							to switch the coupling design off (even if complete options are provided)
				%	NUMERIC_NONLINEAR_EQUALITY:		triggers calculation of coupling conditions as constraints for optimization
				%	NUMERIC_NONLINEAR_INEQUALITY:	triggers calculation of coupling conditions as constraints for optimization
				%	all other strategies:			if they are chosen, the coupling problem is solved outside gammasyn. The results influence gammasyn through K_fixed. Thus, gammasyn is executed normally, just with constrained controller matrices.
				% Number of coupling conditions must always fulfill 0 < number_couplingconditions <= number_references
				if ~isstruct(objectiveoptions.couplingcontrol)
					error('control:design:gamma:input', 'Coupling control options must be of type struct.');
				end
				couplingoptions = checkobjectiveoptions_coupling(objectiveoptions.couplingcontrol);
				if couplingoptions.couplingstrategy ~= GammaCouplingStrategy.NONE
					systemoptions.couplingcontrol = GammaCouplingStrategy_needscouplingconditions(couplingoptions.couplingstrategy);
					if systemoptions.couplingcontrol
						systemoptions.couplingconditions = int32(couplingoptions.couplingconditions);
						systemoptions.usereferences = true;
						objectiveoptions.system.usereferences = true;
					end
				end
			end
		end
	end
	[system, areafun_strict, areafun_loose, weight_strict, weight_loose, dimensions_strict, dimensions_loose, number_states, bounds, nonlcon] = checkandtransformargs(systems, areafun, weights, systemoptions, R_fixed, K_fixed, F_fixed, RKF_fixed, allowvarorder, allownegativeweight, R_bounds, K_bounds, F_bounds, RKF_bounds, R_nonlin);
	if nargin <= 5
		solveroptions = checkandtransform_solveroptions();
	else
		solveroptions = checkandtransform_solveroptions(solveroptions);
	end
	if nargin <= 4
		R_0 = zeros(dimensions_strict.controls, dimensions_strict.measurements);
	end
	if isa(R_0, 'control.design.gamma.InitialValueElement')
		R_0 = control.design.gamma.InitialValue(R_0);
	end
	if isa(R_0, 'control.design.gamma.InitialValue')
		initialoptions = getinitialoptions(struct(), objectiveoptions);
		if isempty(fieldnames(initialoptions))
			initialoptions = [];
		end
		R_0 = R_0.getall(initialoptions, systems, areafun, weights, R_fixed, R_bounds, R_nonlin);
	end
	[R_0, K_0, F_0] = checkinitialRKF(R_0, dimensions_strict);
	x_0 = checkinitialR({R_0, K_0, F_0}, dimensions_strict);
	[A, b, lb, ub] = R2bound(dimensions_strict, bounds);
	[objective_options_strict, objective_options_loose, solution_strategy, errorhandler] = checkobjectiveoptions(objectiveoptions, x_0, number_states, system, solveroptions, dimensions_strict, dimensions_loose, areafun_strict, areafun_loose, weight_strict, weight_loose);
	J_feasibility = [];
	J_coupling = [];
	c_coupling = [];
	needshessian = solveroptions.supportsHessian();
	if solveroptions.ProblemType == optimization.options.ProblemType.CONSTRAINED
		needshessian = needshessian && solveroptions.SpecifyObjectiveHessian && solveroptions.SpecifyConstraintHessian;
	else
		needshessian = needshessian && solveroptions.SpecifyObjectiveHessian;
	end
	if needshessian
		objectivetype_hessiansupport = GammaJType.hashessian(objectiveoptions.type);
		if any(~objectivetype_hessiansupport)
			warning('control:design:gamma', 'Objective type ''%s'' does not support Hessian calculation.', strjoin(cellfun(@char, num2cell(objectiveoptions.type(~objectivetype_hessiansupport)), 'UniformOutput', false), ''', '''));
		end
	end
	solve_coupling_feasibility = solution_strategy == GammaSolutionStrategy.FEASIBILITYITERATION_COUPLING && GammaCouplingStrategy_needscouplingconditions(objective_options_strict.couplingcontrol.couplingstrategy);
	if solveroptions.ProblemType == optimization.options.ProblemType.CONSTRAINED
		if dimensions_loose.areas_max ~= 0
			if all(objective_options_loose.type == GammaJType.ZERO)
				J = J_dispatcher('zero', needshessian, system, weight_strict, areafun_strict, dimensions_strict, objective_options_strict);
				if solution_strategy == GammaSolutionStrategy.FEASIBILITYITERATION
					J_feasibility = J;
				end
				if solve_coupling_feasibility
					J_coupling = J;
				end
			else
				if isa(areafun_loose, 'GammaArea') && objective_options_loose.usecompiled
					J = J_dispatcher('constr_mex', needshessian, system, weight_loose, areafun_loose, dimensions_loose, objective_options_loose);
				else
					J = J_dispatcher('constr_m', needshessian, system, weight_loose, areafun_loose, dimensions_loose, objective_options_loose);
				end
				if solution_strategy == GammaSolutionStrategy.FEASIBILITYITERATION
					J_feasibility = J_dispatcher('zero', needshessian, system, weight_strict, areafun_strict, dimensions_strict, objective_options_strict);
				end
				if solve_coupling_feasibility
					J_coupling = J_dispatcher('zero', needshessian, system, weight_strict, areafun_strict, dimensions_strict, objective_options_strict);
				end
			end
		else
			if any(objective_options_loose.type == GammaJType.EIGENVALUECONDITION) || any(GammaJType_isgainobjective(objective_options_loose.type))
				if isa(areafun_loose, 'GammaArea') && objective_options_loose.usecompiled
					J = J_dispatcher('unconstr_mex', needshessian, system, weight_loose, areafun_loose, dimensions_loose, objective_options_loose);
				else
					J = J_dispatcher('unconstr_m', needshessian, system, weight_loose, areafun_loose, dimensions_loose, objective_options_loose);
				end
			else
				J = J_dispatcher('zero', needshessian, system, weight_strict, areafun_strict, dimensions_strict, objective_options_strict);
			end
			if solution_strategy == GammaSolutionStrategy.FEASIBILITYITERATION
				J_feasibility = J_dispatcher('zero', needshessian, system, weight_strict, areafun_strict, dimensions_strict, objective_options_strict);
			end
			if solve_coupling_feasibility
				J_coupling = J_dispatcher('zero', needshessian, system, weight_strict, areafun_strict, dimensions_strict, objective_options_strict);
			end
		end
		if isa(areafun_strict, 'GammaArea') && objective_options_strict.usecompiled
			c = c_dispatcher('constr_mex', needshessian, system, weight_strict, areafun_strict, dimensions_strict, objective_options_strict, nonlcon);
		else
			c = c_dispatcher('constr_m', needshessian, system, weight_strict, areafun_strict, dimensions_strict, objective_options_strict, nonlcon);
		end
		if solve_coupling_feasibility
			areafun_coupling = GammaArea.empty();
			dimensions_coupling = dimensions_strict;
			dimensions_coupling.area_args = 2*ones(size(areafun_coupling, 1), 1);
			dimensions_coupling.area_parts = size(areafun_coupling, 2);
			dimensions_coupling.areas_max = size(areafun_coupling, 2);
			dimensions_coupling.area_hasgrad = true;
			dimensions_coupling.area_hashess = true;
			dimensions_coupling.area_parameters = control.design.gamma.area.GammaArea.PARAMETERPROTOTYPEEMPTY;
			if objective_options_strict.usecompiled
				c_coupling = c_dispatcher('constr_mex', needshessian, system, zeros(size(system, 1), 0), areafun_coupling, dimensions_strict, objective_options_strict, nonlcon);
			else
				c_coupling = c_dispatcher('constr_m', needshessian, system, zeros(size(system, 1), 0), areafun_coupling, dimensions_strict, objective_options_strict, nonlcon);
			end
		end
	elseif solveroptions.ProblemType == optimization.options.ProblemType.UNCONSTRAINED
		if isa(areafun_strict, 'GammaArea') && objective_options_strict.usecompiled
			J = J_dispatcher('unconstr_mex', needshessian, system, weight_strict, areafun_strict, dimensions_strict, objective_options_strict);
		else
			J = J_dispatcher('unconstr_m', needshessian, system, weight_strict, areafun_strict, dimensions_strict, objective_options_strict);
		end
		c = [];
	elseif solveroptions.ProblemType == optimization.options.ProblemType.CONSTRAINEDMULTI
		if dimensions_loose.areas_max ~= 0
			if all(objective_options_loose.type == GammaJType.ZERO)
				J = J_dispatcher('zero', needshessian, system, weight_strict, areafun_strict, dimensions_strict, objective_options_strict);
				if solution_strategy == GammaSolutionStrategy.FEASIBILITYITERATION
					J_feasibility = J;
				end
				if solve_coupling_feasibility
					J_coupling = J;
				end
			else
				if isa(areafun_loose, 'GammaArea') && objective_options_loose.usecompiled
					J = J_dispatcher('constr_multi_mex', needshessian, system, weight_loose, areafun_loose, dimensions_loose, objective_options_loose);
				else
					J = J_dispatcher('constr_multi_m', needshessian, system, weight_loose, areafun_loose, dimensions_loose, objective_options_loose);
				end
				if solution_strategy == GammaSolutionStrategy.FEASIBILITYITERATION
					J_feasibility = J_dispatcher('zero', needshessian, system, weight_strict, areafun_strict, dimensions_strict, objective_options_strict);
				end
				if solve_coupling_feasibility
					J_coupling = J_dispatcher('zero', needshessian, system, weight_strict, areafun_strict, dimensions_strict, objective_options_strict);
				end
			end
		else
			J = J_dispatcher('zero', needshessian, system, weight_strict, areafun_strict, dimensions_strict, objective_options_strict);
			if solution_strategy == GammaSolutionStrategy.FEASIBILITYITERATION
				J_feasibility = J;
			end
			if solve_coupling_feasibility
				J_coupling = J;
			end
		end
		if isa(areafun_strict, 'GammaArea') && objective_options_strict.usecompiled
			c = c_dispatcher('constr_multi_mex', needshessian, system, weight_strict, areafun_strict, dimensions_strict, objective_options_strict, nonlcon);
		else
			c = c_dispatcher('constr_multi_m', needshessian, system, weight_strict, areafun_strict, dimensions_strict, objective_options_strict, nonlcon);
		end
		if solve_coupling_feasibility
			areafun_coupling = GammaArea.empty();
			dimensions_coupling = dimensions_strict;
			dimensions_coupling.area_args = 2*ones(size(areafun_coupling, 1), 1);
			dimensions_coupling.area_parts = size(areafun_coupling, 2);
			dimensions_coupling.areas_max = size(areafun_coupling, 2);
			dimensions_coupling.area_hasgrad = true;
			dimensions_coupling.area_hashess = true;
			dimensions_coupling.area_parameters = control.design.gamma.area.GammaArea.PARAMETERPROTOTYPEEMPTY;
			if objective_options_strict.usecompiled
				c_coupling = c_dispatcher('constr_mex', needshessian, system, zeros(size(system, 1), 0), areafun_coupling, dimensions_strict, objective_options_strict, nonlcon);
			else
				c_coupling = c_dispatcher('constr_m', needshessian, system, zeros(size(system, 1), 0), areafun_coupling, dimensions_strict, objective_options_strict, nonlcon);
			end
		end
	elseif solveroptions.ProblemType == optimization.options.ProblemType.UNCONSTRAINEDMULTI
		if isa(areafun_strict, 'GammaArea') && objective_options_strict.usecompiled
			J = J_dispatcher('unconstr_multi_mex', needshessian, system, weight_strict, areafun_strict, dimensions_strict, objective_options_strict);
		else
			J = J_dispatcher('unconstr_multi_m', needshessian, system, weight_strict, areafun_strict, dimensions_strict, objective_options_strict);
		end
		c = [];
	else
		error('control:design:gamma', 'Undefined problem type ''%s''.', upper(solveroptions.ProblemType));
	end
	if solution_strategy == GammaSolutionStrategy.FEASIBILITYITERATION_COUPLING && any(solveroptions.ProblemType == [
		optimization.options.ProblemType.CONSTRAINED;
		optimization.options.ProblemType.CONSTRAINEDMULTI
	]) && ~isempty(c_coupling) && ~isempty(J_coupling)
		% solve coupling control feasibility problem before solving actual problem to get feasible initial points
		haderror = false;
		try
			[x_opt_coupling, Jopt_coupling, ~, output_coupling] = optimization.solver.optimize(J_coupling, x_0, A, b, [], [], lb, ub, c_coupling, solveroptions);
		catch e
			haderror = true;
			if strcmpi(e.identifier, 'optimization:solver:optimize:input')
				throw(MException('control:design:gamma', 'Undefined solver ''%s'' for problem type ''%s''.', char(solveroptions.Solver), upper(solveroptions.ProblemType)).addCause(e));
			else
				if errorhandler.type == GammaErrorHandler.WARNING
					warning('control:design:gamma', strrep(e.message, '\', '\\'));
					x_opt_coupling = NaN(numel(dimensions_strict.R_fixed) - dimensions_strict.R_fixed_constraints + numel(dimensions_strict.K_fixed) - dimensions_strict.K_fixed_constraints + numel(dimensions_strict.F_fixed) - dimensions_strict.F_fixed_constraints, 1);
					Jopt_coupling = NaN;
					output_coupling = optimization.options.Options.OUTPUTPROTOTYPEMULTIPLE;
					output_coupling.message = e.message;
					output_coupling.information.output = e.getReport('extended', 'hyperlinks', 'off');
				elseif errorhandler.type == GammaErrorHandler.USER
					errfun = errorhandler.function;
					[x_opt_coupling, Jopt_coupling, output_coupling] = errfun(e, size(x_0, 1));
				else
					throw(MException('control:design:gamma', 'Optimization failed.').addCause(e));
				end
			end
		end
		if ~haderror
			if any(abs(Jopt_coupling) > eps)
				error('control:design:gamma', 'Objective function value must be 0 for coupling feasibility problem solution, possibly wrong objective function selected.');
			end
			if ~isempty(output_coupling.runs)
				runs = length(output_coupling.runs);
				c_val = Inf(runs, 1);
				ceq_val = Inf(runs, 1);
				R_opt_coupling_temp = cell(runs, 1);
				K_opt_coupling_temp = cell(runs, 1);
				F_opt_coupling_temp = cell(runs, 1);
				output_coupling_runs = output_coupling.runs;
				parfor ii = 1:runs
					xmin = output_coupling_runs{ii}.information.xmin;
					[R_opt_coupling_temp{ii, 1}, K_opt_coupling_temp{ii, 1}, F_opt_coupling_temp{ii, 1}] = x2R(xmin, dimensions_strict);
					[c_val_temp, ceq_val_temp] = feval(c_coupling, xmin);
					if ~isempty(c_val_temp)
						c_val(ii, 1) = max(c_val_temp(:));
					else
						c_val(ii, 1) = -Inf;
					end
					if ~isempty(ceq_val_temp)
						ceq_val(ii, 1) = max(abs(ceq_val_temp(:)));
					else
						ceq_val(ii, 1) = -Inf;
					end
				end
				constraintssatisfied = c_val <= solveroptions.ConstraintTolerance & ceq_val <= solveroptions.ConstraintTolerance;
				if any(constraintssatisfied)
					R_opt_coupling = cat(3, R_opt_coupling_temp{constraintssatisfied});
					K_opt_coupling = cat(3, K_opt_coupling_temp{constraintssatisfied});
					F_opt_coupling = cat(3, F_opt_coupling_temp{constraintssatisfied});
				else
					[R_opt_coupling, K_opt_coupling, F_opt_coupling] = x2R(x_opt_coupling, dimensions_strict);
				end
			else
				[c_0, ceq_0] = feval(c_coupling, x_0);
				if ~isempty(c_0)
					c_0 = max(c_0(:));
				else
					c_0 = -Inf;
				end
				if ~isempty(ceq_0)
					ceq_0 = max(abs(ceq_0(:)));
				else
					ceq_0 = -Inf;
				end
				[c_val, ceq_val] = feval(c_coupling, x_opt_coupling);
				if ~isempty(c_val)
					c_val = max(c_val(:));
				else
					c_val = -Inf;
				end
				if ~isempty(ceq_val)
					ceq_val = max(abs(ceq_val(:)));
				else
					ceq_val = -Inf;
				end
				constraintssatisfied = c_val <= solveroptions.ConstraintTolerance & ceq_val <= solveroptions.ConstraintTolerance;
				constraintssatisfied_0 = c_0 <= solveroptions.ConstraintTolerance & ceq_0 <= solveroptions.ConstraintTolerance;
				if constraintssatisfied
					[R_opt_coupling, K_opt_coupling, F_opt_coupling] = x2R(x_opt_coupling, dimensions_strict);
				else
					if constraintssatisfied_0
						[R_opt_coupling, K_opt_coupling, F_opt_coupling] = x2R(x_0, dimensions_strict);
					else
						if c_val < c_0 && ceq_val < ceq_0
							[R_opt_coupling, K_opt_coupling, F_opt_coupling] = x2R(x_opt_coupling, dimensions_strict);
						else
							[R_opt_coupling, K_opt_coupling, F_opt_coupling] = x2R(x_0, dimensions_strict);
						end
					end
				end
			end
			x_0 = checkinitialR({
				R_opt_coupling, K_opt_coupling, F_opt_coupling
			}, dimensions_strict);
		else
			[R_opt, K_opt, F_opt] = x2R(x_opt_coupling, dimensions_strict);
			if dimensions_strict.measurements_xdot > 0
				Ropt = {
					R_opt;
					K_opt;
					F_opt
				};
			elseif dimensions_strict.references > 0
				Ropt = {
					R_opt;
					F_opt
				};
			else
				Ropt = R_opt;
			end
			if nargout >= 2
				Jopt = Jopt_coupling;
			end
			if nargout >= 3
				information = output_coupling;
			end
			return;
		end
	end
	if solution_strategy == GammaSolutionStrategy.FEASIBILITYITERATION && any(solveroptions.ProblemType == [
		optimization.options.ProblemType.CONSTRAINED;
		optimization.options.ProblemType.CONSTRAINEDMULTI
	]) && ~isempty(c) && ~isempty(J_feasibility)
		% solve feasibility problem before solving actual problem to get feasible initial points
		haderror = false;
		try
			[x_opt_feasibility, Jopt_feasibility, ~, output_feasibility] = optimization.solver.optimize(J_feasibility, x_0, A, b, [], [], lb, ub, c, solveroptions);
		catch e
			haderror = true;
			if strcmpi(e.identifier, 'optimization:solver:optimize:input')
				throw(MException('control:design:gamma', 'Undefined solver ''%s'' for problem type ''%s''.', char(solveroptions.Solver), upper(solveroptions.ProblemType)).addCause(e));
			else
				if errorhandler.type == GammaErrorHandler.WARNING
					warning('control:design:gamma', strrep(e.message, '\', '\\'));
					if dimensions_strict.RKF_fixed_has
						x_opt_feasibility = NaN(numel(dimensions_strict.RKF_fixed) - dimensions_strict.RKF_fixed_constraints, 1);
					else
						x_opt_feasibility = NaN(numel(dimensions_strict.R_fixed) - dimensions_strict.R_fixed_constraints + numel(dimensions_strict.K_fixed) - dimensions_strict.K_fixed_constraints + numel(dimensions_strict.F_fixed) - dimensions_strict.F_fixed_constraints, 1);
					end
					Jopt_feasibility = NaN;
					output_feasibility = optimization.options.Options.OUTPUTPROTOTYPEMULTIPLE;
					output_feasibility.message = e.message;
					output_feasibility.information.output = e.getReport('extended', 'hyperlinks', 'off');
				elseif errorhandler.type == GammaErrorHandler.USER
					errfun = errorhandler.function;
					[x_opt_feasibility, Jopt_feasibility, output_feasibility] = errfun(e, size(x_0, 1));
				else
					throw(MException('control:design:gamma', 'Optimization failed.').addCause(e));
				end
			end
		end
		if ~haderror
			if any(abs(Jopt_feasibility) > eps)
				error('control:design:gamma', 'Objective function value must be 0 for feasibility problem solution, possibly wrong objective function selected.');
			end
			if ~isempty(output_feasibility.runs)
				runs = length(output_feasibility.runs);
				c_val = Inf(runs, 1);
				ceq_val = Inf(runs, 1);
				R_opt_feasibility_temp = cell(runs, 1);
				K_opt_feasibility_temp = cell(runs, 1);
				F_opt_feasibility_temp = cell(runs, 1);
				output_feasibility_runs = output_feasibility.runs;
				parfor ii = 1:runs
					xmin = output_feasibility_runs{ii}.information.xmin;
					[R_opt_feasibility_temp{ii, 1}, K_opt_feasibility_temp{ii, 1}, F_opt_feasibility_temp{ii, 1}] = x2R(xmin, dimensions_strict);
					[c_val_temp, ceq_val_temp] = feval(c, xmin);
					if ~isempty(c_val_temp)
						c_val(ii, 1) = max(c_val_temp(:));
					else
						c_val(ii, 1) = -Inf;
					end
					if ~isempty(ceq_val_temp)
						ceq_val(ii, 1) = max(abs(ceq_val_temp(:)));
					else
						ceq_val(ii, 1) = -Inf;
					end
				end
				constraintssatisfied = c_val <= solveroptions.ConstraintTolerance & ceq_val <= solveroptions.ConstraintTolerance;
				if any(constraintssatisfied)
					R_opt_feasibility = cat(3, R_opt_feasibility_temp{constraintssatisfied});
					K_opt_feasibility = cat(3, K_opt_feasibility_temp{constraintssatisfied});
					F_opt_feasibility = cat(3, F_opt_feasibility_temp{constraintssatisfied});
				else
					[R_opt_feasibility, K_opt_feasibility, F_opt_feasibility] = x2R(x_opt_feasibility, dimensions_strict);
				end
			else
				[c_0, ceq_0] = feval(c, x_0);
				if ~isempty(c_0)
					c_0 = max(c_0(:));
				else
					c_0 = -Inf;
				end
				if ~isempty(ceq_0)
					ceq_0 = max(abs(ceq_0(:)));
				else
					ceq_0 = -Inf;
				end
				[c_val, ceq_val] = feval(c, x_opt_feasibility);
				if ~isempty(c_val)
					c_val = max(c_val(:));
				else
					c_val = -Inf;
				end
				if ~isempty(ceq_val)
					ceq_val = max(abs(ceq_val(:)));
				else
					ceq_val = -Inf;
				end
				constraintssatisfied = c_val <= solveroptions.ConstraintTolerance & ceq_val <= solveroptions.ConstraintTolerance;
				constraintssatisfied_0 = c_0 <= solveroptions.ConstraintTolerance & ceq_0 <= solveroptions.ConstraintTolerance;
				if constraintssatisfied
					[R_opt_feasibility, K_opt_feasibility, F_opt_feasibility] = x2R(x_opt_feasibility, dimensions_strict);
				else
					if constraintssatisfied_0
						[R_opt_feasibility, K_opt_feasibility, F_opt_feasibility] = x2R(x_0, dimensions_strict);
					else
						if c_val < c_0 && ceq_val < ceq_0
							[R_opt_feasibility, K_opt_feasibility, F_opt_feasibility] = x2R(x_opt_feasibility, dimensions_strict);
						else
							[R_opt_feasibility, K_opt_feasibility, F_opt_feasibility] = x2R(x_0, dimensions_strict);
						end
					end
				end
			end
			x_0 = checkinitialR({
				R_opt_feasibility, K_opt_feasibility, F_opt_feasibility
			}, dimensions_strict);
		else
			[R_opt, K_opt, F_opt] = x2R(x_opt_feasibility, dimensions_strict);
			if dimensions_strict.measurements_xdot > 0
				Ropt = {
					R_opt;
					K_opt;
					F_opt
				};
			elseif dimensions_strict.references > 0
				Ropt = {
					R_opt;
					F_opt
				};
			else
				Ropt = R_opt;
			end
			if nargout >= 2
				Jopt = Jopt_feasibility;
			end
			if nargout >= 3
				information = output_feasibility;
			end
			return;
		end
	end
	% solve actual gammsyn problem
	try
		[x_opt, Jopt, ~, output] = optimization.solver.optimize(J, x_0, A, b, [], [], lb, ub, c, solveroptions);
	catch e
		if strcmpi(e.identifier, 'optimization:solver:optimize:input')
			throw(MException('control:design:gamma', 'Undefined solver ''%s'' for problem type ''%s''.', char(solveroptions.Solver), upper(solveroptions.ProblemType)).addCause(e));
		elseif strcmpi(e.identifier, 'control:design:gamma:eigenvalues:runtimerecursion')
			throw(MException('control:design:gamma', 'Mex files for constraint and objective function have to be compiled with at least matlab R2016B to support run time recursion in Van der Aa''s method.').addCause(e));
		else
			if errorhandler.type == GammaErrorHandler.WARNING
				warning('control:design:gamma', strrep(e.message, '\', '\\'));
				if dimensions_strict.RKF_fixed_has
					x_opt = NaN(numel(dimensions_strict.RKF_fixed) - dimensions_strict.RKF_fixed_constraints, 1);
				else
					x_opt = NaN(numel(dimensions_strict.R_fixed) - dimensions_strict.R_fixed_constraints + numel(dimensions_strict.K_fixed) - dimensions_strict.K_fixed_constraints + numel(dimensions_strict.F_fixed) - dimensions_strict.F_fixed_constraints, 1);
				end
				Jopt = NaN;
				output = optimization.options.Options.OUTPUTPROTOTYPEMULTIPLE;
				output.message = e.message;
				output.information.output = e.getReport('extended', 'hyperlinks', 'off');
			elseif errorhandler.type == GammaErrorHandler.USER
				errfun = errorhandler.function;
				[x_opt, Jopt, output] = errfun(e, size(x_0, 1));
			else
				throw(MException('control:design:gamma', 'Optimization failed.').addCause(e));
			end
		end
	end
	[R_opt, K_opt, F_opt] = x2R(x_opt, dimensions_strict);
	if dimensions_strict.measurements_xdot > 0
		Ropt = {
			R_opt;
			K_opt;
			F_opt
		};
	elseif dimensions_strict.references > 0
		Ropt = {
			R_opt;
			F_opt
		};
	else
		Ropt = R_opt;
	end
	if nargout >= 3
		information = output;
	end
end

function [handle] = J_dispatcher(fun, hessian, system, weight, areafun, dimensions, objective_options)
	%J_DISPATCHER return handle to objective function to keep unneeded variables out of function scope
	%	Input:
	%		fun:				identifier for function to return
	%		hessian:			indicator, if hessian information should be supplied
	%		system:				transformed/normalized systems argument to use in objective functions
	%		weight:				transformed/normalized weight argument to use in objective functions
	%		areafun:			transformed/normalized areafun argument to use in objective functions
	%		dimensions:			dimension argument to use in objective functions
	%		objective_options:	objective option argument to use in objective functions
	%	Output:
	%		handle:				function handle to objective function to use without variables defined in gammsyn in its scope
	if ~islogical(hessian) || ~isscalar(hessian)
		error('control:design:gamma', 'Hessian indicator must be a logical scalar.');
	end
	function [J, gradJ, hessJ] = J_constr_zero_hessian(x)
		J = 0;
		gradJ = 0*x;
		hessJ = gradJ'*gradJ;
	end
	function [J, gradJ] = J_constr_zero(x)
		J = 0;
		gradJ = 0*x;
	end
	function [J, gradJ, hessJ] = J_constr_hessian_m(x)
		if nargout >= 3
			[J, gradJ, hessJ] = control.design.gamma.J(x, system, weight, areafun, dimensions, objective_options);
		elseif nargout >= 2
			[J, gradJ] = control.design.gamma.J(x, system, weight, areafun, dimensions, objective_options);
		else
			J = control.design.gamma.J(x, system, weight, areafun, dimensions, objective_options);
		end
	end
	function [J, gradJ] = J_constr_m(x)
		if nargout >= 2
			[J, gradJ] = control.design.gamma.J(x, system, weight, areafun, dimensions, objective_options);
		else
			J = control.design.gamma.J(x, system, weight, areafun, dimensions, objective_options);
		end
	end
	function [J, gradJ, hessJ] = J_constr_hessian_mex(x)
		if nargout >= 3
			[J, gradJ, hessJ] = J_mex_hess_mex(x, system, weight, areafun, dimensions, objective_options);
		elseif nargout >= 2
			[J, gradJ] = J_mex_grad_mex(x, system, weight, areafun, dimensions, objective_options);
		else
			J = J_mex_fun_mex(x, system, weight, areafun, dimensions, objective_options);
		end
	end
	function [J, gradJ] = J_constr_mex(x)
		if nargout >= 2
			[J, gradJ] = J_mex_grad_mex(x, system, weight, areafun, dimensions, objective_options);
		else
			J = J_mex_fun_mex(x, system, weight, areafun, dimensions, objective_options);
		end
	end
	function [J, gradJ, hessJ] = J_unconstr_hessian_m(x)
		if nargout >= 3
			[J, gradJ, hessJ] = control.design.gamma.J(x, system, weight, areafun, dimensions, objective_options);
		elseif nargout >= 2
			[J, gradJ] = control.design.gamma.J(x, system, weight, areafun, dimensions, objective_options);
		else
			J = control.design.gamma.J(x, system, weight, areafun, dimensions, objective_options);
		end
	end
	function [J, gradJ] = J_unconstr_m(x)
		if nargout >= 2
			[J, gradJ] = control.design.gamma.J(x, system, weight, areafun, dimensions, objective_options);
		else
			J = control.design.gamma.J(x, system, weight, areafun, dimensions, objective_options);
		end
	end
	function [J, gradJ, hessJ] = J_unconstr_hessian_mex(x)
		if nargout >= 3
			[J, gradJ, hessJ] = J_mex_hess_mex(x, system, weight, areafun, dimensions, objective_options);
		elseif nargout >= 2
			[J, gradJ] = J_mex_grad_mex(x, system, weight, areafun, dimensions, objective_options);
		else
			J = J_mex_fun_mex(x, system, weight, areafun, dimensions, objective_options);
		end
	end
	function [J, gradJ] = J_unconstr_mex(x)
		if nargout >= 2
			[J, gradJ] = J_mex_grad_mex(x, system, weight, areafun, dimensions, objective_options);
		else
			J = J_mex_fun_mex(x, system, weight, areafun, dimensions, objective_options);
		end
	end
	function [J, gradJ] = J_constr_multi_m(x)
		if nargout >= 2
			[J, ~, gradJ] = control.design.gamma.c(x, system, weight, areafun, dimensions, objective_options);
		else
			J = control.design.gamma.c(x, system, weight, areafun, dimensions, objective_options);
		end
	end
	function [J, gradJ] = J_constr_multi_mex(x)
		if nargout >= 2
			[J, ~, gradJ] = c_mex_grad_mex(x, system, weight, areafun, dimensions, objective_options);
		else
			J = c_mex_fun_mex(x, system, weight, areafun, dimensions, objective_options);
		end
	end
	function [J, gradJ] = J_unconstr_multi_m(x)
		if nargout >= 2
			[J, ~, gradJ] = control.design.gamma.c(x, system, weight, areafun, dimensions, objective_options);
		else
			J = control.design.gamma.c(x, system, weight, areafun, dimensions, objective_options);
		end
	end
	function [J, gradJ] = J_unconstr_multi_mex(x)
		if nargout >= 2
			[J, ~, gradJ] = c_mex_grad_mex(x, system, weight, areafun, dimensions, objective_options);
		else
			J = c_mex_fun_mex(x, system, weight, areafun, dimensions, objective_options);
		end
	end
	switch lower(fun)
		case {'zero'}
			if hessian
				handle = @J_constr_zero_hessian;
			else
				handle = @J_constr_zero;
			end
		case {'constr_m'}
			if hessian
				handle = @J_constr_hessian_m;
			else
				handle = @J_constr_m;
			end
		case {'constr_mex'}
			if hessian
				handle = @J_constr_hessian_mex;
			else
				handle = @J_constr_mex;
			end
		case {'unconstr_m'}
			if hessian
				handle = @J_unconstr_hessian_m;
			else
				handle = @J_unconstr_m;
			end
		case {'unconstr_mex'}
			if hessian
				handle = @J_unconstr_hessian_mex;
			else
				handle = @J_unconstr_mex;
			end
		case {'constr_multi_m'}
			handle = @J_constr_multi_m;
		case {'constr_multi_mex'}
			handle = @J_constr_multi_mex;
		case {'unconstr_multi_m'}
			handle = @J_unconstr_multi_m;
		case {'unconstr_multi_mex'}
			handle = @J_unconstr_multi_mex;
		otherwise
			error('control:design:gamma', 'Undefined function handle type ''%s''.', fun);
	end
end

function [handle] = c_dispatcher(fun, hessian, system, weight, areafun, dimensions, objective_options, nonlcon)
	%C_DISPATCHER return handle to constraint function to keep unneeded variables out of function scope
	%	Input:
	%		fun:				identifier for function to return
	%		hessian:			indicator, if hessian information should be supplied
	%		system:				transformed/normalized systems argument to use in constraint functions
	%		weight:				transformed/normalized weight argument to use in constraint functions
	%		areafun:			transformed/normalized areafun argument to use in constraint functions
	%		dimensions:			dimension argument to use in constraint functions
	%		objective_options:	objective option argument to use in constraint functions
	%		nonlcon:			nonlinear constraint function and additional information for gain matrices
	%	Output:
	%		handle:				function handle to constraint function to use without variables defined in gammsyn in its scope
	if nargin <= 7
		nonlcon = struct(...
			'f',		[],...
			'R_c',		0,...
			'R_ceq',	0,...
			'K_c',		0,...
			'K_ceq',	0,...
			'F_c',		0,...
			'F_ceq',	0,...
			'hasfun',	false,...
			'hasgrad',	false,...
			'hashess',	false...
		);
	end
	if ~islogical(hessian) || ~isscalar(hessian)
		error('control:design:gamma', 'Hessian indicator must be a logical scalar.');
	end
	function [c, ceq, gradc, gradceq, hessc, hessceq] = c_constr_hessian_m(x)
		%C_CONSTR_HESSIAN_M nonlinear constraint function for optimization without compiled functions and only pole area constraints
		%	Input:
		%		x:			optimization variable
		%	Output:
		%		c:			nonlinear inequality constraints
		%		ceq:		nonlinear equality constraints
		%		gradc:		gradient of nonlinear inequality constraints
		%		gradceq:	gradient of nonlinear equality constraints
		if nargout <= 2
			[c, ceq] = control.design.gamma.c(x, system, weight, areafun, dimensions, objective_options);
		elseif nargout <= 4
			[c, ceq, gradc, gradceq] = control.design.gamma.c(x, system, weight, areafun, dimensions, objective_options);
		else
			[c, ceq, gradc, gradceq, hessc, hessceq] = control.design.gamma.c(x, system, weight, areafun, dimensions, objective_options);
		end
	end
	function [c, ceq, gradc, gradceq] = c_constr_m(x)
		%C_CONSTR_M nonlinear constraint function for optimization without compiled functions and only pole area constraints
		%	Input:
		%		x:			optimization variable
		%	Output:
		%		c:			nonlinear inequality constraints
		%		ceq:		nonlinear equality constraints
		%		gradc:		gradient of nonlinear inequality constraints
		%		gradceq:	gradient of nonlinear equality constraints
		if nargout <= 2
			[c, ceq] = control.design.gamma.c(x, system, weight, areafun, dimensions, objective_options);
		else
			[c, ceq, gradc, gradceq] = control.design.gamma.c(x, system, weight, areafun, dimensions, objective_options);
		end
	end
	function [c, ceq, gradc, gradceq, hessc, hessceq] = c_constr_hessian_mex(x)
		%C_CONSTR_HESSIAN_MEX nonlinear constraint function for optimization with compiled functions and only pole area constraints
		%	Input:
		%		x:			optimization variable
		%	Output:
		%		c:			nonlinear inequality constraints
		%		ceq:		nonlinear equality constraints
		%		gradc:		gradient of nonlinear inequality constraints
		%		gradceq:	gradient of nonlinear equality constraints
		if nargout <= 2
			[c, ceq] = c_mex(x, system, weight, areafun, dimensions, objective_options);
		elseif nargout <= 4
			[c, ceq, gradc, gradceq] = c_mex(x, system, weight, areafun, dimensions, objective_options);
		else
			[c, ceq, gradc, gradceq, hessc, hessceq] = c_mex(x, system, weight, areafun, dimensions, objective_options);
		end
	end
	function [c, ceq, gradc, gradceq] = c_constr_mex(x)
		%C_CONSTR_MEX nonlinear constraint function for optimization with compiled functions and only pole area constraints
		%	Input:
		%		x:			optimization variable
		%	Output:
		%		c:			nonlinear inequality constraints
		%		ceq:		nonlinear equality constraints
		%		gradc:		gradient of nonlinear inequality constraints
		%		gradceq:	gradient of nonlinear equality constraints
		if nargout <= 2
			[c, ceq] = c_mex(x, system, weight, areafun, dimensions, objective_options);
		else
			[c, ceq, gradc, gradceq] = c_mex(x, system, weight, areafun, dimensions, objective_options);
		end
	end
	function [c, ceq, gradc, gradceq, hessc, hessceq] = c_constr_gain_hessian_m(x)
		%C_CONSTR_GAIN_HESSIAN_M nonlinear constraint function for optimization without compiled functions and pole area constraints as well as nonlinear gain constraints
		%	Input:
		%		x:			optimization variable
		%	Output:
		%		c:			nonlinear inequality constraints
		%		ceq:		nonlinear equality constraints
		%		gradc:		gradient of nonlinear inequality constraints
		%		gradceq:	gradient of nonlinear equality constraints
		if nargout <= 2
			[c, ceq] = control.design.gamma.c(x, system, weight, areafun, dimensions, objective_options);
			[c_gain, ceq_gain] = calculate_gain_constraints(x, dimensions, nonlcon);
			c = [
				c;
				c_gain
			];
			ceq = [
				ceq;
				ceq_gain
			];
		elseif nargout <= 4
			[c, ceq, gradc, gradceq] = control.design.gamma.c(x, system, weight, areafun, dimensions, objective_options);
			[c_gain, ceq_gain, gradc_gain, gradceq_gain] = calculate_gain_constraints(x, dimensions, nonlcon);
			c = [
				c;
				c_gain
			];
			ceq = [
				ceq;
				ceq_gain
			];
			gradc = [
				gradc, gradc_gain
			];
			gradceq = [
				gradceq, gradceq_gain
			];
		else
			[c, ceq, gradc, gradceq, hessc, hessceq] = control.design.gamma.c(x, system, weight, areafun, dimensions, objective_options);
			[c_gain, ceq_gain, gradc_gain, gradceq_gain, hessc_gain, hessceq_gain] = calculate_gain_constraints(x, dimensions, nonlcon);
			c = [
				c;
				c_gain
			];
			ceq = [
				ceq;
				ceq_gain
			];
			gradc = [
				gradc, gradc_gain
			];
			gradceq = [
				gradceq, gradceq_gain
			];
			hessc = cat(3, hessc, hessc_gain);
			hessceq = cat(3, hessceq, hessceq_gain);
		end
	end
	function [c, ceq, gradc, gradceq] = c_constr_gain_m(x)
		%C_CONSTR_GAIN_M nonlinear constraint function for optimization without compiled functions and pole area constraints as well as nonlinear gain constraints
		%	Input:
		%		x:			optimization variable
		%	Output:
		%		c:			nonlinear inequality constraints
		%		ceq:		nonlinear equality constraints
		%		gradc:		gradient of nonlinear inequality constraints
		%		gradceq:	gradient of nonlinear equality constraints
		if nargout <= 2
			[c, ceq] = control.design.gamma.c(x, system, weight, areafun, dimensions, objective_options);
			[c_gain, ceq_gain] = calculate_gain_constraints(x, dimensions, nonlcon);
			c = [
				c;
				c_gain
			];
			ceq = [
				ceq;
				ceq_gain
			];
		else
			[c, ceq, gradc, gradceq] = control.design.gamma.c(x, system, weight, areafun, dimensions, objective_options);
			[c_gain, ceq_gain, gradc_gain, gradceq_gain] = calculate_gain_constraints(x, dimensions, nonlcon);
			c = [
				c;
				c_gain
			];
			ceq = [
				ceq;
				ceq_gain
			];
			gradc = [
				gradc, gradc_gain
			];
			gradceq = [
				gradceq, gradceq_gain
			];
		end
	end
	function [c, ceq, gradc, gradceq, hessc, hessceq] = c_constr_gain_hessian_mex(x)
		%C_CONSTR_GAIN_HESSIAN_MEX nonlinear constraint function for optimization with compiled functions and pole area constraints as well as nonlinear gain constraints
		%	Input:
		%		x:			optimization variable
		%	Output:
		%		c:			nonlinear inequality constraints
		%		ceq:		nonlinear equality constraints
		%		gradc:		gradient of nonlinear inequality constraints
		%		gradceq:	gradient of nonlinear equality constraints
		if nargout <= 2
			[c, ceq] = c_mex(x, system, weight, areafun, dimensions, objective_options);
			[c_gain, ceq_gain] = calculate_gain_constraints(x, dimensions, nonlcon);
			c = [
				c;
				c_gain
			];
			ceq = [
				ceq;
				ceq_gain
			];
		elseif nargout <= 4
			[c, ceq, gradc, gradceq] = c_mex(x, system, weight, areafun, dimensions, objective_options);
			[c_gain, ceq_gain, gradc_gain, gradceq_gain] = calculate_gain_constraints(x, dimensions, nonlcon);
			c = [
				c;
				c_gain
			];
			ceq = [
				ceq;
				ceq_gain
			];
			gradc = [
				gradc, gradc_gain
			];
			gradceq = [
				gradceq, gradceq_gain
			];
		else
			[c, ceq, gradc, gradceq, hessc, hessceq] = c_mex(x, system, weight, areafun, dimensions, objective_options);
			[c_gain, ceq_gain, gradc_gain, gradceq_gain, hessc_gain, hessceq_gain] = calculate_gain_constraints(x, dimensions, nonlcon);
			c = [
				c;
				c_gain
			];
			ceq = [
				ceq;
				ceq_gain
			];
			gradc = [
				gradc, gradc_gain
			];
			gradceq = [
				gradceq, gradceq_gain
			];
			hessc = cat(3, hessc, hessc_gain);
			hessceq = cat(3, hessceq, hessceq_gain);
		end
	end
	function [c, ceq, gradc, gradceq] = c_constr_gain_mex(x)
		%C_CONSTR_GAIN_MEX nonlinear constraint function for optimization with compiled functions and pole area constraints as well as nonlinear gain constraints
		%	Input:
		%		x:			optimization variable
		%	Output:
		%		c:			nonlinear inequality constraints
		%		ceq:		nonlinear equality constraints
		%		gradc:		gradient of nonlinear inequality constraints
		%		gradceq:	gradient of nonlinear equality constraints
		if nargout <= 2
			[c, ceq] = c_mex(x, system, weight, areafun, dimensions, objective_options);
			[c_gain, ceq_gain] = calculate_gain_constraints(x, dimensions, nonlcon);
			c = [
				c;
				c_gain
			];
			ceq = [
				ceq;
				ceq_gain
			];
		else
			[c, ceq, gradc, gradceq] = c_mex(x, system, weight, areafun, dimensions, objective_options);
			[c_gain, ceq_gain, gradc_gain, gradceq_gain] = calculate_gain_constraints(x, dimensions, nonlcon);
			c = [
				c;
				c_gain
			];
			ceq = [
				ceq;
				ceq_gain
			];
			gradc = [
				gradc, gradc_gain
			];
			gradceq = [
				gradceq, gradceq_gain
			];
		end
	end
	switch lower(fun)
		case {'constr_m'}
			if nonlcon.hasfun
				if hessian
					handle = @c_constr_gain_hessian_m;
				else
					handle = @c_constr_gain_m;
				end
			else
				if hessian
					handle = @c_constr_hessian_m;
				else
					handle = @c_constr_m;
				end
			end
		case {'constr_mex'}
			if nonlcon.hasfun
				if hessian
					handle = @c_constr_gain_hessian_mex;
				else
					handle = @c_constr_gain_mex;
				end
			else
				if hessian
					handle = @c_constr_hessian_mex;
				else
					handle = @c_constr_mex;
				end
			end
		case {'unconstr_m'}
			handle = [];
		case {'unconstr_mex'}
			handle = [];
		case {'constr_multi_m'}
			if nonlcon.hasfun
				if hessian
					handle = @c_constr_gain_hessian_m;
				else
					handle = @c_constr_gain_m;
				end
			else
				if hessian
					handle = @c_constr_hessian_m;
				else
					handle = @c_constr_m;
				end
			end
		case {'constr_multi_mex'}
			if nonlcon.hasfun
				if hessian
					handle = @c_constr_gain_hessian_mex;
				else
					handle = @c_constr_gain_mex;
				end
			else
				if hessian
					handle = @c_constr_hessian_mex;
				else
					handle = @c_constr_mex;
				end
			end
		case {'unconstr_multi_m'}
			handle = [];
		case {'unconstr_multi_mex'}
			handle = [];
		otherwise
			error('control:design:gamma', 'Undefined function handle type ''%s''.', fun);
	end
end
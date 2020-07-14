function [objective_options_strict, objective_options_loose, solution_strategy, errorhandler] = checkobjectiveoptions(objectiveoptions, x_0, number_states, system, solveroptions, dimensions_strict, dimensions_loose, areafun_strict, areafun_loose, weight_strict, weight_loose)
	%CHECKOBJECTIVEOPTIONS check options for objective function
	%	Input:
	%		objectiveoptions:			user supplied objective options
	%		x_0:						initial value fo optimization
	%		number_states:				number of states for all models
	%		system:						systems to calculate controller for
	%		solveroptions:				options for optimization
	%		dimensions_strict:			dimensions of problem variables and systems for strict problem formulation
	%		dimensions_loose:			dimensions of problem variables and systems for loose problem formulation
	%		areafun_strict:				pole area for strict problem formulation
	%		areafun_loose:				pole area for loose problem formulation
	%		weight_strict:				weight for strict problem formulation
	%		weight_loose:				weight for loose problem formulation
	%	Output:
	%		objective_options_strict:	objective options for strict problem formulation
	%		objective_options_loose:	objective options for loose problem formulation
	%		solution_strategy:			strategy for solving problem
	%		errorhandler:				function to handle errors during optimization
	if isa(objectiveoptions, 'control.design.gamma.GammasynOptions')
		objectiveoptions = objectiveoptions.userstruct();
	end
	if any(solveroptions.ProblemType == [
		optimization.options.ProblemType.CONSTRAINED;
		optimization.options.ProblemType.CONSTRAINEDMULTI
	])
		if dimensions_loose.areas_max ~= 0
			objective_type = GammaJType.LINEAR;
		else
			objective_type = GammaJType.ZERO;
		end
	else
		objective_type = GammaJType.EXP;
	end
	if isempty(objectiveoptions)
		objectiveoptions = struct(...
			'usecompiled',				configuration.control.design.gamma.hascompiled(),...
			'numthreads',				uint32(configuration.matlab.numthreads()),...
			'type',						objective_type,...
			'weight',					ones(size(objective_type, 1), 1),...
			'allowvarorder',			any(number_states ~= number_states(1)),...
			'eigenvaluederivative',		GammaEigenvalueDerivativeType.getDefaultValue(),...
			'eigenvaluefilter',			GammaEigenvalueFilterType.getDefaultValue(),...
			'eigenvalueignoreinf',		false,...
			'strategy',					GammaSolutionStrategy.getDefaultValue(),...
			'errorhandler',				GammaErrorHandler.getDefaultValue(),...
			'errorhandler_function',	[]...
		);
	else
		if ~isstruct(objectiveoptions)
			if isa(objectiveoptions, 'GammaJType')
				if ~iscolumn(objectiveoptions)
					objectiveoptions = objectiveoptions';
				end
				objectiveoptions = struct(...
					'usecompiled',				configuration.control.design.gamma.hascompiled(),...
					'numthreads',				uint32(configuration.matlab.numthreads()),...
					'type',						objectiveoptions,...
					'weight',					ones(size(objective_type, 1), 1),...
					'allowvarorder',			any(number_states ~= number_states(1)),...
					'eigenvaluederivative',		GammaEigenvalueDerivativeType.getDefaultValue(),...
					'eigenvaluefilter',			GammaEigenvalueFilterType.getDefaultValue(),...
					'eigenvalueignoreinf',		false,...
					'strategy',					GammaSolutionStrategy.getDefaultValue(),...
					'errorhandler',				GammaErrorHandler.getDefaultValue(),...
					'errorhandler_function',	[]...
				);
			else
				error('control:design:gamma', 'Objective options must be of type struct.');
			end
		else
			if ~isfield(objectiveoptions, 'type')
				error('control:design:gamma', 'Objective options must have field ''type''.');
			end
			if ~isfield(objectiveoptions, 'usecompiled')
				objectiveoptions.usecompiled = false;
			end
			if ~isfield(objectiveoptions, 'allowvarorder')
				objectiveoptions.allowvarorder = false;
			end
			if ~isfield(objectiveoptions, 'numthreads')
				objectiveoptions.numthreads = uint32(configuration.matlab.numthreads());
			end
			if ~isfield(objectiveoptions, 'weight')
				objectiveoptions.weight = ones(size(objectiveoptions.type, 1), 1);
			end
			if ~isfield(objectiveoptions, 'eigenvaluederivative')
				objectiveoptions.eigenvaluederivative = GammaEigenvalueDerivativeType.getDefaultValue();
			end
			if ~isfield(objectiveoptions, 'eigenvaluefilter')
				objectiveoptions.eigenvaluefilter = GammaEigenvalueFilterType.getDefaultValue();
			end
			if ~isfield(objectiveoptions, 'eigenvalueignoreinf')
				objectiveoptions.eigenvalueignoreinf = false;
			end
			if ~isfield(objectiveoptions, 'strategy')
				objectiveoptions.strategy = GammaSolutionStrategy.getDefaultValue();
			end
			if ~isfield(objectiveoptions, 'errorhandler')
				objectiveoptions.errorhandler = GammaErrorHandler.getDefaultValue();
			end
			if ~isfield(objectiveoptions, 'errorhandler_function')
				if ~isa(objectiveoptions.errorhandler, 'GammaErrorHandler')
					if isfunctionhandle(objectiveoptions.errorhandler)
						objectiveoptions.errorhandler_function = objectiveoptions.errorhandler;
						objectiveoptions.errorhandler = GammaErrorHandler.USER;
					else
						objectiveoptions.errorhandler_function = [];
					end
				else
					objectiveoptions.errorhandler_function = [];
				end
			end
		end
	end
	if ~islogical(objectiveoptions.usecompiled) || ~isscalar(objectiveoptions.usecompiled)
		error('control:design:gamma', 'Objective option ''usecompiled'' must be of type ''logical''.');
	end
	if ~islogical(objectiveoptions.allowvarorder) || ~isscalar(objectiveoptions.allowvarorder)
		error('control:design:gamma', 'Objective option ''allowvarorder'' must be of type ''logical''.');
	end
	if ~iscolumn(objectiveoptions.type) && ~ischar(objectiveoptions.type)
		objectiveoptions.type = objectiveoptions.type';
	end
	if ~iscolumn(objectiveoptions.weight)
		objectiveoptions.weight = objectiveoptions.weight';
	end
	if isscalar(objectiveoptions.weight)
		objectiveoptions.weight = repmat(objectiveoptions.weight, size(objectiveoptions.type, 1), 1);
	end
	if ~isscalar(objectiveoptions.numthreads)
		error('control:design:gamma', 'Objective option ''numthreads'' must be a scalar.');
	end
	if objectiveoptions.numthreads < 0
		objectiveoptions.numthreads = uint32(configuration.matlab.numthreads());
	end
	if ~isnumeric(objectiveoptions.numthreads) || ~isreal(objectiveoptions.numthreads) || floor(objectiveoptions.numthreads) ~= ceil(objectiveoptions.numthreads) || objectiveoptions.numthreads < 0
		error('control:design:gamma', 'Objective option ''numthreads'' must be a nonnegative integer.');
	end
	objectiveoptions.type = GammaJType.fromname(objectiveoptions.type);
	if ~isa(objectiveoptions.type, 'GammaJType')
		error('control:design:gamma', 'Objective option ''type'' must be of type ''GammaJType''.');
	end
	if size(objectiveoptions.type, 2) ~= 1
		error('control:design:gamma', 'Objective option ''type'' must be a column vector.');
	end
	if ~isnumeric(objectiveoptions.weight)
		error('control:design:gamma', 'Objective option ''weight'' must be numeric.');
	end
	if all(imag(objectiveoptions.weight(:)) == 0)
		objectiveoptions.weight = real(objectiveoptions.weight);
	end
	if ~isreal(objectiveoptions.weight)
		error('control:design:gamma', 'Objective option ''weight'' must be real.');
	end
	if size(objectiveoptions.weight, 2) ~= 1
		error('control:design:gamma', 'Objective option ''weight'' must be a column vector.');
	end
	if size(objectiveoptions.type, 1) ~= size(objectiveoptions.weight, 1)
		error('control:design:gamma', 'Objective option ''type'' and ''weight'' must have %d elements.', max(size(objectiveoptions.type, 1), size(objectiveoptions.weight, 1)));
	end
	if ~objectiveoptions.allowvarorder && any(number_states ~= number_states(1))
		error('control:design:gamma', 'Variable system order is not allowed by the user set options, but systems of different order are supplied.');
	end
	if objectiveoptions.usecompiled && ~configuration.control.design.gamma.hascompiled()
		warning('control:design:gamma', 'Compiled functions should be used, but are not available.');
		objectiveoptions.usecompiled = false;
	end
	if ~isscalar(objectiveoptions.eigenvaluederivative)
		error('control:design:gamma', 'Only one eigenvalue derivative method may be supplied.');
	end
	if ~isa(objectiveoptions.eigenvaluederivative, 'GammaEigenvalueDerivativeType')
		error('control:design:gamma', 'Eigenvalue derivative method must be of type ''GammaEigenvalueDerivativeType''.');
	end
	if isempty(objectiveoptions.eigenvaluefilter)
		objectiveoptions.eigenvaluefilter = GammaEigenvalueFilterType.getDefaultValue();
	end
	if isnumeric(objectiveoptions.eigenvaluefilter) || ischar(objectiveoptions.eigenvaluefilter)
		objectiveoptions.eigenvaluefilter = GammaEigenvalueFilterType.extract(objectiveoptions.eigenvaluefilter);
	end
	if ~isa(objectiveoptions.eigenvaluefilter, 'GammaEigenvalueFilterType')
		error('control:design:gamma', 'Eigenvalue filter type must be of type ''GammaEigenvalueFilterType''.');
	end
	if ~isscalar(objectiveoptions.eigenvaluefilter)
		objectiveoptions.eigenvaluefilter = cat(1, unique(objectiveoptions.eigenvaluefilter(:)));
	end
	if ~isscalar(objectiveoptions.eigenvalueignoreinf)
		error('control:design:gamma', 'Indicator for ignoring infinite eigenvalues must be scalar.');
	end
	if ~islogical(objectiveoptions.eigenvalueignoreinf)
		error('control:design:gamma', 'Indicator for ignoring infinite eigenvalues must be logical.');
	end
	if ~isscalar(objectiveoptions.strategy)
		error('control:design:gamma', 'Only one solution strategy may be supplied.');
	end
	if ~isa(objectiveoptions.strategy, 'GammaSolutionStrategy')
		error('control:design:gamma', 'Solution strategy must be of type ''GammaSolutionStrategy''.');
	end
	preventNaN = NaN;
	if isfield(objectiveoptions, 'objective') && isstruct(objectiveoptions.objective)
		if isfield(objectiveoptions.objective, 'preventNaN')
			if isempty(objectiveoptions.objective.preventNaN)
				preventNaN = NaN;
			else
				if ~isscalar(objectiveoptions.objective.preventNaN)
					error('control:design:gamma', 'Indicator for preventing NaN values in objective function must be scalar.');
				end
				if ~islogical(objectiveoptions.objective.preventNaN)
					error('control:design:gamma', 'Indicator for preventing NaN values in objective function must be of type ''logical'', not ''%s''.', class(objectiveoptions.objective.preventNaN));
				end
				preventNaN = logical(objectiveoptions.objective.preventNaN);
			end
		end
	end
	if isnan(preventNaN)
		preventNaN = false;
	end
	kreisselmeier_rho = NaN;
	kreisselmeier_max = NaN;
	if isfield(objectiveoptions, 'objective') && isstruct(objectiveoptions.objective)
		if isfield(objectiveoptions.objective, 'kreisselmeier') && isstruct(objectiveoptions.objective.kreisselmeier)
			if isfield(objectiveoptions.objective.kreisselmeier, 'rho')
				if isempty(objectiveoptions.objective.kreisselmeier.rho)
					kreisselmeier_rho = NaN;
				else
					if ~isscalar(objectiveoptions.objective.kreisselmeier.rho)
						error('control:design:gamma', 'Scaling factor ''rho'' in Kreisselmeier objective must be scalar.');
					end
					if ~isnumeric(objectiveoptions.objective.kreisselmeier.rho)
						error('control:design:gamma', 'Scaling factor ''rho'' in Kreisselmeier objective must be numeric.');
					end
					if imag(objectiveoptions.objective.kreisselmeier.rho) == 0
						objectiveoptions.objective.kreisselmeier.rho = real(objectiveoptions.objective.kreisselmeier.rho);
					end
					if ~isreal(objectiveoptions.objective.kreisselmeier.rho)
						error('control:design:gamma', 'Scaling factor ''rho'' in Kreisselmeier objective must not be complex.');
					end
					if isinf(objectiveoptions.objective.kreisselmeier.rho)
						error('control:design:gamma', 'Scaling factor ''rho'' in Kreisselmeier objective must be finite.');
					end
					kreisselmeier_rho = double(objectiveoptions.objective.kreisselmeier.rho);
				end
			end
			if isfield(objectiveoptions.objective.kreisselmeier, 'max')
				if isempty(objectiveoptions.objective.kreisselmeier.max)
					kreisselmeier_max = NaN;
				else
					if ~isscalar(objectiveoptions.objective.kreisselmeier.max)
						error('control:design:gamma', 'Shifting factor ''max'' in Kreisselmeier objective must be scalar.');
					end
					if ~isnumeric(objectiveoptions.objective.kreisselmeier.max)
						error('control:design:gamma', 'Shifting factor ''max'' in Kreisselmeier objective must be numeric.');
					end
					if imag(objectiveoptions.objective.kreisselmeier.max) == 0
						objectiveoptions.objective.kreisselmeier.max = real(objectiveoptions.objective.kreisselmeier.max);
					end
					if ~isreal(objectiveoptions.objective.kreisselmeier.max)
						error('control:design:gamma', 'Shifting factor ''max'' in Kreisselmeier objective must not be complex.');
					end
					if isinf(objectiveoptions.objective.kreisselmeier.max)
						error('control:design:gamma', 'Shifting factor ''max'' in Kreisselmeier objective must be finite.');
					end
					kreisselmeier_max = double(objectiveoptions.objective.kreisselmeier.max);
				end
			end
		end
	end
	iskreisselmeier = any(objectiveoptions.type == GammaJType.KREISSELMEIER);
	if iskreisselmeier
		if isnan(kreisselmeier_rho) || isnan(kreisselmeier_max)
			% TODO: worst constraint violation over all initial points is used, so Kreisselmeier objective is nearly useless for other initial values
			if any(solveroptions.ProblemType == [
				optimization.options.ProblemType.CONSTRAINED;
				optimization.options.ProblemType.CONSTRAINEDMULTI
			])
				if dimensions_loose.areas_max ~= 0
					if all(objectiveoptions.type == GammaJType.ZERO)
						usecompiled = objectiveoptions.usecompiled && isa(areafun_strict, 'GammaArea');
						areaval = zeros(size(x_0, 2), dimensions_strict.models, dimensions_strict.areas_max, dimensions_strict.states);
						parfor ii = 1:size(x_0, 2)
							[R_areaval_max, K_areaval_max] = x2R(x_0(:, ii), dimensions_strict);
							eigenvalues = calculate_eigenvalues(system, R_areaval_max, K_areaval_max, dimensions_strict);
							areaval(ii, :, :, :) = calculate_areas(areafun_strict, weight_strict, eigenvalues, dimensions_strict, usecompiled);
						end
					else
						usecompiled = objectiveoptions.usecompiled && isa(areafun_loose, 'GammaArea');
						areaval = zeros(size(x_0, 2), dimensions_loose.models, dimensions_loose.areas_max, dimensions_loose.states);
						parfor ii = 1:size(x_0, 2)
							[R_areaval_max, K_areaval_max] = x2R(x_0(:, ii), dimensions_loose);
							eigenvalues = calculate_eigenvalues(system, R_areaval_max, K_areaval_max, dimensions_loose);
							areaval(ii, :, :, :) = calculate_areas(areafun_loose, weight_loose, eigenvalues, dimensions_loose, usecompiled);
						end
					end
				else
					usecompiled = objectiveoptions.usecompiled && isa(areafun_strict, 'GammaArea') && configuration.control.design.gamma.hascompiled();
					areaval = zeros(size(x_0, 2), dimensions_strict.models, dimensions_strict.areas_max, dimensions_strict.states);
					parfor ii = 1:size(x_0, 2)
						[R_areaval_max, K_areaval_max] = x2R(x_0(:, ii), dimensions_strict);
						eigenvalues = calculate_eigenvalues(system, R_areaval_max, K_areaval_max, dimensions_strict);
						areaval(ii, :, :, :) = calculate_areas(areafun_strict, weight_strict, eigenvalues, dimensions_strict, usecompiled);
					end
				end
			elseif any(solveroptions.ProblemType == [
				optimization.options.ProblemType.UNCONSTRAINED;
				optimization.options.ProblemType.UNCONSTRAINEDMULTI
			])
				usecompiled = objectiveoptions.usecompiled && isa(areafun_strict, 'GammaArea') && configuration.control.design.gamma.hascompiled();
				areaval = zeros(size(x_0, 2), dimensions_strict.models, dimensions_strict.areas_max, dimensions_strict.states);
				parfor ii = 1:size(x_0, 2)
					[R_areaval_max, K_areaval_max] = x2R(x_0(:, ii), dimensions_strict);
					eigenvalues = calculate_eigenvalues(system, R_areaval_max, K_areaval_max, dimensions_strict);
					areaval(ii, :, :, :) = calculate_areas(areafun_strict, weight_strict, eigenvalues, dimensions_strict, usecompiled);
				end
			else
				error('control:design:gamma', 'Undefined problem type ''%s''.', upper(solveroptions.ProblemType));
			end
			areaval_max = max(areaval(:));
		end
		if isnan(kreisselmeier_rho)
			kreisselmeier_rho = min([20, 250/areaval_max]);
		end
		if isnan(kreisselmeier_max)
			kreisselmeier_max = areaval_max;
		end
	else
		kreisselmeier_rho = 1;
		kreisselmeier_max = 0;
	end
	if any(objectiveoptions.type == GammaJType.LYAPUNOV)
		if ~isfield(objectiveoptions, 'objective')
			error('control:design:gamma', 'Options must have field ''objective''.');
		end
		if ~isfield(objectiveoptions.objective, 'lyapunov')
			error('control:design:gamma', 'Options must have field ''lyapunov'' for type LYAPUNOV.');
		end
		if ~isfield(objectiveoptions.objective.lyapunov, 'Q')
			if dimensions_strict.states == 0
				objective_Q = zeros(dimensions_strict.states, dimensions_strict.states, size(system, 1));
			else
				error('control:design:gamma', 'Options must have field ''Q''.');
			end
		else
			objective_Q = objectiveoptions.objective.lyapunov.Q;
		end
		if isempty(objective_Q)
			objective_Q = zeros(dimensions_strict.states, dimensions_strict.states, size(system, 1));
		end
		if size(objective_Q, 3) == 1
			objective_Q = repmat(objective_Q, [1, 1, size(system, 1)]);
		end
		if ~isnumeric(objective_Q)
			error('control:design:gamma', 'Q matrix for lyapunov objective must be numeric.');
		end
		if any(isinf(objective_Q(:)))
			error('control:design:gamma', 'Q matrix for lyapunov objective must be finite.');
		end
		if ndims(objective_Q) > 3
			error('control:design:gamma', 'Q matrix for lyapunov objective must be a matrix.');
		end
		if size(objective_Q, 1) ~= dimensions_strict.states
			error('control:design:gamma', 'Q matrix for lyapunov objective must have %d rows.', dimensions_strict.states);
		end
		if size(objective_Q, 2) ~= dimensions_strict.states
			error('control:design:gamma', 'Q matrix for lyapunov objective must have %d columns.', dimensions_strict.states);
		end
		if size(objective_Q, 3) ~= size(system, 1)
			error('control:design:gamma', 'Q matrix for lyapunov objective must have %d third dimensions.', size(system, 1));
		end
		for ii = 1:size(objective_Q, 3)
			system_order = size(system(ii).A, 1);
			Q = objective_Q(:, :, ii);
			if any(imag(Q(:))) ~= 0
				error('control:design:gamma', 'Q matrix for lyapunov objective must be real.');
			end
			Qnan = isnan(Q);
			if all(Qnan)
				error('control:design:gamma', 'Q matrix for lyapunov objective must not be NaN.');
			end
			nanrows = all(Qnan, 1);
			nancols = all(Qnan, 2);
			if any(nanrows) || any(nancols)
				if nanrows(1)
					error('control:design:gamma', 'Q matrix for lyapunov objective must not start with NaN rows.');
				end
				if nancols(1)
					error('control:design:gamma', 'Q matrix for lyapunov objective must not start with NaN columns.');
				end
				if any(nanrows ~= nancols')
					error('control:design:gamma', 'Q matrix for lyapunov objective must not have isolated NaN rows.');
				end
				if any(nancols ~= nanrows')
					error('control:design:gamma', 'Q matrix for lyapunov objective must not have isolated NaN columns.');
				end
				diffnanrows = abs(diff(nanrows));
				diffnancols = abs(diff(nancols));
				if sum(diffnanrows) > 1
					error('control:design:gamma', 'Q matrix for lyapunov objective must have NaN rows at the end for systems of different order.');
				end
				if sum(diffnancols) > 1
					error('control:design:gamma', 'Q matrix for lyapunov objective must have NaN columns at the end for systems of different order.');
				end
				stateidx_row = find(diffnanrows, 1, 'first') + 1;
				stateidx_col = find(diffnancols, 1, 'first') + 1;
				if stateidx_row <= system_order
					error('control:design:gamma', 'Q matrix for lyapunov objective has too much NaN rows at the end for systems of different order.');
				end
				if stateidx_col <= system_order
					error('control:design:gamma', 'Q matrix for lyapunov objective has too much NaN columns at the end for systems of different order.');
				end
				Q = Q(1:system_order, 1:system_order);
				if any(isnan(Q(:)))
					error('control:design:gamma', 'Q matrix for lyapunov objective must not be NaN.');
				end
			end
			%[~, posdef] = chol(Q);% TODO semidefinite or nothing at all?
			%if posdef > 0
			%	error('control:design:gamma', 'Q matrix for lyapunov objective must be positive definite.');
			%end
			Q = [
				Q, NaN(size(Q, 1), dimensions_strict.states - size(Q, 2));
				NaN(dimensions_strict.states - size(Q, 1), dimensions_strict.states)
			];
			objective_Q(:, :, ii) = Q;
		end
		lyapunov_Q = objective_Q;
	else
		lyapunov_Q = zeros(dimensions_strict.states, dimensions_strict.states, size(system, 1));
	end
	if any(objectiveoptions.type == GammaJType.NORMGAIN)
		if ~isfield(objectiveoptions, 'objective')
			error('control:design:gamma', 'Options must have field ''objective''.');
		end
		if ~isfield(objectiveoptions.objective, 'normgain')
			error('control:design:gamma', 'Options must have field ''normgain'' for type NORMGAIN.');
		end
		if ~isfield(objectiveoptions.objective.normgain, 'R')
			if dimensions_strict.measurements == 0
				weightR = zeros(dimensions_strict.controls, dimensions_strict.measurements);
			else
				error('control:design:gamma', 'Options must have field ''R''.');
			end
		else
			weightR = objectiveoptions.objective.normgain.R;
		end
		if ~isfield(objectiveoptions.objective.normgain, 'R_shift')
			shiftR = zeros(dimensions_strict.controls, dimensions_strict.measurements);
		else
			shiftR = objectiveoptions.objective.normgain.R_shift;
		end
		if ~isfield(objectiveoptions.objective.normgain, 'K')
			if dimensions_strict.measurements_xdot == 0
				weightK = zeros(dimensions_strict.controls, dimensions_strict.measurements_xdot);
			else
				error('control:design:gamma', 'Options must have field ''K''.');
			end
		else
			weightK = objectiveoptions.objective.normgain.K;
		end
		if ~isfield(objectiveoptions.objective.normgain, 'K_shift')
			shiftK = zeros(dimensions_strict.controls, dimensions_strict.measurements_xdot);
		else
			shiftK = objectiveoptions.objective.normgain.K_shift;
		end
		if ~isfield(objectiveoptions.objective.normgain, 'F')
			if dimensions_strict.references == 0
				weightF = zeros(dimensions_strict.controls, dimensions_strict.references);
			else
				error('control:design:gamma', 'Options must have field ''F''.');
			end
		else
			weightF = objectiveoptions.objective.normgain.F;
		end
		if ~isfield(objectiveoptions.objective.normgain, 'F_shift')
			shiftF = zeros(dimensions_strict.controls, dimensions_strict.references);
		else
			shiftF = objectiveoptions.objective.normgain.F_shift;
		end
		if isempty(weightR)
			weightR = zeros(dimensions_strict.controls, dimensions_strict.measurements);
		end
		if isempty(weightK)
			weightK = zeros(dimensions_strict.controls, dimensions_strict.measurements_xdot);
		end
		if isempty(weightF)
			weightF = zeros(dimensions_strict.controls, dimensions_strict.references);
		end
		if isempty(shiftR)
			shiftR = zeros(dimensions_strict.controls, dimensions_strict.measurements);
		end
		if isempty(shiftK)
			shiftK = zeros(dimensions_strict.controls, dimensions_strict.measurements_xdot);
		end
		if isempty(shiftF)
			shiftF = zeros(dimensions_strict.controls, dimensions_strict.references);
		end
		if ~isnumeric(weightR)
			error('control:design:gamma', 'Weight for proportional gain must be numeric.');
		end
		if any(isnan(weightR(:))) || any(isinf(weightR(:)))
			error('control:design:gamma', 'Weight for proportional gain must be finite.');
		end
		if ndims(weightR) > 2 %#ok<ISMAT> compatibility with octave
			error('control:design:gamma', 'Weight for proportional gain must be a matrix.');
		end
		if size(weightR, 1) ~= dimensions_strict.controls
			error('control:design:gamma', 'Weight for proportional gain must have %d rows.', dimensions_strict.controls);
		end
		if size(weightR, 2) ~= dimensions_strict.measurements
			error('control:design:gamma', 'Weight for proportional gain must have %d columns.', dimensions_strict.measurements);
		end
		if all(imag(weightR(:)) == 0)
			weightR = real(weightR);
		end
		if ~isreal(weightR)
			error('control:design:gamma', 'Weight for proportional gain must not be complex.');
		end
		if ~isnumeric(weightK)
			error('control:design:gamma', 'Weight for derivative gain must be numeric.');
		end
		if any(isnan(weightK(:))) || any(isinf(weightK(:)))
			error('control:design:gamma', 'Weight for derivative gain must be finite.');
		end
		if ndims(weightK) > 2 %#ok<ISMAT> compatibility with octave
			error('control:design:gamma', 'Weight for derivative gain must be a matrix.');
		end
		if size(weightK, 1) ~= dimensions_strict.controls
			error('control:design:gamma', 'Weight for derivative gain must have %d rows.', dimensions_strict.controls);
		end
		if size(weightK, 2) ~= dimensions_strict.measurements_xdot
			error('control:design:gamma', 'Weight for derivative gain must have %d columns.', dimensions_strict.measurements_xdot);
		end
		if all(imag(weightK(:)) == 0)
			weightK = real(weightK);
		end
		if ~isreal(weightK)
			error('control:design:gamma', 'Weight for derivative gain must not be complex.');
		end
		if ~isnumeric(weightF)
			error('control:design:gamma', 'Weight for prefilter gain must be numeric.');
		end
		if any(isnan(weightF(:))) || any(isinf(weightF(:)))
			error('control:design:gamma', 'Weight for prefilter gain must be finite.');
		end
		if ndims(weightF) > 2 %#ok<ISMAT> compatibility with octave
			error('control:design:gamma', 'Weight for prefilter gain must be a matrix.');
		end
		if size(weightF, 1) ~= dimensions_strict.controls
			error('control:design:gamma', 'Weight for prefilter gain must have %d rows.', dimensions_strict.controls);
		end
		if size(weightF, 2) ~= dimensions_strict.references
			error('control:design:gamma', 'Weight for prefilter gain must have %d columns.', dimensions_strict.references);
		end
		if all(imag(weightF(:)) == 0)
			weightF = real(weightF);
		end
		if ~isreal(weightF)
			error('control:design:gamma', 'Weight for prefilter gain must not be complex.');
		end
		if ~isnumeric(shiftR)
			error('control:design:gamma', 'Shift for proportional gain must be numeric.');
		end
		if any(isnan(shiftR(:))) || any(isinf(shiftR(:)))
			error('control:design:gamma', 'Shift for proportional gain must be finite.');
		end
		if ndims(shiftR) > 2 %#ok<ISMAT> compatibility with octave
			error('control:design:gamma', 'Shift for proportional gain must be a matrix.');
		end
		if size(shiftR, 1) ~= dimensions_strict.controls
			error('control:design:gamma', 'Shift for proportional gain must have %d rows.', dimensions_strict.controls);
		end
		if size(shiftR, 2) ~= dimensions_strict.measurements
			error('control:design:gamma', 'Shift for proportional gain must have %d columns.', dimensions_strict.measurements);
		end
		if all(imag(shiftR(:)) == 0)
			weightR = real(weightR);
		end
		if ~isreal(shiftR)
			error('control:design:gamma', 'Shift for proportional gain must not be complex.');
		end
		if ~isnumeric(shiftK)
			error('control:design:gamma', 'Shift for derivative gain must be numeric.');
		end
		if any(isnan(shiftK(:))) || any(isinf(shiftK(:)))
			error('control:design:gamma', 'Shift for derivative gain must be finite.');
		end
		if ndims(shiftK) > 2 %#ok<ISMAT> compatibility with octave
			error('control:design:gamma', 'Shift for derivative gain must be a matrix.');
		end
		if size(shiftK, 1) ~= dimensions_strict.controls
			error('control:design:gamma', 'Shift for derivative gain must have %d rows.', dimensions_strict.controls);
		end
		if size(shiftK, 2) ~= dimensions_strict.measurements_xdot
			error('control:design:gamma', 'Shift for derivative gain must have %d columns.', dimensions_strict.measurements_xdot);
		end
		if all(imag(shiftK(:)) == 0)
			shiftK = real(shiftK);
		end
		if ~isreal(shiftK)
			error('control:design:gamma', 'Shift for derivative gain must not be complex.');
		end
		if ~isnumeric(shiftF)
			error('control:design:gamma', 'Shift for prefilter gain must be numeric.');
		end
		if any(isnan(shiftF(:))) || any(isinf(shiftF(:)))
			error('control:design:gamma', 'Shift for prefilter gain must be finite.');
		end
		if ndims(shiftF) > 2 %#ok<ISMAT> compatibility with octave
			error('control:design:gamma', 'Shift for prefilter gain must be a matrix.');
		end
		if size(shiftF, 1) ~= dimensions_strict.controls
			error('control:design:gamma', 'Shift for prefilter gain must have %d rows.', dimensions_strict.controls);
		end
		if size(shiftF, 2) ~= dimensions_strict.references
			error('control:design:gamma', 'Shift for prefilter gain must have %d columns.', dimensions_strict.references);
		end
		if all(imag(shiftF(:)) == 0)
			shiftF = real(shiftF);
		end
		if ~isreal(shiftF)
			error('control:design:gamma', 'Shift for prefilter gain must not be complex.');
		end
	else
		weightR = zeros(dimensions_strict.controls, dimensions_strict.measurements);
		weightK = zeros(dimensions_strict.controls, dimensions_strict.measurements_xdot);
		weightF = zeros(dimensions_strict.controls, dimensions_strict.references);
		shiftR = zeros(dimensions_strict.controls, dimensions_strict.measurements);
		shiftK = zeros(dimensions_strict.controls, dimensions_strict.measurements_xdot);
		shiftF = zeros(dimensions_strict.controls, dimensions_strict.references);
	end
	objective_options_strict = struct(...
		'usecompiled',			objectiveoptions.usecompiled && isa(areafun_strict, 'GammaArea') && configuration.control.design.gamma.hascompiled(),...
		'numthreads',			uint32(objectiveoptions.numthreads),...
		'type',					objectiveoptions.type,...
		'weight',				double(objectiveoptions.weight),...
		'allowvarorder',		objectiveoptions.allowvarorder,...
		'eigenvaluederivative',	objectiveoptions.eigenvaluederivative,...
		'eigenvaluefilter',		objectiveoptions.eigenvaluefilter,...
		'eigenvalueignoreinf',	objectiveoptions.eigenvalueignoreinf,...
		'objective',			struct(...
			'preventNaN',		preventNaN,...
			'kreisselmeier',	struct(...
				'rho',				double(kreisselmeier_rho),...%prevent overflow in objective value
				'max',				double(kreisselmeier_max)...
			),...
			'lyapunov',			struct(...
				'Q',				lyapunov_Q...
			),...
			'normgain',			struct(...
				'R',				double(weightR),...
				'R_shift',			double(shiftR),...
				'K',				double(weightK),...
				'K_shift',			double(shiftK),...
				'F',				double(weightF),...
				'F_shift',			double(shiftF)...
			)...
		)...
	);
	objective_options_loose = struct(...
		'usecompiled',			objectiveoptions.usecompiled && isa(areafun_loose, 'GammaArea') && configuration.control.design.gamma.hascompiled(),...
		'numthreads',			uint32(objectiveoptions.numthreads),...
		'type',					objectiveoptions.type,...
		'weight',				double(objectiveoptions.weight),...
		'allowvarorder',		objectiveoptions.allowvarorder,...
		'eigenvaluederivative',	objectiveoptions.eigenvaluederivative,...
		'eigenvaluefilter',		objectiveoptions.eigenvaluefilter,...
		'eigenvalueignoreinf',	objectiveoptions.eigenvalueignoreinf,...
		'objective',			struct(...
			'preventNaN',		preventNaN,...
			'kreisselmeier',	struct(...
				'rho',				double(kreisselmeier_rho),...%prevent overflow in objective value
				'max',				double(kreisselmeier_max)...
			),...
			'lyapunov',			struct(...
				'Q',				lyapunov_Q...
			),...
			'normgain',			struct(...
				'R',				double(weightR),...
				'R_shift',			double(shiftR),...
				'K',				double(weightK),...
				'K_shift',			double(shiftK),...
				'F',				double(weightF),...
				'F_shift',			double(shiftF)...
			)...
		)...
	);
	solution_strategy = objectiveoptions.strategy;
	if ~isa(objectiveoptions.errorhandler, 'GammaErrorHandler')
		if isfunctionhandle(objectiveoptions.errorhandler)
			objectiveoptions.errorhandler_function = objectiveoptions.errorhandler;
			objectiveoptions.errorhandler = GammaErrorHandler.USER;
		else
			error('control:design:gamma', 'Error handler must be of type ''GammaErrorHandler''.');
		end
	end
	if objectiveoptions.errorhandler == GammaErrorHandler.USER
		if ~isfunctionhandle(objectiveoptions.errorhandler_function)
			error('control:design:gamma', 'Error handler must be a function handle.');
		end
		if nargin(objectiveoptions.errorhandler_function) ~= 2
			error('control:design:gamma', 'Error handler must have 2 input arguments, not %d.', nargin(objectiveoptions.errorhandler_function));
		end
		if nargout(objectiveoptions.errorhandler_function) ~= -1 && nargout(objectiveoptions.errorhandler_function) < 3
			error('control:design:gamma', 'Error handler must have 3 output arguments, not %d.', nargout(objectiveoptions.errorhandler_function));
		end
	else
		if ~isempty(objectiveoptions.errorhandler_function)
			error('control:design:gamma', 'Error handler must not be specified.');
		end
	end
	errorhandler = struct(...
		'type',		objectiveoptions.errorhandler,...
		'function',	objectiveoptions.errorhandler_function...
	);
end
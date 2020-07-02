function [J, gradJ, hessianJ] = Jeval(R, systems, areafun, weight, options, derivativeval)
	%JEVAL evaluate the objective function for a number of gain matrices
	%	Input:
	%		R:				proportional gain matrix to evaluate at
	%		systems:		systems to include into calculation
	%		areafun:		pole areas to use for objective calculation
	%		weight:			weight to use
	%		options:		objective options
	%		derivativeval:	R, K or F o return specific gradient information for corresponding variable
	%	Output:
	%		J:				objective value for gain matrices
	%		gradJ:			objective gradient for gain matrices
	%		hessianJ:		objective hessian for gain matrices
	if nargin <= 5
		derivativeval = 'R';
	end
	if nargin >= 5
		if isa(options, 'control.design.gamma.GammasynOptions')
			options = struct(options);
		end
	end
	if nargin <= 4 || ~isstruct(options)
		options = struct(...
			'usecompiled',	false,...
			'numthreads',	0,...
			'type',			GammaJType.EXP...
		);
	end
	if ~ischar(derivativeval) || ~isscalar(derivativeval)
		error('control:design:gamma:input', 'Derivative variable must be a scalar character.');
	end
	if ~any(derivativeval == ['R', 'K', 'F'])
		error('control:design:gamma:input', 'Derivative variable must be ''R'', ''K'' or ''F''.');
	end
	if isfield(options, 'allowvarorder')
		allowvarorder = options.allowvarorder;
	else
		allowvarorder = false;
	end
	if isfield(options, 'allownegativeweight')
		allownegativeweight = options.allownegativeweight;
	else
		allownegativeweight = false;
	end
	[system, areafun, areafun_loose, weight, weight_loose, dimensions, dimensions_loose] = checkandtransformargs(systems, areafun, weight, [], [], [], [], [], allowvarorder, allownegativeweight);
	[R_0, K_0, F_0] = checkinitialRKF(R, dimensions);
	[options] = checkobjectiveoptions(options, checkinitialR({R_0, K_0, F_0}, dimensions), dimensions.states, system, struct('ProblemType', optimization.options.ProblemType.CONSTRAINED), dimensions, dimensions_loose, areafun, areafun_loose, weight, weight_loose);
	function [J, gradJ, hessianJ] = Jfun(R, K, F)
		x = [
			reshape(R, dimensions.controls*dimensions.measurements, 1);
			reshape(K, dimensions.controls*dimensions.measurements_xdot, 1);
			reshape(F, dimensions.controls*dimensions.references, 1)
		];
		if nargout >= 3
			[J, gradJ, hessianJ] = control.design.gamma.J(x, system, weight, areafun, dimensions, options);
		elseif nargout >= 2
			[J, gradJ] = control.design.gamma.J(x, system, weight, areafun, dimensions, options);
		else
			J = control.design.gamma.J(x, system, weight, areafun, dimensions, options);
		end
	end
	function [J, gradJ, hessianJ] = Jfun_mex(R, K, F)
		x = [
			reshape(R, dimensions.controls*dimensions.measurements, 1);
			reshape(K, dimensions.controls*dimensions.measurements_xdot, 1);
			reshape(F, dimensions.controls*dimensions.references, 1)
		];
		if nargout >= 3
			[J, gradJ, hessianJ] = J_mex(x, system, weight, areafun, dimensions, options);
		elseif nargout >= 2
			[J, gradJ] = J_mex(x, system, weight, areafun, dimensions, options);
		else
			J = J_mex(x, system, weight, areafun, dimensions, options);
		end
	end
	switch derivativeval
		case 'R'
			idx_val = dimensions.index_R_free;
			dimension_measurement = dimensions.measurements;
		case 'K'
			idx_val = dimensions.index_K_free;
			dimension_measurement = dimensions.measurements_xdot;
		case 'F'
			idx_val = dimensions.index_F_free;
			dimension_measurement = dimensions.references;
		otherwise
			idx_val = dimensions.index_R_free;
			dimension_measurement = dimensions.measurements;
	end
	if ~ismatrix(R_0)
		J = zeros(1, size(R_0, 3));
		if nargout >= 2
			gradJ = zeros(dimensions.controls, dimension_measurement, size(R_0, 3));
			if nargout >= 3
				hessianJ = zeros(dimensions.controls*dimension_measurement, dimensions.controls*dimension_measurement, size(R_0, 3));
			end
		end
		for ii = 1:size(R_0, 3)
			if nargout >= 3
				if options.usecompiled
					[J(ii), tempGrad, hesseJtemp] = Jfun_mex(R_0(:, :, ii), K_0(:, :, ii), F_0(:, :, ii));
				else
					[J(ii), tempGrad, hesseJtemp] = Jfun(R_0(:, :, ii), K_0(:, :, ii), F_0(:, :, ii));
				end
				gradJ(:, :, ii) = reshape(tempGrad(idx_val, :), dimensions.controls, dimension_measurement);
				hessianJ(:, :, ii) = reshape(hesseJtemp(idx_val, idx_val, :), dimensions.controls*dimension_measurement, dimensions.controls*dimension_measurement);
			elseif nargout >= 2
				if options.usecompiled
					[J(ii), temp] = Jfun_mex(R_0(:, :, ii), K_0(:, :, ii), F_0(:, :, ii));
				else
					[J(ii), temp] = Jfun(R_0(:, :, ii), K_0(:, :, ii), F_0(:, :, ii));
				end
				gradJ(:, :, ii) = reshape(temp(idx_val, :), dimensions.controls, dimension_measurement);
			else
				if options.usecompiled
					J(ii) = Jfun_mex(R_0(:, :, ii), K_0(:, :, ii), F_0(:, :, ii));
				else
					J(ii) = Jfun(R_0(:, :, ii), K_0(:, :, ii), F_0(:, :, ii));
				end
			end
		end
	else
		if nargout >= 3
			if options.usecompiled
				[J, gradJ, hessianJ] = Jfun_mex(R_0, K_0, F_0);
			else
				[J, gradJ, hessianJ] = Jfun(R_0, K_0, F_0);
			end
			gradJ = gradJ(idx_val, :);
			hessianJ = hessianJ(idx_val, idx_val, :);
		elseif nargout >= 2
			if options.usecompiled
				[J, gradJ] = Jfun_mex(R_0, K_0, F_0);
			else
				[J, gradJ] = Jfun(R_0, K_0, F_0);
			end
			gradJ = gradJ(idx_val, :);
		else
			if options.usecompiled
				J = Jfun_mex(R_0, K_0, F_0);
			else
				J = Jfun(R_0, K_0, F_0);
			end
		end
	end
end
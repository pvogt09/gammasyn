function [c, ceq, gradc, gradceq, hessc, hessceq] = ceval(R, systems, areafun, weight, options, derivativeval)
	%CEVAL evaluate the constraint function for a number of gain matrices
	%	Input:
	%		R:				gain matrix to evaluate at
	%		systems:		systems to include into calculation
	%		areafun:		pole areas to use for objective calculation
	%		weight:			weight to use
	%		options:		objective options
	%		derivativeval:	R, K or F o return specific gradient information for corresponding variable
	%	Output:
	%		c:				constraint value for gain matrices
	%		ceq:			equality constraint value for gain matrices
	%		gradc:			constraint gradient for gain matrices
	%		gradceq:		equality constraint gradient for gain matrices
	%		hessc:			constraint hessian for gain matrices
	%		hessceq:		equality constraint hessian for gain matrices
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
			'usecompiled',			false,...
			'numthreads',			0,...
			'type',					GammaJType.EXP,...
			'decouplingcontrol',	struct(...
				'decouplingstrategy',	GammaDecouplingStrategy.NONE...
			)...
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
	systemoptions = struct();
	if isfield(options, 'decouplingcontrol')
		if ~isstruct(options.decouplingcontrol)
			error('control:design:gamma:input', 'Decoupling control options must be of type struct.');
		end
		decouplingoptions = checkobjectiveoptions_decoupling(options.decouplingcontrol);
		if decouplingoptions.decouplingstrategy ~= GammaDecouplingStrategy.NONE
			systemoptions.decouplingcontrol = GammaDecouplingStrategy_needsdecouplingconditions(options.decouplingcontrol.decouplingstrategy);
			if systemoptions.decouplingcontrol
				systemoptions.decouplingconditions = int32(decouplingoptions.decouplingconditions);
				systemoptions.usereferences = true;
				options.system.usereferences = true;
			end
		end
	end
	[system, areafun, areafun_loose, weight, weight_loose, dimensions, dimensions_loose] = checkandtransformargs(systems, areafun, weight, systemoptions, [], [], [], [], allowvarorder, allownegativeweight);
	[R_0, K_0, F_0] = checkinitialRKF(R, dimensions);
	[options] = checkobjectiveoptions(options, checkinitialR({R_0, K_0, F_0}, dimensions), dimensions.states, system, struct('ProblemType', optimization.options.ProblemType.CONSTRAINED), dimensions, dimensions_loose, areafun, areafun_loose, weight, weight_loose);
	function [c, ceq, gradc, gradceq, hessc, hessceq] = cfun(R, K, F)
		x = [
			reshape(R, dimensions.controls*dimensions.measurements, 1);
			reshape(K, dimensions.controls*dimensions.measurements_xdot, 1);
			reshape(F, dimensions.controls*dimensions.references, 1)
		];
		if nargout >= 6
			[c, ceq, gradc, gradceq, hessc, hessceq] = control.design.gamma.c(x, system, weight, areafun, dimensions, options);
		elseif nargout >= 5
			[c, ceq, gradc, gradceq, hessc] = control.design.gamma.c(x, system, weight, areafun, dimensions, options);
		elseif nargout >= 3
			[c, ceq, gradc, gradceq] = control.design.gamma.c(x, system, weight, areafun, dimensions, options);
		else
			[c, ceq] = control.design.gamma.c(x, system, weight, areafun, dimensions, options);
		end
	end
	function [c, ceq, gradc, gradceq, hessc, hessceq] = cfun_mex(R, K, F)
		x = [
			reshape(R, dimensions.controls*dimensions.measurements, 1);
			reshape(K, dimensions.controls*dimensions.measurements_xdot, 1);
			reshape(F, dimensions.controls*dimensions.references, 1)
		];
		if nargout >= 6
			[c, ceq, gradc, gradceq, hessc, hessceq] = c_mex(x, system, weight, areafun, dimensions, options);
		elseif nargout >= 5
			[c, ceq, gradc, gradceq, hessc] = c_mex(x, system, weight, areafun, dimensions, options);
		elseif nargout >= 3
			[c, ceq, gradc, gradceq] = c_mex(x, system, weight, areafun, dimensions, options);
		else
			[c, ceq] = c_mex(x, system, weight, areafun, dimensions, options);
		end
	end
	if GammaDecouplingStrategy_needsdecouplingconditions(options.decouplingcontrol.decouplingstrategy)
		number_inputconditions = sum(2*(repmat(dimensions.states, dimensions.references, 1) - dimensions.m_invariant) + dimensions.number_decouplingconditions.*dimensions.hasfeedthrough_decoupling);
		number_outputconditions = sum(2*(dimensions.number_decouplingconditions.*dimensions.m_invariant));
		if options.decouplingcontrol.decouplingstrategy == GammaDecouplingStrategy.NUMERIC_NONLINEAR_EQUALITY
			number_decouplingconstraints = 1;
			number_decouplingconstraints_eq = dimensions.models*(number_outputconditions + number_inputconditions);
		elseif options.decouplingcontrol.decouplingstrategy == GammaDecouplingStrategy.NUMERIC_NONLINEAR_INEQUALITY
			number_decouplingconstraints = 2*dimensions.models*(number_outputconditions + number_inputconditions) + 1;
			number_decouplingconstraints_eq = 0;
		else
			number_decouplingconstraints = 0;
			number_decouplingconstraints_eq = 0;
		end
	else
		number_decouplingconstraints = 0;
		number_decouplingconstraints_eq = 0;
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
		c = NaN(dimensions.models*dimensions.areas_max*dimensions.states + number_decouplingconstraints, size(R_0, 3));
		if nargout >= 2
			ceq = NaN(number_decouplingconstraints_eq, size(R_0, 3));
			if nargout >= 3
				gradc = NaN(dimensions.models*dimensions.areas_max*dimensions.states + number_decouplingconstraints, dimensions.controls, dimension_measurement, size(R_0, 3));
				if nargout >= 4
					gradceq = NaN(number_decouplingconstraints_eq, dimensions.controls, dimension_measurement, size(R_0, 3));
					if nargout >= 5
						hessc = NaN(dimensions.controls*dimension_measurement, dimensions.controls*dimension_measurement, dimensions.models*dimensions.areas_max*dimensions.states + number_decouplingconstraints, size(R_0, 3));
						if nargout >= 6
							hessceq = NaN(dimensions.controls*dimension_measurement, dimensions.controls*dimension_measurement, number_decouplingconstraints_eq, size(R_0, 3));
						end
					end
				end
			end
		end
		for ii = 1:size(R_0, 3) %#ok<FORPF> no parfor because of parfor in constraint functions
			if nargout >= 6
				if options.usecompiled
					[cc, cceq, temp, tempeq, temphess, temphesseq] = cfun_mex(R_0(:, :, ii), K_0(:, :, ii), F_0(:, :, ii));
				else
					[cc, cceq, temp, tempeq, temphess, temphesseq] = cfun(R_0(:, :, ii), K_0(:, :, ii), F_0(:, :, ii));
				end
				c(:, ii) = [
					cc;
					NaN(size(c, 1) - size(cc, 1), 1)
				];
				ceq(:, ii) = [
					cceq;
					NaN(size(ceq, 1) - size(cceq, 1), 1)
				];
				for jj = 1:size(temp, 2)
					gradc(jj, :, :, ii) = reshape(temp(idx_val, jj), [], dimensions.controls, dimension_measurement);
					hessc(:, :, jj, ii) = temphess(idx_val, idx_val, jj);
				end
				for jj = 1:size(tempeq, 2)
					gradceq(jj, :, :, ii) = reshape(tempeq(idx_val, jj), [], dimensions.controls, dimension_measurement);
					hessceq(:, :, jj, ii) = temphesseq(idx_val, idx_val, jj);
				end
			elseif nargout >= 5
				if options.usecompiled
					[cc, cceq, temp, tempeq, temphess] = cfun_mex(R_0(:, :, ii), K_0(:, :, ii), F_0(:, :, ii));
				else
					[cc, cceq, temp, tempeq, temphess] = cfun(R_0(:, :, ii), K_0(:, :, ii), F_0(:, :, ii));
				end
				c(:, ii) = [
					cc;
					NaN(size(c, 1) - size(cc, 1), 1)
				];
				ceq(:, ii) = [
					cceq;
					NaN(size(ceq, 1) - size(cceq, 1), 1)
				];
				for jj = 1:size(temp, 2)
					gradc(jj, :, :, ii) = reshape(temp(idx_val, jj), [], dimensions.controls, dimension_measurement);
					hessc(:, :, jj, ii) = temphess(idx_val, idx_val, jj);
				end
				for jj = 1:size(tempeq, 2)
					gradceq(jj, :, :, ii) = reshape(tempeq(idx_val, jj), [], dimensions.controls, dimension_measurement);
				end
			elseif nargout >= 4
				if options.usecompiled
					[cc, cceq, temp, tempeq] = cfun_mex(R_0(:, :, ii), K_0(:, :, ii), F_0(:, :, ii));
				else
					[cc, cceq, temp, tempeq] = cfun(R_0(:, :, ii), K_0(:, :, ii), F_0(:, :, ii));
				end
				c(:, ii) = [
					cc;
					NaN(size(c, 1) - size(cc, 1), 1)
				];
				ceq(:, ii) = [
					cceq;
					NaN(size(ceq, 1) - size(cceq, 1), 1)
				];
				for jj = 1:size(temp, 2)
					gradc(jj, :, :, ii) = reshape(temp(idx_val, jj), [], dimensions.controls, dimension_measurement);
				end
				for jj = 1:size(tempeq, 2)
					gradceq(jj, :, :, ii) = reshape(tempeq(idx_val, jj), [], dimensions.controls, dimension_measurement);
				end
			elseif nargout >= 3
				if options.usecompiled
					[cc, cceq, temp] = cfun_mex(R_0(:, :, ii), K_0(:, :, ii), F_0(:, :, ii));
				else
					[cc, cceq, temp] = cfun(R_0(:, :, ii), K_0(:, :, ii), F_0(:, :, ii));
				end
				c(:, ii) = [
					cc;
					NaN(size(c, 1) - size(cc, 1), 1)
				];
				ceq(:, ii) = [
					cceq;
					NaN(size(ceq, 1) - size(cceq, 1), 1)
				];
				for jj = 1:size(temp, 2)
					gradc(jj, :, :, ii) = reshape(temp(idx_val, jj), [], dimensions.controls, dimension_measurement);
				end
			elseif nargout >= 2
				if options.usecompiled
					[cc, cceq] = cfun_mex(R_0(:, :, ii), K_0(:, :, ii), F_0(:, :, ii));
				else
					[cc, cceq] = cfun(R_0(:, :, ii), K_0(:, :, ii), F_0(:, :, ii));
				end
				c(:, ii) = [
					cc;
					NaN(size(c, 1) - size(cc, 1), 1)
				];
				ceq(:, ii) = [
					cceq;
					NaN(size(ceq, 1) - size(cceq, 1), 1)
				];
			else
				if options.usecompiled
					cc = cfun_mex(R_0(:, :, ii), K_0(:, :, ii), F_0(:, :, ii));
				else
					cc = cfun(R_0(:, :, ii), K_0(:, :, ii), F_0(:, :, ii));
				end
				c(:, ii) = [
					cc;
					NaN(size(c, 1) - size(cc, 1), 1)
				];
			end
		end
	else
		if nargout >= 6
			if options.usecompiled
				[c, ceq, gradc, gradceq, hessc, hessceq] = cfun_mex(R_0, K_0, F_0);
			else
				[c, ceq, gradc, gradceq, hessc, hessceq] = cfun(R_0, K_0, F_0);
			end
			gradc = gradc(idx_val, :);
			gradceq = gradceq(idx_val, :);
			hessc = hessc(idx_val, idx_val, :);
			hessceq = hessceq(idx_val, idx_val, :);
		elseif nargout >= 5
			if options.usecompiled
				[c, ceq, gradc, gradceq, hessc] = cfun_mex(R_0, K_0, F_0);
			else
				[c, ceq, gradc, gradceq, hessc] = cfun(R_0, K_0, F_0);
			end
			gradc = gradc(idx_val, :);
			gradceq = gradceq(idx_val, :);
			hessc = hessc(idx_val, idx_val, :);
		elseif nargout >= 4
			if options.usecompiled
				[c, ceq, gradc, gradceq] = cfun_mex(R_0, K_0, F_0);
			else
				[c, ceq, gradc, gradceq] = cfun(R_0, K_0, F_0);
			end
			gradc = gradc(idx_val, :);
			gradceq = gradceq(idx_val, :);
		elseif nargout >= 3
			if options.usecompiled
				[c, ceq, gradc] = cfun_mex(R_0, K_0, F_0);
			else
				[c, ceq, gradc] = cfun(R_0, K_0, F_0);
			end
			gradc = gradc(idx_val, :);
		elseif nargout >= 2
			if options.usecompiled
				[c, ceq] = cfun_mex(R_0, K_0, F_0);
			else
				[c, ceq] = cfun(R_0, K_0, F_0);
			end
		else
			if options.usecompiled
				c = cfun_mex(R_0, K_0, F_0);
			else
				c = cfun(R_0, K_0, F_0);
			end
		end
	end
end
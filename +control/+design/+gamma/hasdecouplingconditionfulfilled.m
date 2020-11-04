function [has, margin] = hasdecouplingconditionfulfilled(systems, R, options, tolerance)
	%HASDECOUPLINGCONDITIONSFULFILLED return if system fulfills decoupling conditions
	%	Input:
	%		systems:	systems to check decoupling conditions for
	%		R:			gain matrix for control of systems
	%		options:	structure with decoupling controller design opions
	%		tolerance:	tolerance for checking
	%	Output:
	%		has:		true, when decoupling conditions are fulfilled, else false
	%		margin:		maximum distance to decoupling condition fulfillment
	if nargin <= 3
		tolerance = 0;
	end
	if ~isstruct(options)
		error('control:design:gamma:decouplingcondition', 'Decoupling options must be a structure, not a ''%s''.', class(options));
	end
	options = checkobjectiveoptions_decoupling(options);
	options.decouplingstrategy = GammaDecouplingStrategy.NUMERIC_NONLINEAR_EQUALITY;
	systemoptions = struct(...
		'decouplingcontrol',	true,...
		'decouplingconditions',	options.decouplingconditions...
	);
	[system, ~, ~, ~, ~, dimensions, ~] = checkandtransformargs(systems, [], 1, systemoptions, [], [], [], [], true);
	if iscell(R)
		if numel(R) >= 3
			F = R{3};
			K = R{2};
			R = R{1};
		elseif numel(R) >= 2
			K = R{2};
			R = R{1};
			F = zeros(dimensions.controls, dimensions.references);
		else
			K = zeros(dimensions.controls, dimensions.measurements_xdot);
			F = zeros(dimensions.controls, dimensions.references);
		end
	else
		K = zeros(dimensions.controls, dimensions.measurements_xdot);
		F = zeros(dimensions.controls, dimensions.references);
	end
	options = struct(...
		'usecompiled',			configuration.control.design.gamma.hascompiled(),...
		'numthreads',			configuration.matlab.numthreads(),...
		'type',					GammaJType.EXP,...
		'decouplingcontrol',	options,...
		'allowvarorder',		true...
	);
	returnmargin = nargout >= 2;
	if ~ismatrix(R) || ~ismatrix(K) || ~ismatrix(F)
		szR = size(R);
		szK = size(K);
		szF = size(F);
		if prod(szR(3:end)) ~= prod(szK(3:end))
			if prod(szR(3:end)) == 1
				R = repmat(R, [1, 1, prod(szK(3:end))]);
				szR = size(R);
			elseif prod(szK(3:end)) == 1
				K = repmat(K, [1, 1, prod(szR(3:end))]);
				szK = size(K);
			else
				error('control:design:gamma:decouplingcondition', 'Number of gain matrices must be equal.');
			end
		end
		if prod(szR(3:end)) ~= prod(szF(3:end))
			if prod(szR(3:end)) == 1
				R = repmat(R, [1, 1, prod(szF(3:end))]);
				szR = size(R);
			elseif prod(szF(3:end)) == 1
				F = repmat(F, [1, 1, prod(szR(3:end))]);
				szF = size(F);
			else
				error('control:design:gamma:decouplingcondition', 'Number of gain matrices must be equal.');
			end
		end
		if prod(szK(3:end)) ~= prod(szF(3:end))
			if prod(szK(3:end)) == 1
				K = repmat(K, [1, 1, prod(szF(3:end))]);
				szK = size(K);
			elseif prod(szF(3:end)) == 1
				F = repmat(F, [1, 1, prod(szK(3:end))]);
				szF = size(F);
			else
				error('control:design:gamma:decouplingcondition', 'Number of gain matrices must be equal.');
			end
		end
		Rtotest = reshape(R, [szR(1), szR(2), prod(szR(3:end))]);
		Ktotest = reshape(K, [szK(1), szK(2), prod(szK(3:end))]);
		Ftotest = reshape(F, [szF(1), szF(2), prod(szF(3:end))]);
		inareatest = false(prod(szR(3:end)), size(system, 1));
		if returnmargin
			margintest = zeros(prod(szR(3:end)), size(system, 1));
		end
		usecompiled = options.usecompiled;
		parfor ii = 1:size(Rtotest, 3)
			[eigenvalues, eigenvector_right, eigenvector_left] = calculate_eigenvalues(system, Rtotest(:, :, ii), Ktotest(:, :, ii), dimensions, usecompiled);
			[~, ceq] = calculate_decoupling_conditions(system, Rtotest(:, :, ii), Ktotest(:, :, ii), Ftotest(:, :, ii), dimensions, options, eigenvalues, eigenvector_right, eigenvector_left);
			inareatest(ii, :) = all(abs(ceq(:)) <= tolerance);
			if returnmargin
				margintest(ii, :) = max(abs(ceq(:)));
			end
		end
		if size(szR, 2) <= 3
			szR = [szR, 1];
		end
		inareatest = all(inareatest, 2);
		has = reshape(inareatest, szR(3:end));
		if returnmargin
			margin = reshape(max(margintest, [], 2), szR(3:end));
		end
	else
		[eigenvalues, eigenvector_right, eigenvector_left] = calculate_eigenvalues(system, R, K, dimensions, options.usecompiled);
		[~, ceq] = calculate_decoupling_conditions(system, R, K, F, dimensions, options, eigenvalues, eigenvector_right, eigenvector_left);
		has = all(abs(ceq(:)) <= tolerance);
		if returnmargin
			margin = max(abs(ceq(:)));
		end
	end
end
function [inarea, margin] = hasallpolesinarea(systems, R, areafun, forsystem, tolerance)
	%HASALLPOLESINAREA return if poles of the supplied systems lie in the specified pole area
	%	Input:
	%		systems:	systems to check poles for
	%		R:			gain matrix for control of systems
	%		areafun:	area border functions
	%		forsystem:	return indicator for every system if true, else return an aggregated value
	%		tolerance:	tolerance for checking
	%	Output:
	%		inarea:		true, where all poles of all systems lie in the specified area, else false
	%		margin:		maximum distance to area boundary
	if nargin <= 3
		forsystem = true;
	end
	if nargin <= 4
		if ~islogical(forsystem)
			if ~isnumeric(forsystem) || ~isscalar(forsystem)
				error('control:design:gamma:poleinarea', 'Tolerance must be a numeric scalar.');
			end
			tolerance = forsystem;
			forsystem = true;
		else
			tolerance = 0;
		end
	end
	[system, areafun, ~, weight, ~, dimensions, ~] = checkandtransformargs(systems, areafun, 1, struct(), [], [], [], [], true);
	if iscell(R)
		if numel(R) >= 2
			K = R{2};
			R = R{1};
		else
			K = zeros(dimensions.controls, dimensions.measurements_xdot);
		end
	else
		K = zeros(dimensions.controls, dimensions.measurements_xdot);
	end
	options = struct(...
		'usecompiled',		isa(areafun, 'GammaArea') && configuration.control.design.gamma.hascompiled(),...
		'type',				GammaJType.EXP,...
		'allowvarorder',	true...
	);
	returnmargin = nargout >= 2;
	if ~ismatrix(R) || ~ismatrix(K)
		szR = size(R);
		szK = size(K);
		if prod(szR(3:end)) ~= prod(szK(3:end))
			if prod(szR(3:end)) == 1
				R = repmat(R, [1, 1, prod(szK(3:end))]);
				szR = size(R);
			elseif prod(szK(3:end)) == 1
				K = repmat(K, [1, 1, prod(szR(3:end))]);
				szK = size(K);
			else
				error('control:design:gamma:poleinarea', 'Number of gain matrices must be equal.');
			end
		end
		Rtotest = reshape(R, [szR(1), szR(2), prod(szR(3:end))]);
		Ktotest = reshape(K, [szK(1), szK(2), prod(szK(3:end))]);
		inareatest = false(prod(szR(3:end)), size(system, 1));
		if returnmargin
			margintest = zeros(prod(szR(3:end)), size(system, 1));
		end
		usecompiled = options.usecompiled;
		parfor ii = 1:size(Rtotest, 3)
			eigenvalues = calculate_eigenvalues(system, Rtotest(:, :, ii), Ktotest(:, :, ii), dimensions, usecompiled);
			areaval = calculate_areas(areafun, weight, eigenvalues, dimensions, options);
			inareatest(ii, :) = all(all(areaval <= tolerance, 3), 2)';
			if returnmargin
				areaval(areaval <= 0) = 0;
				margintest(ii, :) = max(max(areaval, [], 3), [], 2)';
			end
		end
		if forsystem
			inarea = false([size(system, 1), szR(3:end)]);
			if returnmargin
				margin = zeros([size(system, 1), szR(3:end)]);
			end
			temp = repmat({':'}, 1, ndims(R) - 2);
			sub = substruct('()', [{1}, temp]);
			for ii = 1:size(system, 1) %#ok<FORPF> parfor does not work with subsasgn
				sub.subs{1} = ii;
				if size(szR, 2) > 3
					inarea = subsasgn(inarea, sub, reshape(inareatest(:, ii), szR(3:end)));
					if returnmargin
						margin = subsasgn(margin, sub, reshape(margintest(:, ii), szR(3:end)));
					end
				else
					inarea = subsasgn(inarea, sub, reshape(inareatest(:, ii), [1, szR(3:end)]));
					if returnmargin
						margin = subsasgn(margin, sub, reshape(margintest(:, ii), [1, szR(3:end)]));
					end
				end
			end
		else
			if size(szR, 2) <= 3
				szR = [szR, 1];
			end
			inareatest = all(inareatest, 2);
			inarea = reshape(inareatest, szR(3:end));
			if returnmargin
				margin = reshape(max(margintest, [], 2), szR(3:end));
			end
		end
	else
		eigenvalues = calculate_eigenvalues(system, R, K, dimensions, options.usecompiled);
		areaval = calculate_areas(areafun, weight, eigenvalues, dimensions, options);
		if forsystem
			inarea = all(all(areaval <= tolerance, 3), 2);
			if returnmargin
				areaval(areaval <= 0) = 0;
				margin = max(max(areaval, [], 3), [], 2);
			end
		else
			inarea = all(areaval(:) <= tolerance);
			if returnmargin
				areaval(areaval <= 0) = 0;
				margin = max(areaval(:));
			end
		end
	end
end
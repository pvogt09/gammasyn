function [S, R_0] = convert_constraints_to_oplace(R_fixed, R_bounds, R_nonlin, R_0)
	%CONVERT_CONSTRAINTS_TO_OPLACE convert gammasyn constraint description to oplace constraint description
	%	Input:
	%		R_fixed:	cell array with indicator matrix for gain elements that should be fixed and the values the fixed gains have, empty if no fixed elements are needed. For linear dependent gain components the first element is a 3D matrix with relation in the third dimension and the second element a vector of constant constraint elements
	%		R_bounds:	cell array with indicator matrix for gain elements that should be bounded and the values the gains are bounded by, empty if no bounded elements are needed. For linear dependent gain components the first element is a 3D matrix with relation in the third dimension and the second element a vector of upper bound constraint elements
	%		R_nonlin:	function pointer to a function of nonlinear inequality and equality constraints on gains with signature [c_R, ceq_R, c_K, ceq_K, c_F, ceq_F, gradc_R, gradceq_R, gradc_K, gradceq_K, gradc_F, gradceq_F] = constraint(R, K, F)
	%		R_0:		initial value
	%	Output:
	%		S:			constraint description to use with oplace
	%		R_0:		initial value to use with oplace
	if isstruct(R_fixed) && all(isfield(R_fixed, {
		'R_fixed_has';
		'R_fixed_only';
		'R_fixed_constraints';
		'R_fixed';
		'R_fixed_values';
		'R_fixed_A';
		'R_fixed_b';
		'R_fixed_T';
		'R_fixed_T_inv'
	}))
		if R_fixed.R_fixed_has && R_fixed.R_fixed_only
			S = R_fixed.R_fixed;
			if nargout >= 2
				if size(R_0, 1) ~= size(R_fixed.R_fixed, 1)
					error('control:design:gamma:oplace', 'Initial value must have %d columns.', size(R_fixed.R_fixed, 1));
				end
				if size(R_0, 2) ~= size(R_fixed.R_fixed, 2)
					error('control:design:gamma:oplace', 'Initial value must have %d rows.', size(R_fixed.R_fixed, 2));
				end
				if ndims(R_0) <= 2
					R_0(R_fixed.R_fixed) = R_fixed.R_fixed_values(R_fixed.R_fixed);
				else
					for ii = 1:size(R_0, 3)
						temp = R_0(:, :, ii);
						temp(R_fixed.R_fixed) = R_fixed.R_fixed_values(R_fixed.R_fixed);
						R_0(:, :, ii) = temp;
					end
				end
			end
		else
			error('control:design:gamma:oplace:constrainttype', 'Linear dependent constraints can not be handled by oplace.');
		end
	else
		number_controls = size(R_0, 1);
		number_measurements = size(R_0, 2);
		[R_fixed, ~, ~, ~, ~, ~, hasfixed_R, onlyfixed_R] = checkandtransform_gain_fixed(R_fixed, number_controls, number_measurements, 'proportional');
		if hasfixed_R && onlyfixed_R
			S = R_fixed{1};
			if nargout >= 2
				if size(R_0, 1) ~= size(R_fixed{1}, 1)
					error('control:design:gamma:oplace', 'Initial value must have %d columns.', size(R_fixed{1}, 1));
				end
				if size(R_0, 2) ~= size(R_fixed{1}, 2)
					error('control:design:gamma:oplace', 'Initial value must have %d rows.', size(R_fixed{1}, 2));
				end
				if ndims(R_0) <= 2
					R_0(R_fixed{1}) = R_fixed{2}(R_fixed{1});
				else
					for ii = 1:size(R_0, 3)
						temp = R_0(:, :, ii);
						temp(R_fixed{1}) = R_fixed{2}(R_fixed{1});
						R_0(:, :, ii) = temp;
					end
				end
			end
		else
			error('control:design:gamma:oplace:constrainttype', 'Linear dependent constraints can not be handled by oplace.');
		end
	end
	if ~isempty(R_bounds)
		warning('control:design:gamma:oplace:constrainttype', 'Bound constraints are ignored for oplace.');
	end
	if ~isempty(R_nonlin)
		warning('control:design:gamma:oplace:constrainttype', 'Nonlinear constraints are ignored for oplace.');
	end
end
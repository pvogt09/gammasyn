function [eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative, eigenvalue_derivative_xdot, eigenvector_right_derivative, eigenvector_right_derivative_xdot, eigenvector_left_derivative, eigenvector_left_derivative_xdot, eigenvalue_2derivative, eigenvalue_2derivative_xdot, eigenvalue_2derivative_mixed, eigenvalue_2derivative_xdot_mixed] = calculate_eigenvalue_filter(filtertype, eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative, eigenvalue_derivative_xdot, eigenvector_right_derivative, eigenvector_right_derivative_xdot, eigenvector_left_derivative, eigenvector_left_derivative_xdot, eigenvalue_2derivative, eigenvalue_2derivative_xdot, eigenvalue_2derivative_mixed, eigenvalue_2derivative_xdot_mixed)
	%CALCULATE_EIGENVALUES function for filtering eigenvalues and the corrsponding derivatives
	%	Input:
	%		filtertype:							type of eigenvalue filtering
	%		eigenvalues:						eigenvalues of all systems (NaN + 1i*NaN for systems with fewer states than maximum number of states)
	%		eigenvector_right:					right eigenvectors of all systems (NaN + 1i*NaN for systems with fewer states than maximum number of states)
	%		eigenvector_left:					left eigenvectors of all systems (NaN + 1i*NaN for systems with fewer states than maximum number of states)
	%		eigenvalue_derivative:				derivative of eigenvalues with respect to proportional gain matrix for calculation of objective function gradient
	%		eigenvalue_derivative_xdot:			derivative of eigenvalues with respect to derivative gain matrix for calculation of objective function gradient
	%		eigenvector_right_derivative:		derivative of right eigenvectors with respect to proportional gain matrix for calculation of objective function gradient
	%		eigenvector_right_derivative_xdot:	derivative of right eigenvectors with respect to derivative gain matrix for calculation of objective function gradient
	%		eigenvector_left_derivative:		derivative of left eigenvectors with respect to proportional gain matrix for calculation of objective function gradient
	%		eigenvector_left_derivative_xdot:	derivative of left eigenvectors with respect to derivative gain matrix for calculation of objective function gradient
	%		eigenvalue_2derivative:				second derivative of eigenvalues with respect to proportional gain matrix for calculation of hessian matrix
	%		eigenvalue_2derivative_xdot:		second derivative of eigenvalues with respect to derivative gain matrix for calculation of hessian matrix
	%		eigenvalue_2derivative_mixed:		second derivative of eigenvalues with respect to proportional and derivative gain matrix for calculation of hessian matrix
	%		eigenvalue_2derivative_xdot_mixed:	second derivative of eigenvalues with respect to derivative and proportional gain matrix for calculation of hessian matrix
	%	Output:
	%		eigenvalues:						eigenvalues of all systems (NaN + 1i*NaN for systems with fewer states than maximum number of states)
	%		eigenvector_right:					right eigenvectors of all systems (NaN + 1i*NaN for systems with fewer states than maximum number of states)
	%		eigenvector_left:					left eigenvectors of all systems (NaN + 1i*NaN for systems with fewer states than maximum number of states)
	%		eigenvalue_derivative:				derivative of eigenvalues with respect to proportional gain matrix for calculation of objective function gradient
	%		eigenvalue_derivative_xdot:			derivative of eigenvalues with respect to derivative gain matrix for calculation of objective function gradient
	%		eigenvector_right_derivative:		derivative of right eigenvectors with respect to proportional gain matrix for calculation of objective function gradient
	%		eigenvector_right_derivative_xdot:	derivative of right eigenvectors with respect to derivative gain matrix for calculation of objective function gradient
	%		eigenvector_left_derivative:		derivative of left eigenvectors with respect to proportional gain matrix for calculation of objective function gradient
	%		eigenvector_left_derivative_xdot:	derivative of left eigenvectors with respect to derivative gain matrix for calculation of objective function gradient
	%		eigenvalue_2derivative:				second derivative of eigenvalues with respect to proportional gain matrix for calculation of hessian matrix
	%		eigenvalue_2derivative_xdot:		second derivative of eigenvalues with respect to derivative gain matrix for calculation of hessian matrix
	%		eigenvalue_2derivative_mixed:		second derivative of eigenvalues with respect to proportional and derivative gain matrix for calculation of hessian matrix
	%		eigenvalue_2derivative_xdot_mixed:	second derivative of eigenvalues with respect to derivative and proportional gain matrix for calculation of hessian matrix
	switch filtertype
		case GammaEigenvalueFilterType.NONE
			return;
		case GammaEigenvalueFilterType.NEGATIVEIMAG
			remove = imag(eigenvalues) < 0;
		case GammaEigenvalueFilterType.POSITIVEIMAG
			remove = imag(eigenvalues) > 0;
		otherwise
			error('control:design:gamma:eigenvalues:filter', 'Undefined eigenvalue filter type.');
	end
	remove = remove & ~isnan(eigenvalues);
	if ~any(remove(:))
		return;
	end
	% TODO: can this be done without find and sub2ind only by logical indexing for matrices with different number of planes?
	[idx_state, idx_model] = ind2sub(size(eigenvalues), find(remove));
	eigenvalues(remove) = conj(eigenvalues(remove));
	if nargout >= 2
		if ~isempty(eigenvector_right)
			eigenvector_right(:, idx_state, idx_model) = conj(eigenvector_right(:, idx_state, idx_model));
		end
		if nargout >= 3
			if ~isempty(eigenvector_left)
				eigenvector_left(:, idx_state, idx_model) = conj(eigenvector_left(:, idx_state, idx_model));
			end
			if nargout >= 4
				if ~isempty(eigenvalue_derivative)
					eigenvalue_derivative(idx_state, :, :, idx_model) = conj(eigenvalue_derivative(idx_state, :, :, idx_model));
				end
				if nargout >= 5
					if ~isempty(eigenvalue_derivative_xdot)
						eigenvalue_derivative_xdot(idx_state, :, :, idx_model) = conj(eigenvalue_derivative_xdot(idx_state, :, :, idx_model));
					end
					if nargout >= 6
						if ~isempty(eigenvector_right_derivative)
							eigenvector_right_derivative(:, idx_state, :, :, idx_model) = conj(eigenvector_right_derivative(:, idx_state, :, :, idx_model));
						end
						if nargout >= 7
							if ~isempty(eigenvector_right_derivative_xdot)
								eigenvector_right_derivative_xdot(:, idx_state, :, :, idx_model) = conj(eigenvector_right_derivative_xdot(:, idx_state, :, :, idx_model));
							end
							if nargout >= 8
								if ~isempty(eigenvector_left_derivative)
									eigenvector_left_derivative(:, idx_state, :, :, idx_model) = conj(eigenvector_left_derivative(:, idx_state, :, :, idx_model));
								end
								if nargout >= 9
									if ~isempty(eigenvector_left_derivative_xdot)
										eigenvector_left_derivative_xdot(:, idx_state, :, :, idx_model) = conj(eigenvector_left_derivative_xdot(:, idx_state, :, :, idx_model));
									end
									if nargout >= 10
										eigenvalue_2derivative(idx_state, :, :, idx_model) = conj(eigenvalue_2derivative(idx_state, :, :, idx_model));
										if nargout >= 11
											if ~isempty(eigenvalue_2derivative_xdot)
												eigenvalue_2derivative_xdot(idx_state, :, :, idx_model) = conj(eigenvalue_2derivative_xdot(idx_state, :, :, idx_model));
											end
											if nargout >= 12
												if ~isempty(eigenvalue_2derivative_mixed)
													eigenvalue_2derivative_mixed(idx_state, :, :, idx_model) = conj(eigenvalue_2derivative_mixed(idx_state, :, :, idx_model));
												end
												if nargout >= 13
													if ~isempty(eigenvalue_2derivative_xdot_mixed)
														eigenvalue_2derivative_xdot_mixed(idx_state, :, :, idx_model) = conj(eigenvalue_2derivative_xdot_mixed(idx_state, :, :, idx_model));
													end
												end
											end
										end
									end
								end
							end
						end
					end
				end
			end
		end
	end
end
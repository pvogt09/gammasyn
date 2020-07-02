function [eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative, eigenvalue_derivative_xdot, eigenvector_right_derivative, eigenvector_right_derivative_xdot, eigenvector_left_derivative, eigenvector_left_derivative_xdot, eigenvalue_2derivative, eigenvalue_2derivative_xdot, eigenvalue_2derivative_mixed, eigenvalue_2derivative_xdot_mixed] = calculate_eigenvalues(system, R, K, dimensions, eigenvaluederivativetype, usecompiled, numthreads)
	%CALCULATE_EIGENVALUES helper function for calculation of eigenvalues for all systems
	%	Input:
	%		system:								structure with systems to calculate eigenvalues for
	%		R:									current proportional gain matrix
	%		K:									current derivative gain matrix
	%		dimensions:							structure with information about dimensions of the different variables and systems
	%		eigenvaluederivativetype:			GammaEigenvalueDerivativeType to indicate which method for eigenvalue derivative calculation should be used
	%		usecompiled:						indicator, if compiled version of this function should be used
	%		numthreads:							number of threads to run loops in
	%	Output:
	%		eigenvalues:						eigenvalues of all systems (NaN for systems with fewer states than maximum number of states)
	%		eigenvector_right:					right eigenvectors of all systems (NaN for systems with fewer states than maximum number of states)
	%		eigenvector_left:					left eigenvectors of all systems (NaN for systems with fewer states than maximum number of states)
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
	if nargin <= 4
		eigenvaluederivativetype = GammaEigenvalueDerivativeType.getDefaultValue();
	end
	if nargin <= 5
		usecompiled = false;
	end
	if nargin <= 6
		numthreads = configuration.matlab.numthreads();
	end
	numthreads = uint32(floor(max([0, numthreads])));
	if usecompiled
		if nargout >= 13
			[eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative, eigenvalue_derivative_xdot, eigenvector_right_derivative, eigenvector_right_derivative_xdot, eigenvector_left_derivative, eigenvector_left_derivative_xdot, eigenvalue_2derivative, eigenvalue_2derivative_xdot, eigenvalue_2derivative_mixed, eigenvalue_2derivative_xdot_mixed] = calculate_eigenvalues_mex(system, R, K, dimensions, eigenvaluederivativetype, numthreads);
		elseif nargout >= 12
			[eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative, eigenvalue_derivative_xdot, eigenvector_right_derivative, eigenvector_right_derivative_xdot, eigenvector_left_derivative, eigenvector_left_derivative_xdot, eigenvalue_2derivative, eigenvalue_2derivative_xdot, eigenvalue_2derivative_mixed] = calculate_eigenvalues_mex(system, R, K, dimensions, eigenvaluederivativetype, numthreads);
		elseif nargout >= 11
			[eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative, eigenvalue_derivative_xdot, eigenvector_right_derivative, eigenvector_right_derivative_xdot, eigenvector_left_derivative, eigenvector_left_derivative_xdot, eigenvalue_2derivative, eigenvalue_2derivative_xdot] = calculate_eigenvalues_mex(system, R, K, dimensions, eigenvaluederivativetype, numthreads);
		elseif nargout >= 10
			[eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative, eigenvalue_derivative_xdot, eigenvector_right_derivative, eigenvector_right_derivative_xdot, eigenvector_left_derivative, eigenvector_left_derivative_xdot, eigenvalue_2derivative] = calculate_eigenvalues_mex(system, R, K, dimensions, eigenvaluederivativetype, numthreads);
		elseif nargout >= 9
			[eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative, eigenvalue_derivative_xdot, eigenvector_right_derivative, eigenvector_right_derivative_xdot, eigenvector_left_derivative, eigenvector_left_derivative_xdot] = calculate_eigenvalues_mex(system, R, K, dimensions, eigenvaluederivativetype, numthreads);
		elseif nargout >= 8
			[eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative, eigenvalue_derivative_xdot, eigenvector_right_derivative, eigenvector_right_derivative_xdot, eigenvector_left_derivative] = calculate_eigenvalues_mex(system, R, K, dimensions, eigenvaluederivativetype, numthreads);
		elseif nargout >= 7
			[eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative, eigenvalue_derivative_xdot, eigenvector_right_derivative, eigenvector_right_derivative_xdot] = calculate_eigenvalues_mex(system, R, K, dimensions, eigenvaluederivativetype, numthreads);
		elseif nargout >= 6
			[eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative, eigenvalue_derivative_xdot, eigenvector_right_derivative] = calculate_eigenvalues_mex(system, R, K, dimensions, eigenvaluederivativetype, numthreads);
		elseif nargout >= 5
			[eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative, eigenvalue_derivative_xdot] = calculate_eigenvalues_mex(system, R, K, dimensions, eigenvaluederivativetype, numthreads);
		elseif nargout >= 4
			[eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative] = calculate_eigenvalues_mex(system, R, K, dimensions, eigenvaluederivativetype, numthreads);
		elseif nargout >= 3
			[eigenvalues, eigenvector_right, eigenvector_left] = calculate_eigenvalues_mex(system, R, K, dimensions, eigenvaluederivativetype, numthreads);
		elseif nargout >= 2
			[eigenvalues, eigenvector_right] = calculate_eigenvalues_mex(system, R, K, dimensions, eigenvaluederivativetype, numthreads);
		else
			eigenvalues = calculate_eigenvalues_mex(system, R, K, dimensions, eigenvaluederivativetype, numthreads);
		end
	else
		if nargout >= 13
			[eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative, eigenvalue_derivative_xdot, eigenvector_right_derivative, eigenvector_right_derivative_xdot, eigenvector_left_derivative, eigenvector_left_derivative_xdot, eigenvalue_2derivative, eigenvalue_2derivative_xdot, eigenvalue_2derivative_mixed, eigenvalue_2derivative_xdot_mixed] = calculate_eigenvalues_m(system, R, K, dimensions, eigenvaluederivativetype, numthreads);
		elseif nargout >= 12
			[eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative, eigenvalue_derivative_xdot, eigenvector_right_derivative, eigenvector_right_derivative_xdot, eigenvector_left_derivative, eigenvector_left_derivative_xdot, eigenvalue_2derivative, eigenvalue_2derivative_xdot, eigenvalue_2derivative_mixed] = calculate_eigenvalues_m(system, R, K, dimensions, eigenvaluederivativetype, numthreads);
		elseif nargout >= 11
			[eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative, eigenvalue_derivative_xdot, eigenvector_right_derivative, eigenvector_right_derivative_xdot, eigenvector_left_derivative, eigenvector_left_derivative_xdot, eigenvalue_2derivative, eigenvalue_2derivative_xdot] = calculate_eigenvalues_m(system, R, K, dimensions, eigenvaluederivativetype, numthreads);
		elseif nargout >= 10
			[eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative, eigenvalue_derivative_xdot, eigenvector_right_derivative, eigenvector_right_derivative_xdot, eigenvector_left_derivative, eigenvector_left_derivative_xdot, eigenvalue_2derivative] = calculate_eigenvalues_m(system, R, K, dimensions, eigenvaluederivativetype, numthreads);
		elseif nargout >= 9
			[eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative, eigenvalue_derivative_xdot, eigenvector_right_derivative, eigenvector_right_derivative_xdot, eigenvector_left_derivative, eigenvector_left_derivative_xdot] = calculate_eigenvalues_m(system, R, K, dimensions, eigenvaluederivativetype, numthreads);
		elseif nargout >= 8
			[eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative, eigenvalue_derivative_xdot, eigenvector_right_derivative, eigenvector_right_derivative_xdot, eigenvector_left_derivative] = calculate_eigenvalues_m(system, R, K, dimensions, eigenvaluederivativetype, numthreads);
		elseif nargout >= 7
			[eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative, eigenvalue_derivative_xdot, eigenvector_right_derivative, eigenvector_right_derivative_xdot] = calculate_eigenvalues_m(system, R, K, dimensions, eigenvaluederivativetype, numthreads);
		elseif nargout >= 6
			[eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative, eigenvalue_derivative_xdot, eigenvector_right_derivative] = calculate_eigenvalues_m(system, R, K, dimensions, eigenvaluederivativetype, numthreads);
		elseif nargout >= 5
			[eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative, eigenvalue_derivative_xdot] = calculate_eigenvalues_m(system, R, K, dimensions, eigenvaluederivativetype, numthreads);
		elseif nargout >= 4
			[eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative] = calculate_eigenvalues_m(system, R, K, dimensions, eigenvaluederivativetype, numthreads);
		elseif nargout >= 3
			[eigenvalues, eigenvector_right, eigenvector_left] = calculate_eigenvalues_m(system, R, K, dimensions, eigenvaluederivativetype, numthreads);
		elseif nargout >= 2
			[eigenvalues, eigenvector_right] = calculate_eigenvalues_m(system, R, K, dimensions, eigenvaluederivativetype, numthreads);
		else
			eigenvalues = calculate_eigenvalues_m(system, R, K, dimensions, eigenvaluederivativetype, numthreads);
		end
	end
end
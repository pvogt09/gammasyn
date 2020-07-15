function [V, D, W, V_derv, D_derv, W_derv] = vanDerAa(A, B, options, V_tilde, lambda_tilde, W_tilde)
	%VANDERAA Calculates first order derivatives of eigenvalues and eigenvectors
	%	Input:
	%		A:				matrix to solve eigenvalue problem for
	%		B:				generalized eigenproblem matrix to solve eigenvalue problem for
	%		options:		structure with options for calculation
	%		V_tilde:		matrix of right eigenvectors
	%		lambda_tilde:	vector of eigenvalues
	%		W_tilde:		matrix of left eigenvectors (inv(V_tilde)')
	%	Output:
	%		V:				matrix of right eigenvectors
	%		D:				vector of eigenvalues
	%		W:				matrix of right eigenvectors (inv(V)')
	%		V_derv:			matrix of derivatives of right eigenvectors
	%		D_derv:			vector of derivatives of eigenvalues
	%		W_derv:			matrix of derivatives of left eigenvectors
	if nargin <= 1
		B = [];
	end
	if nargin <= 2
		options = struct(...
			'tolerance',			(1/100000000000),...
			'keepsorting',			false,...
			'multiplicityhandling',	GammaEigenvalueMultiplicityHandlingType.DEFAULT,...% TODO: should be .getDefaultValue() but does not pass code generation
			'problemtype',			struct(...
				'parameterlinear',	false,...
				'maxderivative',	10*size(A, 3)...
			)...
		);
	end
	isgeneral_ev_problem = ~isempty(B);
	if ~isstruct(options)
		error('control:design:gamma:eigenvalues', 'Options must be supplied as structure, not as ''%s''.', class(options));
	end
	if ~isfield(options, 'keepsorting')
		error('control:design:gamma:eigenvalues', 'Options must have field ''keepsorting''.');
	end
	if ~isfield(options, 'tolerance')
		error('control:design:gamma:eigenvalues', 'Options must have field ''tolerance''.');
	end
	if ~isfield(options, 'multiplicityhandling')
		error('control:design:gamma:eigenvalues', 'Options must have field ''multiplicityhandling''.');
	end
	if ~isscalar(options.keepsorting) || ~islogical(options.keepsorting)
		error('control:design:gamma:eigenvalues', 'Option ''keepsorting'' must be a logical scalar.');
	end
	if ~isscalar(options.tolerance) || ~isnumeric(options.tolerance) || options.tolerance < 0
		error('control:design:gamma:eigenvalues', 'Option ''tolerance'' must be a nonegative scalar.');
	end
	if ~isscalar(options.multiplicityhandling) || ~isa(options.multiplicityhandling, 'GammaEigenvalueMultiplicityHandlingType')
		error('control:design:gamma:eigenvalues', 'Option ''multiplicityhandling'' must be of type ''GammaEigenvalueMultiplicityHandlingType''.');
	end
	if options.keepsorting
		% TODO: remove when implemented
		error('control:design:gamma:eigenvalues', 'Keeping the eigenvalue sorting of the input is currently not implemented.');
	end
	% TODO: sort only for calculation
	% system order
	N = size(A, 1);
	if size(A, 2) ~= N
		error('control:design:gamma:eigenvalues', 'A must have %d columns.', N);
	end
	if isgeneral_ev_problem && size(B, 1) ~= N
		error('control:design:gamma:eigenvalues', 'B must have %d rows.', N);
	end
	if isgeneral_ev_problem && size(B, 2) ~= N
		error('control:design:gamma:eigenvalues', 'B must have %d columns.', N);
	end
	% system depth
	sysDepth = size(A, 3);
	if isgeneral_ev_problem
		if size(B, 3) ~= sysDepth
			error('control:design:gamma:eigenvalues', 'Number of derivatives must match for A and B.');
		end
		B_diff = B(:, :, 2:end);
		isgeneral_ev_problem = any(B_diff(:) ~= 0);
		% TODO: remove when implemented
		if isgeneral_ev_problem
			error('control:design:gamma:eigenvalues', 'Van der Aa method not yet implemented for generalized eigenvalue problem.');
		end
		B_eye = B(:, :, 1) - eye(N);
		if any(B_eye(:) ~= 0)
			error('control:design:gamma:eigenvalues', 'Van der Aa method not yet implemented for generalized eigenvalue problem.');
		end
	end
	if sysDepth < 2
		error('control:design:gamma:eigenvalues', 'Derivative of system matrix must be supplied.');
	end
	if nargin <= 3
		if isgeneral_ev_problem
			[V_tilde, lambda_tilde] = eig(A(:, :, 1), B(:, :, 1), 'vector');
		else
			[V_tilde, lambda_tilde] = eig(A(:, :, 1), 'vector');
		end
	end
	if nargin <= 5
		% van der Aa method internally uses different notation for left eigenvectors (i.e. W = inv(V_tilde))
		W_tilde = inv(V_tilde)';
	end
	% initialize
	lambda = NaN(1, 1, sysDepth, sysDepth) + 1i*NaN(1, 1, sysDepth, sysDepth);
	X = NaN(N, N) + 1i*NaN(N, N);
	Y = NaN(N, N) + 1i*NaN(N, N);
	% calculate eigenvalues and -vectors of matrix A
	[X, Lambda, Y, M, lambda(1, 1, 1, 1), permutation_matrix] = vanDerAa_calculateEigSelf(options, V_tilde, lambda_tilde, W_tilde');
%	%Y = inv(X);
	% repeated eigenvalues?
	if M > 1
		% initialize difflambdas
		diffLambdas = NaN(N, N, sysDepth, sysDepth) + 1i*NaN(N, N, sysDepth, sysDepth);
		diffLambdas_size = NaN(sysDepth, 1, sysDepth);
		% size of first diffLambda matrix
		diffLambdas_size(1, 1, 1) = N - M;
		% 0-th derivative of Lambda1
		diffLambdas(1:N - M, 1:N - M, 1) = Lambda(1:N - M, 1:N - M);
		% introduce first partitioning
		part = -ones(sysDepth, 1, sysDepth, 'int32');
		part(1, 1, 1) = N - M;
		part(2, 1, 1) = M;
		% calculate continuously differentiable eigenvector matrix
		[X, Y, lambda(:, :, :, 1), diffLambdas(:, :, :, 1), diffLambdas_size(:, :, 1), part(:, :, 1), permutation] = vanDerAa_calculateV(A, Lambda, X, Y, lambda(:, :, :, 1), diffLambdas(:, :, :, 1), diffLambdas_size(:, :, 1), part(:, :, 1), int32(1), options);
		permutation_matrix = permutation_matrix*blkdiag(eye(N - M), permutation);
		Q = int32(1);
		R = part(1);
		% Is Lambda1 a matrix?
		if N - M > 1
			% repeated eigenvalues in Lambda1
			[Q, lambda(1, 1, 1, 2)] = vanDerAa_eigenvalueMult(Lambda(1:R, 1:R));
		end
		depth = 1;
		% do repeated eigenvalues exist, whose continuously differentiable eigenvectors haven't been found?
		while Q > 1
			% increase depth
			depth = depth + 1;
			% partitioning of current Lambda1
			part(1, 1, depth) = R - Q;
			part(2, 1, depth) = Q;
			% introduce new diffLambda matrix
			diffLambdas_size(1, 1, depth) = R - Q;
			diffLambdas(1:R - Q, 1:R - Q, 1, depth) = Lambda(1:R - Q, 1:R - Q);
			% calculate continuously differentiable eigenvectors
			[X(:, 1:R), Y(1:R, :), lambda(:, :, :, depth), diffLambdas(:, :, :, depth), diffLambdas_size(:, :, depth), part(:, :, depth), permutation] = vanDerAa_calculateV(A, Lambda(1:R, 1:R), X(:, 1:R), Y(1:R, :), lambda(:, :, :, depth), diffLambdas(:, :, :, depth), diffLambdas_size(:, :, depth), part(:, :, depth), int32(1), options);
			permutation_matrix = permutation_matrix*blkdiag(eye(N - Q), permutation);
			% size of new Lambda1
			R = part(1, 1, depth);
			% if Lambda1 is not a matrix, it cannot contain repeated eigenvalues
			if R <= 1
				break;
			end
			% has new Lambda1 repeated eigenvalues?
			[Q, lambda(1, 1, 1, depth + 1)] = vanDerAa_eigenvalueMult(Lambda(1:R, 1:R));
		end
		% normalization of eigenvector matrices
		[Gamma, invGamma, M_norm] = vanDerAa_normalizeEigenvectorMatrix(X, Y);
 		X = X*Gamma;
		Y = invGamma*Y;
		% calculate C for the "lowest" depth
		C = vanDerAa_calculateC(A, X, Y, lambda(:, :, :, 1), diffLambdas(:, :, :, 1), diffLambdas_size(:, :, 1), part(:, :, 1), M_norm, options);
		% iterate through all depths
		for ii = 2:depth %#ok<FORPF> no parfor because of dependent iterations
			% size of C_11
			tmp = part(:, :, ii - 1);
			cSize = tmp(1);
			% values in the current depth
			part1 = part(:, :, ii);
			lambda1 = lambda(:, :, :, ii);
			diffLambdas1 = diffLambdas(:, :, :, ii);
			diffLambdas_size1 = diffLambdas_size(:, :, ii);
			% current eigenvector matrices
			X_curr = X(:, 1:cSize);
			Y_curr = Y(1:cSize, :);
			% calculate C_11
			C_11 = vanDerAa_calculateC(A, X_curr, Y_curr, lambda1, diffLambdas1, diffLambdas_size1, part1, M_norm, options);
			% assign C_11
			C(1:cSize, 1:cSize) = C_11;
		end
		V = X;
		if nargout >= 2
			D = diag(Lambda);
			if nargout >= 3
				W = Y';
				if nargout >= 4
					V_derv = X*C;
					if nargout >= 5
						% HINT: is a diagonal matrix mathematically but can have off diagonal entries due to numerical errors (does this have to be taken into account somehow?)
						D_derv = diag(Y*A(:, :, 2)*X + Lambda*C - C*Lambda);
					end
				end
			end
		end
	else
		[Gamma, invGamma, M_norm] = vanDerAa_normalizeEigenvectorMatrix(X, Y);
		C = zeros(N, N) + 0i;
		D_derv = zeros(N, 1) + 0i;
		for kk = 1:N
			% derivative of k-th eigenvalue
			D_derv(kk, 1) = Y(kk, :)*A(:, :, 2)*X(:, kk);
			for jj = 1:N
				if kk ~= jj
					% off-diagonal elements of C
					C(kk, jj) = Y(kk, :)*A(:, :, 2)*X(:, jj)*Gamma(jj, jj)/(Gamma(kk, kk)*(Lambda(jj, jj) - Lambda(kk, kk)));
				end
			end
		end
		% normalization
		X = X*Gamma;
		Y = invGamma*Y;
		% diagonal elements of C
		for kk = 1:N
			sum = 0 + 0i;
			m = M_norm(kk);
			for ii = 1:N
				if ii ~= kk
					sum = sum + X(m, ii)*C(ii, kk);
				end
			end
			C(kk, kk) = -sum;
		end
		if nargout >= 4
			V_derv = X*C;
		end
		V = X;
		if nargout >= 2
			D = diag(Lambda);
		end
		if nargout >= 3
			W = Y';
		end
	end
	%if options.keepsorting
	%	V = V*permutation_matrix;
	%	W = permutation_matrix'*W;
	%	D = D*permutation_matrix;
	%	V_derv = V_derv*permutation_matrix;
	%	D_derv = D_derv*permutation_matrix;
	%end
	if nargout >= 6
		W_derv = -W*V_derv'*W;
	end
end
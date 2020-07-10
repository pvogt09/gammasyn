function [X_norm, Y_norm, lambda_R, diffLambdas_R, diffLambdas_size_R, part_R, permutation] = vanDerAa_calculateV(A, ~, X, Y, lambda, diffLambdas, diffLambdas_size, part, k, options)
	%VANDERAA_CALCULATEV calculates the continuously differentiable eigenvectors
	%	Input:
	%		A:					system matrix
	%		Lambda:				eigenvector matrix
	%		X:					current right eigenvector matrix
	%		Y:					current left eigenvector matrix
	%		lambda:				derivatives of repeated eigenvalue which are the same
	%		diffLambdas:		derivatives of repeated eigenvalue wo differ
	%		diffLambdas_size:	size of the blocks in diffLambdas
	%		part:				partitioning
	%		k:					current iteration
	%		options:			options for eigenvalue calculation
	%	Output:
	%		X_norm:				normalized right eigenvector matrix
	%		Y_norm:				normalized left eigenvector matrix
	%		lambda_R:			derivatives of repeated eigenvalue which are the same
	%		diffLambdas_R:		derivatives of repeated eigenvalue wo differ
	%		diffLambdas_size_R:	size of the blocks in diffLambdas
	%		part_R:				partitioning
	M = part(k + 1);
	permutation = eye(M);
	while M > 1
		if options.problemtype.parameterlinear
			if size(A, 3) < k + 1
				if options.problemtype.maxderivative < k + 1
					error('control:design:gamma:eigenvalues', 'Van der Aa method needs %d derivatives, but only %d are supplied and maximum number of retries for parameter linear problems is reached.', k + 1, size(A, 3));
				else
					A_diff = zeros(size(A, 1), size(A, 1));
				end
			else
				A_diff = A(:, :, k + 1);
			end
		else
			if size(A, 3) < k + 1
				error('control:design:gamma:eigenvalues', 'Van der Aa method needs %d derivatives, but only %d are supplied.', k + 1, size(A, 3));
			end
			A_diff = A(:, :, k + 1);
		end
		% calculation of current L
		L = vanDerAa_calculateL(A, k, k - 1, k + 1, k + 1, lambda, part, X, Y, diffLambdas, diffLambdas_size, [M, M], options);
		% size of eigenvector matrices
		[~, S] = size(X);% TODO: size(X, 2)?
		% last M rows of Y
		Y_neu = Y(S - M + 1:S, :);
		% last M columns of X
		X_neu = X(:, S - M + 1:S); 
		% matrix for new eigenvalue problem
		A_neu = Y_neu*A_diff*X_neu + L;
		% initialize
		Gamma2 = zeros(M, M) + 0i;
		difflambda = zeros(M, M) + 0i;
		P = int32(0);
		% solve new eigenvalue problem
		[V_tilde, lambda_tilde] = eig(A_neu, 'vector');
		[Gamma2, diffLambda, ~, P, ~, permutation_eig] = vanDerAa_calculateEigSelf(options, V_tilde, lambda_tilde);
		permutation = permutation*blkdiag(eye(size(permutation, 1) - M), permutation_eig);
		%diffLambdas(:, :, k + 1) = NaN(N) + 1i*NaN(N);
		% store new diffLambda
		if P > 1
			diffLambdas(1:M - P, 1:M - P, k + 1) = diffLambda(1:M - P, 1:M - P);
			diffLambdas_size(k + 1) = M - P;
		else
			diffLambdas(1:length(diffLambda), 1:length(diffLambda), k + 1) = diffLambda;
			diffLambdas_size(k + 1) = length(diffLambda);
		end
		% update eigenvector matrices
		X(:, S - M + 1:S) = X_neu*Gamma2;
		Y(S - M + 1:S, :) = Gamma2\Y_neu;
		% do repeated eigenvalues still occur?
		if P > 1
			% new partitioning
			part(k + 1) = M - P;
			%part = [part, P];
			if size(part, 1) <= k + 2
				part(k + 2) = P;
			else
				error('control:design:gamma:eigenvalues', 'Matrix has too many multiple eigenvalues and multiple derivatives and might not be differentiable.');
			end
			lambda(:, :, k + 1) = diffLambda(M, M);
			% last P columns are not determined uniquely yet
			%[X, Y, lambda, diffLambdas, diffLambdas_size, part] = vanDerAa_calculateV(A, Lambda, X, Y, lambda, diffLambdas, diffLambdas_size, part, k + 1, options);
		end
		M = P;
		k = k + 1;
	end
	X_norm = X;
	Y_norm = Y;
	lambda_R = lambda;
	diffLambdas_R = diffLambdas;
	part_R = part;
	diffLambdas_size_R = diffLambdas_size;
end
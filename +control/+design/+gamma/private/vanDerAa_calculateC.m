function [C] = vanDerAa_calculateC(A, X, Y, lambda, diffLambdas, diffLambdas_size, part, M, options)
	%VANDERAA_CALCULATEC calculates the matrix C for van der Aa's method
	%	Input:
	%		A:					system matrix
	%		X:					current right eigenvector matrix
	%		Y:					current left eigenvector matrix
	%		lambda:				derivatives of repeated eigenvalue which are the same
	%		diffLambdas:		derivatives of repeated eigenvalue wo differ
	%		diffLambdas_size:	size of the blocks in diffLambdas
	%		part:				partitioning
	%		M:					indices m of eigenvectors
	%		options:			structure with settings
	%	Output:
	%		C:					C matrix

	% system order (of current system)
	N = size(X, 2);
	% initialize C
	C = NaN(N, N) + 0i*NaN(N, N);
	% nuber of partitionings
	partSize = int32(length(part(part ~= -1)));
	% pointers
	rowPtr = int32(1);
	colPtr = int32(1);
	% calculate off-diagonal blocks
	for m = 1:partSize
		rowSize = part(m);
		for n = 1:partSize
			% size of current matrix
			colSize = part(n);
			if m ~= n
				Y_m = vanDerAa_selectY(m, Y, part);
				X_n = vanDerAa_selectX(n, X, part);
				dim = [
					size(Y_m, 1), size(X_n, 2)
				];
				C_mn = zeros(rowSize, colSize) + 0i;
				if m < n
					L = vanDerAa_calculateL(A, m, m - 1, m, n, lambda, part, X, Y, diffLambdas, diffLambdas_size, dim, options);
					lambdaMSize = int32(diffLambdas_size(m));
					Lambda_m = diffLambdas(1:lambdaMSize, 1:lambdaMSize, m);
					sum1 = lambda(:, :, m)*eye(size(Lambda_m, 1)) - Lambda_m;
					if options.problemtype.parameterlinear
						if size(A, 3) < m + 1
							if options.problemtype.maxderivative < m + 1
								error('control:design:gamma:eigenvalues', 'Van der Aa method needs %d derivatives, but only %d are supplied and maximum number of retries for parameter linear problems is reached.', m + 1, size(A, 3));
							else
								A_diff = zeros(size(A, 1), size(A, 1));
							end
						else
							A_diff = A(:, :, m + 1);
						end
					else
						if size(A, 3) < m + 1
							error('control:design:gamma:eigenvalues', 'Van der Aa method needs %d derivatives, but only %d are supplied.', m + 1, size(A, 3));
						end
						A_diff = A(:, :, m + 1);
					end
					sum2 = Y_m*A_diff*X_n + L;
					C_mn = (sum1\sum2)/double(m);
				else
					L = vanDerAa_calculateL(A, n, m - 1, m, n, lambda, part, X, Y, diffLambdas, diffLambdas_size, dim, options);
					lambdaNSize = int32(diffLambdas_size(n));
					Lambda_n = diffLambdas(1:lambdaNSize, 1:lambdaNSize, n);
					sum1 = lambda(:, :, n)*eye(size(Lambda_n, 1)) - Lambda_n;
					if options.problemtype.parameterlinear
						if size(A, 3) < n + 1
							if options.problemtype.maxderivative < n + 1
								error('control:design:gamma:eigenvalues', 'Van der Aa method needs %d derivatives, but only %d are supplied and maximum number of retries for parameter linear problems is reached.', n + 1, size(A, 3));
							else
								A_diff = zeros(size(A, 1), size(A, 1));
							end
						else
							A_diff = A(:, :, n + 1);
						end
					else
						if size(A, 3) < n + 1
							error('control:design:gamma:eigenvalues', 'Van der Aa method needs %d derivatives, but only %d are supplied.', n + 1, size(A, 3));
						end
						A_diff = A(:, :, n + 1);
					end
					sum2 = Y_m*A_diff*X_n + L;
					C_mn = -(sum2/sum1)/double(n);
				end
				C(rowPtr:rowPtr + rowSize - 1, colPtr:colPtr + colSize - 1) = C_mn;
			end
			% update pointers
			if colPtr + colSize <= N
				colPtr = colPtr + colSize;
			else
				colPtr = int32(1);
				rowPtr = rowPtr + rowSize;
			end
		end
	end
	rowPtr = int32(1);
	colPtr = int32(1);
	% calculate off-diagonal elemnts of diagonal blocks
	for m = 1:partSize %#ok<FORPF> can not be converted to parfor because of variable indexing into C
		Y_m = vanDerAa_selectY(m, Y, part);
		X_m = vanDerAa_selectX(m, X, part);
		% dimension of quadratix matrix C_mm
		q = size(X_m, 2);
		dim = [
			q, q
		];
		if options.problemtype.parameterlinear
			if size(A, 3) < m + 1
				if options.problemtype.maxderivative < m + 1
					error('control:design:gamma:eigenvalues', 'Van der Aa method needs %d derivatives, but only %d are supplied and maximum number of retries for parameter linear problems is reached.', m + 1, size(A, 3));
				else
					A_diff = zeros(size(A, 1), size(A, 1));
				end
			else
				A_diff = A(:, :, m + 1);
			end
		else
			if size(A, 3) < m + 1
				error('control:design:gamma:eigenvalues', 'Van der Aa method needs %d derivatives, but only %d are supplied.', m + 1, size(A, 3));
			end
			A_diff = A(:, :, m + 1);
		end
		Q = (Y_m*A_diff*X_m + vanDerAa_calculateL(A, m, m - 1, m, m, lambda, part, X, Y, diffLambdas, diffLambdas_size, dim, options))/double(m);
		lambdaMSize = int32(diffLambdas_size(m));
		Lambda_m = diffLambdas(1:lambdaMSize, 1:lambdaMSize, m);
		B = Lambda_m;
		offDiagC = NaN(q, q) + 0i*NaN(q, q);
		for i = 1:q
			for j = 1:q
				if i ~= j
					diff = B(i, i) - B(j, j);
					% no 0-division
					if diff ~= 0
						offDiagC(i, j) = Q(i, j)/diff;
					end% TODO: else error?
				end
			end
		end
		% store matrix
		C(rowPtr:rowPtr + q - 1, colPtr:colPtr + q - 1) = offDiagC;
		% update pointers
		rowPtr = rowPtr + q;
		colPtr = colPtr + q;
	end
	% diagonal elements of C
	for k = 1:N
		m = M(k);
		sum = 0 + 0i;
		for j = 1:N
			if j ~= k
				sum = sum + X(m, j)*C(j, k);
			end
		end
		C(k, k) = -sum;
	end
end
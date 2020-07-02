function [L] = vanDerAa_calculateL(A, n, m, p, q, lambda, part, X, Y, diffLambdas, diffLambdas_size, dim, options)
	%VANDERAA_CALCULATEL calculates matrix L for van der Aa's method
	%	Input:
	%		A:					system matrix
	%		n:					input parameters of L
	%		m:					input parameters of L
	%		p:					input parameters of L
	%		q:					input parameters of L
	%		lambda:				derivatives of repeated eigenvalue which are the same
	%		part:				partitioning
	%		X:					current right eigenvector matrix
	%		Y:					current left eigenvector matrix
	%		diffLambdas:		derivatives of repeated eigenvalue wo differ
	%		diffLambdas_size:	size of the blocks in diffLambdas
	%		dim:				dimension of matrix L
	%		options:			structure with settings
	%	Output:
	%		L:					value of L function
	coder.inline('never');
	% initialize L
	L = zeros(dim) + 0i;% TODO: , dim?
	% nothing to calculate here
	if (m < 1) || (n < 2)
		return;
	else
		for l = 1:m
			for k = l:(n - 1)
				v = double(vanDerAa_calculateGamma(n, k, l))/double(l);
				% initialize first sum
				dimSum1 = [
					part(p), part(l)
				];
				sum1 = zeros(dimSum1) + 0i;
				L1 = zeros(dimSum1) + 0i;
				prod1 = zeros(dimSum1) + 0i;
				% first product
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
				sum1 = vanDerAa_selectY(p, Y, part)*A_diff*vanDerAa_selectX(l, X, part);
				L1 = vanDerAa_calculateL(A, k, l - 1, p, l, lambda, part, X, Y, diffLambdas, diffLambdas_size, size(sum1), options);
				prod1 = sum1 + L1;
				if p == l
					prod1 = prod1 - (lambda(:, :, k + 1))*eye(size(sum1));
				end
				% initialize second sum
				lambdaSize = int32(0);
				diffLambda = zeros(diffLambdas_size(l)) + 0i;
				% second product
				lambdaSize = int32(diffLambdas_size(l));
				diffLambda = diffLambdas(1:lambdaSize, 1:lambdaSize, l);
				prod2 = lambda(:, :, l)*eye(length(diffLambda)) - diffLambda;
				% initialize thrid product
				dimSum3 = [
					part(l), part(q)
				];
				sum3 = zeros(dimSum3) + 0i;
				L3 = zeros(dimSum3) + 0i;
				prod3 = zeros(dimSum3) + 0i;
				% third product
				if options.problemtype.parameterlinear
					if size(A, 3) < n - k + l
						if options.problemtype.maxderivative < n - k + l
							error('control:design:gamma:eigenvalues', 'Van der Aa method needs %d derivatives, but only %d are supplied and maximum number of retries for parameter linear problems is reached.', n - k + l, size(A, 3));
						else
							A_diff = zeros(size(A, 1), size(A, 1));
						end
					else
						A_diff = A(:, :, n - k + l);
					end
				else
					if size(A, 3) < n - k + l
						error('control:design:gamma:eigenvalues', 'Van der Aa method needs %d derivatives, but only %d are supplied.', n - k + l, size(A, 3));
					end
					A_diff = A(:, :, n - k + l);
				end
				sum3 = vanDerAa_selectY(l, Y, part)*A_diff*vanDerAa_selectX(q, X, part);
				L3 = vanDerAa_calculateL(A, n - k + l - 1, l, l, q, lambda, part, X, Y, diffLambdas, diffLambdas_size, size(sum3), options);
				prod3 = L3 + sum3;
				if l == q
					prod3 = prod3 - (lambda(:, :, n - k + l))*eye(size(sum3));
				end
				L = L + v*prod1/prod2*prod3;
			end
		end
	end
end
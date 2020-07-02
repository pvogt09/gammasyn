function [V, D, W, M, lambda_multiple, P] = vanDerAa_calculateEigSelf(options, V, d_exact, W)
	%VANDERAA_CALCULATEEIGSELF calculates eigenvalues and eigenvectors of matrix A
	%	Input:
	%		options:			options for calculation
	%		V:					matrix of right eigenvectors
	%		d_exact:			matrix of eigenvalues
	%		W:					matrix of right eigenvectors
	%	Output:
	%		V:					right eigenvector matrix
	%		D:					matrix of eigenvalues, D is constructed like this: D = [D1 ; lambda*I] with lambda being the eigenvalue with the highest multiplicity
	%		W:					left eigenvector matrix
	%		M:					highest multiplicity of all eigenvalues
	%		lambda_multiple:	eigenvalue with highest multiplicity
	%		P:					permutation matrix from d_exact to D
	if nargin <= 3
		W = inv(V);
	end
	tol = options.tolerance;
	rounding_value = 1/tol;
	% system order
	n = size(V, 1);
	% vector for multiplicity of eigenvalues
	mult = zeros(n, 1, 'int32');
	% vector to store the different eigenvalues 
	diffEigs = NaN(n, 1) + 1i*NaN(n, 1);
	% matrix to store locations of the different eigenvalues 
	diffEigsPosition = false(n, n);
	currDiff = 1;
	for index = 1:n %#ok<FORPF> no parfor because of dependent iterations
		% save current eigenvalue 
		current = d_exact(index, 1);
		% if eigenvalue at the current index has not been stored before
		if ~any(abs(diffEigs - current) < tol)
			% variable to count multiplicity
			sameeig = abs(d_exact - current) < tol;
			tmp = sum(int32(sameeig), 'native');
			diffEigs(currDiff) = mean(d_exact(sameeig, 1));
			diffEigsPosition(currDiff, :) = sameeig;
			mult(currDiff) = tmp;
			currDiff = currDiff + 1;
		end
	end
	%diffEigs = diffEigs(1:currDiff - 1);
	mult = mult(1:currDiff - 1);
	diffEigsPosition = diffEigsPosition(1:currDiff - 1, :);
	% M is highest multiplicity
	M = max(mult);
	% amount of different eigenvalues
	q = length(mult);
	% vector and matrix to store eigenvalue/eigenvalues at the right position
	eigValues = zeros(n, 1) + 0i;
	eigVectors_right = zeros(n, n) + 0i;
	eigVectors_left = zeros(n, n) + 0i;
	eigIndex = int32(1);
	sorting = eye(n);
	P = sorting;
	multiplicityhandling = options.multiplicityhandling;
	for ii = 1:q %#ok<FORPF> no parfor because of dependent iterations
		% eigenvalue with lowest multiplicity which has not been stored yet
		[currMult, currIndex] = min(mult);
		% columns of eigenvector matrix which belong to current eigenvalue
		sameColumns = diffEigsPosition(currIndex, :);
		% store eigenvalue with its multiplicity on current position
		curreig = d_exact(sameColumns, 1);
		if size(curreig, 1) ~= currMult || currMult <= 0
			error('control:design:gamma:eigenvalues', 'Programming error in the calculation of eigenvalue multiplicities.');
		end
		if size(eigValues, 1) < eigIndex + currMult - 1
			error('control:design:gamma:eigenvalues', 'Programming error in the calculation of eigenvalue multiplicities.');
		end
		if currMult > 1
			switch multiplicityhandling
				case GammaEigenvalueMultiplicityHandlingType.KEEP
					eigValues(eigIndex:eigIndex + currMult - 1, 1) = curreig;
				case GammaEigenvalueMultiplicityHandlingType.MIN
					eigValues(eigIndex:eigIndex + currMult - 1, 1) = min(curreig(:))*ones(currMult, 1);
				case GammaEigenvalueMultiplicityHandlingType.MEAN
					eigValues(eigIndex:eigIndex + currMult - 1, 1) = mean(curreig(:))*ones(currMult, 1);
				case GammaEigenvalueMultiplicityHandlingType.MAX
					eigValues(eigIndex:eigIndex + currMult - 1, 1) = max(curreig(:))*ones(currMult, 1);
				case GammaEigenvalueMultiplicityHandlingType.ROUND
					% HINT: using round(curreig./tol).*tol results in numerical difficulties while this works
					% TODO: take large values with overflow into account
					eigValues(eigIndex:eigIndex + currMult - 1, 1) = round(curreig.*rounding_value)./rounding_value;
				otherwise
					error('control:design:gamma:eigenvalues', 'Undefined eigenvalue multiplicity handling method.');
			end
		else
			if multiplicityhandling == GammaEigenvalueMultiplicityHandlingType.ROUND
				% HINT: using round(curreig./tol).*tol results in numerical difficulties while this works
				% TODO: take large values with overflow into account
				eigValues(eigIndex:eigIndex + currMult - 1, 1) = round(curreig.*rounding_value)./rounding_value;
			else
				eigValues(eigIndex:eigIndex + currMult - 1, 1) = curreig(1, 1);
			end
		end
		eigVectors_right(:, eigIndex:eigIndex + currMult - 1) = V(:, sameColumns);
		eigVectors_left(:, eigIndex:eigIndex + currMult - 1) = W(:, sameColumns);
		P(:, eigIndex:eigIndex + currMult - 1) = sorting(:, sameColumns);
		eigIndex = eigIndex + currMult;
		% delete the eigenvalue(s) that have been stored
		mult(currIndex, :) = [];
		%diffEigs(currIndex, :) = [];
		diffEigsPosition(currIndex, :) = [];
	end
	D = diag(eigValues);
	V = eigVectors_right;
	W = eigVectors_left;
	lambda_multiple = D(n, n);
end
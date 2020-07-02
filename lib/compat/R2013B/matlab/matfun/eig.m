function [V, D, W] = eig(A, B, algorithm, eigvalOption)
	%EIG    Eigenvalues and eigenvectors.
	%   E = EIG(A) produces a column vector E containing the eigenvalues of 
	%   a square matrix A.
	%
	%   [V,D] = EIG(A) produces a diagonal matrix D of eigenvalues and 
	%   a full matrix V whose columns are the corresponding eigenvectors  
	%   so that A*V = V*D.
	% 
	%   [V,D,W] = EIG(A) also produces a full matrix W whose columns are the
	%   corresponding left eigenvectors so that W'*A = D*W'.
	%
	%   [...] = EIG(A,'nobalance') performs the computation with balancing
	%   disabled, which sometimes gives more accurate results for certain
	%   problems with unusual scaling. If A is symmetric, EIG(A,'nobalance')
	%   is ignored since A is already balanced.
	%
	%   [...] = EIG(A,'balance') is the same as EIG(A).
	%
	%   E = EIG(A,B) produces a column vector E containing the generalized 
	%   eigenvalues of square matrices A and B.
	%
	%   [V,D] = EIG(A,B) produces a diagonal matrix D of generalized
	%   eigenvalues and a full matrix V whose columns are the corresponding
	%   eigenvectors so that A*V = B*V*D.
	%
	%   [V,D,W] = EIG(A,B) also produces a full matrix W whose columns are the
	%   corresponding left eigenvectors so that W'*A = D*W'*B.
	%
	%   [...] = EIG(A,B,'chol') is the same as EIG(A,B) for symmetric A and
	%   symmetric positive definite B.  It computes the generalized eigenvalues
	%   of A and B using the Cholesky factorization of B.
	%
	%   [...] = EIG(A,B,'qz') ignores the symmetry of A and B and uses the QZ
	%   algorithm. In general, the two algorithms return the same result,
	%   however using the QZ algorithm may be more stable for certain problems.
	%   The flag is ignored when A or B are not symmetric.
	%
	%   [...] = EIG(...,'vector') returns eigenvalues in a column vector 
	%   instead of a diagonal matrix.
	%
	%   [...] = EIG(...,'matrix') returns eigenvalues in a diagonal matrix
	%   instead of a column vector.
	%
	%   See also CONDEIG, EIGS, ORDEIG.

	%   Copyright 1984-2013 The MathWorks, Inc.
	%   Built-in function.
	if nargout >= 3
		error('matlab:eig', 'calculation of left eigenvectors is not supported.');
	end
	if nargin <= 3
		eigvalOption = 'matrix';
	end
	if nargin == 1
		if nargout == 1
			V = builtin('eig', A);
		elseif nargout == 2
			[V, D] = builtin('eig', A);
		else
			builtin('eig', A);
		end
	elseif nargin == 2 && (~ischar(B) || (~strcmpi(B, 'vector') && ~strcmpi(B, 'matrix')))
		if nargout == 1
			V = builtin('eig', A, B);
		elseif nargout == 2
			[V, D] = builtin('eig', A, B);
		else
			builtin('eig', A, B);
		end
	elseif nargin == 2 && ischar(B)
		asvector = strcmpi(B, 'vector');
		%asmatrix = strcmpi(B, 'matrix');
		if nargout == 1
			V = builtin('eig', A);
		elseif nargout == 2
			[V, D] = builtin('eig', A);
			if asvector
				D = diag(D);
			end
		else
			builtin('eig', A);
		end
	elseif nargin == 3
		if ischar(B) && (strcmpi(B, 'vector') || strcmpi(B, 'matrix'))
			if nargout == 1
				V = builtin('eig', A, algorithm);
			elseif nargout == 2
				[V, D] = builtin('eig', A, algorithm);
				if strcmpi(B, 'vector')
					D = diag(D);
				end
			else
				builtin('eig', A, algorithm);
			end
		else
			if ischar(algorithm) && (strcmpi(algorithm, 'vector') || strcmpi(algorithm, 'matrix'))
				if nargout == 1
					V = builtin('eig', A, B);
				elseif nargout == 2
					[V, D] = builtin('eig', A, B);
					if strcmpi(algorithm, 'vector')
						D = diag(D);
					end
				else
					builtin('eig', A, B);
				end
			else
				if nargout == 1
					V = builtin('eig', A, B, algorithm);
				elseif nargout == 2
					[V, D] = builtin('eig', A, B, algorithm);
					if strcmpi(algorithm, 'vector')
						D = diag(D);
					end
				else
					builtin('eig', A, B, algorithm);
				end
			end
		end
	elseif nargin == 4
		if ischar(B) && (strcmpi(B, 'vector') || strcmpi(B, 'matrix'))
			if nargout == 1
				V = builtin('eig', A, algorithm, eigvalOption);
			elseif nargout == 2
				[V, D] = builtin('eig', A, algorithm, eigvalOption);
				if strcmpi(B, 'vector')
					D = diag(D);
				end
			else
				builtin('eig', A, algorithm, eigvalOption);
			end
		else
			if ischar(algorithm) && (strcmpi(algorithm, 'matrix') || strcmpi(algorithm, 'matrix'))
				if nargout == 1
					V = builtin('eig', A, B, eigvalOption);
				elseif nargout == 2
					[V, D] = builtin('eig', A, B, eigvalOption);
					if strcmpi(algorithm, 'vector')
						D = diag(D);
					end
				else
					builtin('eig', A, B, eigvalOption);
				end
			else
				if ischar(eigvalOption) && (strcmpi(eigvalOption, 'vector') || strcmpi(eigvalOption, 'matrix'))
					if nargout == 1
						V = builtin('eig', A, B, algorithm);
					elseif nargout == 2
						[V, D] = builtin('eig', A, B, algorithm);
						if strcmpi(eigvalOption, 'vector')
							D = diag(D);
						end
					else
						builtin('eig', A, B, algorithm);
					end
				else
					if nargout == 1
						V = builtin('eig', A, B, algorithm);
					elseif nargout == 2
						[V, D] = builtin('eig', A, B, algorithm);
						if strcmpi(eigvalOption, 'vector')
							D = diag(D);
						end
					else
						builtin('eig', A, B, algorithm);
					end
				end
			end
		end
	else
		error('matlab:eig', 'wrong number of input arguments.');
	end
	if nargout >= 3
		W = inv(V);
	end
end
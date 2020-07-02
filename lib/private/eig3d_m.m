function [V, D] = eig3d_m(A, B, eigenvalueOption, numthreads)
	%EIG3D_M wrapper function for compilation of eig3d
	%	Input:
	%		A:					matrix to calculate eigenvalues of
	%		B:					generalized eigenproblem matrix
	%		eigenvalueOption:	return eigenvalues as matrix or vector
	%		numthreads:			number of threads to run loops in
	%	Output:
	%		V:					eigenvectors of eigenvalue problem element wise along the third dimension
	%		D:					eigenvalues of eigenvalue problem element wise along the third dimension
	isA3d = size(A, 3) > 1;
	isB3d = size(B, 3) > 1;
	asvector = strcmpi(eigenvalueOption, 'vector');
	if isA3d && isB3d
		if size(A, 3) ~= size(B, 3)
			error('MATLAB:innerdim', 'Inner matrix dimensions must agree.');
		end
		if ~isempty(B)
			if nargout >= 2
				V_3 = zeros(size(A, 1), size(A, 2), size(A, 3)) + 0i;
				if asvector
					D_vec = zeros(size(A, 1), size(A, 3)) + 0i;
					parfor (ii = 1:size(A, 3), numthreads)
						[V_3(:, :, ii), D_vec(:, ii)] = eig(A(:, :, ii), B(:, :, ii), 'vector');
					end
					D = D_vec;
				else
					D_mat = zeros(size(A, 1), size(A, 2), size(A, 3)) + 0i;
					parfor (ii = 1:size(A, 3), numthreads)
						[V_3(:, :, ii), D_mat(:, :, ii)] = eig(A(:, :, ii), B(:, :, ii), 'matrix');
					end
					D = D_mat;
				end
				V = V_3;
			else
				if asvector
					V_2 = zeros(size(A, 1), size(A, 3)) + 0i;
					parfor (ii = 1:size(A, 3), numthreads)
						V_2(:, ii) = eig(A(:, :, ii), B(:, :, ii), 'vector');
					end
					V = V_2;
				else
					V_3 = zeros(size(A, 1), size(A, 2), size(A, 3)) + 0i;
					parfor (ii = 1:size(A, 3), numthreads)
						V_3(:, :, ii) = eig(A(:, :, ii), B(:, :, ii), 'matrix');
					end
					V = V_3;
				end
			end
		else
			if nargout >= 2
				V_3 = zeros(size(A, 1), size(A, 2), size(A, 3)) + 0i;
				if asvector
					D_vec = zeros(size(A, 1), size(A, 3)) + 0i;
					parfor (ii = 1:size(A, 3), numthreads)
						[V_3(:, :, ii), D_vec(:, ii)] = eig(A(:, :, ii), 'vector');
					end
					D = D_vec;
				else
					D_mat = zeros(size(A, 1), size(A, 2), size(A, 3)) + 0i;
					parfor (ii = 1:size(A, 3), numthreads)
						[V_3(:, :, ii), D_mat(:, :, ii)] = eig(A(:, :, ii), 'matrix');
					end
					D = D_mat;
				end
				V = V_3;
			else
				if asvector
					V_2 = zeros(size(A, 1), size(A, 3)) + 0i;
					parfor (ii = 1:size(A, 3), numthreads)
						V_2(:, ii) = eig(A(:, :, ii), 'vector');
					end
					V = V_2;
				else
					V_3 = zeros(size(A, 1), size(A, 2), size(A, 3)) + 0i;
					parfor (ii = 1:size(A, 3), numthreads)
						V_3(:, :, ii) = eig(A(:, :, ii), 'matrix');
					end
					V = V_3;
				end
			end
		end
	elseif isA3d
		if ~isempty(B)
			if nargout >= 2
				V_3 = zeros(size(A, 1), size(A, 2), size(A, 3)) + 0i;
				if asvector
					D_vec = zeros(size(A, 1), size(A, 3)) + 0i;
					parfor (ii = 1:size(A, 3), numthreads)
						[V_3(:, :, ii), D_vec(:, ii)] = eig(A(:, :, ii), B, 'vector');
					end
					D = D_vec;
				else
					D_mat = zeros(size(A, 1), size(A, 2), size(A, 3)) + 0i;
					parfor (ii = 1:size(A, 3), numthreads)
						[V_3(:, :, ii), D_mat(:, :, ii)] = eig(A(:, :, ii), B, 'matrix');
					end
					D = D_mat;
				end
				V = V_3;
			else
				if asvector
					V_2 = zeros(size(A, 1), size(A, 3)) + 0i;
					parfor (ii = 1:size(A, 3), numthreads)
						V_2(:, ii) = eig(A(:, :, ii), B, 'vector');
					end
					V = V_2;
				else
					V_3 = zeros(size(A, 1), size(A, 2), size(A, 3)) + 0i;
					parfor (ii = 1:size(A, 3), numthreads)
						V_3(:, :, ii) = eig(A(:, :, ii), B, 'matrix');
					end
					V = V_3;
				end
			end
		else
			if nargout >= 2
				V_3 = zeros(size(A, 1), size(A, 2), size(A, 3)) + 0i;
				if asvector
					D_vec = zeros(size(A, 1), size(A, 3)) + 0i;
					parfor (ii = 1:size(A, 3), numthreads)
						[V_3(:, :, ii), D_vec(:, ii)] = eig(A(:, :, ii), 'vector');
					end
					D = D_vec;
				else
					D_mat = zeros(size(A, 1), size(A, 2), size(A, 3)) + 0i;
					parfor (ii = 1:size(A, 3), numthreads)
						[V_3(:, :, ii), D_mat(:, :, ii)] = eig(A(:, :, ii), 'matrix');
					end
					D = D_mat;
				end
				V = V_3;
			else
				if asvector
					V_2 = zeros(size(A, 1), size(A, 3)) + 0i;
					parfor (ii = 1:size(A, 3), numthreads)
						V_2(:, ii) = eig(A(:, :, ii), 'vector');
					end
					V = V_2;
				else
					V_3 = zeros(size(A, 1), size(A, 2), size(A, 3));
					parfor (ii = 1:size(A, 3), numthreads)
						V_3(:, :, ii) = eig(A(:, :, ii), 'matrix');
					end
					V = V_3;
				end
			end
		end
	elseif isB3d
		if ~isempty(B)
			temp = squeeze(A(:, :, 1));% codegen does not convert ?x?x? to ?x?, when last dimension is 1
			if nargout >= 2
				V_3 = zeros(size(A, 1), size(A, 2), size(B, 3)) + 0i;
				if asvector
					D_vec = zeros(size(A, 1), size(B, 3)) + 0i;
					parfor (ii = 1:size(B, 3), numthreads)
						[V_3(:, :, ii), D_vec(:, ii)] = eig(temp, B(:, :, ii), 'vector');
					end
					D = D_vec;
				else
					D_mat = zeros(size(A, 1), size(A, 2), size(B, 3)) + 0i;
					parfor (ii = 1:size(B, 3), numthreads)
						[V_3(:, :, ii), D_mat(:, :, ii)] = eig(temp, B(:, :, ii), 'matrix');
					end
					D = D_mat;
				end
				V = V_3;
			else
				if asvector
					V_2 = zeros(size(A, 1), size(B, 3)) + 0i;
					parfor (ii = 1:size(B, 3), numthreads)
						V_2(:, ii) = eig(temp, B(:, :, ii), 'vector');
					end
					V = V_2;
				else
					V_3 = zeros(size(A, 1), size(A, 2), size(B, 3)) + 0i;
					parfor (ii = 1:size(B, 3), numthreads)
						V_3(:, :, ii) = eig(temp, B(:, :, ii), 'matrix');
					end
					V = V_3;
				end
			end
		else
			if nargout >= 2
				if asvector
					[V, D] = eig(A, 'vector');
				else
					[V, D] = eig(A, 'matrix');
				end
				V = repmat(V, [1, 1, size(B, 3)]);
				D = repmat(D, [1, 1, size(B, 3)]);
			else
				if asvector
					V = eig(A, 'vector');
				else
					V = eig(A, 'matrix');
				end
				V = repmat(V, [1, 1, size(B, 3)]);
			end
		end
	else
		if ~isempty(B)
			if nargout >= 2
				if asvector
					[V, D] = eig(A, B, 'vector');
				else
					[V, D] = eig(A, B, 'matrix');
				end
			else
				if asvector
					V = eig(A, B, 'vector');
				else
					V = eig(A, B, 'matrix');
				end
			end
		else
			if nargout >= 2
				if asvector
					[V, D] = eig(A, 'vector');
				else
					[V, D] = eig(A, 'matrix');
				end
			else
				if asvector
					V = eig(A, 'vector');
				else
					V = eig(A, 'matrix');
				end
			end
		end
	end
end
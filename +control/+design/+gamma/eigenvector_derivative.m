function [V, D, W, V_derv, D_derv, W_derv] = eigenvector_derivative(A, B, options, V_tilde, lambda_tilde, W_tilde)
	%EIGENVECTOR_DERIVATIVE calculate derivative of eigenvalues and eigenvectors
	%	wrapper method for public access of van der Aa's method with convenient interface
	%	Input:
	%		A:				matrix to calculate eigenvalue derivatives for with derivatives of matrix for a parameter in third dimension
	%		B:				matrix for generalized eigenvalue problem to calculate eigenvalue derivatives for with derivatives of matrix for a parameter in third dimension
	%		options:		structure with options for calculation
	%		V_tilde:		matrix of right eigenvectors for the generalized eigenvalue problem lambda*B*V = A*V
	%		lambda_tilde:	vector of eigenvalues
	%		W_tilde:		matrix of left eigenvactors (W_tilde = inv(V_tilde)')
	%	Output:
	%		V:				continuously differentiable matrix of right eigenvectors
	%		D:				continuously differentiable matrix of eigenvalues
	%		W:				continuously differentiable matrix of left eigenvectors
	%		V_derv:			derivative of right eigenvectors
	%		D_derv:			derivative of eigenvalues
	%		W_derv:			derivative of left eigenvectors
	default_tolerance = 1/cos(1.5707963267);
	if nargin <= 2 || isempty(options) || (isstruct(options) && isempty(fieldnames(options)))
		options = struct(...
			'derivativetype',	GammaEigenvalueDerivativeType.VANDERAA,...
			'tolerance',		default_tolerance...
		);
	end
	if ~isstruct(options)
		error('control:design:gamma:eigenvalues', 'Options must be supplied as a structure.');
	end
	if ~isfield(options, 'derivativetype')
		error('control:design:gamma:eigenvalues', 'Options must have a field ''derivativetype''.');
	end
	if ~isfield(options, 'tolerance')
		error('control:design:gamma:eigenvalues', 'Options must have a field ''tolerance''.');
	end
	if ~isa(options.derivativetype, 'GammaEigenvalueDerivativeType')
		error('control:design:gamma:eigenvalues', 'Derivative type must be of type ''GammaEigenvalueDerivativeType''.');
	end
	if ~isscalar(options.derivativetype)
		error('control:design:gamma:eigenvalues', 'Derivative type must be scalar.');
	end
	if ~isscalar(options.tolerance) || ~isnumeric(options.tolerance) || isinf(options.tolerance)
		error('control:design:gamma:eigenvalues', 'Tolerance must be a finite scalar numerical value.');
	end
	if isnan(options.tolerance)
		options.tolerance = default_tolerance;
	end
	if nargin <= 3
		% copied from calculate_eigenvalues_m
		[V_tilde, lambda_tilde, W_tilde] = eig(A(:, :, 1), 'vector');
		system_order = size(A, 1);
		condeig_tolerance = options.tolerance;
		multiplicity = ones(size(lambda_tilde, 1), 1);
		replacemap = false(0, size(lambda_tilde, 1));
		rankupdate = 0;
		while rank(V_tilde) < size(V_tilde, 2) && rankupdate < 4
			% TODO: arbitrary tolerance should be replaced by a reasonable value
			[multiplicity, multiplicity_map] = eigenvalue_multiplicity(lambda_tilde, 10^rankupdate/condeig_tolerance);
			rankupdate = rankupdate + 1;
			idxtemp = (1:system_order);
			multiplicityidx = idxtemp(multiplicity > 1);
			if isempty(multiplicityidx)
				continue;
			end
			[replacemap, idxmap] = unique(multiplicity_map(multiplicityidx, :), 'rows');
			if isempty(idxmap)
				continue;
			end
			V_tilde_k = NaN(system_order, max(multiplicity), size(idxmap, 1)) + NaN*1i;
			for jj = 1:size(idxmap, 1)
				V_tilde_k(:, 1, jj) = V_tilde(:, multiplicityidx(1, idxmap(jj, 1)));
				k = 2;
				for gg = 2:multiplicity(multiplicityidx(1, idxmap(jj, 1)), 1)
					if isempty(B)
						temp = orth([V_tilde_k(:, 1:k - 1, jj), null((lambda_tilde(multiplicityidx(1, idxmap(jj, 1)))*eye(system_order) - A(:, :, 1))^k)]) + 0i;
					else
						temp = orth([V_tilde_k(:, 1:k - 1, jj), null((lambda_tilde(multiplicityidx(1, idxmap(jj, 1)))*B(:, :, 1) - A(:, :, 1))^k)]) + 0i;
					end
					if size(temp, 1) ~= system_order || size(temp, 2) ~= k
						error('control:design:gamma:eigenvalues', 'No regular basis of right eigenvectors could be found.');
					end
					V_tilde_k(:, 1:k, jj) = temp;
					k = k + 1;
				end
			end
			for ff = 1:size(idxmap, 1)
				if sum(replacemap(ff, :)) ~= multiplicity(multiplicityidx(1, idxmap(ff, 1)), 1)
					% if we get here, multiplicity and number of equal eigenvalues in map are not equal, which should not occur under all circumstances
					error('control:design:gamma:eigenvalues', 'Something went wrong in the eigenvector calculation, check the implementation.');
				end
				V_tilde(:, replacemap(ff, :)) = V_tilde_k(:, 1:multiplicity(multiplicityidx(1, idxmap(ff, 1)), 1), ff);
			end
		end
		if rank(V_tilde) < size(V_tilde, 2)
			error('control:design:gamma:eigenvalues', 'No regular basis of right eigenvectors could be found.');
		end
		W_tilde = inv(V_tilde)';
	end
	if options.derivativetype == GammaEigenvalueDerivativeType.VANDERAA
		[V, D, W, V_derv, D_derv, W_derv] = vanDerAa(A, B, struct(...
			'tolerance',			(1/100000000000),...
			'keepsorting',			false,...
			'multiplicityhandling',	GammaEigenvalueMultiplicityHandlingType.DEFAULT,...% TODO: should be .getDefaultValue() but does not pass code generation
			'problemtype',			struct(...
				'parameterlinear',	false,...
				'maxderivative',	10*size(A, 3)...
			)...
		), V_tilde, lambda_tilde, W_tilde);
		%[V, D, V_derv, D_derv] = vanDerAa(A);
		%W = inv(V)';
		%W_derv = -W*V_derv'*W;
	else
		error('control:design:gamma:eigenvalues', 'Not yet implemented.');
	end
end
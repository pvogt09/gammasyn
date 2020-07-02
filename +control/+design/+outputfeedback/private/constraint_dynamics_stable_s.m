function [c_R, gradc_R] = constraint_dynamics_stable_s(A)
	%CONSTRAINT_DYNAMICS_STABLE_S return inequality constraints and constraint gradients for stable system matrix in continuous time
	%	Input:
	%		A:			system matrix to calculate constraints at
	%	Output:
	%		c_R:		inequality constraints for stable A in s domain (with necessary and sufficient stability conditions for order <= 2 and necessary conditions for order > 2)
	%		gradc_R:	gradient of inequality constraints for stable A in s domain
	if size(A, 1) ~= size(A, 2)
		error('control:design:outputfeedback:input', 'System matrix must be square.');
	end
	switch size(A, 1)
		case 0
			c_R = zeros(0, 1);
			gradc_R = zeros(0, 0, 0);
		case 1
			c_R = A;
			gradc_R = cat(3, 1);
		case 2
			a_1 = -(A(1, 1) + A(2, 2));
			a_0 = A(1, 1)*A(2, 2) - A(1, 2)*A(2, 1);
			c_R = [
				-a_1;
				-a_0
			];
			gradc_R = cat(3, [
				1,	0;
				0,	1
			], [
				-A(2, 2),	A(2, 1);
				A(1, 2),	-A(1, 1)
			]);
		otherwise
			if all(isnan(A(:)))
				% NaN is not allowed in eig
				c_R = NaN(size(A, 1), 1);
				gradc_R = NaN([size(A), size(A, 1)]);
			else
				[V, lambda, W] = eig(A, 'vector');
				if rank(V) ~= size(V, 1)
					[multiplicity, multiplicity_map] = eigenvalue_multiplicity(lambda, 1/cos(1.5707963267));
					idxtemp = (1:size(V, 1));
					multiplicityidx = idxtemp(multiplicity > 1);
					if ~isempty(multiplicityidx)
						[replacemap, idxmap] = unique(multiplicity_map(multiplicityidx, :), 'rows');
						if ~isempty(idxmap)
							eigenvectors_right_k = NaN(size(V, 1), max(multiplicity), size(idxmap, 1)) + NaN*1i;
							for jj = 1:size(idxmap, 1)
								eigenvectors_right_k(:, 1, jj) = V(:, multiplicityidx(1, idxmap(jj, 1)));
								k = 2;
								for gg = 2:multiplicity(multiplicityidx(1, idxmap(jj, 1)), 1)
									temp = orth([eigenvectors_right_k(:, 1:k - 1, jj), null((V(multiplicityidx(1, idxmap(jj, 1)))*eye(size(V, 1)) - A)^k)]) + 0i;
									if size(temp, 1) ~= size(V, 1) || size(temp, 2) ~= k
										error('control:design:outputfeedback:eigenvalues', 'No regular basis of right eigenvectors could be found.');
									end
									eigenvectors_right_k(:, 1:k, jj) = temp;
									k = k + 1;
								end
							end
							for ff = 1:size(idxmap, 1)
								if sum(replacemap(ff, :)) ~= multiplicity(multiplicityidx(1, idxmap(ff, 1)), 1)
									% if we get here, multiplicity and number of equal eigenvalues in map are not equal, which should not occur under all circumstances
									error('control:design:outputfeedback:eigenvalues', 'Something went wrong in the eigenvector calculation, check the implementation.');
								end
								V(:, replacemap(ff, :)) = eigenvectors_right_k(:, 1:multiplicity(multiplicityidx(1, idxmap(ff, 1)), 1), multiplicityidx(1, idxmap(ff, 1)));
							end
							W = inv(V);
						end
					end
				end
				c_R = lambda;
				gradc_R = zeros([size(A), size(lambda, 1)]);
				for ii = 1:size(lambda, 1)
					wv = W(:, ii)'*V(:, ii);
					for jj = 1:size(A, 1)
						for kk = 1:size(A, 2)
							gradc_R(jj, kk, ii) = W(jj, ii)'*V(kk, ii)/wv;
						end
					end
				end
			end
	end
end
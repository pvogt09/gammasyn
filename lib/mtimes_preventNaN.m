function [result] = mtimes_preventNaN(A, B)
	%MTIMES_PREVENTNAN matrix multiply which can handle summation of Inf without NaN
	%	Input:
	%		A:		first matrix to multiply
	%		B:		second matrix to multiply
	%	Output:
	%		result:	result of matrix multiplication
	infA = isinf(A);
	infB = isinf(B);
	if any(infA(:)) || any(infB(:))
		result = zeros(size(A, 1), size(B, 2));
		parfor ii = 1:size(A, 1)
			r = result(ii, :);
			ai = A(ii, :);
			for jj = 1:size(B, 2)
				bj = B(:, jj).';
				ai_inf = isinf(ai);
				ai_posinf = ai_inf & ai > 0;
				ai_neginf = ai_inf & ai < 0;
				bj_inf = isinf(bj);
				bj_posinf = bj_inf & bj > 0;
				bj_neginf = bj_inf & bj < 0;
				infsquare = ai_inf & bj_inf;
				infsquare_pos = ai_posinf & bj_posinf | ai_neginf & bj_neginf;
				infsquare_neg = ai_posinf & bj_neginf | ai_neginf & bj_posinf;
				rij = ai.*bj;
				rij(ai_inf & bj == 0 | bj_inf & ai == 0) = 0;
				if sum(infsquare) > 1
					if any(infsquare_pos) && any(infsquare_neg)
						rij(infsquare) = Inf;
					end
				end
				rij_inf = isinf(rij);
				rij_posinf = rij_inf & rij > 0;
				rij_neginf = rij_inf & rij < 0;
				if any(rij_posinf(:)) && any(rij_neginf(:))
					oppositesign = xor(rij_posinf, rij_neginf);
					if any(infsquare(:))
						sgn = sign(rij(infsquare));
						sgn = sgn(1);
					else
						sgn = 1;
					end
					rij(oppositesign) = sgn*Inf;
				end
				r(1, jj) = sum(rij);
			end
			result(ii, :) = r;
		end
	else
		result = A*B;
	end
end
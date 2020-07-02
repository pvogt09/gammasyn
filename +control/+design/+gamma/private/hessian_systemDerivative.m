function [A_derv] = hessian_systemDerivative(B, C, x, y)
	%HESSIAN_SYSTEMDERIVATIVE calculates first derivative of A-B*R*C with respect to R_{xy}
	%	Input:
	%		B:		control matrix of current multiple model
	%		C:		output matric of current mutliple model
	%		x:		column index of current gain coefficient
	%		y:		row index of current gain coefficient
	%	Output:
	%		A_derv:	derivative of closed loop system matrix  with respect to R_{xy}
	if size(C, 1) < y
		error('control:design:gamma:eigenvalues:vanderaa', 'Wrong index for output matrix.');
	end
	if size(B, 2) < x
		error('control:design:gamma:eigenvalues:vanderaa', 'Wrong index for control matrix.');
	end
	n = size(B, 1);
	A_derv = zeros(n, n);
	Cy = C(y, :);
	Bx = B(:, x);
	for t = 1:n
		for s = 1:n
			A_derv(t, s) = -Cy(:, s)*Bx(t, :);
		end
	end
end
function [P] = calculate_objective_gain_lyap_wrapper_intrinsic(isdiscrete, A, Q, C, E)
	%CALCULATE_OBJECTIVE_GAIN_LYAP_WRAPPER_INTRINSIC lypunov matrix wrapper without extrinsic calls to lyap and dlyap
	%	Input:
	%		isdiscrete:	indicator if system is disrete
	%		A:			system matrix
	%		Q:			right hand side of lyapunov equation
	%		C:			measurement matrix for lyapunov equation
	%		E:			system descriptor matrix
	%	Output:
	%		P:			lyapunov matrix
	if isdiscrete
		if nargin <= 3
			P = dlyap(A, Q);
		elseif nargin <= 4
			P = dlyap(A, Q, C);
		else
			P = dlyap(A, Q, C, E);
		end
	else
		if nargin <= 3
			P = lyap(A, Q);
		elseif nargin <= 4
			P = lyap(A, Q, C);
		else
			P = lyap(A, Q, C, E);
		end
	end
end
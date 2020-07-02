function [U] = calculate_objective_gain_ordschur_wrapper_intrinsic(U, A_Schur, isdiscrete)
	%CALCULATE_OBJECTIVE_GAIN_ORSCHUR_WRAPPER_INTRINSIC Schur transformation matrix wrapper without extrinsic calls to ordschur
	%	Input:
	%		U:			Schur transformation matrix
	%		A_Schur:	right hand side of lyapunov equation
	%		isdiscrete:	indicator if system is disrete
	%	Output:
	%		U:			Schur transformation matrix for ordered eigenvalues
	if isdiscrete
		U = ordschur(U, A_Schur, 'udo');
	else
		U = ordschur(U, A_Schur, 'rhp');
	end
end
function [U] = calculate_objective_gain_ordschur_wrapper_extrinsic(U, A_Schur, isdiscrete)
	%CALCULATE_OBJECTIVE_GAIN_ORSCHUR_WRAPPER_EXTRINSIC Schur transformation matrix wrapper with extrinsic calls to ordschur
	%	Input:
	%		U:			Schur transformation matrix
	%		A_Schur:	right hand side of lyapunov equation
	%		isdiscrete:	indicator if system is disrete
	%	Output:
	%		U:			Schur transformation matrix for ordered eigenvalues
	coder.extrinsic('ordschur');
	if isdiscrete
		U = ordschur(U, A_Schur, 'udo');
	else
		U = ordschur(U, A_Schur, 'rhp');
	end
end
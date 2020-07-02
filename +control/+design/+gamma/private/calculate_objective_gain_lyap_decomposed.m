function [P_stable, P_unstable, P, U, A_unstable_inv, A_stabilized, A_Schur] = calculate_objective_gain_lyap_decomposed(isdiscrete, useextrinsic, tolerance, A, Q)
	%CALCULATE_OBJECTIVE_GAIN_LYAP_DECOMPOSED calculates lyapunov matrices of subsystems in Schur decomposed system, unstable eigenvalues are treated as stable
	%	Input:
	%		isdiscrete:					true if system is discrete
	%		useextrinsic:				indicator if extrinsic calls to lyap are needed
	%		tolerance:					tolerance for eigenvalues to be considered as stable
	%		A:							closed loop system matrix
	%		Q:							weighing matrix
	%	Output:
	%		P_stable:					lyapunov matrix of subsystem with stable eigenvalues
	%		P_unstable:					lyapunov matrix of subsystem with unstable eigenvalues
	%		P:							overall lyapunov matrix
	%		U:							transformation matrix for schur form
	%		A_unstable_inv:				inverse of unstable part of system matrix
	%		A_stabilized:				system matrix with only stable eigenvalues
	%		A_Schur:					system matrix in Schur complement form
	if any(any(isinf(A)))
		error('control:design:gamma:lyap', 'Inputs of schur must be finite.');
	end
	if issparse(A)
		A = full(A);
	end
	[U, A_Schur] = schur(A);
	if useextrinsic
		U = calculate_objective_gain_ordschur_wrapper_extrinsic(U, A_Schur, isdiscrete);
	else
		U = calculate_objective_gain_ordschur_wrapper_intrinsic(U, A_Schur, isdiscrete);
	end
	A_Schur = U'*A*U;
	Q_Schur = U'*Q*U;
	% weighing matrix must be symmetric for use with lyap function
	Q_Schur = (Q_Schur + Q_Schur')/2;
	eigenvalues = eig(A);
	if isdiscrete
		number_unstable_eigenvalues = sum(abs(eigenvalues) > 1 + tolerance);
	else
		number_unstable_eigenvalues = sum(real(eigenvalues) > tolerance);
	end
	% A_Schur = [A_unstable, A_12; A_12', A_stable]
	A_unstable = A_Schur(1:number_unstable_eigenvalues, 1:number_unstable_eigenvalues);
	A_12 = A_Schur(1:number_unstable_eigenvalues, number_unstable_eigenvalues + 1:end);
	A_stable = A_Schur(number_unstable_eigenvalues + 1:end, number_unstable_eigenvalues + 1:end);
	% Q_Schur = [Q_unstable, Q_12; Q_12', Q_stable]
	Q_unstable = Q_Schur(1:number_unstable_eigenvalues, 1:number_unstable_eigenvalues);
	Q_12 = Q_Schur(1:number_unstable_eigenvalues, number_unstable_eigenvalues + 1:end);
	Q_stable = Q_Schur(number_unstable_eigenvalues + 1:end, number_unstable_eigenvalues + 1:end);
	if isdiscrete
		A_unstable_inv = inv(A_unstable);
		if any(isinf(A_unstable_inv(:))) || rank(A_unstable) < size(A_unstable, 1)
			A_unstable_inv = pinv(A_unstable);
		end
		% P_unstable calculated with inverse of A_unstable
		if useextrinsic
			P_unstable = calculate_objective_gain_lyap_wrapper_extrinsic(isdiscrete, A_unstable_inv', Q_unstable);
			P_12 = calculate_objective_gain_lyap_wrapper_extrinsic(isdiscrete, A_unstable_inv', A_stable, Q_12 + A_unstable_inv'*P_unstable*A_12);
		else
			P_unstable = calculate_objective_gain_lyap_wrapper_intrinsic(isdiscrete, A_unstable_inv', Q_unstable);
			P_12 = calculate_objective_gain_lyap_wrapper_intrinsic(isdiscrete, A_unstable_inv', A_stable, Q_12 + A_unstable_inv'*P_unstable*A_12);
		end
		Q_P_stable = Q_stable + A_12'*P_unstable*A_12 + A_stable'*P_12'*A_12 + A_12'*P_12*A_stable;
		Q_P_stable = (Q_P_stable + Q_P_stable')/2;
	else
		A_unstable_inv = NaN(size(A_unstable, 1), size(A_unstable, 2));
		% P_unstable calculated with negative of A_unstable
		if useextrinsic
			P_unstable = calculate_objective_gain_lyap_wrapper_extrinsic(isdiscrete, -A_unstable', Q_unstable);
			P_12 = calculate_objective_gain_lyap_wrapper_extrinsic(isdiscrete, -A_unstable', A_stable, Q_12 + P_unstable*A_12);
		else
			P_unstable = calculate_objective_gain_lyap_wrapper_intrinsic(isdiscrete, -A_unstable', Q_unstable);
			P_12 = calculate_objective_gain_lyap_wrapper_intrinsic(isdiscrete, -A_unstable', A_stable, Q_12 + P_unstable*A_12);
		end
		Q_P_stable = Q_stable + P_12'*A_12 + A_12'*P_12;
		Q_P_stable = (Q_P_stable + Q_P_stable')/2;
	end
	if useextrinsic
		P_stable = calculate_objective_gain_lyap_wrapper_extrinsic(isdiscrete, A_stable', Q_P_stable);
	else
		P_stable = calculate_objective_gain_lyap_wrapper_intrinsic(isdiscrete, A_stable', Q_P_stable);
	end
	P = U*[
		P_unstable,	P_12;
		P_12',		P_stable
	]*U';
	if isdiscrete
		A_stabilized = U*[
			A_unstable_inv,																	A_12;
			zeros(size(A, 1) - number_unstable_eigenvalues, number_unstable_eigenvalues),	A_stable
		]*U';
	else
		A_stabilized = U*[
			-A_unstable,																	A_12;
			zeros(size(A, 1) - number_unstable_eigenvalues, number_unstable_eigenvalues),	A_stable
		]*U';
	end
end
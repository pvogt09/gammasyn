function [eigenvalues, eigenvector_right, eigenvector_left, eigenvalue_derivative, eigenvalue_derivative_xdot, eigenvector_right_derivative, eigenvector_right_derivative_xdot, eigenvector_left_derivative, eigenvector_left_derivative_xdot, eigenvalue_2derivative, eigenvalue_2derivative_xdot, eigenvalue_2derivative_mixed, eigenvalue_2derivative_xdot_mixed] = calculate_eigenvalues_m(system, R, K, dimensions, eigenvaluederivativetype, numthreads, eigenvaluefilter)
	%CALCULATE_EIGENVALUES helper function for calculation of eigenvalues for all systems
	%	Input:
	%		system:								structure with systems to calculate eigenvalues for
	%		R:									current gain matrix for proportional feedback
	%		K:									current gain matrix for derivative feedback
	%		dimensions:							structure with information about dimensions of the different variables and systems
	%		eigenvaluederivativetype:			GammaEigenvalueDerivativeType to indicate which method for eigenvalue derivative calculation should be used
	%		numthreads:							number of threads to run loops in
	%		eigenvaluefilter:					filter for calculated eigenvalues
	%	Output:
	%		eigenvalues:						eigenvalues of all systems (NaN for systems with fewer states than maximum number of states)
	%		eigenvector_right:					right eigenvectors of all systems (NaN for systems with fewer states than maximum number of states)
	%		eigenvector_left:					left eigenvectors of all systems (NaN for systems with fewer states than maximum number of states)
	%		eigenvalue_derivative:				derivative of eigenvalues with respect to proportional gain matrix for calculation of objective function gradient
	%		eigenvalue_derivative_xdot:			derivative of eigenvalues with respect to derivative gain matrix for calculation of objective function gradient
	%		eigenvector_right_derivative:		derivative of right eigenvectors with respect to proportional gain matrix for calculation of objective function gradient
	%		eigenvector_right_derivative_xdot:	derivative of right eigenvectors with respect to derivative gain matrix for calculation of objective function gradient
	%		eigenvector_left_derivative:		derivative of left eigenvectors with respect to proportional gain matrix for calculation of objective function gradient
	%		eigenvector_left_derivative_xdot:	derivative of left eigenvectors with respect to derivative gain matrix for calculation of objective function gradient
	%		eigenvalue_2derivative:				second derivative of eigenvalues with respect to proportional gain matrix for calculation of hessian matrix
	%		eigenvalue_2derivative_xdot:		second derivative of eigenvalues with respect to derivative gain matrix for calculation of hessian matrix
	%		eigenvalue_2derivative_mixed:		second derivative of eigenvalues with respect to proportional and derivative gain matrix for calculation of hessian matrix
	%		eigenvalue_2derivative_xdot_mixed:	second derivative of eigenvalues with respect to derivative and proportional gain matrix for calculation of hessian matrix
	codegen_supports_recursion = coder.const(~feval('verLessThan', 'matlab', '9.1'));
	codegen_is_generating = coder.const(~coder.target('MATLAB'));
	numthreads = uint32(floor(max([0, numthreads])));
	number_models = dimensions.models;
	number_states = dimensions.states;
	number_controls = dimensions.controls;
	number_measurements = dimensions.measurements;
	number_measurements_xdot = dimensions.measurements_xdot;
	R_fixed = dimensions.R_fixed & ~dimensions.RKF_fixed_has;% checking RKF_fixed_has should not be neccessary, just to make sure
	K_fixed = dimensions.K_fixed & ~dimensions.RKF_fixed_has;% checking RKF_fixed_has should not be neccessary, just to make sure
	derivative_feedback = number_measurements_xdot > 0;
	if dimensions.K_fixed_only
		% if all K are fixed to 0, solving a generalized eigenvalue problem with E = I (in case of no descriptor matrix) is numerically worse than solving the same normal eigenvalue problem
		% all other occurences of the variable "derivative_feedback" in other functions are still the same, because no eigenvalue problem is solved there
		if all(dimensions.K_fixed(:)) && all(dimensions.K_fixed_values(:) == 0)
			derivative_feedback = false;
		end
	end
	if dimensions.K_isforced2zero
		derivative_feedback = false;
	end
	descriptor = dimensions.descriptor;
	eigenvalues = zeros(number_states, number_models) + 0i;
	needseigenvectorsright = nargout >= 2;
	needseigenvectorsleft = nargout >= 3;
	needseigenvectors = needseigenvectorsleft || needseigenvectorsright;
	needsgradient = nargout >= 4;
	needsgradient_xdot = nargout >= 5;
	needsgradienteigenvectorsright = nargout >= 6;
	needsgradienteigenvectorsright_xdot = (nargout >= 7) && needsgradient_xdot;
	needsgradienteigenvectorsleft = nargout >= 8;
	needsgradienteigenvectorsleft_xdot = (nargout >= 9) && needsgradient_xdot;
	needshessian = nargout >= 10;
	needsgradienteigenvectors = needsgradienteigenvectorsright || needsgradienteigenvectorsright_xdot || needsgradienteigenvectorsleft || needsgradienteigenvectorsleft_xdot;
	condeig_tolerance = 1/cos(1.5707963267);
	eigenvalue_options = struct(...
		'tolerance',			1/condeig_tolerance,...
		'keepsorting',			false,...
		'multiplicityhandling',	GammaEigenvalueMultiplicityHandlingType.DEFAULT,...% TODO: should be .getDefaultValue() but does not pass code generation
		'problemtype',			struct(...
			'parameterlinear',	true,...
			'maxderivative',	10*number_states...
		)...
	);
	eigenvector_right = zeros(number_states, number_states, number_models) + 0i;
	eigenvector_left = zeros(number_states, number_states, number_models) + 0i;
	if needsgradient
		eigenvalue_derivative = zeros(number_states, number_controls, number_measurements, number_models) + 0i;
		if needsgradient_xdot
			eigenvalue_derivative_xdot = zeros(number_states, number_controls, number_measurements_xdot, number_models) + 0i;
		end
		if needsgradienteigenvectorsright
			eigenvector_right_derivative = zeros(number_states, number_states, number_controls, number_measurements, number_models) + 0i;
			if needsgradienteigenvectorsright_xdot
				eigenvector_right_derivative_xdot = zeros(number_states, number_states, number_controls, number_measurements_xdot, number_models) + 0i;
			end
		end
		if needsgradienteigenvectorsleft
			eigenvector_left_derivative = zeros(number_states, number_states, number_controls, number_measurements, number_models) + 0i;
			if needsgradienteigenvectorsleft_xdot
				eigenvector_left_derivative_xdot = zeros(number_states, number_states, number_controls, number_measurements_xdot, number_models) + 0i;
			end
		end
		if needshessian
			eigenvalue_2derivative = zeros(number_states, number_controls*number_measurements, number_controls*number_measurements, number_models) + 0i;
			if needsgradient_xdot
				eigenvalue_2derivative_xdot = zeros(number_states, number_controls*number_measurements_xdot, number_controls*number_measurements_xdot, number_models) + 0i;
				eigenvalue_2derivative_mixed = zeros(number_states, number_controls*number_measurements_xdot, number_controls*number_measurements, number_models) + 0i;
				eigenvalue_2derivative_xdot_mixed = zeros(number_states, number_controls*number_measurements, number_controls*number_measurements_xdot, number_models) + 0i;
			end
		end
	end
	if needshessian
		if needsgradienteigenvectorsleft
			eigenvector_left_derivative = NaN(number_states, number_states, number_controls, number_measurements, number_models) + 0i;
			if needsgradienteigenvectorsleft_xdot
				eigenvector_left_derivative_xdot = NaN(number_states, number_states, number_controls, number_measurements_xdot, number_models) + 0i;
			end
		end
	end
	if needshessian
		if eigenvaluederivativetype == GammaEigenvalueDerivativeType.RUDISILLCHU
			error('control:design:gamma:eigenvalues', 'Hessian of eigenvalues and eigenvectors can not be calculated with Rudisill and Chu method.');
		end
	end
	allRNaN = all(isnan(R(:)));% needed for structural information calls from IPOPT, SNOPT and possibly others to avoid calling eig with NaN
	if derivative_feedback || descriptor
		% K can be constrained to fixed values, so "or" is used here
		allRNaN = allRNaN || (~isempty(K) && all(isnan(K(:))));
	end
	parfor (ii = 1:number_models, numthreads)
		A = system(ii).A - system(ii).B*R*system(ii).C;
		system_order = size(A, 1);
		if needshessian% only distinct eigenvalues
			eigenvalue_derivative_xdot_temp = zeros(system_order, number_controls, number_measurements_xdot) + 0i;
			eigenvalue_derivative_temp = zeros(system_order, number_controls, number_measurements) + 0i;
			eigenvector_right_derivative_temp = zeros(system_order, system_order, number_controls, number_measurements) + 0i;
			eigenvector_right_derivative_xdot_temp = zeros(system_order, system_order, number_controls, number_measurements_xdot) + 0i;
			eigenvector_left_derivative_temp = zeros(system_order, system_order, number_controls, number_measurements) + 0i;
			eigenvector_left_derivative_xdot_temp = zeros(system_order, system_order, number_controls, number_measurements_xdot) + 0i;
			% TODO: for codegen, left eigenvectormatrix can not be calculated by eig and therefore W = V^-H
			%[eigenvectors_right, eigenvalues(:, ii), eigenvectors_left] = eig(system(ii).A - system(ii).B*K*system(ii).C, 'vector');
			if descriptor
				if derivative_feedback
					E = system(ii).E - system(ii).B*K*system(ii).C_dot;
				else
					E = system(ii).E;
				end
			else
				if derivative_feedback
					E = eye(system_order) - system(ii).B*K*system(ii).C_dot;
				else
					E = eye(system_order);
				end
			end
			if allRNaN
				V_tilde = ones(system_order) + eye(system_order) + 0i;
				ev = double(1:system_order)' + 0i;
				eigenvalues(:, ii) = [
					ev;
					(1 + 1i)*NaN(number_states - system_order, 1)
				];
			else
				if derivative_feedback || descriptor
					[V_tilde, ev] = eig(A, E, 'vector');
					% generalized eigenvalue problem can result in NaN eigenvalues
					ev(isnan(ev)) = Inf;
				else
					[V_tilde, ev] = eig(A, 'vector');
				end
				ev = calculate_eigenvalue_filter_immediate(eigenvaluefilter, ev);
				eigenvalues(:, ii) = [
					ev;
					(1 + 1i)*NaN(number_states - system_order, 1)
				];
			end
			%ev = eigenvalues(:, ii);
			multiplicity = ones(size(ev, 1), 1, 'uint32');
			replacemap = false(0, size(ev, 1));
			rankupdate = 0;
			while rank(V_tilde) < size(V_tilde, 2) && rankupdate < 4
				% TODO: arbitrary tolerance should be replaced by a reasonable value
				[multiplicity, multiplicity_map] = eigenvalue_multiplicity(ev, 10^rankupdate/condeig_tolerance);
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
						if descriptor || derivative_feedback
							if isinf(ev(multiplicityidx(1, idxmap(jj, 1)))) || isnan(isinf(ev(multiplicityidx(1, idxmap(jj, 1)))))
								evsysA = E;
								evsysb = A*V_tilde_k(:, gg - 1, jj);
							else
								evsysA = (ev(multiplicityidx(1, idxmap(jj, 1)))*E - A);
								evsysb = -E*V_tilde_k(:, gg - 1, jj);
							end
							if rank(evsysA) < size(evsysA, 1)
								sol_particular = pinv(evsysA)*evsysb;
							else
								sol_particular = evsysA\evsysb;
							end
							sol = null(evsysA);
							sol_particular(any(sol, 2), :) = 0;
							sol = sol + repmat(sol_particular, 1, size(sol, 2));
							temp = orth([V_tilde_k(:, 1:k - 1, jj), sol]) + 0i;
							if size(temp, 2) > k
								temp = temp(:, 1:k);
							end
						else
							temp = orth([V_tilde_k(:, 1:k - 1, jj), null((ev(multiplicityidx(1, idxmap(jj, 1)))*E - A)^k)]) + 0i;
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
					V_tilde(:, replacemap(ff, :)) = V_tilde_k(:, 1:multiplicity(multiplicityidx(1, idxmap(ff, 1)), 1), multiplicityidx(1, idxmap(ff, 1)));
				end
			end
			if rank(V_tilde) < size(V_tilde, 2)
				error('control:design:gamma:eigenvalues', 'No regular basis of right eigenvectors could be found.');
			end
			if system_order ~= size(V_tilde, 1) || system_order ~= size(V_tilde, 2)
				% if we get here, eigenvectors were removed from the eigenvector matrix, which should not occur under all circumstances
				error('control:design:gamma:eigenvalues', 'Something went wrong in the eigenvector calculation, check the implementation.');
			end
			eigenvectors_right = V_tilde;% initialize eigenvectors_right
			W = V_tilde;% initialize W_H
			% calculate gradient
			for jj = 1:number_controls
				A_derv = zeros(system_order, system_order, 2);
				A_derv(:, :, 1) = A;
				B_derv = zeros(system_order, system_order, 2);
				B_derv(:, :, 1) = E;
				for kk = 1:number_measurements
					if allRNaN
						eigenvectors_right = ones(system_order) + eye(system_order) + 0i;
						W = ones(system_order) + eye(system_order) + 0i;
						eigenvector_right_derivative_temp(:, :, jj, kk) = ones(system_order) + eye(system_order) + 0i;
						D_derv = double(1:system_order)' + 0i;
					else
						A_derv(:, :, 2) = hessian_systemDerivative(system(ii).B, system(ii).C, jj, kk);
						if needsgradienteigenvectorsleft
							[eigenvector_right_derivative_temp(:, :, jj, kk), D_derv, eigenvector_left_derivative_temp(:, :, jj, kk), eigenvectors_right, W] = hessian_firstDerivative(A_derv, B_derv, V_tilde, ev);
						else
							[eigenvector_right_derivative_temp(:, :, jj, kk), D_derv, ~, eigenvectors_right, W] = hessian_firstDerivative(A_derv, B_derv, V_tilde, ev);
						end
					end
					% TODO: check if usage of last W_H and eigenvectors_right in subsequent calculations is correct or can lead to errors
					if needseigenvectorsright
						eigenvector_right(:, :, ii) = [
							eigenvectors_right,	(1 + 1i)*NaN(system_order, number_states - system_order);
							(1 + 1i)*NaN(number_states - system_order, number_states)
						];
					end
					if needseigenvectorsleft
						eigenvector_left(:, :, ii) = [
							W,	(1 + 1i)*NaN(system_order, number_states - system_order);
							(1 + 1i)*NaN(number_states - system_order, number_states)
						];
					end
					eigenvalue_derivative_temp(:, jj, kk) = D_derv;
				end
				if needsgradient_xdot && number_measurements_xdot > 0
					A_derv = zeros(system_order, system_order, 2);
					A_derv(:, :, 1) = A;
					B_derv = zeros(system_order, system_order, 2);
					B_derv(:, :, 1) = E;
					for kk = 1:number_measurements_xdot
						if allRNaN
							eigenvector_right_derivative_xdot_temp(:, :, jj, kk) = ones(system_order) + eye(system_order) + 0i;
							D_derv = double(1:number_states)' + 0i;
						else
							B_derv(:, :, 2) = hessian_systemDerivative(system(ii).B, system(ii).C_dot, jj, kk);
							if needsgradienteigenvectorsleft_xdot
								[eigenvector_right_derivative_xdot_temp(:, :, jj, kk), D_derv, eigenvector_left_derivative_xdot_temp(:, :, jj, kk)] = hessian_firstDerivative(A_derv, B_derv, V_tilde, ev);
							else
								[eigenvector_right_derivative_xdot_temp(:, :, jj, kk), D_derv] = hessian_firstDerivative(A_derv, B_derv, V_tilde, ev);
							end
						end
						eigenvalue_derivative_xdot_temp(:, jj, kk) = D_derv;
					end
				end
			end
			if needsgradient
				if number_states - system_order > 0
					eigenvalue_derivative(:, :, :, ii) = cat(1, eigenvalue_derivative_temp, (1 + 1i)*NaN(number_states - system_order, number_controls, number_measurements));
				else
					eigenvalue_derivative(:, :, :, ii) = eigenvalue_derivative_temp;
				end
				if needsgradienteigenvectors
					if needsgradienteigenvectorsright
						if number_states - system_order > 0
							eigenvector_right_derivative_temp_cat = NaN(number_states, number_states, number_controls, number_measurements) + NaN*1i;
							eigenvector_right_derivative_temp_cat(1:system_order, 1:system_order, :, :, :, :) = eigenvector_right_derivative_temp;
							eigenvector_right_derivative(:, :, :, :, ii) = eigenvector_right_derivative_temp_cat;
						else
							eigenvector_right_derivative(:, :, :, :, ii) = eigenvector_right_derivative_temp;
						end
					end
					if needsgradienteigenvectorsleft
						if number_states - system_order > 0
							eigenvector_left_derivative_temp_cat = NaN(number_states, number_states, number_controls, number_measurements) + NaN*1i;
							eigenvector_left_derivative_temp_cat(1:system_order, 1:system_order, :, :, :, :) = eigenvector_left_derivative_temp;
							eigenvector_left_derivative(:, :, :, :, ii) = eigenvector_left_derivative_temp_cat;
						else
							eigenvector_left_derivative(:, :, :, :, ii) = eigenvector_left_derivative_temp;
						end
					end
				end
			end
			if needsgradient_xdot && number_measurements_xdot > 0
				% cat can not concatenate empty matrices in compiled code, so number of derivative measurements has to be greater than 0 to do anything
				if number_states - system_order > 0
					eigenvalue_derivative_xdot(:, :, :, ii) = cat(1, eigenvalue_derivative_xdot_temp, (1 + 1i)*NaN(number_states - system_order, number_controls, number_measurements_xdot));
				else
					eigenvalue_derivative_xdot(:, :, :, ii) = eigenvalue_derivative_xdot_temp;
				end
				if needsgradienteigenvectors
					if needsgradienteigenvectorsright_xdot
						if number_states - system_order > 0
							eigenvector_right_derivative_xdot_temp_cat = NaN(number_states, number_states, number_controls, number_measurements_xdot) + NaN*1i;
							eigenvector_right_derivative_xdot_temp_cat(1:system_order, 1:system_order, :, :, :, :) = eigenvector_right_derivative_xdot_temp;
							eigenvector_right_derivative_xdot(:, :, :, :, ii) = eigenvector_right_derivative_xdot_temp_cat;
						else
							eigenvector_right_derivative_xdot(:, :, :, :, ii) = eigenvector_right_derivative_xdot_temp;
						end
					end
					if needsgradienteigenvectorsleft_xdot
						if number_states - system_order > 0
							eigenvector_left_derivative_xdot_temp_cat = NaN(number_states, number_states, number_controls, number_measurements_xdot) + NaN*1i;
							eigenvector_left_derivative_xdot_temp_cat(1:system_order, 1:system_order, :, :, :, :) = eigenvector_left_derivative_xdot_temp;
							eigenvector_left_derivative_xdot(:, :, :, :, ii) = eigenvector_left_derivative_xdot_temp_cat;
						else
							eigenvector_left_derivative_xdot(:, :, :, :, ii) = eigenvector_left_derivative_xdot_temp;
						end
					end
				end
			end
			% calculate hessian
			eigenvalue_2derivative_temp = zeros(system_order, number_controls*number_measurements, number_controls*number_measurements) + 0i;
			eigenvalue_2derivative_xdot_temp = zeros(system_order, number_controls*number_measurements_xdot, number_controls*number_measurements_xdot) + 0i;
			eigenvalue_2derivative_mixed_temp = zeros(system_order, number_controls*number_measurements_xdot, number_controls*number_measurements) + 0i;
			eigenvalue_2derivative_xdot_mixed_temp = zeros(system_order, number_controls*number_measurements, number_controls*number_measurements_xdot) + 0i;
			% calculate d^2lambda/dR^2
			N = number_measurements*number_controls;
			A_derv = zeros(system_order, system_order, 2);
			B_derv = zeros(system_order, system_order, 3);
			B_derv(:, :, 1) = E;
			Lambda_derv = zeros(system_order, system_order, 3) + 0i;
			Lambda_derv(:, :, 1) = diag(ev);
			V_derv = zeros(system_order, system_order, 3) + 0i;
			V_derv(:, :, 1) = eigenvectors_right;
			W_tilde = W;
			for ll = 1:N
				for zz = 1:N
					qq = mod(ll - 1, number_controls) + 1;
					rr = idivide(ll - 1, number_controls) + 1;
					ss = mod(zz - 1, number_controls) + 1;
					tt = idivide(zz - 1, number_controls) + 1;
					A_derv(:, :, 1) = hessian_systemDerivative(system(ii).B, system(ii).C, qq, rr);
					A_derv(:, :, 2) = hessian_systemDerivative(system(ii).B, system(ii).C, ss, tt);
					Lambda_derv(:, :, 2) = diag(eigenvalue_derivative_temp(:, qq, rr));
					Lambda_derv(:, :, 3) = diag(eigenvalue_derivative_temp(:, ss, tt));
					V_derv(:, :, 2) = eigenvector_right_derivative_temp(:, :, qq, rr);
					V_derv(:, :, 3) = eigenvector_right_derivative_temp(:, :, ss, tt);
					eigenvalue_2derivative_temp(:, zz, ll) = hessian_secondDerivativeDifferentParams(A_derv, B_derv, Lambda_derv, V_derv, W_tilde);
				end
			end
			if number_states - system_order > 0
				eigenvalue_2derivative(:, :, :, ii) = cat(1, eigenvalue_2derivative_temp, (1 + 1i)*NaN(number_states - system_order, number_controls*number_measurements, number_controls*number_measurements));
			else
				eigenvalue_2derivative(:, :, :, ii) = eigenvalue_2derivative_temp;
			end
			if needsgradient_xdot && number_measurements_xdot > 0
				% calculate d^2lambda/dRdK
				M = number_measurements_xdot*number_controls;
				A_derv = zeros(system_order, system_order, 2);
				B_derv = zeros(system_order, system_order, 3);
				B_derv(:, :, 1) = E;
				Lambda_derv = zeros(system_order, system_order, 3) + 0i;
				Lambda_derv(:, :, 1) = diag(ev);
				V_derv = zeros(system_order, system_order, 3) + 0i;
				V_derv(:, :, 1) = eigenvectors_right;
				for ll = 1:N
					for zz = 1:M
						qq = mod(ll - 1, number_controls) + 1;
						rr = idivide(ll - 1, number_controls) + 1;
						ss = mod(zz - 1, number_controls) + 1;
						tt = idivide(zz - 1, number_controls) + 1;
						A_derv(:, :, 1) = hessian_systemDerivative(system(ii).B, system(ii).C, qq, rr);
						B_derv(:, :, 3) = hessian_systemDerivative(system(ii).B, system(ii).C_dot, ss, tt);% 3 is the correct index since B contains [B, dB/dK, dB/dD]
						Lambda_derv(:, :, 2) = diag(eigenvalue_derivative_temp(:, qq, rr));
						Lambda_derv(:, :, 3) = diag(eigenvalue_derivative_xdot_temp(:, ss, tt));
						V_derv(:, :, 2) = eigenvector_right_derivative_temp(:, :, qq, rr);
						V_derv(:, :, 3) = eigenvector_right_derivative_xdot_temp(:, :, ss, tt);
						eigenvalue_2derivative_mixed_temp(:, zz, ll) = hessian_secondDerivativeDifferentParams(A_derv, B_derv, Lambda_derv, V_derv, W_tilde);
					end
				end
				if number_states - system_order > 0
					eigenvalue_2derivative_mixed(:, :, :, ii) = cat(1, eigenvalue_2derivative_mixed_temp, (1 + 1i)*NaN(number_states - system_order, number_controls*number_measurements_xdot, number_controls*number_measurements));
				else
					eigenvalue_2derivative_mixed(:, :, :, ii) = eigenvalue_2derivative_mixed_temp;
				end
				% calculate d^2lambda/dKdR
				A_derv = zeros(system_order, system_order, 2);
				B_derv = zeros(system_order, system_order, 3);
				B_derv(:, :, 1) = E;
				Lambda_derv = zeros(system_order, system_order, 3) + 0i;
				Lambda_derv(:, :, 1) = diag(ev);
				V_derv = zeros(system_order, system_order, 3) + 0i;
				V_derv(:, :, 1) = eigenvectors_right;
				for ll = 1:M
					for zz = 1:N
						qq = mod(ll - 1, number_controls) + 1;
						rr = idivide(ll - 1, number_controls) + 1;
						ss = mod(zz - 1, number_controls) + 1;
						tt = idivide(zz - 1, number_controls) + 1;
						A_derv(:, :, 2) = hessian_systemDerivative(system(ii).B, system(ii).C, ss, tt);
						B_derv(:, :, 2) = hessian_systemDerivative(system(ii).B, system(ii).C_dot, qq, rr);
						Lambda_derv(:, :, 2) = diag(eigenvalue_derivative_xdot_temp(:, qq, rr));
						Lambda_derv(:, :, 3) = diag(eigenvalue_derivative_temp(:, ss, tt));
						V_derv(:, :, 2) = eigenvector_right_derivative_xdot_temp(:, :, qq, rr);
						V_derv(:, :, 3) = eigenvector_right_derivative_temp(:, :, ss, tt);
						eigenvalue_2derivative_xdot_mixed_temp(:, zz, ll) = hessian_secondDerivativeDifferentParams(A_derv, B_derv, Lambda_derv, V_derv, W_tilde);
					end
				end
				if number_states - system_order > 0
					eigenvalue_2derivative_xdot_mixed(:, :, :, ii) = cat(1, eigenvalue_2derivative_xdot_mixed_temp, (1 + 1i)*NaN(number_states - system_order, number_controls*number_measurements, number_controls*number_measurements_xdot));
				else
					eigenvalue_2derivative_xdot_mixed(:, :, :, ii) = eigenvalue_2derivative_xdot_mixed_temp;
				end
				% calculate d^2lambda/dK^2
				A_derv = zeros(system_order, system_order, 2);
				B_derv = zeros(system_order, system_order, 3);
				B_derv(:, :, 1) = E;
				Lambda_derv = zeros(system_order, system_order, 3) + 0i;
				Lambda_derv(:, :, 1) = diag(ev);
				V_derv = zeros(system_order, system_order, 3) + 0i;
				V_derv(:, :, 1) = eigenvectors_right;
				for ll = 1:M
					for zz = 1:M
						qq = mod(ll - 1, number_controls) + 1;
						rr = idivide(ll - 1, number_controls) + 1;
						ss = mod(zz - 1, number_controls) + 1;
						tt = idivide(zz - 1, number_controls) + 1;
						B_derv(:, :, 2) = hessian_systemDerivative(system(ii).B, system(ii).C_dot, qq, rr);
						B_derv(:, :, 3) = hessian_systemDerivative(system(ii).B, system(ii).C_dot, ss, tt);
						Lambda_derv(:, :, 2) = diag(eigenvalue_derivative_xdot_temp(:, qq, rr));
						Lambda_derv(:, :, 3) = diag(eigenvalue_derivative_xdot_temp(:, ss, tt));
						V_derv(:, :, 2) = eigenvector_right_derivative_xdot_temp(:, :, qq, rr);
						V_derv(:, :, 3) = eigenvector_right_derivative_xdot_temp(:, :, ss, tt);
						eigenvalue_2derivative_xdot_temp(:, zz, ll) = hessian_secondDerivativeDifferentParams(A_derv, B_derv, Lambda_derv, V_derv, W_tilde);
					end
				end
				if number_states - system_order > 0
					eigenvalue_2derivative_xdot(:, :, :, ii) = cat(1, eigenvalue_2derivative_xdot_temp, (1 + 1i)*NaN(number_states - system_order, number_controls*number_measurements_xdot, number_controls*number_measurements_xdot));
				else
					eigenvalue_2derivative_xdot(:, :, :, ii) = eigenvalue_2derivative_xdot_temp;
				end
			end
		else
			% if left eigenvecor matrix of eig and not W = V^-T is used, w^T*v has to be w^H*v
			if needsgradient || needseigenvectors
				% TODO: for codegen, left eigenvectormatrix can not be calculated by eig and therefore W = V^-H
				%[eigenvectors_right, eigenvalues(:, ii), eigenvectors_left] = eig(system(ii).A - system(ii).B*K*system(ii).C, 'vector');
				if descriptor
					if derivative_feedback
						E = system(ii).E - system(ii).B*K*system(ii).C_dot;
					else
						E = system(ii).E;
					end
				else
					if derivative_feedback
						E = eye(system_order) - system(ii).B*K*system(ii).C_dot;
					else
						E = eye(system_order);
					end
				end
				if allRNaN
					eigenvectors_right = ones(system_order) + eye(system_order) + 0i;
					eigenvalues(:, ii) = [
						double(1:system_order)' + 0i;
						(1 + 1i)*NaN(number_states - system_order, 1)
					];
					ev = double(1:system_order)' + 0i;
				else
					if derivative_feedback || descriptor
						[eigenvectors_right, ev] = eig(A, E, 'vector');
						% generalized eigenvalue problem can result in NaN eigenvalues
						ev(isnan(ev)) = Inf;
					else
						[eigenvectors_right, ev] = eig(A, 'vector');
					end
					ev = calculate_eigenvalue_filter_immediate(eigenvaluefilter, ev);
					eigenvalues(:, ii) = [
						ev;
						(1 + 1i)*NaN(number_states - system_order, 1)
					];
				end
				%ev = eigenvalues(:, ii);
				multiplicity = ones(size(ev, 1), 1, 'uint32');
				replacemap = false(0, size(ev, 1));
				rankupdate = 0;
				while rank(eigenvectors_right) < size(eigenvectors_right, 2) && rankupdate < 4
					% TODO: arbitrary tolerance should be replaced by a reasonable value
					[multiplicity, multiplicity_map] = eigenvalue_multiplicity(ev, 10^rankupdate/condeig_tolerance);
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
					eigenvectors_right_k = NaN(system_order, max(multiplicity), size(idxmap, 1)) + NaN*1i;
					for jj = 1:size(idxmap, 1)
						eigenvectors_right_k(:, 1, jj) = eigenvectors_right(:, multiplicityidx(1, idxmap(jj, 1)));
						k = 2;
						for gg = 2:multiplicity(multiplicityidx(1, idxmap(jj, 1)), 1)
							if descriptor || derivative_feedback
								if isinf(ev(multiplicityidx(1, idxmap(jj, 1)))) || isnan(isinf(ev(multiplicityidx(1, idxmap(jj, 1)))))
									evsysA = E;
									evsysb = A*eigenvectors_right_k(:, gg - 1, jj);
								else
									evsysA = (ev(multiplicityidx(1, idxmap(jj, 1)))*E - A);
									evsysb = -E*eigenvectors_right_k(:, gg - 1, jj);
								end
								if rank(evsysA) < size(evsysA, 1)
									sol_particular = pinv(evsysA)*evsysb;
								else
									sol_particular = evsysA\evsysb;
								end
								sol = null(evsysA);
								sol_particular(any(sol, 2), :) = 0;
								sol = sol + repmat(sol_particular, 1, size(sol, 2));
								temp = orth([eigenvectors_right_k(:, 1:k - 1, jj), sol]) + 0i;
								if size(temp, 2) > k
									temp = temp(:, 1:k);
								end
							else
								temp = orth([eigenvectors_right_k(:, 1:k - 1, jj), null((ev(multiplicityidx(1, idxmap(jj, 1)))*E - A)^k)]) + 0i;
							end
							if size(temp, 1) ~= system_order || size(temp, 2) ~= k
								error('control:design:gamma:eigenvalues', 'No regular basis of right eigenvectors could be found.');
							end
							eigenvectors_right_k(:, 1:k, jj) = temp;
							k = k + 1;
						end
					end
					for ff = 1:size(idxmap, 1)
						if sum(replacemap(ff, :)) ~= multiplicity(multiplicityidx(1, idxmap(ff, 1)), 1)
							% if we get here, multiplicity and number of equal eigenvalues in map are not equal, which should not occur under all circumstances
							error('control:design:gamma:eigenvalues', 'Something went wrong in the eigenvector calculation, check the implementation.');
						end
						eigenvectors_right(:, replacemap(ff, :)) = eigenvectors_right_k(:, 1:multiplicity(multiplicityidx(1, idxmap(ff, 1)), 1), multiplicityidx(1, idxmap(ff, 1)));
					end
				end
				if rank(eigenvectors_right) < size(eigenvectors_right, 2)
					error('control:design:gamma:eigenvalues', 'No regular basis of right eigenvectors could be found.');
				end
				if system_order ~= size(eigenvectors_right, 1) || system_order ~= size(eigenvectors_right, 2)
					% if we get here, eigenvectors were removed from the eigenvector matrix, which should not occur under all circumstances
					error('control:design:gamma:eigenvalues', 'Something went wrong in the eigenvector calculation, check the implementation.');
				end
				if (codegen_supports_recursion || ~codegen_is_generating) && (eigenvaluederivativetype == GammaEigenvalueDerivativeType.VANDERAA || (any(multiplicity(:) > 1) && ~(descriptor || derivative_feedback)))
					% TODO: remove descriptor || derivative_feedback condition when generalized eigenvalue problem is implemented for van der Aa method
					if ~codegen_supports_recursion && codegen_is_generating
						error('control:design:gamma:eigenvalues:runtimerecursion', 'Van der Aa''s method is recursive, but runtime recursion is only supported since R2016B and this mex file was compiled with an older version.');
					end
					% use van der Aa method for calculation of derivatives if requested or multiple eigenvalues occured
					if number_measurements_xdot > 0 && needsgradienteigenvectorsright_xdot
						error('control:design:gamma:eigenvalues', 'Van der Aa method not yet implemented for generalized eigenvalue problem.');
					end
					% initialization
					eigenvalue_derivative_temp = zeros(system_order, number_controls, number_measurements) + 0i;
					eigenvalue_derivative_xdot_temp = zeros(system_order, number_controls, number_measurements_xdot) + 0i;
					eigenvector_right_derivative_temp = zeros(system_order, system_order, number_controls, number_measurements) + 0i;
					eigenvector_right_derivative_xdot_temp = zeros(system_order, system_order, number_controls, number_measurements_xdot) + 0i;
					eigenvector_left_derivative_temp = zeros(system_order, system_order, number_controls, number_measurements) + 0i;
					eigenvector_left_derivative_xdot_temp = zeros(system_order, system_order, number_controls, number_measurements_xdot) + 0i;
					A_derv = zeros(system_order, system_order, system_order + 1);
					A_derv(:, :, 1) = A;
					B_derv = zeros(system_order, system_order, system_order + 1);
					B_derv(:, :, 1) = E;
					V = zeros(system_order, system_order) + 0i; %#ok<NASGU> prevent "The temporary variable will be cleared at the beginning of each iteration of the parfor loop." warnings
					D_eig = zeros(system_order, 1) + 0i; %#ok<NASGU> prevent "The temporary variable will be cleared at the beginning of each iteration of the parfor loop." warnings
					V_derv = zeros(system_order, system_order) + 0i; %#ok<NASGU> prevent "The temporary variable will be cleared at the beginning of each iteration of the parfor loop." warnings
					D_derv = zeros(system_order, 1) + 0i; %#ok<NASGU> prevent "The temporary variable will be cleared at the beginning of each iteration of the parfor loop." warnings
					W_derv = zeros(system_order, system_order) + 0i; %#ok<NASGU> prevent "The temporary variable will be cleared at the beginning of each iteration of the parfor loop." warnings
					V_derv_xdot = zeros(system_order, system_order) + 0i; %#ok<NASGU> prevent "The temporary variable will be cleared at the beginning of each iteration of the parfor loop." warnings
					D_derv_xdot = zeros(system_order, 1) + 0i; %#ok<NASGU> prevent "The temporary variable will be cleared at the beginning of each iteration of the parfor loop." warnings
					W_derv_xdot = zeros(system_order, system_order) + 0i; %#ok<NASGU> prevent "The temporary variable will be cleared at the beginning of each iteration of the parfor loop." warnings
					% calculation of derivative of A - B*R*C
					for kk = 1:number_controls
						for jj = 1:number_measurements
							if ~R_fixed(kk, jj)
								A_derv(:, :, 2) = hessian_systemDerivative(system(ii).B, system(ii).C, kk, jj);
								if allRNaN
									V = ones(system_order) + eye(system_order) + 0i;
									W = ones(system_order) + eye(system_order) + 0i;
									D_eig = double(1:system_order)' + 0i;
									V_derv = ones(system_order) + eye(system_order) + 0i;
									D_derv = double(1:system_order)' + 0i;
									W_derv = ones(system_order) + eye(system_order) + 0i;
								else
									% TODO: only sort inside vanDerAa
									% TODO: make D_eig and D_derv vectors
									% TODO: transform V_derv and W_derv to original W and V for all coefficients
									if descriptor || derivative_feedback
										[V, D_eig, W, V_derv, D_derv, W_derv] = vanDerAa(A_derv, B_derv, eigenvalue_options, eigenvectors_right, ev);
									else
										[V, D_eig, W, V_derv, D_derv, W_derv] = vanDerAa(A_derv, [], eigenvalue_options, eigenvectors_right, ev);
									end
								end
								if kk == 1 && jj == 1
									eigenvalues(:, ii) = [
										D_eig;
										(1 + 1i)*NaN(number_states - system_order, 1)
									];
									if needseigenvectors
										eigenvector_right(:, :, ii) = [
											V,	(1 + 1i)*NaN(system_order, number_states - system_order);
											(1 + 1i)*NaN(number_states - system_order, number_states)
										];
										if rank(V) < size(V, 2)
											error('control:design:gamma:eigenvalues', 'No regular basis of right eigenvectors could be found.');
										end
										if needseigenvectorsleft
											eigenvector_left(:, :, ii) = [
												W,	(1 + 1i)*NaN(system_order, number_states - system_order);
												(1 + 1i)*NaN(number_states - system_order, number_states)
											];
										end
									end
								end
								if needsgradient
									eigenvalue_derivative_temp(:, kk, jj) = D_derv;
								end
								if needsgradienteigenvectorsright
									eigenvector_right_derivative_temp(:, :, kk, jj) = V_derv;
								end
								if needsgradienteigenvectorsleft
									eigenvector_left_derivative_temp(:, :, kk, jj) = W_derv;
								end
							end
						end
						if descriptor || derivative_feedback
							for jj = 1:number_measurements_xdot
								if ~K_fixed(kk, jj)
									B_derv(:, :, 2) = hessian_systemDerivative(system(ii).B, system(ii).C_dot, kk, jj);
									if allRNaN
										V_derv_xdot = ones(system_order) + eye(system_order) + 0i;
										D_derv_xdot = double(1:system_order)' + 0i;
										W_derv_xdot = ones(system_order) + eye(system_order) + 0i;
									else
										% TODO: implementation for generalized eigenvalue problem
										[~, ~, ~, V_derv_xdot, D_derv_xdot, W_derv_xdot] = vanDerAa(A_derv, B_derv, eigenvalue_options, eigenvectors_right, ev);
									end
									if needsgradient_xdot
										eigenvalue_derivative_xdot_temp(:, kk, jj) = D_derv_xdot;
									end
									if needsgradienteigenvectorsright_xdot
										eigenvector_right_derivative_xdot_temp(:, :, kk, jj) = V_derv_xdot;
									end
									if needsgradienteigenvectorsleft_xdot
										eigenvector_left_derivative_xdot_temp(:, :, kk, jj) = W_derv_xdot;
									end
								end
							end
						end
					end
					if needsgradient
						if number_states - system_order > 0
							eigenvalue_derivative(:, :, :, ii) = cat(1, eigenvalue_derivative_temp, (1 + 1i)*NaN(number_states - system_order, number_controls, number_measurements));
						else
							eigenvalue_derivative(:, :, :, ii) = eigenvalue_derivative_temp;
						end
						if needsgradienteigenvectors
							if needsgradienteigenvectorsright
								if number_states - system_order > 0
									eigenvector_right_derivative_temp_cat = NaN(number_states, number_states, number_controls, number_measurements) + NaN*1i;
									eigenvector_right_derivative_temp_cat(1:system_order, 1:system_order, :, :, :, :) = eigenvector_right_derivative_temp;
									eigenvector_right_derivative(:, :, :, :, ii) = eigenvector_right_derivative_temp_cat;
								else
									eigenvector_right_derivative(:, :, :, :, ii) = eigenvector_right_derivative_temp;
								end
							end
							if needsgradienteigenvectorsleft
								if number_states - system_order > 0
									eigenvector_left_derivative_temp_cat = NaN(number_states, number_states, number_controls, number_measurements) + NaN*1i;
									eigenvector_left_derivative_temp_cat(1:system_order, 1:system_order, :, :, :, :) = eigenvector_left_derivative_temp;
									eigenvector_left_derivative(:, :, :, :, ii) = eigenvector_left_derivative_temp_cat;
								else
									eigenvector_left_derivative(:, :, :, :, ii) = eigenvector_left_derivative_temp;
								end
							end
						end
					end
					if needsgradient_xdot && number_measurements_xdot > 0
						% cat can not concatenate empty matrices in compiled code, so number of derivative measurements has to be greater than 0 to do anything
						if number_states - system_order > 0
							eigenvalue_derivative_xdot(:, :, :, ii) = cat(1, eigenvalue_derivative_xdot_temp, (1 + 1i)*NaN(number_states - system_order, number_controls, number_measurements_xdot));
						else
							eigenvalue_derivative_xdot(:, :, :, ii) = eigenvalue_derivative_xdot_temp;
						end
						if needsgradienteigenvectors
							if needsgradienteigenvectorsright_xdot
								if number_states - system_order > 0
									eigenvector_right_derivative_xdot_temp_cat = NaN(number_states, number_states, number_controls, number_measurements_xdot) + NaN*1i;
									eigenvector_right_derivative_xdot_temp_cat(1:system_order, 1:system_order, :, :, :, :) = eigenvector_right_derivative_xdot_temp;
									eigenvector_right_derivative_xdot(:, :, :, :, ii) = eigenvector_right_derivative_xdot_temp_cat;
								else
									eigenvector_right_derivative_xdot(:, :, :, :, ii) = eigenvector_right_derivative_xdot_temp;
								end
							end
							if needsgradienteigenvectorsleft_xdot
								if number_states - system_order > 0
									eigenvector_left_derivative_xdot_temp_cat = NaN(number_states, number_states, number_controls, number_measurements_xdot) + NaN*1i;
									eigenvector_left_derivative_xdot_temp_cat(1:system_order, 1:system_order, :, :, :, :) = eigenvector_left_derivative_xdot_temp;
									eigenvector_left_derivative_xdot(:, :, :, :, ii) = eigenvector_left_derivative_xdot_temp_cat;
								else
									eigenvector_left_derivative_xdot(:, :, :, :, ii) = eigenvector_left_derivative_xdot_temp;
								end
							end
						end
					end
				elseif ~codegen_supports_recursion && codegen_is_generating && (eigenvaluederivativetype == GammaEigenvalueDerivativeType.VANDERAA || (any(multiplicity(:) > 1) && ~(descriptor || derivative_feedback)))
					error('control:design:gamma:eigenvalues:runtimerecursion', 'Van der Aa''s method is recursive, but runtime recursion is only supported since R2016B and this mex file was compiled with an older version.');
				else
					% use eigenvalue sensitivity or Rudisill/Chu method
					if allRNaN
						eigenvectors_left = eigenvectors_right';
					else
						if descriptor || derivative_feedback
							if rank(E) < size(E, 2)
								% TODO: replace by adequate algorithm for left eigenvectors
								eigenvectors_left = pinv(E*eigenvectors_right)';
							else
								eigenvectors_left = inv(E*eigenvectors_right)';
							end
						else
							eigenvectors_left = inv(eigenvectors_right)';
						end
					end
					if needseigenvectorsright
						eigenvector_right(:, :, ii) = [
							eigenvectors_right,	(1 + 1i)*NaN(system_order, number_states - system_order);
							(1 + 1i)*NaN(number_states - system_order, number_states)
						];
					end
					if needseigenvectorsleft
						eigenvector_left(:, :, ii) = [
							eigenvectors_left,	(1 + 1i)*NaN(system_order, number_states - system_order);
							(1 + 1i)*NaN(number_states - system_order, number_states)
						];
					end
					if needsgradient
						eigenvalue_derivative_xdot_temp = zeros(system_order, number_controls, number_measurements_xdot) + 0i;
						eigenvalue_derivative_temp = zeros(system_order, number_controls, number_measurements) + 0i;
						for kk = 1:system_order
							if multiplicity(kk, 1) == 1
								% distinct eigenvalues
								% if left eigenvecor matrix of eig and not W = V^-T is used, w^T*v has to be w^H*v
								if derivative_feedback || descriptor
									wv = eigenvectors_left(:, kk)'*E*eigenvectors_right(:, kk);
				% 					if wv == 0
				% 						wv = eps;
				% 					end
								else
									wv = eigenvectors_left(:, kk)'*eigenvectors_right(:, kk);
								end
								for ll = 1:number_controls
									b_ll = system(ii).B(:, ll);
									for hh = 1:number_measurements
										if ~R_fixed(ll, hh)
											eigenvalue_derivative_temp(kk, ll, hh) = -eigenvectors_left(:, kk)'*b_ll*system(ii).C(hh, :)*eigenvectors_right(:, kk)/wv;
										end
									end
									if needsgradient_xdot
										for jj = 1:number_measurements_xdot
											if ~K_fixed(ll, jj)
												eigenvalue_derivative_xdot_temp(kk, ll, jj) = ev(kk)*eigenvectors_left(:, kk)'*b_ll*system(ii).C_dot(jj, :)*eigenvectors_right(:, kk)/wv;
											end
										end
									end
								end
							else
								% repeated eigenvalues
								currentidx = find(replacemap(:, kk), 1, 'first');
								if isempty(currentidx)
									% if we get here, an eigenvalue with multiplicity > 1 is not detected as such and was replaced in the map by unique, which should not occur under all circumstances
									error('control:design:gamma:eigenvalues', 'Something went wrong in the eigenvector calculation, check the implementation.');
								end
								currentidx = currentidx(1);
								currentreplacemap = replacemap(currentidx, :);
								if kk > 1 && any(currentreplacemap(1, 1:kk - 1))
									continue;
								end
								jordan_basis_right = eigenvectors_right(:, currentreplacemap);
								jordan_basis_left = eigenvectors_left(:, currentreplacemap);
								if derivative_feedback || descriptor
									wEv = jordan_basis_left'*E*jordan_basis_right;
								else
									wEv = jordan_basis_left'*jordan_basis_right;
								end
								currentreplacemapcount = sum(currentreplacemap);
								for ll = 1:number_controls
									b_ll = system(ii).B(:, ll);
									for hh = 1:number_measurements
										if ~R_fixed(ll, hh)
											eigenvalue_derivative_tempval = eig(-jordan_basis_left'*b_ll*system(ii).C(hh, :)*jordan_basis_right, wEv, 'vector');
											if currentreplacemapcount ~= size(eigenvalue_derivative_tempval, 1)
												% if we get here, something went wrong and we only want to tell the code generator to respect dimensions when compiling
												error('control:design:gamma:eigenvalues', 'Something went wrong in the eigenvector calculation, check the implementation.');
											end
											eigenvalue_derivative_temp(currentreplacemap, ll, hh) = eigenvalue_derivative_tempval;
										end
									end
									if needsgradient_xdot
										for jj = 1:number_measurements_xdot
											if ~K_fixed(ll, jj)
												eigenvalue_derivative_xdot_tempval = eig(ev(kk)*jordan_basis_left'*b_ll*system(ii).C_dot(jj, :)*jordan_basis_right, wEv, 'vector');
												if currentreplacemapcount ~= size(eigenvalue_derivative_xdot_tempval, 1)
													% if we get here, something went wrong and we only want to tell the code generator to respect dimensions when compiling
													error('control:design:gamma:eigenvalues', 'Something went wrong in the eigenvector calculation, check the implementation.');
												end
												eigenvalue_derivative_xdot_temp(currentreplacemap, ll, jj) = eigenvalue_derivative_xdot_tempval;
											end
										end
									end
								end
							end
						end
						if number_states - system_order > 0
							eigenvalue_derivative(:, :, :, ii) = cat(1, eigenvalue_derivative_temp, (1 + 1i)*NaN(number_states - system_order, number_controls, number_measurements));
						else
							eigenvalue_derivative(:, :, :, ii) = eigenvalue_derivative_temp;
						end
						if needsgradient_xdot && number_measurements_xdot > 0
							% cat can not concatenate empty matrices in compiled code, so number of derivative measurements has to be greater than 0 to do anything
							if number_states - system_order > 0
								eigenvalue_derivative_xdot(:, :, :, ii) = cat(1, eigenvalue_derivative_xdot_temp, (1 + 1i)*NaN(number_states - system_order, number_controls, number_measurements_xdot));
							else
								eigenvalue_derivative_xdot(:, :, :, ii) = eigenvalue_derivative_xdot_temp;
							end
						end
						if needsgradienteigenvectors
							eigenvectors_right_derivative_temp = zeros(system_order, system_order, number_controls, number_measurements) + 0i;
							eigenvectors_right_derivative_xdot_temp = zeros(system_order, system_order, number_controls, number_measurements_xdot) + 0i;
							%if needsgradienteigenvectorsleft% TODO: condition is not allowed to prevent "The temporary variable will be cleared at the beginning of each iteration of the parfor loop." warnings
								eigenvectors_left_derivative_temp = zeros(system_order, system_order, number_controls, number_measurements) + 0i;
							%end
							%if needsgradienteigenvectorsleft_xdot
								eigenvectors_left_derivative_xdot_temp = zeros(system_order, system_order, number_controls, number_measurements_xdot) + 0i;
							%end
							if eigenvaluederivativetype == GammaEigenvalueDerivativeType.DEFAULT || eigenvaluederivativetype == GammaEigenvalueDerivativeType.RUDISILLCHU
								for kk = 1:system_order
									if multiplicity(kk, 1) == 1
										% distinct eigenvalues
										mat_right = [
											eigenvectors_right(:, kk)',	0;
											A - ev(kk)*E,				-E*eigenvectors_right(:, kk)
										];
										%if needsgradienteigenvectorsleft% TODO: condition is not allowed to prevent "The temporary variable will be cleared at the beginning of each iteration of the parfor loop." warnings
											%mat_left = [
											%	A - ev(kk)*E,					eigenvectors_left(:, kk);
											%	-eigenvectors_left(:, kk)'*E,	0
											%].';
										%end
										for ll = 1:number_controls
											b_ll = system(ii).B(:, ll);
											for hh = 1:number_measurements
												if ~R_fixed(ll, hh)
													vec_right = [
														0;
														-b_ll*system(ii).C(hh, :)*eigenvectors_right(:, kk)
													];
													temp = mat_right\vec_right;
													eigenvectors_right_derivative_temp(kk, :, ll, hh) = temp(1:system_order);
													%if needsgradienteigenvectorsleft
														% TODO: does not work, is instead calculated later in dependence of right eigenvector derivative like in [#STRUBEL2014]
														%vec_left = [
														%	-eigenvectors_left(:, kk)'*b_ll*system(ii).C(hh, :),	0
														%].';
														%temp = (mat_left\vec_left).';
														%eigenvectors_left_derivative_temp(kk, :, ll, hh) = temp(1:system_order);
													%end
												end
											end
											if needsgradienteigenvectorsright_xdot
												for jj = 1:number_measurements_xdot
													if ~K_fixed(ll, jj)
														vec_right = [
															0;
															ev(kk)*b_ll*system(ii).C_dot(jj, :)*eigenvectors_right(:, kk)
														];
														temp = mat_right\vec_right;
														eigenvectors_right_derivative_xdot_temp(kk, :, ll, jj) = temp(1:system_order);
														%if needsgradienteigenvectorsleft_xdot
															% TODO: does not work, is instead calculated later in dependence of right eigenvector derivative like in [#STRUBEL2014]
															%vec_left = [
															%	ev(kk)*eigenvectors_left(:, kk)'*b_ll*system(ii).C_dot(jj, :),	0
															%].';
															%temp = (mat_left\vec_left).';
															%eigenvectors_left_derivative_xdot_temp(kk, :, ll, jj) = temp(1:system_order);
														%end
													end
												end
											end
										end
									else
										% repeated eigenvalues
										currentidx = find(replacemap(:, kk), 1, 'first');
										if isempty(currentidx)
											% if we get here, an eigenvalue with multiplicity > 1 is not detected as such and was replaced in the map by unique, which should not occur under all circumstances
											error('control:design:gamma:eigenvalues', 'Something went wrong in the eigenvector calculation, check the implementation.');
										end
										currentidx = currentidx(1);
										currentreplacemap = replacemap(currentidx, :);
										if kk > 1 && any(currentreplacemap(1, 1:kk - 1))
											continue;
										end
										%eigenvectors_right_derivative_temp(currentreplacemap, :, :, :) = NaN;
										%if needsgradienteigenvectorsright_xdot
										%	eigenvectors_right_derivative_xdot_temp(currentreplacemap, :, :, :) = NaN;
										%end
										error('control:design:gamma:eigenvalues', 'Calculation of derivative of eigenvectors is only supported with van der Aa''s method, there must be either a programming error or the generated mex file was not created with a matlab verion that supports runtime recursion.');
									end
								end
								% calculate left eigenvector derivative in dependence of right eigenvector derivative according to [#STRUBEL2014]
								if needsgradienteigenvectorsleft
									for ll = 1:number_controls
										for hh = 1:number_measurements
											eigenvectors_left_derivative_temp(:, :, ll, hh) = -eigenvectors_left*eigenvectors_right_derivative_temp(:, :, ll, hh)*eigenvectors_left;
										end
									end
								end
								if needsgradienteigenvectorsleft_xdot && number_measurements_xdot > 0
									for ll = 1:number_controls
										for hh = 1:number_measurements_xdot
											eigenvectors_left_derivative_xdot_temp(:, :, ll, hh) = -eigenvectors_left*eigenvectors_right_derivative_xdot_temp(:, :, ll, hh)*eigenvectors_left;
										end
									end
								end
							else
								error('control:design:gamma:eigenvalues', 'Only methods of #FOELLINGER1994, #RUDISILL1975 and #VANDERAA2007 are implemented for calculation of eigenvector derivatives.');
							end
							if needsgradienteigenvectorsright
								if number_states - system_order > 0
									eigenvector_right_derivative_temp_cat = NaN(number_states, number_states, number_controls, number_measurements) + NaN*1i;
									eigenvector_right_derivative_temp_cat(1:system_order, 1:system_order, :, :, :, :) = eigenvectors_right_derivative_temp;
									eigenvector_right_derivative(:, :, :, :, ii) = eigenvector_right_derivative_temp_cat;
								else
									eigenvector_right_derivative(:, :, :, :, ii) = eigenvectors_right_derivative_temp;
								end
							end
							if needsgradienteigenvectorsright_xdot && number_measurements_xdot > 0
								if number_states - system_order > 0
									eigenvector_right_derivative_xdot_temp_cat = NaN(number_states, number_states, number_controls, number_measurements_xdot) + NaN*1i;
									eigenvector_right_derivative_xdot_temp_cat(1:system_order, 1:system_order, :, :, :, :) = eigenvectors_right_derivative_xdot_temp;
									eigenvector_right_derivative_xdot(:, :, :, :, ii) = eigenvector_right_derivative_xdot_temp_cat;
								else
									eigenvector_right_derivative_xdot(:, :, :, :, ii) = eigenvectors_right_derivative_xdot_temp;
								end
							end
							if needsgradienteigenvectorsleft
								if number_states - system_order > 0
									eigenvector_left_derivative_temp_cat = NaN(number_states, number_states, number_controls, number_measurements) + NaN*1i;
									eigenvector_left_derivative_temp_cat(1:system_order, 1:system_order, :, :, :, :) = eigenvectors_left_derivative_temp;
									eigenvector_left_derivative(:, :, :, :, ii) = eigenvector_left_derivative_temp_cat;
								else
									eigenvector_left_derivative(:, :, :, :, ii) = eigenvectors_left_derivative_temp;
								end
							end
							if needsgradienteigenvectorsleft_xdot && number_measurements_xdot > 0
								if number_states - system_order > 0
									eigenvector_left_derivative_xdot_temp_cat = NaN(number_states, number_states, number_controls, number_measurements_xdot) + NaN*1i;
									eigenvector_left_derivative_xdot_temp_cat(1:system_order, 1:system_order, :, :, :, :) = eigenvectors_left_derivative_xdot_temp;
									eigenvector_left_derivative_xdot(:, :, :, :, ii) = eigenvector_left_derivative_xdot_temp_cat;
								else
									eigenvector_left_derivative_xdot(:, :, :, :, ii) = eigenvectors_left_derivative_xdot_temp;
								end
							end
						end
					end
				end
			else
				if allRNaN
					eigenvalues(:, ii) = [
						double(1:system_order)' + 0i;
						(1 + 1i)*NaN(number_states - system_order, 1)
					];
				else
					if derivative_feedback
						if descriptor
							ev = eig(A, system(ii).E - system(ii).B*K*system(ii).C_dot, 'vector');
						else
							ev = eig(A, eye(number_states) - system(ii).B*K*system(ii).C_dot, 'vector');
						end
						% generalized eigenvalue problem can result in NaN eigenvalues
						ev(isnan(ev)) = Inf;
					else
						if descriptor
							ev = eig(A, system(ii).E, 'vector');
							% generalized eigenvalue problem can result in NaN eigenvalues
							ev(isnan(ev)) = Inf;
						else
							ev = eig(A, 'vector');
						end
					end
					ev = calculate_eigenvalue_filter_immediate(eigenvaluefilter, ev);
					eigenvalues(:, ii) = [
						ev;
						(1 + 1i)*NaN(number_states - system_order, 1)
					];
				end
			end
		end
	end
end
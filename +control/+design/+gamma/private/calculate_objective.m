function [J, gradJ, hessianJ] = calculate_objective(areaval, weight, eigenvalue_derivative, eigenvalue_derivative_xdot, areaval_derivative, dimensions, options, eigenvalue_2derivative, areval_2_derivative, eigenvalue_2derivative_xdot, eigenvalue_2derivative_mixed, eigenvalue_2derivative_xdot_mixed)
	%CALCULATE_OBJECTIVE helper function for calculation of objective function and gradient for gamma pole placement
	%	Input:
	%		areaval:							values of areafunctions as matrix
	%		weight:								weighting matrix with number of systems columns and number of pole area border functions rows
	%		eigenvalue_derivative:				derivative of eigenvalues with respect to proportional gain used for calculation of gradient of objective function
	%		eigenvalue_derivative_xdot:			derivative of eigenvalues with respect to derivative gain used for calculation of gradient of objective function
	%		areaval_derivative:					derivative of pole area border functions used for calculation of gradient of objective function
	%		dimensions:							structure with information about dimensions of the different variables and systems
	%		options:							structure with options for objective function
	%		eigenvalue_2derivative:				second derivative of eigenvalues with respect to proportional gain
	%		areval_2_derivative:				second derivative of area functions
	%		eigenvalue_2derivative_xdot:		second derivative of eigenvalues with respect to derivative gain
	%		eigenvalue_2derivative_mixed:		second derivative of eigenvalues with respect to proportional gain and derivative gain
	%		eigenvalue_2derivative_xdot_mixed:	second derivative of eigenvalues with respect to derivative gain and proportional gain
	%	Output:
	%		J:									objective function value for current optimization value
	%		gradJ:								gradient of objective function value for current optimization value
	%		hessianJ:							hessian of objective function value for current optimization value
	codegen_is_generating = coder.const(~coder.target('MATLAB'));
	number_models = dimensions.models;
	number_states = dimensions.states;
	number_controls = dimensions.controls;
	number_measurements = dimensions.measurements;
	number_measurements_xdot = dimensions.measurements_xdot;
	number_references = dimensions.references;
	number_areas_max = dimensions.areas_max;
	derivative_feedback = number_measurements_xdot > 0;
	objective_type = options.type;
	objective_weight = options.weight;
	objective_settings_KREISSELMEIER_rho = options.objective.kreisselmeier.rho;
	objective_settings_KREISSELMEIER_max = options.objective.kreisselmeier.max;
	numthreads = options.numthreads;
	J_objective = zeros(size(objective_type, 1), 1);
	Jtempgrad = zeros([size(objective_type, 1), size(areaval)]);
	Jtemphesse = zeros([size(objective_type, 1), size(areaval)]);
	needsgradient = nargout >= 2;
	needshessian = nargout >= 3;
	areaval_zero = zeros(size(areaval));% prevent NaN in 0*areaval if areaval is Inf
	if options.eigenvalueignoreinf
		areaval(isinf(areaval)) = 0;
	end
	parfor (ii = 1:int32(size(objective_type, 1)), numthreads)
		Jtemp = areaval; %#ok<NASGU> prevent "The temporary variable Jtemp will be cleared at the beginning of each iteration of the parfor loop."
		switch objective_type(ii, 1)
			case GammaJType.ZERO
				Jtemp = areaval_zero;
				if needsgradient
					Jtempgrad(ii, :, :, :) = Jtemp;
					if needshessian
						Jtemphesse(ii, :, :, :) = Jtemp;
					end
				end
			case GammaJType.LINEAR
				Jtemp = areaval;
				if needsgradient
					Jtempgrad(ii, :, :, :) = ones(size(areaval));
					if needshessian
						Jtemphesse(ii, :, :, :) = zeros(size(areaval));
					end
				end
			case GammaJType.SQUARE
				Jtemp = sign(areaval).*areaval.^2;
				if needsgradient
					Jtempgrad(ii, :, :, :) = 2*abs(areaval);
					if needshessian
						Jtemphesse(ii, :, :, :) = 2*sign(areaval);
					end
				end
			case GammaJType.SQUAREPENALTY
				Jtemp = areaval_zero;
				%Jtemp(areaval >= 0) = areaval(areaval >= 0).^2;
				if ~isempty(Jtemp)
					ispos = areaval >= 0;
					if codegen_is_generating
						% workaround for case with singleton dimensions that is not handled correctly in generated code
						% otherwise results in "Code generation assumption about size violated. Unexpected run-time vector changed predicted orientation."
						if any(ispos(:)) && ~isempty(ispos)
							if isvector(ispos) || isvector(Jtemp)
								squarevec = areaval(:);
								Jtemp(ispos(:)) = squarevec(ispos(:)).^2;
							else
								if number_models == 1 && size(Jtemp, 1) == 1
									for jj = 1:size(Jtemp, 1)
										for kk = 1:size(Jtemp, 2)
											for ll = 1:size(Jtemp, 3)
												if ispos(jj, kk, ll)
													Jtemp(jj, kk, ll) = areaval(jj, kk, ll).^2;
												end
											end
										end
									end
								else
									areavaltmp = areaval(ispos);
									Jtemp(ispos) = areavaltmp.^2;
								end
							end
						end
					else
						areavaltmp = areaval(ispos);
						Jtemp(ispos) = areavaltmp.^2;
					end
				end
				if needsgradient
					temp = 2*areaval;
					temp(areaval < 0) = 0;
					Jtempgrad(ii, :, :, :) = temp;
					if needshessian
						temp = 2*ones(size(areaval));
						temp(areaval < 0) = 0;
						Jtemphesse(ii, :, :, :) = temp;
					end
				end
			case GammaJType.CUBIC
				Jtemp = areaval.^3;
				if needsgradient
					Jtempgrad(ii, :, :, :) = 3*areaval.^2;
					if needshessian
						Jtemphesse(ii, :, :, :) = 6*areaval;
					end
				end
			case GammaJType.EXP
				Jtemp = exp(areaval);
				if needsgradient
					Jtempgrad(ii, :, :, :) = Jtemp;
					if needshessian
						Jtemphesse(ii, :, :, :) = Jtemp;
					end
				end
			case GammaJType.LOG
				Jtemp = areaval_zero;
				Jtemp(:) = Inf;
				if ~isempty(Jtemp)
					isnonneg = areaval < 0;
					if codegen_is_generating
						% workaround for case with singleton dimensions that is not handled correctly in generated code
						% otherwise results in "Code generation assumption about size violated. Unexpected run-time vector changed predicted orientation."
						if any(isnonneg(:)) && ~isempty(isnonneg)
							if isvector(isnonneg) || isvector(Jtemp)
								reallogvec = areaval(:);
								Jtemp(isnonneg(:)) = -reallog(-reallogvec(isnonneg(:)));
							else
								if number_models == 1 && size(Jtemp, 1) == 1
									for jj = 1:size(Jtemp, 1)
										for kk = 1:size(Jtemp, 2)
											for ll = 1:size(Jtemp, 3)
												if isnonneg(jj, kk, ll)
													Jtemp(jj, kk, ll) = -reallog(-areaval(jj, kk, ll));
												end
											end
										end
									end
								else
									areavaltmp = -areaval(isnonneg);
									Jtemp(isnonneg) = -reallog(areavaltmp);
								end
							end
						end
					else
						areavaltmp = -areaval(isnonneg);
						Jtemp(isnonneg) = -reallog(areavaltmp);
					end
				end
				if needsgradient
					Jtempgrad(ii, :, :, :) = -1./areaval;
					if needshessian
						Jtemphesse(ii, :, :, :) = 1./(areaval.^2);
					end
				end
			case GammaJType.MAX
				Jtemp = max(areaval, 0);
				if needsgradient
					temp = ones(size(areaval));
					temp(areaval < 0) = 0;
					Jtempgrad(ii, :, :, :) = temp;
					if needshessian
						Jtemphesse(ii, :, :, :) = zeros(size(areaval));
					end
				end
			case GammaJType.KREISSELMEIER
				tempexp = objective_settings_KREISSELMEIER_rho*(areaval - objective_settings_KREISSELMEIER_max);
				Jtemp = exp(tempexp);
				if needsgradient
					sumJ = sum(Jtemp(:));
					if isinf(sumJ)
						% TODO: replace gradient with limit value, if any exponential function in sum overflows
						%Jtempgrad(ii, :, :, :) = double(isinf(Jtemp));
						[~, maxidx] = max(tempexp(:));
						tempexp = zeros(size(areaval));
						tempexp(maxidx) = 1;
						Jtempgrad(ii, :, :, :) = tempexp;
					elseif sumJ == 0 || abs(sumJ) < abs(realmin('double'))
						% prevent division by 0 if all exponential functions are close to zero
						Jtempgrad(ii, :, :, :) = 1;
					else
						Jtempgrad(ii, :, :, :) = 1./sumJ.*Jtemp;
					end
					if needshessian
						if isinf(sumJ)
							% TODO: replace hessian with limit value, if any exponential function in sum overflows
							[~, maxidx] = max(tempexp(:));
							tempexp = zeros(size(areaval));
							tempexp(maxidx) = 1;
							Jtemphesse(ii, :, :, :) = tempexp;
						elseif sumJ == 0 || abs(sumJ) < abs(realmin('double'))
							% prevent division by 0 if all exponential functions are close to zero
							Jtemphesse(ii, :, :, :) = 1;
						else
							Jtemphesse(ii, :, :, :) = objective_settings_KREISSELMEIER_rho./sumJ.*Jtemp;
						end
					end
				end
			case GammaJType.EIGENVALUECONDITION
				Jtemp = areaval_zero;
				if needsgradient
					Jtempgrad(ii, :, :, :) = 0*Jtemp;
					if needshessian
						Jtemphesse(ii, :, :, :) = zeros(size(areaval));
					end
				end
			case GammaJType.NORMGAIN
				Jtemp = areaval_zero;
				if needsgradient
					Jtempgrad(ii, :, :, :) = 0*Jtemp;
					if needshessian
						Jtemphesse(ii, :, :, :) = zeros(size(areaval));
					end
				end
			case GammaJType.LYAPUNOV
				Jtemp = areaval_zero;
				if needsgradient
					Jtempgrad(ii, :, :, :) = 0*Jtemp;
					if needshessian
						Jtemphesse(ii, :, :, :) = zeros(size(areaval));
					end
				end
			otherwise
				Jtemp = areaval_zero;
				if needsgradient
					Jtempgrad(ii, :, :, :) = 0*Jtemp;
					if needshessian
						Jtemphesse(ii, :, :, :) = zeros(size(areaval));
					end
				end
		end
		if number_models == 1 || isvector(Jtemp)
			% codegen somehow messes up the dimensions of Jtemp and generated code crashes at runtime, so colon indexing is used here to convert to 2D-vectors
			Jidx = ~isnan(Jtemp);
			Jidx = Jidx(:);
			Jtempvec = Jtemp(:);
			if any(isinf(Jtempvec) & Jtempvec > 0) && any(isinf(Jtempvec) & Jtempvec <= 0)
				% avoid -Inf + Inf = NaN
				Jsum = Inf;
			else
				Jsum = sum(Jtempvec(Jidx(:)));
			end
		else
			Jpos = Jtemp(:) > 0;
			Jinf = isinf(Jtemp(:));
			if any(Jinf & Jpos) && any(Jinf & ~Jpos)
				% avoid -Inf + Inf = NaN
				Jsum = Inf;
			else
				Jsum = sum(Jtemp(~isnan(Jtemp)));
			end
		end
		if objective_type(ii, 1) == GammaJType.KREISSELMEIER
			J_objective(ii, 1) = objective_weight(ii, 1)*reallog(Jsum)/objective_settings_KREISSELMEIER_rho;
		else
			J_objective(ii, 1) = objective_weight(ii, 1)*Jsum;%=sum(sum(sum(Jtemp)));
		end
	end
	if any(isinf(J_objective) & J_objective > 0) && any(isinf(J_objective) & J_objective <= 0)
		% avoid -Inf + Inf = NaN
		J = Inf;
	else
		J = sum(J_objective);
	end
	if needsgradient
% see calculate_objective_gradient_helper for reason to not calculate gradient in this function (codegen crash)
% 		gradtemp = zeros(number_controls, number_measurements, number_models);
% 		gradtemp_xdot = zeros(number_controls, number_measurements_xdot, number_models);
%		gradtemp_prefilter = zeros(number_controls, number_references, number_models);
% 		if derivative_feedback && isempty(eigenvalue_derivative_xdot)
% 			error('control:design:gamma:gradient', 'Gradient for eigenvalues was not supplied.');
% 		end
% 		for jj = 1:size(objective_type, 1) %#ok<FORPF> number of objective functions is usually smaller than number of models, so parfor is used for models
% 			Jgrad_objective = gradtemp;
% 			Jgrad_xdot_objective = gradtemp_xdot;
% 			parfor (ii = 1:number_models, numthreads)
% 				tempgradJ = zeros(number_controls, number_measurements);
% 				tempgradJ_xdot = zeros(number_controls, number_measurements_xdot);
% 				tempareaval_derivative = areaval_derivative(:, :, :, ii);
% 				for kk = 1:number_states
% 					re = real(squeeze(eigenvalue_derivative(kk, :, :, ii)));
% 					im = imag(squeeze(eigenvalue_derivative(kk, :, :, ii)));
% 					if derivative_feedback && ~isempty(eigenvalue_derivative_xdot)
% 						re_xdot = real(squeeze(eigenvalue_derivative_xdot(kk, :, :, ii)));
% 						im_xdot = imag(squeeze(eigenvalue_derivative_xdot(kk, :, :, ii)));
% 					else
% 						re_xdot = zeros(number_controls, number_measurements_xdot);
% 						im_xdot = zeros(number_controls, number_measurements_xdot);
% 					end
% 					for ll = 1:number_areas_max
% 						if any(isnan(tempareaval_derivative(kk, ll, :, 1)))
% 							continue;
% 						end
% 						tempgradJ = tempgradJ + Jtempgrad(jj, ii, ll, kk)*weight(ii, ll)*(tempareaval_derivative(kk, ll, 1, 1)*re + tempareaval_derivative(kk, ll, 2, 1)*im);
% 						if derivative_feedback
% 							tempgradJ_xdot = tempgradJ_xdot + Jtempgrad(jj, ii, ll, kk)*weight(ii, ll)*(tempareaval_derivative(kk, ll, 1, 1)*re_xdot + tempareaval_derivative(kk, ll, 2, 1)*im_xdot);
% 						end
% 					end
% 				end
% 				Jgrad_objective(:, :, ii) = tempgradJ;
% 				if derivative_feedback
% 					Jgrad_xdot_objective(:, :, ii) = tempgradJ_xdot;
% 				end
% 			end
% 			gradtemp = gradtemp + objective_weight(jj, 1)*Jgrad_objective;
% 			gradtemp_xdot = gradtemp_xdot + objective_weight(jj, 1)*Jgrad_xdot_objective;
% 		end
% 		if derivative_feedback
% 			gradJ = cat(2, sum(gradtemp, 3), sum(gradtemp_xdot, 3), sum(gradtemp_prefilter, 3));
% 		else
% 			gradJ = cat(2, sum(gradtemp, 3), sum(gradtemp_prefilter, 3));
% 		end
		gradJ = calculate_objective_gradient_helper(numthreads, derivative_feedback, number_models, number_states, number_controls, number_measurements, number_measurements_xdot, number_references, number_areas_max, eigenvalue_derivative, eigenvalue_derivative_xdot, areaval_derivative, weight, objective_type, objective_weight, Jtempgrad, options.objective.preventNaN);
		if needshessian
			hessianJ = calculate_objective_hesse_helper(number_models, number_states, number_controls, number_measurements, number_measurements_xdot, number_references, number_areas_max, eigenvalue_derivative, eigenvalue_derivative_xdot, eigenvalue_2derivative, eigenvalue_2derivative_xdot, eigenvalue_2derivative_mixed, eigenvalue_2derivative_xdot_mixed,areaval_derivative, areval_2_derivative,weight, objective_type, objective_weight, Jtempgrad, Jtemphesse, options.objective.preventNaN, options);
		end
	end
end

function [gradJ] = calculate_objective_gradient_helper(numthreads, derivative_feedback, number_models, number_states, number_controls, number_measurements, number_measurements_xdot, number_references, number_areas_max, eigenvalue_derivative, eigenvalue_derivative_xdot, areaval_derivative, weight, objective_type, objective_weight, Jtempgrad, preventNaN)
	% TODO: this function should be part of calculate_objective, but for some unknown reason codegen reuses some of the internal variables needed in calculate_objective and then the function crashes at runtime because corrupted memory is freed, so gradient calculation is put in its own function
	gradtemp = zeros(number_controls, number_measurements, number_models);
	gradtemp_xdot = zeros(number_controls, number_measurements_xdot, number_models);
	gradtemp_prefilter = zeros(number_controls, number_references, number_models);
	if derivative_feedback && isempty(eigenvalue_derivative_xdot)
		error('control:design:gamma:gradient', 'Gradient for eigenvalues was not supplied.');
	end
	for jj = 1:size(objective_type, 1) %#ok<FORPF> number of objective functions is usually smaller than number of models, so parfor is used for models
		Jgrad_objective = gradtemp;
		Jgrad_xdot_objective = gradtemp_xdot;
		parfor (ii = 1:number_models, numthreads)
			tempgradJ = zeros(number_controls, number_measurements);
			tempgradJ_xdot = zeros(number_controls, number_measurements_xdot);
			tempareaval_derivative = areaval_derivative(:, :, :, ii);
			for kk = 1:number_states
				if number_controls == 1
					% matlab transposes NxPx1xM and Nx1xQxM matrices
					if number_measurements == 1
						re = real(squeeze(eigenvalue_derivative(kk, :, :, ii)));
						im = imag(squeeze(eigenvalue_derivative(kk, :, :, ii)));
					else
						re = real(squeeze(eigenvalue_derivative(kk, :, :, ii))).';
						im = imag(squeeze(eigenvalue_derivative(kk, :, :, ii))).';
					end
				else
					if number_measurements == 1
						re = real(squeeze(eigenvalue_derivative(kk, :, :, ii))).';
						im = imag(squeeze(eigenvalue_derivative(kk, :, :, ii))).';
					else
						re = real(squeeze(eigenvalue_derivative(kk, :, :, ii)));
						im = imag(squeeze(eigenvalue_derivative(kk, :, :, ii)));
					end
				end
				if derivative_feedback && ~isempty(eigenvalue_derivative_xdot)
					if number_controls == 1
						% matlab transposes NxPx1xM and Nx1xQxM matrices
						if number_measurements_xdot == 1
							re_xdot = real(squeeze(eigenvalue_derivative_xdot(kk, :, :, ii)));
							im_xdot = imag(squeeze(eigenvalue_derivative_xdot(kk, :, :, ii)));
						else
							re_xdot = real(squeeze(eigenvalue_derivative_xdot(kk, :, :, ii))).';
							im_xdot = imag(squeeze(eigenvalue_derivative_xdot(kk, :, :, ii))).';
						end
					else
						if number_measurements_xdot == 1
							re_xdot = real(squeeze(eigenvalue_derivative_xdot(kk, :, :, ii))).';
							im_xdot = imag(squeeze(eigenvalue_derivative_xdot(kk, :, :, ii))).';
						else
							re_xdot = real(squeeze(eigenvalue_derivative_xdot(kk, :, :, ii)));
							im_xdot = imag(squeeze(eigenvalue_derivative_xdot(kk, :, :, ii)));
						end
					end
				else
					re_xdot = zeros(number_controls, number_measurements_xdot);
					im_xdot = zeros(number_controls, number_measurements_xdot);
				end
				for ll = 1:number_areas_max
					if any(isnan(tempareaval_derivative(kk, ll, :, 1)))
						continue;
					end
					if weight(ii, ll) ~= 0
						tempgradJ_add = Jtempgrad(jj, ii, ll, kk)*weight(ii, ll)*(tempareaval_derivative(kk, ll, 1, 1)*re + tempareaval_derivative(kk, ll, 2, 1)*im);
						if preventNaN
							nanJ = isnan(tempgradJ_add);
							if any(nanJ(:))
								if isinf(Jtempgrad(jj, ii, ll, kk))
									tempgradJ_add(nanJ) = Jtempgrad(jj, ii, ll, kk)*weight(ii, ll);
								else
									tempgradJ_add(nanJ) = weight(ii, ll)*(tempareaval_derivative(kk, ll, 1, 1)*re(nanJ) + tempareaval_derivative(kk, ll, 2, 1)*im(nanJ));
								end
							end
							% prevent -Inf + Inf = NaN
							tempgradJ_add(isinf(tempgradJ) & isinf(tempgradJ_add) & (tempgradJ > 0 & tempgradJ_add < 0 | tempgradJ < 0 & tempgradJ_add > 0)) = 0;
						end
						tempgradJ = tempgradJ + tempgradJ_add;
						if derivative_feedback
							tempgradJ_add_xdot = Jtempgrad(jj, ii, ll, kk)*weight(ii, ll)*(tempareaval_derivative(kk, ll, 1, 1)*re_xdot + tempareaval_derivative(kk, ll, 2, 1)*im_xdot);
							if preventNaN
								nanJ_xdot = isnan(tempgradJ_add_xdot);
								if any(nanJ_xdot(:))
									% TODO: result depends on objective type and limit value
									if isinf(Jtempgrad(jj, ii, ll, kk))
										tempgradJ_add_xdot(nanJ_xdot) = Jtempgrad(jj, ii, ll, kk)*weight(ii, ll);
									else
										tempgradJ_add_xdot(nanJ_xdot) = weight(ii, ll)*(tempareaval_derivative(kk, ll, 1, 1)*re_xdot(nanJ_xdot) + tempareaval_derivative(kk, ll, 2, 1)*im_xdot(nanJ_xdot));
									end
								end
								% prevent -Inf + Inf = NaN
								tempgradJ_add_xdot(isinf(tempgradJ_xdot) & isinf(tempgradJ_add_xdot) & (tempgradJ_xdot > 0 & tempgradJ_add_xdot < 0 | tempgradJ_xdot < 0 & tempgradJ_add_xdot > 0)) = 0;
							end
							tempgradJ_xdot = tempgradJ_xdot + tempgradJ_add_xdot;
						end
					end
				end
			end
			Jgrad_objective(:, :, ii) = tempgradJ;
			if derivative_feedback
				Jgrad_xdot_objective(:, :, ii) = tempgradJ_xdot;
			end
		end
		isinfoppositesignneg = isinf(gradtemp) & gradtemp > 0 & isinf(Jgrad_objective) & Jgrad_objective <= 0;
		isinfoppositesignpos = isinf(gradtemp) & gradtemp <= 0 & isinf(Jgrad_objective) & Jgrad_objective > 0;
		if any(isinfoppositesignneg(:))
			% avoid Inf + -Inf = NaN
			Jgrad_objective(isinfoppositesignneg) = Inf;
		elseif any(isinfoppositesignpos(:))
			% avoid -Inf + Inf = NaN
			Jgrad_objective(isinfoppositesignpos) = -Inf;
		end
		gradtemp = gradtemp + objective_weight(jj, 1)*Jgrad_objective;
		if derivative_feedback
			isinfoppositesignneg_xdot = isinf(gradtemp_xdot) & gradtemp_xdot > 0 & isinf(Jgrad_xdot_objective) & Jgrad_xdot_objective <= 0;
			isinfoppositesignpos_xdot = isinf(gradtemp_xdot) & gradtemp_xdot <= 0 & isinf(Jgrad_xdot_objective) & Jgrad_xdot_objective > 0;
			if any(isinfoppositesignneg_xdot(:))
				% avoid Inf + -Inf = NaN
				Jgrad_xdot_objective(isinfoppositesignneg_xdot) = Inf;
			elseif any(isinfoppositesignneg_xdot(:))
				% avoid -Inf + Inf = NaN
				Jgrad_xdot_objective(isinfoppositesignpos_xdot) = -Inf;
			end
			gradtemp_xdot = gradtemp_xdot + objective_weight(jj, 1)*Jgrad_xdot_objective;
		end
	end
	isinfoppositesign = any(isinf(gradtemp) & gradtemp > 0, 3) & any(isinf(gradtemp) & gradtemp <= 0, 3);
	if derivative_feedback
		isinfoppositesign_xdot = any(isinf(gradtemp_xdot) & gradtemp_xdot > 0, 3) & any(isinf(gradtemp_xdot) & gradtemp_xdot <= 0, 3);
	else
		isinfoppositesign_xdot = false(size(gradtemp_xdot, 1), size(gradtemp_xdot, 2));
	end
	% codegen crashes when concatenating empty variable size matrices
	if derivative_feedback
		sumgradtemp_xdot = sum(gradtemp_xdot, 3);
		sumgradtemp_xdot(isinfoppositesign_xdot) = Inf;
		if isempty(gradtemp_prefilter)
			if isempty(gradtemp)
				gradJ = sumgradtemp_xdot;
			else
				sumgradtemp = sum(gradtemp, 3);
				sumgradtemp(isinfoppositesign) = Inf;
				gradJ = cat(2, sumgradtemp, sumgradtemp_xdot);
			end
		else
			if isempty(gradtemp)
				gradJ = cat(2, sumgradtemp_xdot, sum(gradtemp_prefilter, 3));
			else
				sumgradtemp = sum(gradtemp, 3);
				sumgradtemp(isinfoppositesign) = Inf;
				gradJ = cat(2, sumgradtemp, sumgradtemp_xdot, sum(gradtemp_prefilter, 3));
			end
		end
	else
		if isempty(gradtemp_prefilter)
			sumgradtemp = sum(gradtemp, 3);
			sumgradtemp(isinfoppositesign) = Inf;
			gradJ = sumgradtemp;
		else
			if isempty(gradtemp)
				gradJ = sum(gradtemp_prefilter, 3);
			else
				sumgradtemp = sum(gradtemp, 3);
				sumgradtemp(isinfoppositesign) = Inf;
				gradJ = cat(2, sumgradtemp, sum(gradtemp_prefilter, 3));
			end
		end
	end
end

function [hessianJ] = calculate_objective_hesse_helper(number_models, number_states, number_controls, number_measurements, number_measurements_xdot, number_references, number_areas_max, eigenvalue_derivative, eigenvalue_derivative_xdot, eigenvalue_2derivative, eigenvalue_2derivative_xdot, eigenvalue_2derivative_mixed, eigenvalue_2derivative_xdot_mixed, areaval_derivative, areaval_2_derivative, weight, objective_type, objective_weight, Jtempgrad, Jtemphesse, preventNaN, options)
	objective_settings_KREISSELMEIER_rho = options.objective.kreisselmeier.rho;
	number_R_coefficients = number_controls*number_measurements;% number of proportional parameters
	number_K_coefficients = number_controls*number_measurements_xdot; % number of derivative parameters
	number_gain_coefficients = number_R_coefficients + number_K_coefficients; % number of parameters
	hessianJ_RK = zeros(number_gain_coefficients, number_gain_coefficients);
	for jj = 1:size(objective_type, 1) %#ok<FORPF> number of objective functions is usually smaller than number of models, so parfor is used for models
		if objective_weight(jj, 1) == 0
			continue;
		end
		Jhesse_objective = NaN(number_gain_coefficients, number_gain_coefficients);
		iskreisselmeier = objective_type(jj, 1) == GammaJType.KREISSELMEIER;
		for z = 1:number_gain_coefficients % first control parameter
			for q = 1:number_gain_coefficients % second control parameter
				% controller indices
				if z <= number_R_coefficients
					i = mod(z - 1,number_controls) + 1;
					j = idivide(z - 1, number_controls) + 1;
				else
					tmpZ = z - number_R_coefficients;
					i = mod(tmpZ - 1, number_controls) + 1;
					j = idivide(tmpZ - 1, number_controls) + 1;
				end

				if q <= number_R_coefficients
					s = mod(q - 1, number_controls) + 1;
					t = idivide(q - 1, number_controls) + 1;
				else
					tmpQ = q - number_R_coefficients;
					s = mod(tmpQ - 1, number_controls) + 1;
					t = idivide(tmpQ - 1, number_controls) + 1;
				end
				J_2_grad_temp = 0;
				J_2_grad_temp_kreisselmeier = zeros(2, 1);
				% TODO: parfor?
				for ii = 1:number_models
					for kk = 1:number_states
						% read eigenvalue derivatives
						if z <= number_R_coefficients
							if q <= number_R_coefficients
								eigDerivative_ij = eigenvalue_derivative(kk, i, j, ii);
								eigDerivative_st = eigenvalue_derivative(kk, s, t, ii);

								eig2Derivative_re = real(eigenvalue_2derivative(kk, q, z, ii));
								eig2Derivative_im = imag(eigenvalue_2derivative(kk, q, z, ii));
							else
								eigDerivative_ij = eigenvalue_derivative(kk, i, j, ii);
								eigDerivative_st = eigenvalue_derivative_xdot(kk, s, t, ii);

								eig2Derivative_re = real(eigenvalue_2derivative_mixed(kk, q - number_R_coefficients, z, ii));
								eig2Derivative_im = imag(eigenvalue_2derivative_mixed(kk, q - number_R_coefficients, z, ii));
							end
						else
							if q <= number_R_coefficients
								eigDerivative_ij = eigenvalue_derivative_xdot(kk, i, j, ii);
								eigDerivative_st = eigenvalue_derivative(kk, s, t, ii);

								eig2Derivative_re = real(eigenvalue_2derivative_xdot_mixed(kk, q, z - number_R_coefficients, ii));
								eig2Derivative_im = imag(eigenvalue_2derivative_xdot_mixed(kk, q, z - number_R_coefficients, ii));
							else
								eigDerivative_ij = eigenvalue_derivative_xdot(kk, i, j, ii);
								eigDerivative_st = eigenvalue_derivative_xdot(kk, s, t, ii);

								eig2Derivative_re = real(eigenvalue_2derivative_xdot(kk, q - number_R_coefficients, z - number_R_coefficients, ii));
								eig2Derivative_im = imag(eigenvalue_2derivative_xdot(kk, q - number_R_coefficients, z - number_R_coefficients, ii));
							end
						end
						if isnan(eigDerivative_ij) || isnan(eigDerivative_st) || isnan(eig2Derivative_re) || isnan(eig2Derivative_im)
							continue;
						end
						for ll = 1:number_areas_max
							dfdre = areaval_derivative(kk, ll, 1, ii);
							dfdim = areaval_derivative(kk, ll, 2, ii);
							d2fdredre = areaval_2_derivative(kk, ll, 1, ii);
							d2fdimdre = areaval_2_derivative(kk, ll, 2, ii);
							d2fdredim = areaval_2_derivative(kk, ll, 3, ii);
							d2fdimdim = areaval_2_derivative(kk, ll, 4, ii);
							%weighted_Jtemphesse = zeros(size(Jtemphesse(jj, ii, ll, kk)));
							switch objective_type(jj, 1)
								case GammaJType.ZERO
									% TODO: replace by zeros(...) to prevent 0*Inf = NaN?
									weighted_Jtemphesse = 0*Jtemphesse(jj, ii, ll, kk);
								case GammaJType.LINEAR
									weighted_Jtemphesse = 0*Jtemphesse(jj, ii, ll, kk);
								case GammaJType.SQUARE
									weighted_Jtemphesse = weight(ii, ll)^2*Jtemphesse(jj, ii, ll, kk);
								case GammaJType.SQUAREPENALTY
									weighted_Jtemphesse = weight(ii, ll)^2*Jtemphesse(jj, ii, ll, kk);
								case GammaJType.CUBIC
									weighted_Jtemphesse = weight(ii, ll)^2*Jtemphesse(jj, ii, ll, kk);
								case GammaJType.EXP
									weighted_Jtemphesse = weight(ii, ll)^2*Jtemphesse(jj, ii, ll, kk);
								case GammaJType.LOG
									weighted_Jtemphesse = weight(ii, ll)^2*Jtemphesse(jj, ii, ll, kk);
								case GammaJType.MAX
									weighted_Jtemphesse = 0*Jtemphesse(jj, ii, ll, kk);
								case GammaJType.KREISSELMEIER
									weighted_Jtemphesse = weight(ii, ll)^2*Jtemphesse(jj, ii, ll, kk);
								case GammaJType.EIGENVALUECONDITION
									% TODO: case GammaJType.EIGENVALUECONDITION
									error('control:design:gamma:hessian', 'Hessian for eigenvector matrix condition objective not yet implemented.');
								case GammaJType.NORMGAIN
									weighted_Jtemphesse = zeros(size(Jtemphesse(jj, ii, ll, kk)));
								case GammaJType.LYAPUNOV
									% TODO: case GammaJType.LYAPUNOV
									error('control:design:gamma:hessian', 'Hessian for Lyapunov objective not yet implemented.');
								otherwise
									weighted_Jtemphesse = zeros(size(Jtemphesse(jj, ii, ll, kk)));
							end
							tmpVal_parts = [
								weighted_Jtemphesse*(dfdre*real(eigDerivative_st) + dfdim*imag(eigDerivative_st))*(dfdre*real(eigDerivative_ij) + dfdim*imag(eigDerivative_ij));
								Jtempgrad(jj, ii, ll, kk)*((d2fdredre*real(eigDerivative_st) + d2fdimdre*imag(eigDerivative_st))*real(eigDerivative_ij) + dfdre*eig2Derivative_re + (d2fdredim*real(eigDerivative_st) + d2fdimdim*imag(eigDerivative_st))*imag(eigDerivative_ij) + dfdim*eig2Derivative_im)
							];
							if preventNaN
								nanJ = isnan(tmpVal_parts);
								% TODO: result depends on objective type and limit value
								if nanJ(1)
									if isinf(weighted_Jtemphesse)
										tmpVal_parts(1) = weighted_Jtemphesse;
									else
										tmpVal_parts(1) = (dfdre*real(eigDerivative_st) + dfdim*imag(eigDerivative_st))*(dfdre*real(eigDerivative_ij) + dfdim*imag(eigDerivative_ij));
									end
								end
								if nanJ(2)
									if isinf(Jtempgrad(jj, ii, ll, kk))
										tmpVal_parts(2) = Jtempgrad(jj, ii, ll, kk);
									else
										tmpVal_parts(2) = (dfdre*real(eigDerivative_st) + dfdim*imag(eigDerivative_st))*(dfdre*real(eigDerivative_ij) + dfdim*imag(eigDerivative_ij));
									end
								end
							end
							if all(isinf(tmpVal_parts))
								if tmpVal_parts(1) > 0 && tmpVal_parts(2) < 0 || tmpVal_parts(1) < 0 && tmpVal_parts(2) > 0
									tmpVal_parts(:) = Inf;
								end
							end
							tmpVal = sum(tmpVal_parts(:));
							if iskreisselmeier
								tmpVal_kreisselmeier = [
									Jtempgrad(jj, ii, ll, kk)*weight(ii, ll)*(dfdre*real(eigDerivative_st) + dfdim*imag(eigDerivative_st));
									Jtempgrad(jj, ii, ll, kk)*weight(ii, ll)*(dfdre*real(eigDerivative_ij) + dfdim*imag(eigDerivative_ij))
								];
								if isinf(tmpVal_kreisselmeier(1)) && tmpVal_kreisselmeier(1) > 0 && isinf(J_2_grad_temp_kreisselmeier(1)) && J_2_grad_temp_kreisselmeier(1) <= 0
									% prevent -Inf + Inf = NaN
								elseif isinf(tmpVal_kreisselmeier(1)) && tmpVal_kreisselmeier(1) <= 0 && isinf(J_2_grad_temp_kreisselmeier(1)) && J_2_grad_temp_kreisselmeier(1) > 0
									% prevent Inf + -Inf = NaN
								else
									J_2_grad_temp_kreisselmeier(1) = J_2_grad_temp_kreisselmeier(1) + tmpVal_kreisselmeier(1);
								end
								if isinf(tmpVal_kreisselmeier(2)) && tmpVal_kreisselmeier(2) > 0 && isinf(J_2_grad_temp_kreisselmeier(2)) && J_2_grad_temp_kreisselmeier(2) <= 0
									% prevent -Inf + Inf = NaN
								elseif isinf(tmpVal_kreisselmeier(2)) && tmpVal_kreisselmeier(2) <= 0 && isinf(J_2_grad_temp_kreisselmeier(2)) && J_2_grad_temp_kreisselmeier(2) > 0
									% prevent Inf + -Inf = NaN
								else
									J_2_grad_temp_kreisselmeier(2) = J_2_grad_temp_kreisselmeier(2) + tmpVal_kreisselmeier(2);
								end
							end
							if isinf(tmpVal) && tmpVal > 0 && isinf(J_2_grad_temp) && J_2_grad_temp <= 0
								% prevent -Inf + Inf = NaN
							elseif isinf(tmpVal) && tmpVal <= 0 && isinf(J_2_grad_temp) && J_2_grad_temp > 0
								% prevent Inf + -Inf = NaN
							else
								J_2_grad_temp = J_2_grad_temp + tmpVal;
							end
						end
					end
				end
				Jhesse_objective(z, q) = J_2_grad_temp - J_2_grad_temp_kreisselmeier(1)*J_2_grad_temp_kreisselmeier(2)*objective_settings_KREISSELMEIER_rho;
			end
		end
		if objective_weight(jj, 1) ~= 0
			hessianJ_RK = hessianJ_RK + objective_weight(jj, 1)*Jhesse_objective;
		end
	end
	if number_references ~= 0
		hessianJ = blkdiag(hessianJ_RK, zeros(number_controls*number_references, number_controls*number_references));
	else
		hessianJ = hessianJ_RK;
	end
end
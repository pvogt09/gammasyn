function [areaval, areaval_derivative, areaval_2_derivative] = calculate_areas_fixed_m(areafun, weight, eigenvalues, dimensions, numthreads, eigenvalueignoreinf)
	%CALCULATE_AREAS_FIXED_M helper function for calculation of border function, gradient and hessian values for gamma pole placement for use with generated code
	%	Input:
	%		areafun:				border functions as cell array of function handles or matrix of GammaArea objects
	%		weight:					weighting matrix with number of systems columns and number of pole area border functions rows
	%		eigenvalues:			eigenvalues used of all systems
	%		dimensions:				structure with information about dimensions of the different variables and systems
	%		numthreads:				number of threads to run loops in
	%		eigenvalueignoreinf:	indicator wheter infinite eigenvalues should be ignored
	%	Output:
	%		areaval:				area border function value for current optimization value
	%		areaval_derivative:		gradient of area border function for current optimization value
	%		areaval_2_derivative:	hessian of area border function for current optimization value
	numthreads = uint32(floor(max([0, numthreads])));
	number_models = dimensions.models;
	number_states = dimensions.states;
	number_areas_max = dimensions.areas_max;
	parameters = dimensions.area_parameters;
	isemptyareafun = isempty(areafun);
	if isemptyareafun
		areaval = zeros(number_models, number_areas_max, number_states);
		if nargout >= 2
			areaval_derivative = zeros(number_states, number_areas_max, 2, number_models);
			if nargout >= 3
				areaval_2_derivative = zeros(number_states, number_areas_max, 4, number_models);
			end
		end
		return;
	else
		areaval = NaN(number_models, number_areas_max, number_states);
		if nargout >= 2
			areaval_derivative = NaN(number_states, number_areas_max, 2, number_models);
			if nargout >= 3
				areaval_2_derivative = NaN(number_states, number_areas_max, 4, number_models);
			end
		end
	end
	needsgradient = nargout >= 2;
	needshessian = nargout >= 3;
	parfor (ii = 1:number_models, numthreads)
		for kk = 1:number_states
			if isnan(eigenvalues(kk, ii))
				continue;
			end
			if eigenvalueignoreinf && isinf(eigenvalues(kk, ii))
				signedinf = -sign(weight(ii, :)).*Inf(1, number_areas_max);
				signedinf(isnan(signedinf)) = 0;% handle weight = 0
				areaval(ii, :, kk) = signedinf;
				if needsgradient
					areaval_derivative(kk, :, :, ii) = zeros(1, number_areas_max, 2);
					if needshessian
						areaval_2_derivative(kk, :, :, ii) = zeros(number_areas_max, 4);
					end
				end
				continue;
			end
			if needshessian
				areavaltemp = zeros(1, number_areas_max);
				derivative_deltatemp = zeros(1, number_areas_max);
				derivative_omegatemp = zeros(1, number_areas_max);
				derivate_2_deltadelta_temp = zeros(1, number_areas_max);
				derivate_2_omegadelta_temp = zeros(1, number_areas_max);
				derivate_2_deltaomega_temp = zeros(1, number_areas_max);
				derivate_2_omegaomega_temp = zeros(1, number_areas_max);
				if isemptyareafun
				else
					for ll = 1:number_areas_max
						switch areafun(ii, ll)
							case GammaArea.CIRCLE
								[areavaltemp(1, ll), derivative_deltatemp(1, ll), derivative_omegatemp(1, ll), derivate_2_deltadelta_temp(1, ll), derivate_2_omegadelta_temp(1, ll), derivate_2_deltaomega_temp(1, ll), derivate_2_omegaomega_temp(1, ll)] = control.design.gamma.area.Circle_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.CIRCLESQUARE
								[areavaltemp(1, ll), derivative_deltatemp(1, ll), derivative_omegatemp(1, ll), derivate_2_deltadelta_temp(1, ll), derivate_2_omegadelta_temp(1, ll), derivate_2_deltaomega_temp(1, ll), derivate_2_omegaomega_temp(1, ll)] = control.design.gamma.area.Circlesquare_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.CIRCLEDISCRETE
								[areavaltemp(1, ll), derivative_deltatemp(1, ll), derivative_omegatemp(1, ll), derivate_2_deltadelta_temp(1, ll), derivate_2_omegadelta_temp(1, ll), derivate_2_deltaomega_temp(1, ll), derivate_2_omegaomega_temp(1, ll)] = control.design.gamma.area.CircleDiscrete_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.ELLIPSE
								[areavaltemp(1, ll), derivative_deltatemp(1, ll), derivative_omegatemp(1, ll), derivate_2_deltadelta_temp(1, ll), derivate_2_omegadelta_temp(1, ll), derivate_2_deltaomega_temp(1, ll), derivate_2_omegaomega_temp(1, ll)] = control.design.gamma.area.Ellipse_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.ELLIPSESQUARE
								[areavaltemp(1, ll), derivative_deltatemp(1, ll), derivative_omegatemp(1, ll), derivate_2_deltadelta_temp(1, ll), derivate_2_omegadelta_temp(1, ll), derivate_2_deltaomega_temp(1, ll), derivate_2_omegaomega_temp(1, ll)] = control.design.gamma.area.Ellipsesquare_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.HYPERBOLA
								[areavaltemp(1, ll), derivative_deltatemp(1, ll), derivative_omegatemp(1, ll), derivate_2_deltadelta_temp(1, ll), derivate_2_omegadelta_temp(1, ll), derivate_2_deltaomega_temp(1, ll), derivate_2_omegaomega_temp(1, ll)] = control.design.gamma.area.Hyperbola_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.HYPERBOLASQUARE
								[areavaltemp(1, ll), derivative_deltatemp(1, ll), derivative_omegatemp(1, ll), derivate_2_deltadelta_temp(1, ll), derivate_2_omegadelta_temp(1, ll), derivate_2_deltaomega_temp(1, ll), derivate_2_omegaomega_temp(1, ll)] = control.design.gamma.area.Hyperbolasquare_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.IMAG
								[areavaltemp(1, ll), derivative_deltatemp(1, ll), derivative_omegatemp(1, ll), derivate_2_deltadelta_temp(1, ll), derivate_2_omegadelta_temp(1, ll), derivate_2_deltaomega_temp(1, ll), derivate_2_omegaomega_temp(1, ll)] = control.design.gamma.area.Imag_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.LINE
								[areavaltemp(1, ll), derivative_deltatemp(1, ll), derivative_omegatemp(1, ll), derivate_2_deltadelta_temp(1, ll), derivate_2_omegadelta_temp(1, ll), derivate_2_deltaomega_temp(1, ll), derivate_2_omegaomega_temp(1, ll)] = control.design.gamma.area.Line_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.LOGSPIRAL
								[areavaltemp(1, ll), derivative_deltatemp(1, ll), derivative_omegatemp(1, ll), derivate_2_deltadelta_temp(1, ll), derivate_2_omegadelta_temp(1, ll), derivate_2_deltaomega_temp(1, ll), derivate_2_omegaomega_temp(1, ll)] = control.design.gamma.area.LogSpiral_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.POLYELLIPSE
								[areavaltemp(1, ll), derivative_deltatemp(1, ll), derivative_omegatemp(1, ll), derivate_2_deltadelta_temp(1, ll), derivate_2_omegadelta_temp(1, ll), derivate_2_deltaomega_temp(1, ll), derivate_2_omegaomega_temp(1, ll)] = control.design.gamma.area.PolyEllipse_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.POLYELLIPSESQUARE
								[areavaltemp(1, ll), derivative_deltatemp(1, ll), derivative_omegatemp(1, ll), derivate_2_deltadelta_temp(1, ll), derivate_2_omegadelta_temp(1, ll), derivate_2_deltaomega_temp(1, ll), derivate_2_omegaomega_temp(1, ll)] = control.design.gamma.area.PolyEllipsesquare_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.NONE
								[areavaltemp(1, ll), derivative_deltatemp(1, ll), derivative_omegatemp(1, ll), derivate_2_deltadelta_temp(1, ll), derivate_2_omegadelta_temp(1, ll), derivate_2_deltaomega_temp(1, ll), derivate_2_omegaomega_temp(1, ll)] = control.design.gamma.area.None_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.CUSTOM
								areavaltemp(1, ll) = NaN;
								derivative_deltatemp(1, ll) = NaN;
								derivative_omegatemp(1, ll) = NaN;
								derivate_2_deltadelta_temp(1, ll) = NaN;
								derivate_2_omegadelta_temp(1, ll) = NaN;
								derivate_2_deltaomega_temp(1, ll)= NaN;
								derivate_2_omegaomega_temp(1, ll) = NaN;
							otherwise
								error('control:design:gamma:area', 'Undefined pole area.');
						end
					end
				end
				areaval(ii, :, kk) = weight(ii, :).*areavaltemp;
				areaval_derivative(kk, :, :, ii) = [
					derivative_deltatemp;
					derivative_omegatemp
				]';
				areaval_2_derivative(kk, :, :, ii) = [
					derivate_2_deltadelta_temp;
					derivate_2_omegadelta_temp;
					derivate_2_deltaomega_temp;
					derivate_2_omegaomega_temp
				]';
			elseif needsgradient
				areavaltemp = zeros(1, number_areas_max);
				derivative_deltatemp = zeros(1, number_areas_max);
				derivative_omegatemp = zeros(1, number_areas_max);
				if isemptyareafun
				else
					for ll = 1:number_areas_max
						switch areafun(ii, ll)
							case GammaArea.CIRCLE
								[areavaltemp(1, ll), derivative_deltatemp(1, ll), derivative_omegatemp(1, ll)] = control.design.gamma.area.Circle_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.CIRCLESQUARE
								[areavaltemp(1, ll), derivative_deltatemp(1, ll), derivative_omegatemp(1, ll)] = control.design.gamma.area.Circlesquare_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.CIRCLEDISCRETE
								[areavaltemp(1, ll), derivative_deltatemp(1, ll), derivative_omegatemp(1, ll)] = control.design.gamma.area.CircleDiscrete_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.ELLIPSE
								[areavaltemp(1, ll), derivative_deltatemp(1, ll), derivative_omegatemp(1, ll)] = control.design.gamma.area.Ellipse_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.ELLIPSESQUARE
								[areavaltemp(1, ll), derivative_deltatemp(1, ll), derivative_omegatemp(1, ll)] = control.design.gamma.area.Ellipsesquare_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.HYPERBOLA
								[areavaltemp(1, ll), derivative_deltatemp(1, ll), derivative_omegatemp(1, ll)] = control.design.gamma.area.Hyperbola_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.HYPERBOLASQUARE
								[areavaltemp(1, ll), derivative_deltatemp(1, ll), derivative_omegatemp(1, ll)] = control.design.gamma.area.Hyperbolasquare_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.IMAG
								[areavaltemp(1, ll), derivative_deltatemp(1, ll), derivative_omegatemp(1, ll)] = control.design.gamma.area.Imag_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.LINE
								[areavaltemp(1, ll), derivative_deltatemp(1, ll), derivative_omegatemp(1, ll)] = control.design.gamma.area.Line_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.LOGSPIRAL
								[areavaltemp(1, ll), derivative_deltatemp(1, ll), derivative_omegatemp(1, ll)] = control.design.gamma.area.LogSpiral_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.POLYELLIPSE
								[areavaltemp(1, ll), derivative_deltatemp(1, ll), derivative_omegatemp(1, ll)] = control.design.gamma.area.PolyEllipse_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.POLYELLIPSESQUARE
								[areavaltemp(1, ll), derivative_deltatemp(1, ll), derivative_omegatemp(1, ll)] = control.design.gamma.area.PolyEllipsesquare_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.NONE
								[areavaltemp(1, ll), derivative_deltatemp(1, ll), derivative_omegatemp(1, ll)] = control.design.gamma.area.None_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.CUSTOM
								areavaltemp(1, ll) = NaN;
								derivative_deltatemp(1, ll) = NaN;
								derivative_omegatemp(1, ll) = NaN;
							otherwise
								error('control:design:gamma:area', 'Undefined pole area.');
						end
					end
				end
				areaval(ii, :, kk) = weight(ii, :).*areavaltemp;
				areaval_derivative(kk, :, :, ii) = [
					derivative_deltatemp;
					derivative_omegatemp
				]';
			else
				areavaltemp = zeros(1, number_areas_max);
				if isemptyareafun
				else
					for ll = 1:number_areas_max
						switch areafun(ii, ll)
							case GammaArea.CIRCLE
								areavaltemp(1, ll) = control.design.gamma.area.Circle_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.CIRCLESQUARE
								areavaltemp(1, ll) = control.design.gamma.area.Circlesquare_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.CIRCLEDISCRETE
								areavaltemp(1, ll) = control.design.gamma.area.CircleDiscrete_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.ELLIPSE
								areavaltemp(1, ll) = control.design.gamma.area.Ellipse_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.ELLIPSESQUARE
								areavaltemp(1, ll) = control.design.gamma.area.Ellipsesquare_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.HYPERBOLA
								areavaltemp(1, ll) = control.design.gamma.area.Hyperbola_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.HYPERBOLASQUARE
								areavaltemp(1, ll) = control.design.gamma.area.Hyperbolasquare_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.IMAG
								areavaltemp(1, ll) = control.design.gamma.area.Imag_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.LINE
								areavaltemp(1, ll) = control.design.gamma.area.Line_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.LOGSPIRAL
								areavaltemp(1, ll) = control.design.gamma.area.LogSpiral_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.POLYELLIPSE
								areavaltemp(1, ll) = control.design.gamma.area.PolyEllipse_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.POLYELLIPSESQUARE
								areavaltemp(1, ll) = control.design.gamma.area.PolyEllipsesquare_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.NONE
								areavaltemp(1, ll) = control.design.gamma.area.None_border(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)), parameters(ii, ll));
							case GammaArea.CUSTOM
								areavaltemp(1, ll) = NaN;
							otherwise
								error('control:design:gamma:area', 'Undefined pole area.');
						end
					end
				end
				areaval(ii, :, kk) = weight(ii, :).*areavaltemp;
			end
		end
	end
end
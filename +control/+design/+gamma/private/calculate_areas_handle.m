function [areaval, areaval_derivative, areaval_2_derivative] = calculate_areas_handle(areafun, weight, eigenvalues, dimensions, numthreads, eigenvalueignoreinf)
	%CALCULATE_AREAS_HANDLE helper function for calculation of border function, gradient and hessian values for gamma pole placement for use with not generated code since function handles are not supported for code generation
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
	if nargin <= 4
		numthreads = configuration.matlab.numthreads();
	end
	numthreads = uint32(floor(max([0, numthreads])));
	number_models = dimensions.models;
	number_states = dimensions.states;
	number_areas_max = dimensions.areas_max;
	number_areaargs = dimensions.area_args;
	area_hasgrad = dimensions.area_hasgrad;
	area_hashess = dimensions.area_hashess;
	area_parts = dimensions.area_parts;
	areaval = NaN(number_models, number_areas_max, number_states);
	areaval_derivative = NaN(number_states, number_areas_max, 2, number_models);
	areaval_2_derivative = NaN(number_states, number_areas_max, 4, number_models);
	for ii = 1:number_models
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
			z = zeros(number_areas_max - area_parts(ii), 1);
			if area_hashess && nargout >= 3
				if number_areaargs(ii) == 1
					[areavaltemp, areaval_derivative_deltatemp, areaval_derivative_omegatemp, areaval_2_derivative_deltadeltatemp, areaval_2_derivative_deltaomegatemp, areaval_2_derivative_omegadeltatemp, areaval_2_derivative_omegaomegatemp] = areafun{ii}(eigenvalues(kk, ii));
					areaval(ii, :, kk) = weight(ii, :).*[
						areavaltemp, z
					];
					areaval_derivative(kk, :, 1, ii) = [
						areaval_derivative_deltatemp.';
						z
					];
					areaval_derivative(kk, :, 2, ii) = [
						areaval_derivative_omegatemp.';
						z
					];
					areaval_2_derivative(kk, :, 1, ii) = [
						areaval_2_derivative_deltadeltatemp.';
						z
					];
					areaval_2_derivative(kk, :, 2, ii) = [
						areaval_2_derivative_deltaomegatemp.';
						z
					];
					areaval_2_derivative(kk, :, 3, ii) = [
						areaval_2_derivative_omegadeltatemp.';
						z
					];
					areaval_2_derivative(kk, :, 4, ii) = [
						areaval_2_derivative_omegaomegatemp.';
						z
					];
				else
					[areavaltemp, areaval_derivative_deltatemp, areaval_derivative_omegatemp, areaval_2_derivative_deltadeltatemp, areaval_2_derivative_deltaomegatemp, areaval_2_derivative_omegadeltatemp, areaval_2_derivative_omegaomegatemp] = areafun{ii}(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)));
					areaval(ii, :, kk) = weight(ii, :).*[
						areavaltemp, z
					];
					areaval_derivative(kk, :, 1, ii) = [
						areaval_derivative_deltatemp.';
						z
					];
					areaval_derivative(kk, :, 2, ii) = [
						areaval_derivative_omegatemp.';
						z
					];
					areaval_2_derivative(kk, :, 1, ii) = [
						areaval_2_derivative_deltadeltatemp.';
						z
					];
					areaval_2_derivative(kk, :, 2, ii) = [
						areaval_2_derivative_deltaomegatemp.';
						z
					];
					areaval_2_derivative(kk, :, 3, ii) = [
						areaval_2_derivative_omegadeltatemp.';
						z
					];
					areaval_2_derivative(kk, :, 4, ii) = [
						areaval_2_derivative_omegaomegatemp.';
						z
					];
				end
			elseif area_hasgrad && nargout >= 2
				if number_areaargs(ii) == 1
					[areavaltemp, areaval_derivative_deltatemp, areaval_derivative_omegatemp] = areafun{ii}(eigenvalues(kk, ii));
					areaval(ii, :, kk) = weight(ii, :).*[
						areavaltemp, z
					];
					areaval_derivative(kk, :, 1, ii) = [
						areaval_derivative_deltatemp.';
						z
					];
					areaval_derivative(kk, :, 2, ii) = [
						areaval_derivative_omegatemp.';
						z
					];
				else
					[areavaltemp, areaval_derivative_deltatemp, areaval_derivative_omegatemp] = areafun{ii}(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii)));
					areaval(ii, :, kk) = weight(ii, :).*[
						areavaltemp, z
					];
					areaval_derivative(kk, :, 1, ii) = [
						areaval_derivative_deltatemp.';
						z
					];
					areaval_derivative(kk, :, 2, ii) = [
						areaval_derivative_omegatemp.';
						z
					];
				end
			else
				if number_areaargs(ii) == 1
					areaval(ii, :, kk) = weight(ii, :).*[
						areafun{ii}(eigenvalues(kk, ii)), z
					];
				else
					areaval(ii, :, kk) = [
						areafun{ii}(real(eigenvalues(kk, ii)), imag(eigenvalues(kk, ii))), z
					];
				end
			end
		end
	end
end
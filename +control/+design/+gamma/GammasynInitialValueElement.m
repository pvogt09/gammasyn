classdef GammasynInitialValueElement < control.design.gamma.InitialValueElement
	%GAMMASYNINITIALVALUEELEMENT class for representing an initial value to use with gammasyn

	properties
		% initial value
		x_init
	end

	methods
		function [this] = GammasynInitialValueElement(x_init)
			%GAMMASYNINITIALVALUEELEMENT initial value for gammasyn
			%	Input:
			%		x_init:		initial value
			%	Output:
			%		this:		instance
			this@control.design.gamma.InitialValueElement(true);
			[R, K, F] = input_initial_value2RKF(x_init);
			if ~isnumeric(R)
				error('control:design:gamma:initial', 'Proportional initial value must be numeric.');
			end
			if ndims(R) > 3
				error('control:design:gamma:initial', 'Proportional initial value must not have more than 3 dimensions.');
			end
			if ~isnumeric(K)
				error('control:design:gamma:initial', 'Derivative initial value must be numeric.');
			end
			if ndims(K) > 3
				error('control:design:gamma:initial', 'Derivative initial value must not have more than 3 dimensions.');
			end
			if ~isnumeric(F)
				error('control:design:gamma:initial', 'Prefilter initial value must be numeric.');
			end
			if ndims(F) > 3
				error('control:design:gamma:initial', 'Prefilter initial value must not have more than 3 dimensions.');
			end
			this.x_init = {
				R;
				K;
				F
			};
		end

		function [] = set.x_init(this, x_init)
			%X_INIT setter for initial value
			%	Input:
			%		this:	instance
			%		x_init:	initial value
			[R, K, F] = input_initial_value2RKF(x_init);
			if ~isnumeric(R)
				error('control:design:gamma:initial', 'Proportional initial value must be numeric.');
			end
			if ndims(R) > 3
				error('control:design:gamma:initial', 'Proportional initial value must not have more than 3 dimensions.');
			end
			if ~isnumeric(K)
				error('control:design:gamma:initial', 'Derivative initial value must be numeric.');
			end
			if ndims(K) > 3
				error('control:design:gamma:initial', 'Derivative initial value must not have more than 3 dimensions.');
			end
			if ~isnumeric(F)
				error('control:design:gamma:initial', 'Prefilter initial value must be numeric.');
			end
			if ndims(F) > 3
				error('control:design:gamma:initial', 'Prefilter initial value must not have more than 3 dimensions.');
			end
			this.x_init = {
				R;
				K;
				F
			};
		end
	end

	methods(Access=protected)
		function [initialvalue, valid, errorid, errormessage] = get_initial(this, ~, ~, ~, ~, ~, ~, ~, varargin) %#ok<VANUS> varargin is used for compatibility with interface
			%GET_INITIAL get a parameter dependent initial value from the element
			%	Input:
			%		this:			instance
			%		idx:			indices of initial values to return
			%		options:		options for creatin of dependent initial values in list
			%		systems:		structure/cell array or matrix with dynamic systems to take into consideration
			%		areafun:		area border functions and gradients as cell array with number of system entries, each function returns a row vector with as much elements as area border functions are defined for the corresponding system
			%		weights:		weighting matrix with number of systems columns and number of pole area border functions rows
			%		fixed:			cell array with indicator matrix for gain elements that should be fixed and the values the fixed gains have, empty if no fixed elements are needed. For linear dependent gain components the first element is a 3D matrix with relation in the third dimension and the second element a vector of constant constraint elements
			%		bounds:			cell array with indicator matrix for gain elements that should be bounded and the values the gains are bounded by, empty if no bounded elements are needed. For linear dependent gain components the first element is a 3D matrix with relation in the third dimension and the second element a vector of upper bound constraint elements
			%		nonlcon:		function pointer to a function of nonlinear inequality and equality constraints on gains with signature [c_R, ceq_R, c_K, ceq_K, c_F, ceq_F, gradc_R, gradceq_R, gradc_K, gradceq_K, gradc_F, gradceq_F] = constraint(R, K, F)
			%		varargin:		additional arguments needed for calculation
			%	Output:
			%		initialvalue:	cell array with initial proportional, derivative and prefilter values
			initialvalue = this.x_init;
			valid = true;
			errorid = '';
			errormessage = '';
		end
	end

end
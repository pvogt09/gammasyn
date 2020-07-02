classdef RandomInitialValue < control.design.gamma.InitialValueElement
	%RANDOMINITIALVALUE random initial value for gammasyn
	
	properties
		% number of initial values
		n
	end
	
	methods
		function [this] = RandomInitialValue(n)
			%RANDOMINITIALVALUE initial value for gammasyn
			%	Input:
			%		n:			number of initial value to return
			%	Output:
			%		this:		instance
			this@control.design.gamma.InitialValueElement(false);
			this.n = n;
		end
		
		function [] = set.n(this, n)
			%N setter for number of initial values
			%	Input:
			%		this:	instance
			%		n:		number of initial values
			if ~isscalar(n) || ~isnumeric(n)
				error('control:design:gamma:initial', 'Number of initial values must be a numeric scalar.');
			end
			if n <= 0 || isinf(n) || isnan(n)
				error('control:design:gamma:initial', 'Number of initial values must be greater than 0.');
			end
			this.n = n;
		end
		
		function [n] = get.n(this)
			%N getter for number of initial values
			%	Input:
			%		this:	instance
			%	Output:
			%		n:		number of initial values
			n = this.n;
		end
	end
	
	methods(Access=protected)
		function [initialvalue, valid, errorid, errormessage] = get_initial(this, ~, systems, ~, ~, ~, ~, ~, varargin) %#ok<VANUS> varargin is used for compatibility with interface
			%GET_INITIAL get a parameter dependent initial value from the element
			%	Input:
			%		this:			instance
			%		idx:			indices of initial values to return
			%		options:		options for creation of dependent initial values in list
			%		systems:		structure array with dynamic systems to take into consideration
			%		areafun:		area border functions and gradients as cell array with number of system entries, each function returns a row vector with as much elements as area border functions are defined for the corresponding system
			%		weights:		weighting matrix with number of systems columns and number of pole area border functions rows
			%		fixed:			cell array with indicator matrix for gain elements that should be fixed and the values the fixed gains have, empty if no fixed elements are needed. For linear dependent gain components the first element is a 3D matrix with relation in the third dimension and the second element a vector of constant constraint elements
			%		bounds:			cell array with indicator matrix for gain elements that should be bounded and the values the gains are bounded by, empty if no bounded elements are needed. For linear dependent gain components the first element is a 3D matrix with relation in the third dimension and the second element a vector of upper bound constraint elements
			%		nonlcon:		function pointer to a function of nonlinear inequality and equality constraints on gains with signature [c_R, ceq_R, c_K, ceq_K, c_F, ceq_F, gradc_R, gradceq_R, gradc_K, gradceq_K, gradc_F, gradceq_F] = constraint(R, K, F)
			%		varargin:		additional arguments needed for calculation
			%	Output:
			%		initialvalue:	cell array with numerical initial proportional, derivative and prefilter values
			number_controls = size(systems(1).B, 2);
			number_measurements = size(systems(1).C, 1);
			number_measurements_xdot = size(systems(1).C_dot, 1);
			number_references = size(systems(1).C_ref, 1);
			initialvalue = {
				randn(number_controls, number_measurements, this.n);
				randn(number_controls, number_measurements_xdot, this.n);
				randn(number_controls, number_references, this.n)
			};
			valid = true;
			errorid = '';
			errormessage = '';
		end
	end
end
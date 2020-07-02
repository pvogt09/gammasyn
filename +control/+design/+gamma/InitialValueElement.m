classdef(Abstract) InitialValueElement < handle
	%INITIALVALUEELEMENT Interface class for initial value elements that depend on parameters
	
	properties(Constant=true)
		% prototype for options of iniital value elements
		OPTION_PROTOTYPE = struct(...
			'usesystem',	[],...
			'eigenvalues',	[],...
			'optimization',	struct(...
				'algorithm',	[],...
				'options',		[],...
				'x_0',			[]...
			)...
		);
	end
	
	properties(Hidden=true)
		% indicator, if arguments of gammasyn (systems, areafun, weights, R_fixed, R_bounds, R_nonlin) are used. If not, the first argument for the get function is in varargin
		usesgammasynargs = true;
	end
	
	methods
		function [this] = InitialValueElement(usesgammasynargs)
			%INITIALVALUEELEMENT create new initial value element
			%	Input:
			%		usesgammasynargs:	indicator if the element uses the arguments of gammasyn
			%	Output:
			%		this:				instance
			if ~isscalar(usesgammasynargs) || ~islogical(usesgammasynargs)
				error('control:design:gamma:initial', 'gammasyn argument compliance must be a scalar logical.');
			end
			this.usesgammasynargs = usesgammasynargs;
		end
		
		function [] = set.usesgammasynargs(this, usesgammasynargs)
			%USESGAMMASYNARGS setter for indicator of gammasyn argument compliance
			%	Input:
			%		this:	instance
			%		usesgammasynargs:	inidcator if the element uses the arguments of gammasyn
			if ~isscalar(usesgammasynargs) || ~islogical(usesgammasynargs)
				error('control:design:gamma:initial', 'gammasyn argument compliance must be a scalar logical.');
			end
			this.usesgammasynargs = usesgammasynargs;
		end
		
	end
	
	methods(Access={?control.design.gamma.InitialValue})
		function [initialvalue] = get(this, options, systems, areafun, weights, fixed, bounds, nonlcon, varargin)
			%GET get a parameter dependent initial value from the element
			%	Input:
			%		this:			instance
			%		idx:			indices of initial values to return
			%		options:		options for creatin of dependent initial values in list
			%		systems:		structure array with dynamic systems to take into consideration
			%		areafun:		area border functions and gradients as cell array with number of system entries, each function returns a row vector with as much elements as area border functions are defined for the corresponding system
			%		weights:		weighting matrix with number of systems columns and number of pole area border functions rows
			%		fixed:			cell array with indicator matrix for gain elements that should be fixed and the values the fixed gains have, empty if no fixed elements are needed. For linear dependent gain components the first element is a 3D matrix with relation in the third dimension and the second element a vector of constant constraint elements
			%		bounds:			cell array with indicator matrix for gain elements that should be bounded and the values the gains are bounded by, empty if no bounded elements are needed. For linear dependent gain components the first element is a 3D matrix with relation in the third dimension and the second element a vector of upper bound constraint elements
			%		nonlcon:		function pointer to a function of nonlinear inequality and equality constraints on gains with signature [c_R, ceq_R, c_K, ceq_K, c_F, ceq_F, gradc_R, gradceq_R, gradc_K, gradceq_K, gradc_F, gradceq_F] = constraint(R, K, F)
			%		varargin:		additional arguments needed for calculation
			%	Output:
			%		initialvalue:	cell array with numerical initial proportional, derivative and prefilter values
			if nargin <= 1
				options = this.OPTION_PROTOTYPE;
			end
			switch nargin
				case 2
					[initialvalue, valid, errorid, errormessage] = this.get_initial(options, systems);
				case 3
					[initialvalue, valid, errorid, errormessage] = this.get_initial(options, systems, areafun);
				case 4
					[initialvalue, valid, errorid, errormessage] = this.get_initial(options, systems, areafun, weights);
				case 5
					[initialvalue, valid, errorid, errormessage] = this.get_initial(options, systems, areafun, weights, fixed);
				case 6
					[initialvalue, valid, errorid, errormessage] = this.get_initial(options, systems, areafun, weights, fixed, bounds);
				case 7
					[initialvalue, valid, errorid, errormessage] = this.get_initial(options, systems, areafun, weights, fixed, bounds, nonlcon);
				otherwise
					[initialvalue, valid, errorid, errormessage] = this.get_initial(options, systems, areafun, weights, fixed, bounds, nonlcon, varargin{:});
			end
			if ~valid
				error(errorid, errormessage);
			end
		end
	end
	
	methods(Abstract=true, Access=protected)
		%GET_INITIAL get a parameter dependent initial value from the element
		%	Input:
		%		this:			instance
		%		idx:			indices of initial values to return
		%		options:		options for creatin of dependent initial values in list
		%		systems:		structure array with dynamic systems to take into consideration
		%		areafun:		area border functions and gradients as cell array with number of system entries, each function returns a row vector with as much elements as area border functions are defined for the corresponding system
		%		weights:		weighting matrix with number of systems columns and number of pole area border functions rows
		%		fixed:			cell array with indicator matrix for gain elements that should be fixed and the values the fixed gains have, empty if no fixed elements are needed. For linear dependent gain components the first element is a 3D matrix with relation in the third dimension and the second element a vector of constant constraint elements
		%		bounds:			cell array with indicator matrix for gain elements that should be bounded and the values the gains are bounded by, empty if no bounded elements are needed. For linear dependent gain components the first element is a 3D matrix with relation in the third dimension and the second element a vector of upper bound constraint elements
		%		nonlcon:		function pointer to a function of nonlinear inequality and equality constraints on gains with signature [c_R, ceq_R, c_K, ceq_K, c_F, ceq_F, gradc_R, gradceq_R, gradc_K, gradceq_K, gradc_F, gradceq_F] = constraint(R, K, F)
		%		varargin:		additional arguments needed for calculation
		%	Output:
		%		initialvalue:	cell array with initial proportional, derivative and prefilter values
		[initialvalue, valid, errorid, errormessage] = get_initial(this, options, systems, areafun, weights, fixed, bounds, nonlcon, varargin);
	end
end
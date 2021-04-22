classdef gammasyn_decouplingcontrol_Test_Class < handle
	%GAMMASYN_DECOUPLINGCONTROL_TEST_CLASS class for handling testcases used for gammasyn_decouplingcontrol_Test

	properties(SetAccess=protected)
		% default input arguments for gammasyn_decouplingcontrol
		systems_default
		areafun_default
		weights_default
		R_fixed_default
		R_0_default
		solveroptions_default
		objectiveoptions_default
		R_bounds_default
		R_nonlin_default
		number_passed_parameters_default

		% testcase-individual input arguments for gammasyn_decouplingcontrol
		systems
		areafun
		weights
		R_fixed
		R_0
		solveroptions
		objectiveoptions
		R_bounds
		R_nonlin
		number_passed_parameters
	end

	methods
		function [this] = gammasyn_decouplingcontrol_Test_Class(systems, areafun, weights, R_fixed, R_0, solveroptions, objectiveoptions, R_bounds, R_nonlin, number_passed_parameters)
			%GAMMASYN_DECOUPLINGCONTROL_TEST_CLASS create new gammasyn_decouplingcontrol_Test_Class
			%	Input:
			%		systems:					systems struct
			%		areafun:					area border functions and gradients as cell array
			%		weights:					weighting matrix with number of systems columns and number of pole area border functions rows
			%		R_fixed:					R_fixed_cell
			%		R_0:						R_0_cell
			%		solveroptions:				options for optimization algorithm to use
			%		objectiveoptions:			options for problem functions
			%		R_bounds:					R_bounds_cell
			%		R_nonlin:					R_nonlin pointer
			%		number_passed_parameters:	how many parameters should be passed to gammasyn_decouplingcontrol. -1 for all.
			%	Output:
			%		this:						instance
			narginchk(9, Inf);
			this.systems_default = systems;
			this.areafun_default = areafun;
			this.weights_default = weights;
			this.R_fixed_default = R_fixed;
			this.R_0_default = R_0;
			this.solveroptions_default = solveroptions;
			this.objectiveoptions_default = objectiveoptions;
			this.R_bounds_default = R_bounds;
			this.R_nonlin_default = R_nonlin;
			this.number_passed_parameters_default = number_passed_parameters;
			this.reset();
		end

		function [this] = amend_systems(this, idx_sys, field_string, idx_elements, value)
			%AMEND_SYSTEMS changes the chosen property in systems
			%	Input:
			%		this:			instance
			%		idx_sys:		index indicating which system should be changed
			%		field_string:	string indicating which field should be changed
			%		idx_elements:	index of the field-elements that should be changed. If empty, then whole field is assigned anew.
			%		value:			value that should be assigned to the chosen option
			%	Output:
			%		this:			updated instance
			if ~isempty(idx_elements)
				this.systems(idx_sys).(field_string)(idx_elements) = value;
			elseif ~strcmp(field_string, '')
				this.systems(idx_sys).(field_string) = value;
			else
				this.systems(idx_sys) = value;
			end
		end

		function [this] = amend_areafun(this, idx_sys, value)
			%AMEND_AREAFUN changes the chosen property in areafun
			%	Input:
			%		this:			instance
			%		idx_sys:		index indicating which system's area should be changed
			%		value:			value that should be assigned to the chosen option
			%	Output:
			%		this:			updated instance
			if idx_sys > size(this.areafun, 1)
				this.areafun = [
					this.areafun;
					value
				];
			else
				this.areafun(idx_sys, :) = value;
			end
		end

		function [this] = amend_R_fixed_bounds(this, idx_RKF, idx_Ab, value, type)
			%AMEND_R_fixed_BOUNDS changes the chosen property in R_fixed or R_bounds
			%	Input:
			%		this:			instance
			%		idx_RKF:		index indicating which constraint system should be changed (1 = R_fixed, 2 = K_fixed, 3 = F_fixed, 4 = RKF_fixed)
			%		idx_Ab:			index indicating which field should be changed (1 = A, 2 = b)
			%		value:			value that should be assigned to the chosen option
			%		type:			specifies 'bounds' or 'fixed'
			%	Output:
			%		this:			updated instance
			if ~strcmp(type, 'fixed') && ~strcmp(type, 'bounds')
				error('control:gamma:decoupling:test', 'Wrong identifier. type must be ''fixed'' or ''bounds''.');
			end
			type_string = ['R_', type];
			if isempty(idx_RKF)
				this.(type_string) = value;
			elseif isempty(idx_Ab)
				this.(type_string){idx_RKF} = value;
			else
				this.(type_string){idx_RKF}{idx_Ab} = value;
			end
		end

		function [this] = amend_R_0(this, idx_RKF, idx_Ab, value)
			%AMEND_R_0 changes the chosen property in R_0
			%	Input:
			%		this:			instance
			%		idx_RKF:		index indicating which constraint system should be changed (1 = R_fixed, 2 = K_fixed, 3 = F_fixed, 4 = RKF_fixed)
			%		idx_Ab:			string indicating which field should be changed (1 = A, 2 = b)
			%		value:			value that should be assigned to the chosen option
			%	Output:
			%		this:			updated instance
			if isempty(idx_Ab)
				this.R_0{idx_RKF} = value;
			else
				this.R_0{idx_RKF}(idx_Ab) = value;
			end
		end

		function [this] = amend_objectiveoptions(this, option_string, value)
			%AMEND_OBJECTIVEOPTIONS changes the chosen property in objectiveoptions
			%	Input:
			%		this:			instance
			%		option_string:	string indicating which field should be changed. May be a (1x2) cell to chose a nested option
			%		value:			value that should be assigned to the chosen option
			%	Output:
			%		this:			updated instance
			if iscell(option_string)
				this.objectiveoptions.(option_string{1}).(option_string{2}) = value;
			elseif isempty(option_string)
				this.objectiveoptions = value;
			else
				this.objectiveoptions.(option_string) = value;
			end
		end

		function [this] = amend_solveroptions(this, option_string, value)
			%AMEND_SOLVEROPTIONS changes the chosen property in solveroptions
			%	Input:
			%		this:			instance
			%		option_string:	string indicating which field should be changed.
			%		value:			value that should be assigned to the chosen option
			%	Output:
			%		this:			updated instance
			this.solveroptions.(option_string) = value;
		end

		function [this] = amend_R_nonlin(this, value)
			%AMEND_R_nonlin changes handle for nonlinear constraints
			%	Input:
			%		this:	instance
			%		value:	handler for nonlinear constraints function
			%	Output:
			%		this:	updated instance
			this.R_nonlin = value;
		end

		function [this] = amend_number_passed_parameters(this, value)
			%AMEND_NUMBER_PASSED_PARAMETERS changes number of passed parameters
			%	Input:
			%		this:	instance
			%		value:	number of passed parameters
			%	Output:
			%		this:	updated instance
			this.number_passed_parameters = value;
		end

		function [systems, areafun, weights, R_fixed, R_0, solveroptions, objectiveoptions, R_bounds, R_nonlin, number_passed_parameters] = implement_testcase(this, testcase_struct)
			%IMPLEMENT_TESTCASE returns input arguments for gammasyn_decouplingcontrol according to specified testcase
			%	Input:
			%		this:						instance
			%		testcase_struct:			struct specifying testcase options. Must have fields: systems, areafun, weights, R_fixed, R_0, solveroptions, objectiveoptions, R_bounds, R_nonlin
			%	Output:
			%		systems:					systems struct
			%		areafun:					area border functions and gradients as cell array
			%		weights:					weighting matrix with number of systems columns and number of pole area border functions rows
			%		R_fixed:					R_fixed_cell
			%		R_0:						R_0_cell
			%		solveroptions:				options for optimization algorithm to use
			%		objectiveoptions:			options for problem functions
			%		R_bounds:					R_bounds_cell
			%		R_nonlin:					R_nonlin pointer
			%		number_passed_parameters:	number of passed parameters to gammasyn_decouplingcontrol
			for ii = 1:size(testcase_struct.systems, 1)
				this.amend_systems(testcase_struct.systems{ii}{1}, testcase_struct.systems{ii}{2}, testcase_struct.systems{ii}{3}, testcase_struct.systems{ii}{4});
			end
			for ii = 1:size(testcase_struct.R_fixed, 1)
				this.amend_R_fixed_bounds(testcase_struct.R_fixed{ii}{1}, testcase_struct.R_fixed{ii}{2}, testcase_struct.R_fixed{ii}{3}, 'fixed');
			end
			for ii = 1:size(testcase_struct.R_0, 1)
				this.amend_R_0(testcase_struct.R_0{ii}{1}, testcase_struct.R_0{ii}{2}, testcase_struct.R_0{ii}{3});
			end
			for ii = 1:size(testcase_struct.solveroptions, 1)
				this.amend_solveroptions(testcase_struct.solveroptions{ii, 1}, testcase_struct.solveroptions{ii, 2});
			end
			for ii = 1:size(testcase_struct.objectiveoptions, 1)
				this.amend_objectiveoptions(testcase_struct.objectiveoptions{ii, 1}, testcase_struct.objectiveoptions{ii, 2});
			end
			for ii = 1:size(testcase_struct.R_bounds, 1)
				this.amend_R_fixed_bounds(testcase_struct.R_bounds{ii}{1}, testcase_struct.R_bounds{ii}{2}, testcase_struct.R_bounds{ii}{3}, 'bounds');
			end
			for ii = 1:size(testcase_struct.R_nonlin, 1)
				this.amend_R_nonlin(testcase_struct.R_nonlin{ii, 1});
			end
			for ii = 1:size(testcase_struct.number_passed_parameters, 1)
				this.amend_number_passed_parameters(testcase_struct.number_passed_parameters{ii, 1});
			end
			for ii = 1:size(testcase_struct.areafun, 1)
				this.amend_areafun(testcase_struct.areafun{ii, 1}, testcase_struct.areafun{ii, 2});
			end
			[systems, areafun, weights, R_fixed, R_0, solveroptions, objectiveoptions, R_bounds, R_nonlin, number_passed_parameters] = this.get_options();
		end

		function [systems, areafun, weights, R_fixed, R_0, solveroptions, objectiveoptions, R_bounds, R_nonlin, number_passed_parameters] = get_options(this)
			%GET_OPTIONS returns options of current testcase
			%	Input:
			%		this:						instance
			%	Output:
			%		systems:					systems struct
			%		areafun:					area border functions and gradients as cell array
			%		weights:					weighting matrix with number of systems columns and number of pole area border functions rows
			%		R_fixed:					R_fixed_cell
			%		R_0:						R_0_cell
			%		solveroptions:				options for optimization algorithm to use
			%		objectiveoptions:			options for problem functions
			%		R_bounds:					R_bounds_cell
			%		R_nonlin:					R_nonlin pointer
			%		number_passed_parameters:	number of passed parameters to gammasyn_decouplingcontrol
			systems = this.systems;
			areafun = this.areafun;
			weights = this.weights;
			R_fixed = this.R_fixed;
			R_0 = this.R_0;
			solveroptions = this.solveroptions;
			objectiveoptions = this.objectiveoptions;
			R_bounds = this.R_bounds;
			R_nonlin = this.R_nonlin;
			number_passed_parameters = this.number_passed_parameters;
		end

		function [] = reset(this)
			%RESET resets options systems, areafun, weights, R_fixed, R_0, solveroptions, objectiveoptions, R_bounds, R_nonlin to default values
			%	Input:
			%		this:	instance
			%	Output:
			%		this:	updated instance
			this.systems = this.systems_default;
			this.areafun = this.areafun_default;
			this.weights = this.weights_default;
			this.R_fixed = this.R_fixed_default;
			this.R_0 = this.R_0_default;
			this.solveroptions = this.solveroptions_default;
			this.objectiveoptions = this.objectiveoptions_default;
			this.R_bounds = this.R_bounds_default;
			this.R_nonlin = this.R_nonlin_default;
			this.number_passed_parameters = this.number_passed_parameters_default;
		end
	end
end
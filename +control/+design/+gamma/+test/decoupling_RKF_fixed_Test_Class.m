classdef decoupling_RKF_fixed_Test_Class
%DECOUPLING_RKF_FIXED_TEST_CLASS class for handling testcases used for decoupling_RKF_fixed_Test
	
	properties(SetAccess=protected)
		% default input arguments for decoupling_RKF_fixed
		systems_default
		R_fixed_default
		objectiveoptions_default
		solveroptions_default
		descriptor_default

		% testcase-individual input arguments for decoupling_RKF_fixed
		systems
		R_fixed
		objectiveoptions
		solveroptions
		descriptor
	end

	methods
		function [this] = decoupling_RKF_fixed_Test_Class(systems, R_fixed, objectiveoptions, solveroptions, descriptor)
			%DECOUPLING_RKF_FIXED_TEST_CLASS create new decoupling_RKF_fixed_Test_Class
			%	Input:
			%		systems:			systems struct
			% 		R_fixed:			R_fixed cell
			% 		objectiveoptions:	struct of objective options
			% 		solveroptions:		struct of solver options
			% 		descriptor:			logical indicator if descriptor system is considered
			%	Output:
			%		this:						instance
			narginchk(5, Inf);
			this.systems_default = systems;
			this.R_fixed_default = R_fixed;
			this.objectiveoptions_default = objectiveoptions;
			this.solveroptions_default = solveroptions;
			this.descriptor_default = descriptor;
			this = this.reset();
		end
		
		function this = amend_systems(this, idx_sys, field_string, idx_elements, value)
			%AMEND_SYSTEMS changes the chosen property in systems
			%	Input:
			%		this:			instance
			%		idx_sys:		index indicating which system should be changed
			%		field_string:	string indicating which field should be changed
			%		idx_elements:	index of the field-elements that should be changed
			%		value:			value that should be assigned to the chosen option
			%	Output:
			%		this:			updated instance
			this.systems(idx_sys).(field_string)(idx_elements) = value;
		end
		
		function this = amend_R_fixed(this, idx_RKF, idx_Ab, value)
			%AMEND_R_fixed changes the chosen property in R_fixed
			%	Input:
			%		this:			instance
			%		idx_RKF:		index indicating which constraint system should be changed (1 = R_fixed, 2 = K_fixed, 3 = F_fixed, 4 = RKF_fixed)
			%		idx_Ab:			string indicating which field should be changed (1 = A, 2 = b)
			%		value:			value that should be assigned to the chosen option
			%	Output:
			%		this:			updated instance
			this.R_fixed{idx_RKF}{idx_Ab} = value;
		end
		
		function this = amend_objectiveoptions(this, option_string, value)
			%AMEND_OBJECTIVEOPTIONS changes the chosen property in objectiveoptions
			%	Input:
			%		this:			instance
			%		option_string:	string indicating which field should be changed. May be a (1x2) cell to chose a nested option
			%		value:			value that should be assigned to the chosen option
			%	Output:
			%		this:			updated instance
			if iscell(option_string)
				this.objectiveoptions.(option_string{1}).(option_string{2}) = value;
			else
				this.objectiveoptions.(option_string) = value;
			end
		end
		
		function this = amend_solveroptions(this, option_string, value)
			%AMEND_SOLVEROPTIONS changes the chosen property in solveroptions
			%	Input:
			%		this:			instance
			%		option_string:	string indicating which field should be changed.
			%		value:			value that should be assigned to the chosen option
			%	Output:
			%		this:			updated instance
			this.solveroptions.(option_string) = value;
		end
		
		function this = amend_descriptor(this, descriptor)
			%AMEND_descriptor changes descriptor
			%	Input:
			%		this:	instance
			%		value:	logical scalar that should be assigned to descriptor
			%	Output:
			%		this:	updated instance
			this.descriptor = descriptor;
		end
		
		function [systems, R_fixed, objectiveoptions, solveroptions, descriptor] = implement_testcase(this, testcase_struct)
			%IMPLEMENT_TESTCASE returns input arguments for decoupling_RKF_fixed according to specified testcase
			%	Input:
			%		this:				instance
			%		testcase_struct:	struct specifying testcase options. Must have fields: systems, R_fixed, objectiveoptions, solveroptions, descriptor
			%	Output:
			%		systems:			systems structure
			% 		R_fixed:			cell array with controller constraints
			% 		objectiveoptions:	struct with objective options
			% 		solveroptions:		struct with solver options
			% 		descriptor:			logical indicator if descriptor system is considered
			for ii = 1:size(testcase_struct.systems, 1)
				this = this.amend_systems(testcase_struct.systems{ii}{1}, testcase_struct.systems{ii}{2}, testcase_struct.systems{ii}{3}, testcase_struct.systems{ii}{4});
			end
			for ii = 1:size(testcase_struct.R_fixed, 1)
				this = this.amend_R_fixed(testcase_struct.R_fixed{ii}{1}, testcase_struct.R_fixed{ii}{2}, testcase_struct.R_fixed{ii}{3});
			end
			for ii = 1:size(testcase_struct.objectiveoptions, 1)
				this = this.amend_objectiveoptions(testcase_struct.objectiveoptions{ii}{1}, testcase_struct.objectiveoptions{ii}{2});
			end
			for ii = 1:size(testcase_struct.solveroptions, 1)
				this = this.amend_solveroptions(testcase_struct.solveroptions{ii}{1}, testcase_struct.solveroptions{ii}{2});
			end
			for ii = 1:size(testcase_struct.descriptor, 1)
				this = this.amend_descriptor(testcase_struct.descriptor{ii});
			end
			[systems, R_fixed, objectiveoptions, solveroptions, descriptor] = this.get_options();
		end

		function [systems, R_fixed, objectiveoptions, solveroptions, descriptor] = get_options(this)
			%GET_OPTIONS returns options of current testcase
			%	Input:
			%		this:				instance
			%	Output:
			%		systems:			systems structure
			% 		R_fixed:			cell array with controller constraints
			% 		objectiveoptions:	struct with objective options
			% 		solveroptions:		struct with solver options
			% 		descriptor:			logical indicator if descriptor system is considered
			systems = this.systems;
			R_fixed = this.R_fixed;
			objectiveoptions = this.objectiveoptions;
			solveroptions = this.solveroptions;
			descriptor = this.descriptor;
		end
		function this = reset(this)
			%RESET resets options systems, R_fixed, objectiveoptions, solveroptions, descriptor to default values
			%	Input:
			%		this:	instance
			%	Output:
			%		this:	updated instance
			this.systems = this.systems_default;
			this.R_fixed = this.R_fixed_default;
			this.objectiveoptions = this.objectiveoptions_default;
			this.solveroptions = this.solveroptions_default;
			this.descriptor = this.descriptor_default;
		end
	end
end
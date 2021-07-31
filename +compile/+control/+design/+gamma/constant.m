function [config] = constant(config)
	%CONSTANT settings for code generation for gamma pole area assignment
	%	Input:
	%		config:	settings to change
	%	Output:
	%		config:	settings for code generation
	if nargin < 1
		config = struct();
	end
	config = isset(config, 'max_system_states',							20);
	config = isset(config, 'max_system_controls',						10);
	config = isset(config, 'max_system_measurements',					20);
	config = isset(config, 'max_system_references',						10);
	config = isset(config, 'max_number_of_systems',						1000);
	config = isset(config, 'max_number_of_objective_functions',			max([10, length(enumeration('GammaJType')) + 1]));
	config = isset(config, 'max_number_of_area_functions',				max([10, length(enumeration('GammaArea')) + 1]));
	config = isset(config, 'max_number_of_area_function_parameters',	20);
end
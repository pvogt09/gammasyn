function [decouplingoptions] = checkobjectiveoptions_decoupling(number_references, decouplingoptions)
	%CHECKOBJECTIVEOPTIONS_DECOUPLING check decoupling control design options for objective function
	%	Input:
	%		number_references:			number of references
	%		decouplingoptions:			user supplied objective options for decoupling control
	%	Output:
	%		decouplingoptions:			objective options for decoupling control
	decoupling_prototype = control.design.gamma.objectiveoptions_prototype_decoupling(number_references);
	if nargin < 2
		decouplingoptions = decoupling_prototype;
	end
	if isempty(decouplingoptions)
		decouplingoptions = decoupling_prototype;
	else
		if ~isstruct(decouplingoptions)
			if isa(coulingoptions, 'GammaDecouplingStrategy')
				temp = decoupling_prototype;
				temp.decouplingstrategy = decouplingoptions;
				decouplingoptions = temp;
			else
				error('control:design:gamma', 'Objective options must be of type struct.');
			end
		else
			if ~isfield(decouplingoptions, 'tf_structure')
				decouplingoptions.tf_structure = decoupling_prototype.tf_structure;
			end
			if ~isfield(decouplingoptions, 'decouplingstrategy')
				% use default strategy in case decoupling controller design is requested
				decouplingoptions.decouplingstrategy = [];
			end
			if ~isfield(decouplingoptions, 'sortingstrategy_decoupling')
				% use default sorting strategy for decoupling conditions
				decouplingoptions.sortingstrategy_decoupling = [];
			end
			if ~isfield(decouplingoptions, 'tolerance_decoupling')
				decouplingoptions.tolerance_decoupling = decoupling_prototype.tolerance_decoupling;
			end
			if ~isfield(decouplingoptions, 'tolerance_prefilter')
				decouplingoptions.tolerance_prefilter = decoupling_prototype.tolerance_prefilter;
			end
			if ~isfield(decouplingoptions, 'weight_decoupling')
				decouplingoptions.weight_decoupling = decoupling_prototype.weight_decoupling;
			end
			if ~isfield(decouplingoptions, 'weight_prefilter')
				decouplingoptions.weight_prefilter = decoupling_prototype.weight_prefilter;
			end
			if ~isfield(decouplingoptions, 'solvesymbolic')
				decouplingoptions.solvesymbolic = decoupling_prototype.solvesymbolic;
			end
			if ~isfield(decouplingoptions, 'round_equations_to_digits')
				decouplingoptions.round_equations_to_digits = decoupling_prototype.round_equations_to_digits;
			end
			if ~isfield(decouplingoptions, 'allowoutputdecoupling')
				decouplingoptions.allowoutputdecoupling = decoupling_prototype.allowoutputdecoupling;
			end
		end
	end
	if ~isstruct(decouplingoptions)
		error('control:design:gamma', 'Options for decoupling control must be of type struct.');
	end
	if size(decouplingoptions.tf_structure, 1) ~= number_references || size(decouplingoptions.tf_structure, 2) ~= number_references
		error('control:design:gamma', 'tf_structure must be a %dX%d matrix.', number_references, number_references);
	end
	if size(decouplingoptions.tf_structure, 3) > 1
		error('control:design:gamma', 'Stacking of tf_structures in the third dimension is not allowed.');
	end
	if sum(isnan(decouplingoptions.tf_structure(:))) + sum(decouplingoptions.tf_structure(:) == 0) ~= number_references^2
		error('control:design:gamma', 'tf_structure must only contains 0 and NaN elements.');
	end
	if isempty(decouplingoptions.decouplingstrategy)
		if any(decouplingoptions.tf_structure(:) == 0)
			% default value if decoupling controller design is requested, but strategy not specified
			decouplingoptions.decouplingstrategy = GammaDecouplingStrategy.NUMERIC_NONLINEAR_EQUALITY;
		else
			% default value if decoupling controller design is not requested, but strategy not specified
			decouplingoptions.decouplingstrategy = GammaDecouplingStrategy.getDefaultValue();
		end
	end
	if ~isscalar(decouplingoptions.decouplingstrategy)
		error('control:design:gamma', 'Decoupling control strategy must be scalar.');
	end
	if ~isa(decouplingoptions.decouplingstrategy, 'GammaDecouplingStrategy')
		try
			decouplingoptions.decouplingstrategy = GammaDecouplingStrategy.fromname(decouplingoptions.decouplingstrategy);
		catch e
			rethrow(e);
		end
	end
	if isempty(decouplingoptions.sortingstrategy_decoupling)
		decouplingoptions.sortingstrategy_decoupling = GammaDecouplingconditionSortingStrategy.getDefaultValue();
	end
	if ~isscalar(decouplingoptions.sortingstrategy_decoupling)
		error('control:design:gamma', 'Sorting strategy for decoupling conditions must be scalar.');
	end
	if ~isa(decouplingoptions.sortingstrategy_decoupling, 'GammaDecouplingconditionSortingStrategy')
		try
			decouplingoptions.sortingstrategy_decoupling = GammaDecouplingconditionSortingStrategy.fromname(decouplingoptions.sortingstrategy_decoupling);
		catch e
			rethrow(e);
		end
	end
	if isempty(decouplingoptions.weight_decoupling)
		decouplingoptions.weight_decoupling = 1;
	end
	if ~isscalar(decouplingoptions.weight_decoupling) || ~isnumeric(decouplingoptions.weight_decoupling)
		error('control:design:gamma', 'Weight for decoupling condition constraints must be a numeric scalar.');
	end
	if isnan(decouplingoptions.weight_decoupling) || isinf(decouplingoptions.weight_decoupling)
		error('control:design:gamma', 'Weight for decoupling condition constraints must be finite.');
	end
	if decouplingoptions.weight_decoupling < 0
		error('control:design:gamma', 'Weight for decoupling constraints must be nonnegative.');
	end
	if isempty(decouplingoptions.weight_prefilter)
		decouplingoptions.weight_prefilter = 1;
	end
	if ~isscalar(decouplingoptions.weight_prefilter) || ~isnumeric(decouplingoptions.weight_prefilter)
		error('control:design:gamma', 'Weight for prefilter decoupling condition constraints must be a numeric scalar.');
	end
	if isnan(decouplingoptions.weight_prefilter) || isinf(decouplingoptions.weight_prefilter)
		error('control:design:gamma', 'Weight for prefilter decoupling condition constraints must be finite.');
	end
	if decouplingoptions.weight_prefilter < 0
		error('control:design:gamma', 'Weight for prefilter decoupling constraints must be nonnegative.');
	end
	if isempty(decouplingoptions.tolerance_decoupling)
		decouplingoptions.tolerance_decoupling = NaN;
	end
	if ~isscalar(decouplingoptions.tolerance_decoupling) || ~isnumeric(decouplingoptions.tolerance_decoupling)
		error('control:design:gamma', 'Tolerance for decouplingconditions must be a numeric scalar.');
	end
	if isnan(decouplingoptions.tolerance_decoupling)
		if decouplingoptions.decouplingstrategy == GammaDecouplingStrategy.NUMERIC_NONLINEAR_INEQUALITY
			decouplingoptions.tolerance_decoupling = sqrt(eps);
		else
			decouplingoptions.tolerance_decoupling = 0;
		end
	end
	if isinf(decouplingoptions.tolerance_decoupling)
		error('control:design:gamma', 'Tolerance for decouplingconditions must be finite.');
	end
	if decouplingoptions.tolerance_decoupling < 0
		error('control:design:gamma', 'Tolerance for decouplingconditions must be nonnegative.');
	end
	if isempty(decouplingoptions.tolerance_prefilter)
		decouplingoptions.tolerance_prefilter = NaN;
	end
	if ~isscalar(decouplingoptions.tolerance_prefilter) || ~isnumeric(decouplingoptions.tolerance_prefilter)
		error('control:design:gamma', 'Tolerance for prefilter regularization must be a numeric scalar.');
	end
	if isnan(decouplingoptions.tolerance_prefilter)
		decouplingoptions.tolerance_prefilter = 1;
	end
	if isinf(decouplingoptions.tolerance_prefilter)
		error('control:design:gamma', 'Tolerance for prefilter regularization must be finite.');
	end
	if decouplingoptions.tolerance_prefilter < 0
		error('control:design:gamma', 'Tolerance for prefilter regularization must be nonnegative.');
	end
	if ~isscalar(decouplingoptions.solvesymbolic) || ~islogical(decouplingoptions.solvesymbolic)
		error('control:design:gamma', 'Indicator for symbolic solution of decoupling conditions must be a logical scalar.');
	end
	if isempty(decouplingoptions.round_equations_to_digits)
		decouplingoptions.round_equations_to_digits = NaN;
	end
	if ~isscalar(decouplingoptions.round_equations_to_digits) || ~isnumeric(decouplingoptions.round_equations_to_digits)
		error('control:design:gamma', 'Places to round decoupling equations to must be a numeric scalar.');
	end
	if isnan(decouplingoptions.round_equations_to_digits)
		decouplingoptions.round_equations_to_digits = double(NaN);
	end
	if isinf(decouplingoptions.round_equations_to_digits)
		error('control:design:gamma', 'Places to round decoupling equations to must be finite.');
	end
	if ~isnan(decouplingoptions.round_equations_to_digits) && (floor(decouplingoptions.round_equations_to_digits) ~= ceil(decouplingoptions.round_equations_to_digits))
		error('control:design:gamma', 'Places to round decoupling equations to must be an integer.');
	end
	if ~isscalar(decouplingoptions.allowoutputdecoupling) || ~islogical(decouplingoptions.allowoutputdecoupling)
		error('control:design:gamma', 'Indicator for output feedback in decoupling control must be a logical scalar.');
	end
	decouplingoptions = struct(...
		'tf_structure',					decouplingoptions.tf_structure,...
		'decouplingstrategy',			decouplingoptions.decouplingstrategy,...
		'sortingstrategy_decoupling',	decouplingoptions.sortingstrategy_decoupling,...
		'weight_decoupling',			double(decouplingoptions.weight_decoupling),...
		'weight_prefilter',				double(decouplingoptions.weight_prefilter),...
		'tolerance_decoupling',			double(decouplingoptions.tolerance_decoupling),...
		'tolerance_prefilter',			double(decouplingoptions.tolerance_prefilter),...
		'solvesymbolic',				logical(decouplingoptions.solvesymbolic),...
		'round_equations_to_digits',	double(decouplingoptions.round_equations_to_digits),...
		'allowoutputdecoupling',		logical(decouplingoptions.allowoutputdecoupling)...
	);
end
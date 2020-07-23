function [couplingoptions] = checkobjectiveoptions_coupling(couplingoptions)
	%CHECKOBJECTIVEOPTIONS_COUPLING check coupling control design options for objective function
	%	Input:
	%		couplingoptions:			user supplied objective options for coupling control
	%	Output:
	%		couplingoptions:			objective options for coupling control
	coupling_prototype = control.design.gamma.objectiveoptions_prototype_coupling();
	if nargin <= 0
		couplingoptions = coupling_prototype;
	end
	if isempty(couplingoptions)
		couplingoptions = coupling_prototype;
	else
		if ~isstruct(couplingoptions)
			if isa(coulingoptions, 'GammaCouplingStrategy')
				temp = coupling_prototype;
				temp.couplingstrategy = couplingoptions;
				couplingoptions = temp;
			else
				error('control:design:gamma', 'Objective options must be of type struct.');
			end
		else
			if ~isfield(couplingoptions, 'couplingconditions')
				couplingoptions.couplingconditions = coupling_prototype.couplingconditions;
			end
			if ~isfield(couplingoptions, 'couplingstrategy')
				% use default strategy in case coupling controller design is requested
				couplingoptions.couplingstrategy = [];
			end
			if ~isfield(couplingoptions, 'tolerance_coupling')
				couplingoptions.tolerance_coupling = coupling_prototype.tolerance_coupling;
			end
			if ~isfield(couplingoptions, 'tolerance_prefilter')
				couplingoptions.tolerance_prefilter = coupling_prototype.tolerance_prefilter;
			end
			if ~isfield(couplingoptions, 'weight_coupling')
				couplingoptions.weight_coupling = coupling_prototype.weight_coupling;
			end
			if ~isfield(couplingoptions, 'weight_prefilter')
				couplingoptions.weight_prefilter = coupling_prototype.weight_prefilter;
			end
			if ~isfield(couplingoptions, 'solvesymbolic')
				couplingoptions.solvesymbolic = coupling_prototype.solvesymbolic;
			end
			if ~isfield(couplingoptions, 'round_equations_to_digits')
				couplingoptions.round_equations_to_digits = coupling_prototype.round_equations_to_digits;
			end
		end
	end
	if ~isstruct(couplingoptions)
		error('control:design:gamma', 'Options for coupling control must be of type struct.');
	end
	if ~isscalar(couplingoptions.couplingconditions) || ~isnumeric(couplingoptions.couplingconditions)
		error('control:design:gamma', 'Number of couplingconditions must be numeric scalar.');
	end
	if isnan(couplingoptions.couplingconditions) || isinf(couplingoptions.couplingconditions)
		error('control:design:gamma', 'Number of couplingconditions must be finite.');
	end
	if couplingoptions.couplingconditions < 0
		error('control:design:gamma', 'Number of couplingconditions must be nonnegative.');
	end
	if floor(couplingoptions.couplingconditions) ~= ceil(couplingoptions.couplingconditions)
		error('control:design:gamma', 'Number of couplingconditions must be an integer.');
	end
	if isempty(couplingoptions.couplingstrategy)
		if couplingoptions.couplingconditions > 0
			% default value if coupling controller design is requested, but strategy not specified
			couplingoptions.couplingstrategy = GammaCouplingStrategy.NUMERIC_NONLINEAR_EQUALITY;
		else
			% default value if coupling controller design is not requested, but strategy not specified
			couplingoptions.couplingstrategy = GammaCouplingStrategy.getDefaultValue();
		end
	end
	if ~isscalar(couplingoptions.couplingstrategy)
		error('control:design:gamma', 'Coupling control strategy must be scalar.');
	end
	if ~isa(couplingoptions.couplingstrategy, 'GammaCouplingStrategy')
		try
			couplingoptions.couplingstrategy = GammaCouplingStrategy.fromname(couplingoptions.couplingstrategy);
		catch e
			rethrow(e);
		end
	end
	if isempty(couplingoptions.weight_coupling)
		couplingoptions.weight_coupling = 1;
	end
	if ~isscalar(couplingoptions.weight_coupling) || ~isnumeric(couplingoptions.weight_coupling)
		error('control:design:gamma', 'Weight for coupling condition constraints must be a numeric scalar.');
	end
	if isnan(couplingoptions.weight_coupling) || isinf(couplingoptions.weight_coupling)
		error('control:design:gamma', 'Weight for coupling condition constraints must be finite.');
	end
	if couplingoptions.weight_coupling < 0
		error('control:design:gamma', 'Weight for coupling constraints must be nonnegative.');
	end
	if isempty(couplingoptions.weight_prefilter)
		couplingoptions.weight_prefilter = 1;
	end
	if ~isscalar(couplingoptions.weight_prefilter) || ~isnumeric(couplingoptions.weight_prefilter)
		error('control:design:gamma', 'Weight for prefilter coupling condition constraints must be a numeric scalar.');
	end
	if isnan(couplingoptions.weight_prefilter) || isinf(couplingoptions.weight_prefilter)
		error('control:design:gamma', 'Weight for prefilter coupling condition constraints must be finite.');
	end
	if couplingoptions.weight_prefilter < 0
		error('control:design:gamma', 'Weight for prefilter coupling constraints must be nonnegative.');
	end
	if isempty(couplingoptions.tolerance_coupling)
		couplingoptions.tolerance_coupling = NaN;
	end
	if ~isscalar(couplingoptions.tolerance_coupling) || ~isnumeric(couplingoptions.tolerance_coupling)
		error('control:design:gamma', 'Tolerance for couplingconditions must be a numeric scalar.');
	end
	if isnan(couplingoptions.tolerance_coupling)
		if couplingoptions.couplingstrategy == GammaCouplingStrategy.NUMERIC_NONLINEAR_INEQUALITY
			couplingoptions.tolerance_coupling = sqrt(eps);
		else
			couplingoptions.tolerance_coupling = 0;
		end
	end
	if isinf(couplingoptions.tolerance_coupling)
		error('control:design:gamma', 'Tolerance for couplingconditions must be finite.');
	end
	if couplingoptions.tolerance_coupling < 0
		error('control:design:gamma', 'Tolerance for couplingconditions must be nonnegative.');
	end
	if isempty(couplingoptions.tolerance_prefilter)
		couplingoptions.tolerance_prefilter = NaN;
	end
	if ~isscalar(couplingoptions.tolerance_prefilter) || ~isnumeric(couplingoptions.tolerance_prefilter)
		error('control:design:gamma', 'Tolerance for prefilter regularization must be a numeric scalar.');
	end
	if isnan(couplingoptions.tolerance_prefilter)
		couplingoptions.tolerance_prefilter = 1;
	end
	if isinf(couplingoptions.tolerance_prefilter)
		error('control:design:gamma', 'Tolerance for prefilter regularization must be finite.');
	end
	if couplingoptions.tolerance_prefilter < 0
		error('control:design:gamma', 'Tolerance for prefilter regularization must be nonnegative.');
	end
	if ~isscalar(couplingoptions.solvesymbolic) || ~islogical(couplingoptions.solvesymbolic)
		error('control:design:gamma', 'Indicator for symbolic solution of coupling conditions must be logical scalar.');
	end
	if isempty(couplingoptions.round_equations_to_digits)
		couplingoptions.round_equations_to_digits = NaN;
	end
	if ~isscalar(couplingoptions.round_equations_to_digits) || ~isnumeric(couplingoptions.round_equations_to_digits)
		error('control:design:gamma', 'Places to round coupling equations to must be a numeric scalar.');
	end
	if isnan(couplingoptions.round_equations_to_digits)
		couplingoptions.round_equations_to_digits = double(NaN);
	end
	if isinf(couplingoptions.round_equations_to_digits)
		error('control:design:gamma', 'Places to round coupling equations to must be finite.');
	end
	if ~isnan(couplingoptions.round_equations_to_digits) && (floor(couplingoptions.round_equations_to_digits) ~= ceil(couplingoptions.round_equations_to_digits))
		error('control:design:gamma', 'Places to round coupling equations to must be an integer.');
	end
	couplingoptions = struct(...
		'couplingconditions',			uint32(couplingoptions.couplingconditions),...
		'couplingstrategy',				couplingoptions.couplingstrategy,...
		'weight_coupling',				double(couplingoptions.weight_coupling),...
		'weight_prefilter',				double(couplingoptions.weight_prefilter),...
		'tolerance_coupling',			double(couplingoptions.tolerance_coupling),...
		'tolerance_prefilter',			double(couplingoptions.tolerance_prefilter),...
		'solvesymbolic',				logical(couplingoptions.solvesymbolic),...
		'round_equations_to_digits',	double(couplingoptions.round_equations_to_digits)...
	);
end
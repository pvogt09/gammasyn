classdef(Abstract) AbstractCouplingFeedback < control.design.outputfeedback.OutputFeedback
	%ABSTRACTCOUPLINGFEEDBACK abstract class for casting a control system in output feedback form and specify the needed constraints on the resulting gain matrix, when a coupling controller feedback is needed

	properties(SetAccess=protected)
		% number of coupling conditions
		number_couplingconditions
	end

	methods
		function [this] = AbstractCouplingFeedback(number_couplingconditions)
			%ABSTRACTCOUPLINGFEEDBACK create new coupling feedback
			%	Input:
			%		number_couplingconditions:	number of coupling conditions in C_ref
			%	Output:
			%		this:						instance
			this@control.design.outputfeedback.OutputFeedback();
			narginchk(1, 1);
			this.number_couplingconditions = number_couplingconditions;
		end

		function [this] = set.number_couplingconditions(this, number_couplingconditions)
			%NUMBER_COUPLINGCONDITIONS set number of coupling conditions in C_ref
			%	Input:
			%		this:						instance
			%		number_couplingconditions:	number of coupling conditions in C_ref
			%	Output:
			%		this:						instance
			if ~isnumeric(number_couplingconditions) || ~isscalar(number_couplingconditions)
				error('control:design:outputfeedback:input', 'Number of coupling conditions must be a numeric scalar.');
			end
			if isinf(number_couplingconditions) || isnan(number_couplingconditions)
				error('control:design:outputfeedback:input', 'Number of coupling conditions must be finite.');
			end
			if floor(number_couplingconditions) ~= ceil(number_couplingconditions)
				error('control:design:outputfeedback:input', 'Number of coupling conditions must be an integer.');
			end
			if number_couplingconditions <= 0
				error('control:design:outputfeedback:input', 'Number of coupling conditions must be greater than 0.');
			end
			this.number_couplingconditions = uint32(number_couplingconditions);
		end

		function [couplingoptions] = get_couplingoptions(this, couplingoptions)
			%GET_COUPLINGOPTIONS return structure with options for coupling controller design
			%	Input:
			%		this:				instance
			%		options:			structure with options to append to
			%	Output:
			%		couplingoptions:	structure with appended coupling options
			if nargin <= 1
				couplingoptions = struct();
			end
			if ~isstruct(couplingoptions)
				error('control:design:outputfeedback:input', 'Coupling options must be a structure.');
			end
			if ~isfield(couplingoptions, 'couplingcontrol')
				couplingstruct = control.design.gamma.objectiveoptions_prototype_coupling();
			else
				couplingstruct = mergestruct(couplingoptions.couplingcontrol, control.design.gamma.objectiveoptions_prototype_coupling());
			end
			couplingoptions.couplingcontrol = this.get_couplingoptions_system(couplingstruct);
		end
	end

	methods(Abstract=true, Access=protected)
		%GET_COUPLINGOPTIONS_SYSTEM return structure with options for coupling controller design
		%	Input:
		%		this:				instance
		%		options:			structure with options to append to
		%	Output:
		%		couplingoptions:	structure with appended coupling options
		[couplingoptions] = get_couplingoptions_system(this, couplingoptions);
	end
end
classdef(Abstract) AbstractCouplingFeedback < control.design.outputfeedback.AbstractDecouplingFeedback
	%ABSTRACTCOUPLINGFEEDBACK abstract class for casting a control system in output feedback form and specify the needed constraints on the resulting gain matrix, when a coupling controller feedback is needed

	properties(SetAccess=protected)
		% number of coupling conditions
		number_couplingconditions
		% number of references
		number_references
	end

	methods
		function [this] = AbstractCouplingFeedback(number_couplingconditions, number_references)
			%ABSTRACTCOUPLINGFEEDBACK create new coupling feedback
			%	Input:
			%		number_couplingconditions:	number of coupling conditions in C_ref
			%		number_references:			number of references
			%	Output:
			%		this:						instance
			this@control.design.outputfeedback.AbstractDecouplingFeedback();
			narginchk(2, 2);
			this.number_couplingconditions = number_couplingconditions;
			this.number_references = number_references;
			if number_couplingconditions >=	number_references
				error('control:design:outputfeedback:input', 'Number of coupling conditions (%d) must be smaller than number of references (%d).', number_couplingconditions, number_references);
			end
			tf_structure = NaN(number_references, number_references);
			tf_structure(end - number_couplingconditions + 1:end, 1:number_references - number_couplingconditions) = 0;
			this.tf_structure = tf_structure;
		end

		function set.number_couplingconditions(this, number_couplingconditions)
			%NUMBER_COUPLINGCONDITIONS set number of coupling conditions in C_ref
			%	Input:
			%		this:						instance
			%		number_couplingconditions:	number of coupling conditions in C_ref
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

		function set.number_references(this, number_references)
			%NUMBER_REFERENCES set number of references
			%	Input:
			%		this:				instance
			%		number_references:	number of reference signals
			if ~isnumeric(number_references) || ~isscalar(number_references)
				error('control:design:outputfeedback:input', 'Number of references must be a numeric scalar.');
			end
			if isinf(number_references) || isnan(number_references)
				error('control:design:outputfeedback:input', 'Number of references must be finite.');
			end
			if floor(number_references) ~= ceil(number_references)
				error('control:design:outputfeedback:input', 'Number of references must be an integer.');
			end
			if number_references <= 0
				error('control:design:outputfeedback:input', 'Number of references must be greater than 0.');
			end
			this.number_references = uint32(number_references);
		end
	end
end
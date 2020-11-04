classdef(Abstract) AbstractDecouplingFeedback < control.design.outputfeedback.OutputFeedback
	%ABSTRACTDECOUPLINGFEEDBACK abstract class for casting a control system in output feedback form and specify the needed constraints on the resulting gain matrix, when a decoupling controller feedback is needed

	properties(SetAccess=protected)
		% structure of the closed-loop transfer matrix
		tf_structure
	end

	methods
		function [this] = AbstractDecouplingFeedback()
			%ABSTRACTDECOUPLINGFEEDBACK create new decoupling feedback
			%	Input:
			%		tf_structure:	structure of the closed-loop transfer matrix
			%	Output:
			%		this:			instance
			this@control.design.outputfeedback.OutputFeedback();
			narginchk(0, Inf);
		end

		function [this] = set.tf_structure(this, tf_structure)
			%TF_STRUCTURE structure of the closed-loop transfer matrix
			%	Input:
			%		this:			instance
			%		tf_structure:	structure of the closed-loop transfer matrix
			%	Output:
			%		this:			instance
			if ~isnumeric(tf_structure) || ndims(tf_structure) > 2 %#ok<ISMAT> compatibility with Octave
				error('control:design:outputfeedback:input', 'Transfer function strucure must be a numeric matrix.');
			end
			if size(tf_structure, 1) ~= size(tf_structure, 2)
				error('control:design:outputfeedback:input', 'Transfer function strucure must be a quadratic matrix.');
			end
			tf_structure_tmp = tf_structure(:);
			tf_structure_tmp(isnan(tf_structure_tmp)) = [];
			tf_structure_tmp(tf_structure_tmp == 0) = [];
			if ~isempty(tf_structure_tmp)
				error('control:design:outputfeedback:input', 'Transfer function strucure must only contain zero and NaN elements.');
			end
			this.tf_structure = tf_structure;
		end

		function [decouplingoptions] = get_decouplingoptions(this, decouplingoptions)
			%GET_DECOUPLINGOPTIONS return structure with options for decoupling controller design
			%	Input:
			%		this:				instance
			%		options:			structure with options to append to
			%	Output:
			%		decouplingoptions:	structure with appended decoupling options
			number_references = size(this.tf_structure, 1);
			if nargin <= 2
				decouplingoptions = struct();
			end
			if ~isstruct(decouplingoptions)
				error('control:design:outputfeedback:input', 'Decoupling options must be a structure.');
			end
			if ~isfield(decouplingoptions, 'decouplingcontrol')
				decouplingstruct = control.design.gamma.objectiveoptions_prototype_decoupling(number_references);
			else
				decouplingstruct = mergestruct(decouplingoptions.decouplingcontrol, control.design.gamma.objectiveoptions_prototype_decoupling(number_references));
			end
			decouplingoptions.decouplingcontrol = this.get_decouplingoptions_system(decouplingstruct);
			decouplingoptions.decouplingcontrol.tf_structure = this.tf_structure;
		end
	end

	methods(Abstract=true, Access=protected)
		%GET_DECOUPLINGOPTIONS_SYSTEM return structure with options for decoupling controller design
		%	Input:
		%		this:				instance
		%		options:			structure with options to append to
		%	Output:
		%		decouplingoptions:	structure with appended decoupling options
		[decouplingoptions] = get_decouplingoptions_system(this, decouplingoptions);
	end
end
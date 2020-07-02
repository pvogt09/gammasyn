classdef(Abstract) AbstractDynamicFeedback < control.design.outputfeedback.OutputFeedback
	%ABSTRACTREFERENCEMODELFEEDBACK abstract class for casting a control system in output feedback form and specify the needed constraints on the resulting gain matrix, when a dynamic feedback of specified order is needed
	
	properties(SetAccess=protected)
		% order of dynamic feedback
		n
	end
	
	methods
		function [this] = AbstractDynamicFeedback(order)
			%ABSTRACTDYNAMICFEEDBACK create new dynamic feedback
			%	Input:
			%		order:		order of dynamic feedback
			%	Output:
			%		this:		instance
			this@control.design.outputfeedback.OutputFeedback();
			if ~isscalar(order) || ~isnumeric(order) || order < 0 || floor(order) ~= ceil(order)
				error('control:design:outputfeedback:input', 'Dynamic system order must be a positive number.');
			end
			this.n = order;
		end
	end
end
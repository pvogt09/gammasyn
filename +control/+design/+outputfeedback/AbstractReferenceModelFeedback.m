classdef(Abstract) AbstractReferenceModelFeedback < control.design.outputfeedback.AbstractDynamicModelFeedback
	%ABSTRACTREFERENCEMODELFEEDBACK abstract class for casting a control system in output feedback form and specify the needed constraints on the resulting gain matrix, when a reference model is needed

	methods
		function [this] = AbstractReferenceModelFeedback(system, varargin)
			%ABSTRACTREFERENCEMODELFEEDBACK create new feedback with reference model
			%	Input:
			%		this:		instance
			%		system:		state space system or structure with system matrices or descriptor matrix
			%		A:			system matrix or sample time if system is given as first argument
			%		B:			control matrix
			%		C:			output matrix
			%		C_dot:		derivative output matrix
			%		D:			throughput matrix
			%		C_ref:		measurement matrix for reference outputs
			%		D_ref:		throughput matrix for reference outputs
			%		T:			sampling time
			%	Output:
			%		this:		instance
			this@control.design.outputfeedback.AbstractDynamicModelFeedback(system, varargin{:});
			if any(isnan(this.E_model_ref(:)))
				error('control:design:outputfeedback:input', 'Reference model descriptor matrix E must not contain NaN.');
			end
			if any(isnan(this.A_model_ref(:)))
				error('control:design:outputfeedback:input', 'Reference model system matrix A must not contain NaN.');
			end
			if any(isnan(this.B_model_ref(:)))
				error('control:design:outputfeedback:input', 'Reference model control matrix B must not contain NaN.');
			end
			if any(isnan(this.C_model_ref(:)))
				error('control:design:outputfeedback:input', 'Reference model output matrix C must not contain NaN.');
			end
			if any(isnan(this.C_dot_model_ref(:)))
				error('control:design:outputfeedback:input', 'Reference model derivative output matrix C_dot must not contain NaN.');
			end
			if any(isnan(this.D_model_ref(:)))
				error('control:design:outputfeedback:input', 'Reference model throughput matrix D must not contain NaN.');
			end
			if any(isnan(this.C_ref_model_ref(:)))
				error('control:design:outputfeedback:input', 'Reference model reference output matrix C_ref must not contain NaN.');
			end
			if any(isnan(this.D_ref_model_ref(:)))
				error('control:design:outputfeedback:input', 'Reference model reference throughput matrix D_ref must not contain NaN.');
			end
		end
	end
end
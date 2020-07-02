classdef(Abstract) AbstractDynamicOrderModelFeedback < control.design.outputfeedback.AbstractDynamicModelFeedback
	%ABSTRACTDYNAMICORDERMODELFEEDBACK abstract class for casting a control system in output feedback form and specify the needed constraints on the resulting gain matrix, when a reference model is needed
	
	methods
		function [this] = AbstractDynamicOrderModelFeedback(system, varargin)
			%ABSTRACTDYNAMICORDERMODELFEEDBACK create new feedback with dynamic model of specified order
			%	Input:
			%		this:		instance
			%		system:		state space system or structure with system matrices or descriptor matrix
			%		A:			system matrix or sample time if system is given as first argument
			%		B:			control matrix
			%		C:			output matrix
			%		C_dot:		derivative output matrix
			%		D:			throughput matrix
			%		T:			sampling time
			%	Output:
			%		this:		instance
			this@control.design.outputfeedback.AbstractDynamicModelFeedback(system, varargin{:});
			nnom = size(this.A_model_ref, 1);
			if size(this.B_model_ref, 2) < nnom
				this.B_model_ref = [
					this.B_model_ref, NaN(nnom, nnom - size(this.B_model_ref, 2))
				];
			end
			if size(this.C_model_ref, 1) < nnom
				this.C_model_ref = [
					this.C_model_ref;
					NaN(nnom - size(this.C_model_ref, 1), nnom)
				];
			end
			if size(this.C_dot_model_ref, 1) < nnom
				this.C_dot_model_ref = [
					this.C_dot_model_ref;
					NaN(nnom - size(this.C_dot_model_ref, 1), nnom)
				];
			end
			if size(this.D_model_ref, 2) < size(this.B_model_ref, 2)
				this.D_model_ref = [
					this.D_model_ref, NaN(size(this.D_model_ref, 1), size(this.B_model_ref, 1) - size(this.D_model_ref, 2))
				];
			end
			if size(this.D_model_ref, 1) < size(this.C_model_ref, 1)
				this.D_model_ref = [
					this.D_model_ref;
					NaN(size(this.C_model_ref, 1) - size(this.D_model_ref, 1), size(this.D_model_ref, 2))
				];
			end
		end
	end
end
function [R, K, F, RKF] = input_gain_constraint2RKF(constraint)
	%INPUT_GAIN_CONSTRAINT2RKF convert constraint information (equality or inequality) to constraints for different gain matrices
	%	Input:
	%		constraint:	cell array with indicator matrix for proportional gain elements that should be fixed and the values the fixed gains have or 3D matrix where fixed elements are the non zero elements in the first and second dimension or (experimental) cell array with symbolic expression for the gain coefficients in the first dimension and equation system in the second
	%	Output:
	%		R:			part of the constraint system belonging to proportional gain
	%		K:			part of the constraint system belonging to derivative gain
	%		F:			part of the constraint system belonging to prefilter gain
	%		RKF:		part of the constraint system belonging to all gains
	if nargin <= 0
		constraint = [];
	end
	if iscell(constraint)
		if numel(constraint) >= 4
			emptyarg = [
				isempty(constraint{1});
				isempty(constraint{2});
				isempty(constraint{3});
				isempty(constraint{4})
			];
			cellarg = [
				iscell(constraint{1});
				iscell(constraint{2});
				iscell(constraint{3});
				iscell(constraint{4})
			];
			if all(cellarg(:)) || all(emptyarg(:)) || all(cellarg | emptyarg)
				RKF = constraint{4};
				F = constraint{3};
				K = constraint{2};
				R = constraint{1};
			else
				if numel(constraint) >= 8
					RKF = constraint{7:8};
					F = constraint{5:6};
					K = constraint{3:4};
					R = constraint{1:2};
				elseif numel(constraint) >= 6
					RKF = [];
					F = constraint{5:6};
					K = constraint{3:4};
					R = constraint{1:2};
				elseif numel(constraint) >= 4
					RKF = [];
					F = [];
					K = constraint{3:4};
					R = constraint{1:2};
				else
					RKF = [];
					F = [];
					K = [];
					R = constraint;
				end
			end
		elseif numel(constraint) >= 3
			emptyarg = [
				isempty(constraint{1});
				isempty(constraint{2});
				isempty(constraint{3})
			];
			cellarg = [
				iscell(constraint{1});
				iscell(constraint{2});
				iscell(constraint{3})
			];
			if all(cellarg(:)) || all(emptyarg(:)) || all(cellarg & emptyarg)
				RKF = [];
				F = constraint{3};
				K = constraint{2};
				R = constraint{1};
			else
				if numel(constraint) >= 8
					RKF = constraint{7:8};
					F = constraint{5:6};
					K = constraint{3:4};
					R = constraint{1:2};
				elseif numel(constraint) >= 6
					RKF = [];
					F = constraint{5:6};
					K = constraint{3:4};
					R = constraint{1:2};
				elseif numel(constraint) >= 4
					RKF = [];
					F = [];
					K = constraint{3:4};
					R = constraint{1:2};
				else
					RKF = [];
					F = [];
					K = [];
					R = constraint;
				end
			end
		elseif numel(constraint) >= 2
			emptyarg = [
				isempty(constraint{1});
				isempty(constraint{2})
			];
			cellarg = [
				iscell(constraint{1});
				iscell(constraint{2})
			];
			if all(cellarg(:)) || all(emptyarg(:)) || all(cellarg & emptyarg)
				RKF = [];
				F = [];
				K = constraint{2};
				R = constraint{1};
			else
				if numel(constraint) >= 8
					RKF = constraint{7:8};
					F = constraint{5:6};
					K = constraint{3:4};
					R = constraint{1:2};
				elseif numel(constraint) >= 6
					RKF = [];
					F = constraint{5:6};
					K = constraint{3:4};
					R = constraint{1:2};
				elseif numel(constraint) >= 4
					RKF = [];
					F = [];
					K = constraint{3:4};
					R = constraint{1:2};
				else
					RKF = [];
					F = [];
					K = [];
					R = constraint;
				end
			end
		else
			RKF = [];
			F = [];
			K = [];
			R = constraint;
		end
	else
		RKF = [];
		F = [];
		K = [];
		R = constraint;
	end
end
function [grad, funEvals] = finitedifference(x, fun, funelements, type, argumentidx, lb, ub, FinDiffRelStep, TypicalX, AlwaysHonorBounds, nosanitycheck)
	%FINITEDIFFERENCE calculate finite difference gradient for function
	%	Input:
	%		x:					point to evaluate the gradient at
	%		fun:				function to evaluate
	%		funelements:		number of elements, the function returns
	%		type:				type of finite difference
	%		argumentidx:		index of argument in return list of function
	%		lb:					lower bound for evaluation point
	%		ub:					upper bound for evaluation point
	%		FinDiffRelStep:		relative finite difference step
	%		TypicalX:			typical values of the evaluation point
	%		AlwaysHonorBounds:	indicator, if bounds should be honored when calculating evaluation points
	%		nosanitycheck:		indicator, if sanity checks for input arguments should be skipped
	%	Output:
	%		grad:				finite difference gradient
	%		funEvals:			number of function evaluations needed for gradient evaluation
	if nargin <= 6
		TypicalX = x;
	end
	if nargin <= 7
		AlwaysHonorBounds = false;
	end
	if nargin <= 8
		nosanitycheck = false;
	end
	dim = size(x, 1);
	if isempty(TypicalX)
		TypicalX = x;
	end
	if isempty(AlwaysHonorBounds)
		AlwaysHonorBounds = false;
	end
	workers = uint32(min([max([1, floor(dim/5) - 20]), configuration.matlab.numthreads()]));
	if ~nosanitycheck
		if ndims(x) >= 3
			error('optimization:general:finitedifference', 'evaluation point must be a vector.');
		end
		if size(x, 2) ~= 1
			error('optimization:general:finitedifference', 'evaluation point must be a column vector.');
		end
		if ischar(type)
			type = optimization.general.FinDiffType.fromname(type);
		elseif isnumeric(type)
			type = optimization.general.FinDiffType.frominteger(type);
		end
		if ~isa(type, 'optimization.general.FinDiffType')
			error('optimization:general:finitedifference', 'finite difference type must be of type ''optimization.general.FinDiffType''.');
		end
		if isempty(lb)
			lb = -Inf(dim, 1);
		end
		if isempty(ub)
			ub = Inf(dim, 1);
		end
		if ndims(lb) ~= ndims(x) || any(size(lb) ~= size(x))
			error('optimization:general:finitedifference', 'size of lower bounds must match size of evaluation point.');
		end
		if ndims(ub) ~= ndims(x) || any(size(ub) ~= size(x))
			error('optimization:general:finitedifference', 'size of upper bounds must match size of evaluation point.');
		end
		if isempty(FinDiffRelStep)
			switch type
				case optimization.general.FinDiffType.FORWARD
					FinDiffRelStep = sqrt(eps);
				case optimization.general.FinDiffType.BACKWARD
					FinDiffRelStep = sqrt(eps);
				case optimization.general.FinDiffType.CENTRAL
					FinDiffRelStep = eps^(1/3);
				otherwise
					FinDiffRelStep = sqrt(eps);
			end
		end
		if isscalar(FinDiffRelStep)
			FinDiffRelStep = FinDiffRelStep*ones(dim, 1);
		end
		if ndims(FinDiffRelStep) ~= ndims(x) || any(size(FinDiffRelStep) ~= size(x))
			error('optimization:general:finitedifference', 'size of finite difference relative step must match size of evaluation point.');
		end
		if isempty(TypicalX)
			TypicalX = x;
		end
		if ndims(TypicalX) ~= ndims(x) || any(size(TypicalX) ~= size(x))
			error('optimization:general:finitedifference', 'size of typical x must match size of evaluation point.');
		end
		if ~islogical(AlwaysHonorBounds) || ~isscalar(AlwaysHonorBounds)
			error('optimization:general:finitedifference', 'indicator for bound honoration must be a logical scalar.');
		end
	end
	if isempty(FinDiffRelStep)
		switch type
			case optimization.general.FinDiffType.FORWARD
				FinDiffRelStep = sqrt(eps)*ones(dim, 1);
			case optimization.general.FinDiffType.BACKWARD
				FinDiffRelStep = sqrt(eps)*ones(dim, 1);
			case optimization.general.FinDiffType.CENTRAL
				FinDiffRelStep = eps^(1/3)*ones(dim, 1);
			otherwise
				FinDiffRelStep = sqrt(eps)*ones(dim, 1);
		end
	end
	if AlwaysHonorBounds
		if isempty(lb)
			lb = -Inf(dim, 1);
		end
		if isempty(ub)
			ub = Inf(dim, 1);
		end
	end
	delta = FinDiffRelStep.*(1 + max([norm(x), norm(TypicalX)]));
	switch type
		case optimization.general.FinDiffType.FORWARD
			if ~isnan(funelements) && funelements == 0
				diff = zeros(dim, funelements);
			else
				if argumentidx == 1
					f = fun(x);
				elseif argumentidx == 2
					[~, f] = fun(x);
				elseif argumentidx == 3
					[~, ~, f] = fun(x);
				elseif argumentidx == 4
					[~, ~, ~, f] = fun(x);
				elseif argumentidx == 5
					[~, ~, ~, ~, f] = fun(x);
				else
					error('optimization:general:finitedifference', 'argument index %d is not implemented.', argumentidx);
				end
				mu = 2*delta;
				diff = zeros(dim, size(f, 1));
				if isempty(f)
					funEvals = 1;
				else
					funEvals = 1 + dim;
					if workers <= 1
						for jj = 1:dim %#ok<FORPF> no parfor because of user setting
							e_j = zeros(dim, 1);
							e_j(jj, 1) = 1;
							if AlwaysHonorBounds
								if argumentidx == 1
									diff(jj, :) = fun(min([max([lb, x + mu.*e_j], [], 2), ub], [], 2));
								elseif argumentidx == 2
									[~, diff(jj, :)] = fun(min([max([lb, x + mu.*e_j], [], 2), ub], [], 2));
								elseif argumentidx == 3
									[~, ~, diff(jj, :)] = fun(min([max([lb, x + mu.*e_j], [], 2), ub], [], 2));
								elseif argumentidx == 4
									[~, ~, ~, diff(jj, :)] = fun(min([max([lb, x + mu.*e_j], [], 2), ub], [], 2));
								elseif argumentidx == 5
									[~, ~, ~, ~, diff(jj, :)] = fun(min([max([lb, x + mu.*e_j], [], 2), ub], [], 2));
								else
									error('optimization:general:finitedifference', 'argument index %d is not implemented.', argumentidx);
								end
							else
								if argumentidx == 1
									diff(jj, :) = fun(x + mu.*e_j);
								elseif argumentidx == 2
									[~, diff(jj, :)] = fun(x + mu.*e_j);
								elseif argumentidx == 3
									[~, ~, diff(jj, :)] = fun(x + mu.*e_j);
								elseif argumentidx == 4
									[~, ~, ~, diff(jj, :)] = fun(x + mu.*e_j);
								elseif argumentidx == 5
									[~, ~, ~, ~, diff(jj, :)] = fun(x + mu.*e_j);
								else
									error('optimization:general:finitedifference', 'argument index %d is not implemented.', argumentidx);
								end
							end
							diff(jj, :) = (diff(jj, :) - f.')/mu(jj, 1);
						end
					else
						parfor (jj = 1:dim, workers)
							e_j = zeros(dim, 1);
							e_j(jj, 1) = 1;
							if AlwaysHonorBounds
								if argumentidx == 1
									diff(jj, :) = fun(min([max([lb, x + mu.*e_j], [], 2), ub], [], 2));
								elseif argumentidx == 2
									[~, diff(jj, :)] = fun(min([max([lb, x + mu.*e_j], [], 2), ub], [], 2));
								elseif argumentidx == 3
									[~, ~, diff(jj, :)] = fun(min([max([lb, x + mu.*e_j], [], 2), ub], [], 2));
								elseif argumentidx == 4
									[~, ~, ~, diff(jj, :)] = fun(min([max([lb, x + mu.*e_j], [], 2), ub], [], 2));
								elseif argumentidx == 5
									[~, ~, ~, ~, diff(jj, :)] = fun(min([max([lb, x + mu.*e_j], [], 2), ub], [], 2));
								else
									error('optimization:general:finitedifference', 'argument index %d is not implemented.', argumentidx);
								end
							else
								if argumentidx == 1
									diff(jj, :) = fun(x + mu.*e_j);
								elseif argumentidx == 2
									[~, diff(jj, :)] = fun(x + mu.*e_j);
								elseif argumentidx == 3
									[~, ~, diff(jj, :)] = fun(x + mu.*e_j);
								elseif argumentidx == 4
									[~, ~, ~, diff(jj, :)] = fun(x + mu.*e_j);
								elseif argumentidx == 5
									[~, ~, ~, ~, diff(jj, :)] = fun(x + mu.*e_j);
								else
									error('optimization:general:finitedifference', 'argument index %d is not implemented.', argumentidx);
								end
							end
							diff(jj, :) = (diff(jj, :) - f.')/mu(jj, 1);
						end
					end
				end
			end
			grad = diff.';
		case optimization.general.FinDiffType.BACKWARD
			if ~isnan(funelements) && funelements == 0
				diff = zeros(dim, funelements);
			else
				if argumentidx == 1
					f = fun(x);
				elseif argumentidx == 2
					[~, f] = fun(x);
				elseif argumentidx == 3
					[~, ~, f] = fun(x);
				elseif argumentidx == 4
					[~, ~, ~, f] = fun(x);
				elseif argumentidx == 5
					[~, ~, ~, ~, f] = fun(x);
				else
					error('optimization:general:finitedifference', 'argument index %d is not implemented.', argumentidx);
				end
				mu = 2*delta;
				diff = zeros(dim, size(f, 1));
				if isempty(f)
					funEvals = 1;
				else
					funEvals = 1 + dim;
					if workers <= 1
						for jj = 1:dim %#ok<FORPF> no parfor because of user setting
							e_j = zeros(dim, 1);
							e_j(jj, 1) = 1;
							if AlwaysHonorBounds
								if argumentidx == 1
									diff(jj, :) = fun(min([max([lb, x - mu.*e_j], [], 2), ub], [], 2));
								elseif argumentidx == 2
									[~, diff(jj, :)] = fun(min([max([lb, x - mu.*e_j], [], 2), ub], [], 2));
								elseif argumentidx == 3
									[~, ~, diff(jj, :)] = fun(min([max([lb, x - mu.*e_j], [], 2), ub], [], 2));
								elseif argumentidx == 4
									[~, ~, ~, diff(jj, :)] = fun(min([max([lb, x - mu.*e_j], [], 2), ub], [], 2));
								elseif argumentidx == 5
									[~, ~, ~, ~, diff(jj, :)] = fun(min([max([lb, x - mu.*e_j], [], 2), ub], [], 2));
								else
									error('optimization:general:finitedifference', 'argument index %d is not implemented.', argumentidx);
								end
							else
								if argumentidx == 1
									diff(jj, :) = fun(x - mu.*e_j);
								elseif argumentidx == 2
									[~, diff(jj, :)] = fun(x - mu.*e_j);
								elseif argumentidx == 3
									[~, ~, diff(jj, :)] = fun(x - mu.*e_j);
								elseif argumentidx == 4
									[~, ~, ~, diff(jj, :)] = fun(x - mu.*e_j);
								elseif argumentidx == 5
									[~, ~, ~, ~, diff(jj, :)] = fun(x - mu.*e_j);
								else
									error('optimization:general:finitedifference', 'argument index %d is not implemented.', argumentidx);
								end
							end
							diff(jj, :) = (diff(jj, :) - f.')/mu(jj, 1);
						end
					else
						parfor (jj = 1:dim, workers)
							e_j = zeros(dim, 1);
							e_j(jj, 1) = 1;
							if AlwaysHonorBounds
								if argumentidx == 1
									diff(jj, :) = fun(min([max([lb, x - mu.*e_j], [], 2), ub], [], 2));
								elseif argumentidx == 2
									[~, diff(jj, :)] = fun(min([max([lb, x - mu.*e_j], [], 2), ub], [], 2));
								elseif argumentidx == 3
									[~, ~, diff(jj, :)] = fun(min([max([lb, x - mu.*e_j], [], 2), ub], [], 2));
								elseif argumentidx == 4
									[~, ~, ~, diff(jj, :)] = fun(min([max([lb, x - mu.*e_j], [], 2), ub], [], 2));
								elseif argumentidx == 5
									[~, ~, ~, ~, diff(jj, :)] = fun(min([max([lb, x - mu.*e_j], [], 2), ub], [], 2));
								else
									error('optimization:general:finitedifference', 'argument index %d is not implemented.', argumentidx);
								end
							else
								if argumentidx == 1
									diff(jj, :) = fun(x - mu.*e_j);
								elseif argumentidx == 2
									[~, diff(jj, :)] = fun(x - mu.*e_j);
								elseif argumentidx == 3
									[~, ~, diff(jj, :)] = fun(x - mu.*e_j);
								elseif argumentidx == 4
									[~, ~, ~, diff(jj, :)] = fun(x - mu.*e_j);
								elseif argumentidx == 5
									[~, ~, ~, ~, diff(jj, :)] = fun(x - mu.*e_j);
								else
									error('optimization:general:finitedifference', 'argument index %d is not implemented.', argumentidx);
								end
							end
							diff(jj, :) = (diff(jj, :) - f.')/mu(jj, 1);
						end
					end
				end
			end
			grad = diff.';
		case optimization.general.FinDiffType.CENTRAL
			if ~isnan(funelements) && funelements == 0
				grad = zeros(dim, funelements);
			else
				mu = 2*delta;
				if argumentidx == 1
					f = fun(x);
				elseif argumentidx == 2
					[~, f] = fun(x);
				elseif argumentidx == 3
					[~, ~, f] = fun(x);
				elseif argumentidx == 4
					[~, ~, ~, f] = fun(x);
				elseif argumentidx == 5
					[~, ~, ~, ~, f] = fun(x);
				else
					error('optimization:general:finitedifference', 'argument index %d is not implemented.', argumentidx);
				end
				diff1 = zeros(dim, size(f, 1));
				diff2 = zeros(dim, size(f, 1));
				funEvals = 2*dim + 1;
				if workers <= 1
					for jj = 1:dim %#ok<FORPF> no parfor because of user setting
						e_j = zeros(dim, 1);
						e_j(jj, 1) = 1;
						if AlwaysHonorBounds
							if argumentidx == 1
								diff1(jj, :) = fun(min([max([lb, x + mu.*e_j], [], 2), ub], [], 2));
								diff2(jj, :) = fun(min([max([lb, x - mu.*e_j], [], 2), ub], [], 2));
							elseif argumentidx == 2
								[~, diff1(jj, :)] = fun(min([max([lb, x + mu.*e_j], [], 2), ub], [], 2));
								[~, diff2(jj, :)] = fun(min([max([lb, x - mu.*e_j], [], 2), ub], [], 2));
							elseif argumentidx == 3
								[~, ~, diff1(jj, :)] = fun(min([max([lb, x + mu.*e_j], [], 2), ub], [], 2));
								[~, ~, diff2(jj, :)] = fun(min([max([lb, x - mu.*e_j], [], 2), ub], [], 2));
							elseif argumentidx == 4
								[~, ~, ~, diff1(jj, :)] = fun(min([max([lb, x + mu.*e_j], [], 2), ub], [], 2));
								[~, ~, ~, diff2(jj, :)] = fun(min([max([lb, x - mu.*e_j], [], 2), ub], [], 2));
							elseif argumentidx == 5
								[~, ~, ~, ~, diff1(jj, :)] = fun(min([max([lb, x + mu.*e_j], [], 2), ub], [], 2));
								[~, ~, ~, ~, diff2(jj, :)] = fun(min([max([lb, x - mu.*e_j], [], 2), ub], [], 2));
							else
								error('optimization:general:finitedifference', 'argument index %d is not implemented.', argumentidx);
							end
						else
							if argumentidx == 1
								diff1(jj, :) = fun(x + mu.*e_j);
								diff2(jj, :) = fun(x - mu.*e_j);
							elseif argumentidx == 2
								[~, diff1(jj, :)] = fun(x + mu.*e_j);
								[~, diff2(jj, :)] = fun(x - mu.*e_j);
							elseif argumentidx == 3
								[~, ~, diff1(jj, :)] = fun(x + mu.*e_j);
								[~, ~, diff2(jj, :)] = fun(x - mu.*e_j);
							elseif argumentidx == 4
								[~, ~, ~, diff1(jj, :)] = fun(x + mu.*e_j);
								[~, ~, ~, diff2(jj, :)] = fun(x - mu.*e_j);
							elseif argumentidx == 5
								[~, ~, ~, ~, diff1(jj, :)] = fun(x + mu.*e_j);
								[~, ~, ~, ~, diff2(jj, :)] = fun(x - mu.*e_j);
							else
								error('optimization:general:finitedifference', 'argument index %d is not implemented.', argumentidx);
							end
						end
					end
				else
					parfor (jj = 1:dim, workers)
						e_j = zeros(dim, 1);
						e_j(jj, 1) = 1;
						if AlwaysHonorBounds
							if argumentidx == 1
								diff1(jj, :) = fun(min([max([lb, x + mu.*e_j], [], 2), ub], [], 2));
								diff2(jj, :) = fun(min([max([lb, x - mu.*e_j], [], 2), ub], [], 2));
							elseif argumentidx == 2
								[~, diff1(jj, :)] = fun(min([max([lb, x + mu.*e_j], [], 2), ub], [], 2));
								[~, diff2(jj, :)] = fun(min([max([lb, x - mu.*e_j], [], 2), ub], [], 2));
							elseif argumentidx == 3
								[~, ~, diff1(jj, :)] = fun(min([max([lb, x + mu.*e_j], [], 2), ub], [], 2));
								[~, ~, diff2(jj, :)] = fun(min([max([lb, x - mu.*e_j], [], 2), ub], [], 2));
							elseif argumentidx == 4
								[~, ~, ~, diff1(jj, :)] = fun(min([max([lb, x + mu.*e_j], [], 2), ub], [], 2));
								[~, ~, ~, diff2(jj, :)] = fun(min([max([lb, x - mu.*e_j], [], 2), ub], [], 2));
							elseif argumentidx == 5
								[~, ~, ~, ~, diff1(jj, :)] = fun(min([max([lb, x + mu.*e_j], [], 2), ub], [], 2));
								[~, ~, ~, ~, diff2(jj, :)] = fun(min([max([lb, x - mu.*e_j], [], 2), ub], [], 2));
							else
								error('optimization:general:finitedifference', 'argument index %d is not implemented.', argumentidx);
							end
						else
							if argumentidx == 1
								diff1(jj, :) = fun(x + mu.*e_j);
								diff2(jj, :) = fun(x - mu.*e_j);
							elseif argumentidx == 2
								[~, diff1(jj, :)] = fun(x + mu.*e_j);
								[~, diff2(jj, :)] = fun(x - mu.*e_j);
							elseif argumentidx == 3
								[~, ~, diff1(jj, :)] = fun(x + mu.*e_j);
								[~, ~, diff2(jj, :)] = fun(x - mu.*e_j);
							elseif argumentidx == 4
								[~, ~, ~, diff1(jj, :)] = fun(x + mu.*e_j);
								[~, ~, ~, diff2(jj, :)] = fun(x - mu.*e_j);
							elseif argumentidx == 5
								[~, ~, ~, ~, diff1(jj, :)] = fun(x + mu.*e_j);
								[~, ~, ~, ~, diff2(jj, :)] = fun(x - mu.*e_j);
							else
								error('optimization:general:finitedifference', 'argument index %d is not implemented.', argumentidx);
							end
						end
					end
				end
				grad = ((diff1 - diff2)./(2*repmat(mu.', size(f, 1), 1))).';
			end
		otherwise
			error('optimization:general:finitedifference', 'unknown finite difference type.');
	end
end
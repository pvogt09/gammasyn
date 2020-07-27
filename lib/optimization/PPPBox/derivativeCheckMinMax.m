function [diff, g, g2] = derivativeCheckMinMax(funObj, x, order, type, varargin)
	% diff = derivativeCheck(funObj,x,order,useComplex,varargin)
	%
	% type = 1 (simple forward-difference)
	% type = 2 (central differencing - default)
	% type = 3 (complex-step deriative)
	diff = [];
	g = [];
	g2 = [];

	if nargin < 3
		order = 1; % Only check gradient by default
		if nargin < 4
			type = 2; % Use central-differencing by default
		end
	end
	isfunvararg = nargin(funObj) ~= 1;
	if order == 2
		if isfunvararg
			[f, g, H] = funObj(x, varargin{:});
		else
			[f, g, H] = funObj(x);
		end

		fprintf('Checking Hessian...\n');
		[f2, g2, H2] = autoHess(x, type, funObj, varargin{:});

		fprintf('Max difference between user and numerical hessian: %e\n',max(abs(H(:)-H2(:))));
		if max(abs(H(:)-H2(:))) > 1e-4
			H
			H2
			diff = abs(H-H2)
			pause;
		end
	else
		if isfunvararg
			[f, g] = funObj(x, varargin{:});
		else
			[f, g] = funObj(x);
		end

		fprintf('Checking Gradient...\n\n');
		[f2, g2] = autoGradMinMax(x, type, funObj, varargin{:});

		for k = 1:length(f)
			fprintf('Max difference between user and numerical gradient in function %d: %e\n',k,max(abs(g(:,k)-g2(:,k))));
		end
		% Absolute error
		[maxC, indC] = max(abs(g-g2));
		[maxA, indA] = max(maxC);

		% Relative Error
		[maxCg, indCg] = max(abs(g));
		[maxRel, indAg] = max(maxC./maxCg);
		if maxA > 1e-4
			fprintf('\nWarning! Gradients might be inaccurate.\n')
			fprintf('Maximal absolute error in function %d, element %d: %e\n', indA, indC(indA), maxA);
			[g g2];
			diff = abs(g-g2);
		end
	%     if maxRel > 1e-6
	% 		fprintf('\nWarning! Gradients might be inaccurate.\n')
	%         fprintf('Maximal relative error in function %d, element %d: %e\n', indA, indC(indA), maxRel);
	% 		[g g2]
	% 		diff = abs(g-g2);
	%     end
		fprintf('\n')
	end

end % function

function [f,g] = autoGradMinMax(x,type,funObj,varargin)
% [f,g] = autoGrad(x,useComplex,funObj,varargin)
%
% Numerically compute gradient of objective function from function values
%
% type =
%     1 - forward-differencing (p+1 evaluations)
%     2 - central-differencing (more accurate, but requires 2p evaluations)
%     3 - complex-step derivative (most accurate and only requires p evaluations, but only works for certain objectives)

p = length(x);

if type == 1 % Use Finite Differencing
	f = funObj(x,varargin{:});
	mu = 2*sqrt(1e-12)*(1+norm(x));
	diff = zeros(p,1);
	for j = 1:p
		e_j = zeros(p,1);
		e_j(j) = 1;
		diff(j,1) = funObj(x + mu*e_j,varargin{:});
	end
	g = (diff-f)/mu;
elseif type == 3 % Use Complex Differentials
	mu = 1e-150;
	diff = zeros(p,1);
	for j = 1:p
		e_j = zeros(p,1);
		e_j(j) = 1;
		diff(j,1) = funObj(x + mu*i*e_j,varargin{:});
	end

	f = mean(real(diff));
	g = imag(diff)/mu;
else % Use Central Differencing
	f = funObj(x,varargin{:});
	mu = 2*sqrt(1e-12)*(1+norm(x));
	diff1 = zeros(p,length(f));
	diff2 = zeros(p,length(f));
	for j = 1:p
		e_j = zeros(p,1);
		e_j(j) = 1;
		diff1(j,:) = funObj(x + mu*e_j,varargin{:})';
		diff2(j,:) = funObj(x - mu*e_j,varargin{:})';
	end
	f = mean([diff1;diff2]);
	g = (diff1 - diff2)/(2*mu);
end

if 0 % DEBUG CODE
	[fReal gReal] = funObj(x,varargin{:});
	[fReal f]
	[gReal g]
	diff
	pause;
end

end % function
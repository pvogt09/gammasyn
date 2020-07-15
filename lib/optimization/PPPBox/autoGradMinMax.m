function [f, g] = autoGradMinMax(x, type, funObj, varargin)
% [f,g] = autoGrad(x,useComplex,funObj,varargin)
%
% Numerically compute gradient of objective function from function values
%
% type =
%     1 - forward-differencing (p+1 evaluations)
%     2 - central-differencing (more accurate, but requires 2p evaluations)
%     3 - complex-step derivative (most accurate and only requires p evaluations, but only works for certain objectives)

p = length(x);
isfunvararg = nargin(funObj) ~= 1;

if type == 1 % Use Finite Differencing
	if isfunvararg
		f = funObj(x, varargin{:});
	else
		f = funObj(x);
	end
	mu = 2*sqrt(1e-12)*(1+norm(x));
	diff = zeros(p, 1);
	for j = 1:p
		e_j = zeros(p, 1);
		e_j(j) = 1;
		if isfunvararg
			diff(j, 1) = funObj(x + mu*e_j, varargin{:});
		else
			diff(j, 1) = funObj(x + mu*e_j);
		end
	end
	g = (diff - f)/mu;
elseif type == 3 % Use Complex Differentials
	mu = 1e-150;
	diff = zeros(p, 1);
	for j = 1:p
		e_j = zeros(p, 1);
		e_j(j) = 1;
		if isfunvararg
			diff(j, 1) = funObj(x + mu*1i*e_j, varargin{:});
		else
			diff(j, 1) = funObj(x + mu*1i*e_j);
		end
	end

	f = mean(real(diff));
	g = imag(diff)/mu;
else % Use Central Differencing
	if isfunvararg
		 f = funObj(x, varargin{:});
	else
	    f = funObj(x);
	end
	mu = 2*sqrt(1e-12)*(1+norm(x));
	diff1 = zeros(p, length(f));
	diff2 = zeros(p, length(f));
	for j = 1:p
		e_j = zeros(p, 1);
		e_j(j) = 1;
		if isfunvararg
			diff1(j, :) = funObj(x + mu*e_j, varargin{:})';
			diff2(j, :) = funObj(x - mu*e_j, varargin{:})';
		else
			diff1(j, :) = funObj(x + mu*e_j)';
			diff2(j, :) = funObj(x - mu*e_j)';
		end
	end
	f = mean([diff1;diff2]);
	g = (diff1 - diff2)/(2*mu);
end

if 0 % DEBUG CODE
	if isfunvararg
		[fReal gReal] = funObj(x, varargin{:});
	else
		[fReal gReal] = funObj(x);
	end
	[fReal f]
	[gReal g]
	diff
	pause;
end

end % function
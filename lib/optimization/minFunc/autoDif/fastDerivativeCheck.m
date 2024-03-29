function diff = derivativeCheck(funObj,x,order,type,varargin)
% diff = fastDerivativeCheck(funObj,x,order,varargin)

if nargin < 3
	order = 1; % Only check gradient by default
	if nargin < 4
		type = 2; % Use central-differencing by default
	end
end

p = length(x);
d = sign(randn(p,1));
if order == 2
	fprintf('Checking Hessian-vector product along random direction:\n');
	[f,g,H] = funObj(x,varargin{:});
	Hv = H*d;

	if type == 1 % Use Finite Differencing
		mu = 2*sqrt(1e-12)*(1+norm(x))/(1+norm(x));
		[diff,diffa] = funObj(x+d*mu,varargin{:});
		Hv2 = (diffa-g)/mu;
	elseif type == 3 % Use Complex Differentials
		mu = 1e-150;
		[diff,diffa] = funObj(x+d*mu*i,varargin{:});
		Hv2 = imag(diffa-g)/mu;
	else % Use Central Differencing
		mu = 2*sqrt(1e-12)*(1+norm(x))/(1+norm(x));
			[diff1,diffa] = funObj(x+d*mu,varargin{:});
			[diff2,diffb] = funObj(x-d*mu,varargin{:});
			Hv2 = (diffa-diffb)/(2*mu);
	end

	fprintf('Max difference between user and numerical Hessian-vector product: %e\n',max(abs(Hv-Hv2)));
else
	fprintf('Checking Gradient along random direction:\n');
	[f,g] = funObj(x,varargin{:});
	gtd = g'*d;

	if type == 1 % Use Finite Differencing
		mu = 2*sqrt(1e-12)*(1+norm(x))/(1+norm(x));
		diff = funObj(x+d*mu,varargin{:});
		gtd2 = (diff-f)/mu;
	elseif type == 3 % Use Complex Differentials
		mu = 1e-150;
		[diff,diffa] = funObj(x+d*mu*i,varargin{:});
		gtd2 = imag(diff)/mu;
	else % Use Central Differencing
		mu = 2*sqrt(1e-12)*(1+norm(x))/(1+norm(x));
		diff1 = funObj(x+d*mu,varargin{:});
		diff2 = funObj(x-d*mu,varargin{:});
		gtd2 = (diff1-diff2)/(2*mu);
	end

	fprintf('Max difference between user and numerical directional-derivative: %e\n',max(abs(gtd-gtd2)));
end
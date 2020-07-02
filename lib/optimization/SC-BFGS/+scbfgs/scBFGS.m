function [x, F, f, k, d, a, g] = scBFGS(obj, x, dmax, a0, a1, eta, theta, C, rho, tau, M, b)

% function [x,F] = scBFGS
%
% Author      : Frank E. Curtis
% Description : Self-correcting BFGS (scBFGS) algorithm
% Input       : obj   ~ loss function
%               x     ~ initial parameter vector
%               dmax  ~ accessed data point (ADP) limit
%               a0    ~ stepsize constants
%               a1    ~ ... for the formula alpha = 1/(a0 + k*a1)
%               eta   ~ BFGS displacement vector constants
%               theta ~ ... equation (9) in reference paper
%               C     ~ max iterations in SC-BFGS-sub in reference paper
%               rho   ~ input parameters
%               tau   ~ ... for SC-BFGS-sub in reference paper
%               M     ~ L-BFGS history length (0 ~ full BFGS)
% Output      : x     ~ final parameter vector
%               F     ~ vector of training errors obtained over the optimization process
%               f     ~ final objective function value
%               k     ~ number of iterations
%               d     ~ number of function evaluations
%               a     ~ steplength
%               g     ~ final gradient value

% Set global variables
%global b

% Initialize training error history
F = inf*ones(ceil(dmax/b+1),1);

% Set problem size
n = length(x);

% Initialize scaling matrix
if M == 0, W = eye(n,n); else m = 1; S = zeros(n,M); V = zeros(n,M); end;

% Compute stochastic gradient
[g,d] = stoch_grad(obj,x,0,b);

% Compute norm of subgradient
norms.g = norm(g);

% Compute matrix-vector product
if M == 0, Wg = W*g; else Wg = g; end;

% Compute norm of matrix-vector product
norms.Wg = norm(Wg);

% Initialize iteration counter
k = 1;

% Compute function value
f = obj_func(obj,x); F(k) = f;

% Compute stepsize
a = stepsize(a0,a1,k);

% Iteration loop
while d < dmax

  % Print info line
  if mod(k,20) == 1
    fprintf('============================================================+===================================================\n');
    fprintf('   ADP    Objective       ||g||      ||W*g||      Stepsize  |   theta(1)     theta(2)    sv/||s||^2   ||v||^2/sv\n');
    fprintf('============================================================+===================================================\n');
  end

  % Print information
  fprintf(' %6d  %+.4e  %+.4e  %+.4e  %+.4e',d,f,norms.g,norms.Wg,a);
  
  % Store current point
  x_ = x;
  
  % Store current gradient
  g_ = g;
  
  % Set step
  s = -a*Wg; if M > 0, S(:,m) = s; end;
  
  % Compute new point
  x = x_ + s;
  
  % Compute stochastic gradient
  [g,d] = stoch_grad(obj,x,d,b);

  % Compute norm of subgradient
  norms.g = norm(g);
  
  % Compute independent estimate
  if C > 0, [ghat,d] = stoch_grad(obj,x,d,b); end;
  
  % Store current scaling matrix
  if M == 0, W_ = W; end;
  
  % Initialize counter
  inner_count = 1;
  
  % Loop for consistency
  while 1
  
    % Set y
    y = g - g_;
  
    % Get betas
    betas = beta(s,a*y,eta,theta);
  
    % Set v
    v = max(betas)*s + (1-max(betas))*a*y; if M > 0, V(:,m) = v; end;
  
    % Set inner product
    sv = s'*v;

    % Print (s,v) info
    fprintf(' | %+.4e  %+.4e  %+.4e  %+.4e\n',betas(1),betas(2),sv/(s'*s),(v'*v)/sv);
  
    % Check history length
    if M == 0
    
      % Update scaling matrix
      W = (eye(n,n) - (v*s')/sv)'*W_*(eye(n,n) - (v*s')/sv) + (s*s')/sv;

      % Compute matrix-vector product
      Wg = W*g;
      
    else
      
      % L-BFGS two-loop recursion
      Rho   = zeros(M,1);
      alpha = zeros(M,1);
      beta  = zeros(M,1);
      Wg = g;
      for ii = 1:M
        if S(:,ii)'*V(:,ii) > 0, Rho(ii) = 1/(S(:,ii)'*V(:,ii)); else Rho(ii) = 0; end;
      end
      for ii = m:-1:m-M+1
        if ii >= 1, ind = ii; else ind = ii + M; end;
        if Rho(ind) > 0
          alpha(ind) = Rho(ind)*(S(:,ind)'*Wg);
          Wg = Wg - alpha(ind)*V(:,ind);
        end
      end
      for ii = m-M+1:m
        if ii >= 1, ind = ii; else ind = ii + M; end;
        if Rho(ind) > 0
          beta(ind) = Rho(ind)*(V(:,ind)'*Wg);
          Wg = Wg + S(:,ind)*(alpha(ind) - beta(ind));
        end
      end
      
    end
    
    % Compute norm of matrix-vector product
    norms.Wg = norm(Wg);
    
    % Break in unsafe mode
    if C == 0, break; end;
    
    % Check consistency
    if inner_count >= C, if M == 0, W = W_; else m = m - 1; end; 
      break; end;
    if rho*norm(ghat)^2 <= ghat'*Wg && norms.Wg <= tau*norm(ghat) || d >= dmax, break; end;
    
    % Print blank info
    fprintf('                                                           ');
    
    % Compute SG
    [g_temp   ,d] = stoch_grad(obj,x,d,b);
    [ghat_temp,d] = stoch_grad(obj,x,d,b);
    
    % Average gradients
    g    = (inner_count*g    + g_temp   )/(inner_count + 1);
    ghat = (inner_count*ghat + ghat_temp)/(inner_count + 1);
    
    % Compute norm of subgradient
    norms.g = norm(g);
    
    % Increment inner counter
    inner_count = inner_count + 1;
    
  end
  
  % Increment L-BFGS history
  if M > 0, m = m + 1; if m > M, m = 1; end; end;

  % Increment iteration counter
  k = k + 1;
  
  % Compute function value
  f = obj_func(obj,x); F(k) = f;

  % Compute stepsize
  a = stepsize(a0,a1,k);

end

% Print info line
if mod(k,20) == 1
  fprintf('=============================================================+===================================================\n');
  fprintf('   ADP    Objective       ||g||       ||W*g||      Stepsize  |   theta(1)     theta(2)    sv/||s||^2   ||v||^2/sv\n');
  fprintf('=============================================================+===================================================\n');
end

% Print information
fprintf(' %6d  %+.4e  %+.4e  %+.4e  %+.4e\n',d,f,norms.g,norms.Wg,a);

%% Objective function
function f = obj_func(obj,x)

% Evaluate objective function
f = feval(obj,x,0);

%% Stochastic gradient function
function [g,d] = stoch_grad(obj,x,d,b)

% Set global variables
%global b

% Evaluate stochastic gradient
g = full(feval(obj,x,1));

% Increment ADP counter
d = d + b;

%% Stepsize function
function alpha = stepsize(a0,a1,k)

% Set stepsize
alpha = 1/(a0 + k*a1);

%% Scaling factor function
function betas = beta(s,y,eta,theta)

% Compute products
u = norm(s)^2;
v = s'*y;
w = norm(y)^2;

% Compute for lower bound
betas(1) = 0;
if eta > v/u
    betas(1) = (eta*u-v)/(u-v);
end
vv = betas(1)*s + (1-betas(1))*y;
if betas(1) > 0 && vv'*vv > theta*s'*vv
  betas(1) = 1;
end

% Compute for upper bound
betas(2) = 0;
if w/v > theta
  betas(2) = min(roots([(u-2*v+w);(2*v-2*w-theta*u+theta*v);(w-theta*v)]));
end

% Avoid complex
betas = real(betas);
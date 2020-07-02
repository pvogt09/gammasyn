function v = logloss(x,o)

% function v = logloss(x,o)
%
% Author      : Frank E. Curtis
% Description : Evaluator for empirical loss objective function
% Input       : x ~ parameter vector
%               o ~ evaluation option (0 ~ loss, 1 ~ batch stochastic gradient)
% Output      : v ~ evaluation value

% Set global values
global X y m b

% Check option
if o == 0
  
  % Compute objective function
  e = exp(-1*y.*(X*x));
  v = (1/length(y))*sum(log(1+e));
  S = [];
  
elseif o == 1
  
  % Choose indices
  S = zeros(b,1);
  for j = 1:b, S(j) = ceil(m*rand); end;
  
  % Compute objective stochastic gradient
  e = exp(-1*y(S,:).*(X(S,:)*x));
  s = e./(1+e);
  v = -(1/length(S))*((s.*y(S,:))'*X(S,:))';
  v = full(v);
      
else

  error('bad option');
    
end
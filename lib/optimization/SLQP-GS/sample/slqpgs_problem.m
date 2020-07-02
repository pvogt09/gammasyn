function v = slqpgs_problem(x,d,j,o)

% function v = slqpgs_problem(x,d,j,o)
%
% Author      : Frank E. Curtis
% Description : Problem functions for sample problem
% Input       : x ~ point
%               d ~ problem function data structure
%               j ~ constraint number
%               o ~ evaluation option
%                     0 ~ objective function
%                     1 ~ objective gradient
%                     2 ~ j-th equality constraint value
%                     3 ~ j-th equality constraint gradient
%                     4 ~ j-th inequality constraint value
%                     5 ~ j-th inequality constraint gradient
% Output      : v ~ value

% Switch on evaluation option
switch o
  
  case 0 % Objective function value
    
    v = d.w*abs(x(1)^2 - x(2)) + (1 - x(1))^2;
    
  case 1 % Objective gradient value (as column vector)
    
    v = [-2*(1-x(1)); 0];
    if x(1)^2 - x(2) >= 0
      v = v + d.w*[ 2*x(1); -1];
    else
      v = v + d.w*[-2*x(1);  1];
    end
    
  case 2 % j-th equality constraint value
    
  case 3 % j-th equality constraint gradient value (as row vector)
    
  case 4 % j-th inequality constraint value
    
    v = max(sqrt(2)*x(1),2*x(2)) - 1;
    
  case 5 % j-th inequality constraint gradient value (as row vector)
    
    if sqrt(2)*x(1) >= 2*x(2), v = [sqrt(2),0]; else v = [0, 2]; end;

  otherwise
    
    % This error is not supposed to happen!
    error('SLQP-GS: Option not given to problem function evaluator.');
    
end
% Acceptance class
classdef Acceptance < handle

  % Class properties (private set access)
  properties (SetAccess = private)

    alpha % Primal stepsize
    
  end
  
  % Class methods
  methods
    
    % Backtracking line search
    function lineSearch(a,i,c,p,z,d)
      import slqpgs.*;
      % Check for epsilon update
      if d.eps_msg > 0, a.alpha = 0; return; end;
      
      % Store current values
      phi = z.phi;
      
      % Initialize stepsize
      a.alpha = 1;
      
      % Backtracking loop
      while a.alpha >= eps
        
        % Set trial point
        z.updateIterate(i,c,p,d,a,0);

        % Check Armijo condition
        if z.phi - phi <= -p.ls_suff_dec*a.alpha*max(d.dHd,0), return; end;
        
        % Reduce stepsize
        a.alpha = p.ls_factor*a.alpha;
        
      end
      
    end
    
  end
  
end
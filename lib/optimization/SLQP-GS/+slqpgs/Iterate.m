% Iterate class
classdef Iterate < handle
  
  % Class properties (private set access)
  properties (SetAccess = private)
    
    x       % Primal points
    x_      % Previous primal iterate
    rho     % Penalty parameter value
    theta   % Feasibility forcing tolerance
    epsilon % Sampling radius
    f       % Objective function value
    g       % Objective gradient values
    cE      % Equality constraint values
    JE      % Equality constraint Jacobian values
    cI      % Inequality constraint values
    JI      % Inequality constraint Jacobian values
    H       % Hessian approximation
    v       % Feasibility violation measure value
    v_eq    % Feasibility violation measure value for equalities
    v_ineq  % Feasibility violation measure value for inequalities
    phi     % Merit function value
    kkt     % KKT optimality error estimate
    s       % Primal point displacements
    y       % Gradient of penalty function displacements
    
  end
  
  % Class methods
  methods

    % Constructor
    function z = Iterate(i,c,p)
      
      % Initialize point
      z.x = i.x;
      
      % Initialize penalty parameter
      z.rho = p.rho_init;
      
      % Initialize sampling radius
      z.epsilon = p.epsilon_init;
      
      % Initialize sampling radius
      z.theta = p.theta_init;
      
      % Set sample points
      z.setSamplePoints(i);
      
      % Evaluate function values
      z.evalFunctions(i,c);
      
      % Evaluate gradient values
      z.evalGradients(i,c,1);
      
      % Evaluate infeasibility measure
      z.evalInfeasibility(i);
      
      % Evaluate merit function
      z.evalMerit;
      
      % Initialize Hessian information
      if i.algorithm == 0
        
        % Initialize s and y
        z.s = sparse(i.nV,p.lbfgs_hist);
        z.y = sparse(i.nV,p.lbfgs_hist);
        
        % Initialize H
        z.H = speye(i.nV);
        
      end
         
    end
    
    % Termination checker
    function b = checkTermination(z,i,c,d)

      % Initialize boolean
      b = 0;
      
      % Update termination based on stationarity condition for optimization problem
      if max([d.phi_red,z.epsilon]) <= i.stat_tol % Stationarity condition
        if z.v_eq <= i.eq_tol && z.v_ineq <= i.ineq_tol
          b = 1;    % Converged to a feasible stationary point
        else
          b = 2;    % Converged to an infeasible stationary point
        end
        return
      end
            
      % Update termination based on iteration count
      if c.k >= i.iter_max, b = 3; return; end;
	  
	  % Update termination based on function evaluation count
      if c.f >= i.funevals_max, b = 4; return; end;
      
    end
    
    % Function evaluator
    function evalFunctions(z,i,c)
      
      % Increment function evaluation counter
      c.incrementFunctionCount;
      
      % Evaluate objective value
      z.f = i.f(z.x(:,1),i.d,[],0);

      % Initialize equality constraint values
      z.cE = sparse(i.nE,1);
      
      % Loop through equality constraints
      for k = 1:i.nE

        % Evaluate kth equality constraint value
        z.cE(k) = i.f(z.x(:,1),i.d,k,2);
  
      end
      
      % Initialize inequality constraint values
      z.cI = sparse(i.nI,1);

      % Loop through inequality constraints
      for k = 1:i.nI

        % Evaluate kth inequality constraint value
        z.cI(k) = i.f(z.x(:,1),i.d,k,4);

      end
      
    end
    
    % Gradient evaluator
    function evalGradients(z,i,c,opt)
      
      % Increment gradient evaluation counter
      c.incrementGradientCount;
      
      % Initialize marker
      mark = 1;

      % Loop through sample points for objective
      for j = 1:1+opt*i.pO
  
        % Evaluate objective gradient value at sample point j
        if j == 1, z.g(:,j) = i.f(z.x(:,1)       ,i.d,[],1);
        else       z.g(:,j) = i.f(z.x(:,mark+j-1),i.d,[],1); end;

      end

      % Increment marker
      mark = mark + i.pO;

      % Loop through equality constraints
      for k = 1:i.nE

        % Loop through sample points for equality constraint k
        for j = 1:1+opt*i.pE(k)

          % Evaluate kth equality constraint gradient value at sample point j
          if j == 1, z.JE(k,:,j) = i.f(z.x(:,1)       ,i.d,k,3);
          else       z.JE(k,:,j) = i.f(z.x(:,mark+j-1),i.d,k,3); end;

        end
  
        % Increment marker
        mark = mark + i.pE(k);
  
      end

      % Loop through inequality constraints
      for k = 1:i.nI

        % Loop through sample points for inequality constraint k
        for j = 1:1+opt*i.pI(k)
  
          % Evaluate kth inequality constraint gradient value at sample point j
          if j == 1, z.JI(k,:,j) = i.f(z.x(:,1)       ,i.d,k,5);
          else       z.JI(k,:,j) = i.f(z.x(:,mark+j-1),i.d,k,5); end;

        end

        % Increment marker
        mark = mark + i.pI(k);

      end

    end
    
    % Hessian evaluator
    function evalHessian(z,i,c,p)

      % Increment Hessian evaluation counter
      c.incrementHessianCount;
      
      % Initialize Hessian
      z.H = eye(i.nV);
      
      % Loop through (s,y) pairs
      for j = 1:p.lbfgs_hist
                
        % Check curvature condition
        if norm(z.s(:,j)) <= 1e3*z.epsilon & norm(z.y(:,j)) <= 1e3*z.epsilon & z.s(:,j)'*z.y(:,j) > p.lbfgs_tol*z.epsilon^2
          
          % Compute matrix-vector product
          Hs = z.H*z.s(:,j);
          
          % Update Hessian
          z.H = z.H - (Hs*Hs')/(z.s(:,j)'*Hs) + (z.y(:,j)*z.y(:,j)')/(z.s(:,j)'*z.y(:,j));

        end
        
      end
            
    end
    
    % Infeasibility evaluator
    function evalInfeasibility(z,i)
      
      % Initialize violations to zero (in case of no constraints)
      [z.v_eq,z.v_ineq] = deal(0);

      % Separate violations for equality and inequality constraints
      % Used for separate termination feasibility tolerances
      if i.nE > 0
          z.v_eq = norm( z.cE(:,1), 1);               
      end
      if i.nI > 0
          z.v_ineq = norm( max(z.cI(:,1),0), 1); 
      end
      
      % Evaluate total violation
      z.v = z.v_eq + z.v_ineq;
      
    end
    
    % Merit evaluator
    function evalMerit(z)
      
      % Initialize merit for objective
      z.phi = z.rho*z.f + z.v;
      
    end

    % Gets primal point
    function x = getx(z)
      x = z.x(:,1);
    end
    
    % Sample point generator
    function setSamplePoints(z,i)
      
      % Initialize points
      z.x(:,2:1+i.pO+sum(i.pE)+sum(i.pI)) = zeros(i.nV,i.pO+sum(i.pE)+sum(i.pI));

      % Loop over number of sample points
      for j = 1:i.pO+sum(i.pE)+sum(i.pI)

        % Generate normal deviates
        u = randn(i.nV,1);
  
        % Transform into neighborhood
        u = z.epsilon*rand^(1/i.nV)*u/norm(u);

        % Add to set of sample points
        z.x(:,j+1) = z.x(:,1) + u;

      end
      
    end
    
    % Iterate updater
    function updateIterate(z,i,c,p,d,a,opt)
      
      % Save current point information
      x = z.x(:,1);
      
      % Update point
      z.x(:,1) = z.x(:,1) + a.alpha*d.x;
            
      % Set sample points
      if opt == 1, z.setSamplePoints(i); end;
      
      % Evaluate functions
      z.evalFunctions(i,c);

      % Evaluate gradients
      if opt == 1, z.evalGradients(i,c,opt); end;
      
      % Evaluate infeasibility
      z.evalInfeasibility(i);
      
      % Evaluate merit
      z.evalMerit;
      
      % Evaluate Hessian approximation
      if opt == 1 & i.algorithm == 0 & c.k > 0
        
        % Loop through history
        for j = p.lbfgs_hist-1:-1:1
          
          % Move columns in s and y
          z.s(:,j+1) = z.s(:,j);
          z.y(:,j+1) = z.y(:,j);
          
        end
        
        % Add new column to s and y
        z.s(:,1) = x - z.x_;
        z.y(:,1) = d.grad - d.grad_;
        
        % Evaluate Hessian
        z.evalHessian(i,c,p);
        
      end
      
      % Reset or store current point
      if opt == 0, z.x(:,1) = x;
      else         z.x_     = x; end;
      
    end
    
    % Parameter updater
    function b = updateParameters(z,p,d)

      % Save KKT error estimate
      z.kkt = d.kkt;

      % Initialize epsilon update message
      b = 0;
      
      % Check merit model reduction
      if d.phi_red <= p.update_tol*z.epsilon^2
        
        % Update epsilon update message
        b = 1;
        
        % Decrease sampling radius
        if z.epsilon > 1e-8
          z.epsilon = p.epsilon_factor*z.epsilon;
          d.resetKKT;
        end
        
        % Check feasibility level
        if z.v > z.theta
                    
          % Decrease penalty parameter
          z.rho = p.rho_factor*z.rho;
          
        else

          % Decrease feasibility tolerance
          z.theta = p.theta_factor*z.theta;
          
        end
        
      end

    end
    
  end
 
end
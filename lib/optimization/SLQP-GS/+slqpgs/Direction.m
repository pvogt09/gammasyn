% Direction class
classdef Direction < handle
  
  % Class properties (private set access)
  properties (SetAccess = private)
    
    x       % Primal direction
    x_norm  % Primal direction norm value
    dHd     % Primal direction inner product with Hessian
    g       % Subproblem linear objective data
    H       % Subproblem quadratic objective data
    lO      % Objective multipliers
    AE      % Subproblem equality constraint Jacobian data
    bE      % Subproblem equality constraint right-hand side data
    lE      % Equality constraint multipliers
    AI      % Subproblem inequality constraint Jacobian data
    bI      % Subproblem inequality constraint right-hand side data
    lI      % Inequality constraint multipliers
    l       % Subproblem lower bounds
    u       % Subproblem upper bounds
    phi_red % Merit model reduction
    flag    % Subproblem solver termination flag
    eps_msg % Null step boolean
    grad    % Gradient of penalty function
    grad_   % Last gradient of penalty function
    grad_b  % Boolean to check if gradient has been set
    kkt     % KKT error approximation

  end
  
  % Class methods
  methods
    
    % Constructor
    function d = Direction(i)
      
      % Initialize merit model reduction
      d.phi_red = inf;
      
      % Initialize KKT error estimate
      d.kkt = inf;

      % Initialize subproblem data
      d.initSubproblemData(i);
      
      % Initialize multiplier sizes
      d.lO = sparse(1+    i.pO ,1   );
      if i.nE > 0, d.lE = sparse(1+max(i.pE),i.nE); end;
      if i.nI > 0, d.lI = sparse(1+max(i.pI),i.nI); end;
      
      % Initialize gradient computation boolean
      d.grad_b = 0;
      
    end
    
    % Evaluate KKT error approximation
    function evalKKT(d,i,z)
      
      % Initialize dual feasibility vector for objective
      vec1 = z.g*d.lO;
      
      % Loop through equality constraints
      for k = 1:i.nE
        
        % Loop through sample points
        for j = 1:1+i.pE(k)
        
          % Update dual feasibility vector for equality constraint j
          vec1 = vec1 + z.JE(k,:,j)'*d.lE(j,k);
          
        end
        
      end
      
      % Loop through inequality constraints
      for k = 1:i.nI
        
        % Loop through sample points
        for j = 1:1+i.pI(k)
          
          % Update dual feasibility vector for inequality constraint j
          vec1 = vec1 + z.JI(k,:,j)'*d.lI(j,k);
        
        end
        
      end
      
      % Save last gradient of penalty function
      if d.grad_b > 0, d.grad_ = d.grad; end;
      
      % Save gradient of penalty function
      d.grad = vec1; d.grad_b = 1;
      
      % Scale dual feasibility vector by numbers of sample points
      val1 = norm(vec1,inf);

      % Initialize violation vector
      vec2 = [];
      
      % Update vector for constraint values
      if i.nE > 0, vec2 = z.cE(:,1);               end;
      if i.nI > 0, vec2 = [vec2; max(z.cI(:,1),0)]; end;
      
      % Set inf-norm of constraint violation
      val2 = norm(vec2,inf);
      
      % Initialize multiplier vector
      val3 = 0;

      % Loop through inequality constraints
      for k = 1:i.nI
        
        % Update multiplier vector
        val3 = max(val3,norm(max(0,-d.lI(1+i.pI(k),k)),inf));
        
      end
      
      % Initialize complementarity vector
      val4 = 0;
      
      % Loop through inequality constraints
      for k = 1:i.nI
                  
        % Update dual feasibility vector for inequality constraint j
        val4 = max(val4,norm(z.cI(k,1)*d.lI(1+i.pI(k),k),inf));
                
      end
            
      % Evaluate KKT error approximation
      d.kkt = min(d.kkt,norm([val1;val2;val3;val4],inf));
      
    end
    
    % Evaluate model reductions
    function evalModels(d,i,z)

      % Initialize linear term
      l = 0;

      % Loop through equality constraints
      for k = 1:i.nE
  
        % Initialize temporary term
        term = 0;
  
        % Loop through sample points for equality constraint k
        for j = 1:1+i.pE(k)
  
          % Evaluate linear term for equality constraint k
          piece = abs(z.cE(k,1) + z.JE(k,:,j)*d.x); if piece > term, term = piece; end;
    
        end
  
        % Update linear term for equality constraint k
        l = l + term;
  
      end

      % Loop through inequality constraints
      for k = 1:i.nI
  
        % Initialize temporary term
        term = 0;
  
        % Loop through sample points for inequality constraint k
        for j = 1:1+i.pI(k)
  
          % Evaluate linear term for inequality constraint k
          piece = max(z.cI(k,1) + z.JI(k,:,j)*d.x,0); if piece > term, term = piece; end;
    
        end
  
        % Update linear term for inequality constraint k
        l = l + term;
  
      end

      % Update linear term for objective
      l = l + max(z.rho*(z.f+z.g'*d.x));

      % Evaluate merit model reduction
      d.phi_red = z.phi - l;
      
      % Evaluate quadratic term
      if i.algorithm == 0, q = (1/2)*d.x'*z.H*d.x; else q = 0; end;

      % Update merit model reduction
      d.phi_red = d.phi_red - q;
      
    end

    % Evaluate search direction quantities
    function evalStep(d,i,c,p,z)
      
      % Update subproblem data
      d.updateSubproblemData(i,p,z);

      % Solve subproblem
      d.solveSubproblem(i,c,z);

      % Evaluate model and model reduction
      d.evalModels(i,z);
      
      % Update parameters
      d.eps_msg = z.updateParameters(p,d);
   
    end
    
    % Initialize subproblem data
    function initSubproblemData(d,i)
      
      % Check subproblem problem to solve
      if i.sp_problem == 0

        % Initialize Hessian
        if i.algorithm == 0, d.H = sparse(i.nV+1+i.nE+i.nI,i.nV+1+i.nE+i.nI); end;
  
        % Initialize linear objective term
        d.g = [sparse(i.nV+1,1); ones(i.nE+i.nI,1)];

        % Initialize subproblem equality constraint matrix
        d.AE = [];

        % Initialize subproblem inequality constraint matrix
        d.AI = sparse(1+i.pO+2*(i.nE+sum(i.pE))+i.nI+sum(i.pI),i.nV+1+i.nE+i.nI);

        % Initialize row counters and column counter
        row = 1; col = i.nV+1;

        % Place auxiliary variable coefficients for objective
        d.AI(row:row+i.pO,col) = -ones(1+i.pO,1);
  
        % Increment row and column counters
        row = row + i.pO + 1; col = col + 1;

        % Loop through equality constraints
        for j = 1:i.nE

          % Place auxiliary variable coefficients for equality constraint j
          d.AI(row               :row               +i.pE(j),col) = -ones(1+i.pE(j),1);
          d.AI(row+i.nE+sum(i.pE):row+i.nE+sum(i.pE)+i.pE(j),col) = -ones(1+i.pE(j),1);

          % Increment row and column counters
          row = row + i.pE(j) + 1; col = col + 1;

        end
  
        % Increment row counter
        row = row + i.nE + sum(i.pE);

        % Loop through inequality constraints
        for j = 1:i.nI

          % Place auxiliary variable coefficients for inequality constraint j
          d.AI(row:row+i.pI(j),col) = -ones(1+i.pI(j),1);

          % Increment row and column counters
          row = row + i.pI(j) + 1; col = col + 1;

        end

        % Initialize subproblem equality constraint right-hand-side
        d.bE = [];

        % Initialize subproblem inequality constraint right-hand-side
        d.bI = zeros(1+i.pO+2*(i.nE+sum(i.pE))+i.nI+sum(i.pI),1);

        % Set subproblem bounds
        d.l = [-inf*ones(i.nV+1,1);   sparse(i.nE+i.nI,1)];
        d.u = [ inf*ones(i.nV+1,1); inf*ones(i.nE+i.nI,1)];

      elseif i.algorithm == 0

        % Initialize Hessian
        d.H = sparse(i.nV+1+i.pO+2*(i.nE+sum(i.pE))+i.nI+sum(i.pI),i.nV+1+i.pO+2*(i.nE+sum(i.pE))+i.nI+sum(i.pI));
  
        % Initialize linear objective term
        d.g = sparse(i.nV+1+i.pO+2*(i.nE+sum(i.pE))+i.nI+sum(i.pI),1);
  
        % Initialize subproblem inequality constraint Jacobian
        d.AI = sparse(i.nE+i.nI,i.nV+1+i.pO+2*(i.nE+sum(i.pE))+i.nI+sum(i.pI));
  
        % Initialize subproblem equality constraint Jacobian
        d.AE = sparse(i.nV+1,i.nV+1+i.pO+2*(i.nE+sum(i.pE))+i.nI+sum(i.pI));
  
        % Place dual variable coefficients for primal auxiliary variable coefficient for objective
        d.AE(i.nV+1,i.nV+1:i.nV+1+i.pO) = ones(1,1+i.pO);
  
        % Initialize row and column counters
        row = 1; col = i.nV+1+i.pO+1;

        % Loop through equality constraints
        for j = 1:i.nE

          % Place dual variable coefficients for primal auxiliary variable coefficients for equality constraints
          d.AI(row,col               :col               +i.pE(j)) = ones(1,1+i.pE(j));
          d.AI(row,col+i.nE+sum(i.pE):col+i.nE+sum(i.pE)+i.pE(j)) = ones(1,1+i.pE(j));
    
          % Increment row and column counters
          row = row + 1; col = col + i.pE(j) + 1;
    
        end
  
        % Increment column counter
        col = col + i.nE + sum(i.pE);

        % Loop through inequality constraints
        for j = 1:i.nI
  
          % Place dual variable coefficients for primal auxiliary variable coefficients for inequality constraints
          d.AI(row,col:col+i.pI(j)) = ones(1,1+i.pI(j));
    
          % Increment row and column counters
          row = row + 1; col = col + i.pI(j) + 1;
    
        end

        % Initialize subproblem equality constraint right-hand-side
        d.bE = sparse(i.nV+1,1);
  
        % Set subproblem inequality constraint right-hand-side
        d.bI = ones(i.nE+i.nI,1);
  
        % Set subproblem bounds
        d.l = [-inf*ones(i.nV,1);   sparse(1+i.pO+2*(i.nE+sum(i.pE))+i.nI+sum(i.pI),1)];
        d.u = [ inf*ones(i.nV,1); inf*ones(1+i.pO+2*(i.nE+sum(i.pE))+i.nI+sum(i.pI),1)];

      else

        % Initialize linear objective term
        d.g = sparse(1+i.pO+2*(i.nE+sum(i.pE))+i.nI+sum(i.pI)+2*i.nV,1);
  
        % Initialize subproblem inequality constraint Jacobian
        d.AI = sparse(i.nE+i.nI,1+i.pO+2*(i.nE+sum(i.pE))+i.nI+sum(i.pI)+2*i.nV);
  
        % Initialize subproblem equality constraint Jacobian
        d.AE = sparse(i.nV+1,1+i.pO+2*(i.nE+sum(i.pE))+i.nI+sum(i.pI)+2*i.nV);
  
        % Place dual variable coefficients for primal bounds
        d.AE(1:i.nV,1+i.pO+2*(i.nE+sum(i.pE))+i.nI+sum(i.pI)+1:1+i.pO+2*(i.nE+sum(i.pE))+i.nI+sum(i.pI)+2*i.nV) = [-speye(i.nV) speye(i.nV)];
  
        % Place dual variable coefficients for primal auxiliary variable coefficient for objective
        d.AE(i.nV+1,1:1+i.pO) = ones(1,1+i.pO);
  
        % Initialize row and column counters
        row = 1; col = 1+i.pO+1;

        % Loop through equality constraints
        for j = 1:i.nE

          % Place dual variable coefficients for primal auxiliary variable coefficients for equality constraints
          d.AI(row,col               :col               +i.pE(j)) = ones(1,1+i.pE(j));
          d.AI(row,col+i.nE+sum(i.pE):col+i.nE+sum(i.pE)+i.pE(j)) = ones(1,1+i.pE(j));
    
          % Increment row and column counters
          row = row + 1; col = col + i.pE(j) + 1;
    
        end
  
        % Increment column counter
        col = col + i.nE + sum(i.pE);

        % Loop through inequality constraints
        for j = 1:i.nI
  
          % Place dual variable coefficients for primal auxiliary variable coefficients for inequality constraints
          d.AI(row,col:col+i.pI(j)) = ones(1,1+i.pI(j));
    
          % Increment row and column counters
          row = row + 1; col = col + i.pI(j) + 1;
    
        end

        % Initialize subproblem equality constraint right-hand-side
        d.bE = sparse(i.nV+1,1);
  
        % Set subproblem inequality constraint right-hand-side
        d.bI = ones(i.nE+i.nI,1);
  
        % Set subproblem bounds
        d.l =   sparse(1+i.pO+2*(i.nE+sum(i.pE))+i.nI+sum(i.pI)+2*i.nV,1);
        d.u = inf*ones(1+i.pO+2*(i.nE+sum(i.pE))+i.nI+sum(i.pI)+2*i.nV,1);

      end
      
    end

    % Resetter of KKT error estimate
    function resetKKT(d)
    
      % Reset KKT error estimate for new sampling radius
      d.kkt = inf;

    end
    
    % Update subproblem data
    function updateSubproblemData(d,i,p,z)
            
      % Check subproblem problem to solve
      if i.sp_problem == 0
        
        % Set subproblem penalty parameter
        d.g(i.nV+1) = z.rho;

        % Set Hessian (for SQPGS) or trust region (for SLPGS)
        if i.algorithm == 0
  
          % Set Hessian matrix
          d.H(1:i.nV,1:i.nV) = z.H;
    
        else
  
          % Set trust region bounds
          d.l(1:i.nV) = -p.tr_radius*z.epsilon;
          d.u(1:i.nV) =  p.tr_radius*z.epsilon;

        end

        % Loop through sample points for objective
        for j = 1:1+i.pO

          % Set constraint values for objective for sample point j
          d.AI(j,1:i.nV) =  z.g(:,j)';
          d.bI(j       ) = -z.f;
      
        end

        % Loop through equality constraints
        for k = 1:i.nE
  
          % Loop through sample points for equality constraint k
          for j = 1:1+i.pE(k)
  
            % Set constraint values for equality constraint k for sample point j
            d.AI(1+i.pO               +k-1+sum(i.pE(1:k-1))+j,1:i.nV) =  z.JE(k,:,j);
            d.AI(1+i.pO+i.nE+sum(i.pE)+k-1+sum(i.pE(1:k-1))+j,1:i.nV) = -z.JE(k,:,j);
            d.bI(1+i.pO               +k-1+sum(i.pE(1:k-1))+j       ) = -z.cE(k,1);
            d.bI(1+i.pO+i.nE+sum(i.pE)+k-1+sum(i.pE(1:k-1))+j       ) =  z.cE(k,1);
      
          end

        end

        % Loop through inequality constraints
        for k = 1:i.nI

          % Loop through sample points for inequality constraint k
          for j = 1:1+i.pI(k)
  
            % Set constraint values for inequality constraint k for sample point j
            d.AI(1+i.pO+2*(i.nE+sum(i.pE))+k-1+sum(i.pI(1:k-1))+j,1:i.nV) =  z.JI(k,:,j);
            d.bI(1+i.pO+2*(i.nE+sum(i.pE))+k-1+sum(i.pI(1:k-1))+j       ) = -z.cI(k,1);
      
          end

        end

      elseif i.algorithm == 0

        % Set Hessian matrix
        d.H (1:i.nV,1:i.nV) = z.H;
        d.AE(1:i.nV,1:i.nV) = z.H;
  
        % Reset variable upper bounds
        d.u = [inf*ones(i.nV,1); inf*ones(1+i.pO+2*(i.nE+sum(i.pE))+i.nI+sum(i.pI),1)];

        % Loop through sample points for objective
        for j = 1:1+i.pO

          % Set constraint and linear objective term values for objective for sample point j
          d.AE(1:i.nV,i.nV+j) =  z.g(:,j);
          d.g (       i.nV+j) = -z.f;
      
        end

        % Loop through equality constraints
        for k = 1:i.nE
  
          % Loop through sample points for equality constraint k
          for j = 1:1+i.pE(k)
  
            % Set constraint and linear objective term values for equality constraint k for sample point j
            d.AE(1:i.nV,i.nV+1+i.pO               +k-1+sum(i.pE(1:k-1))+j) =  z.JE(k,:,j)';
            d.AE(1:i.nV,i.nV+1+i.pO+i.nE+sum(i.pE)+k-1+sum(i.pE(1:k-1))+j) = -z.JE(k,:,j)';
            d.g (       i.nV+1+i.pO               +k-1+sum(i.pE(1:k-1))+j) = -z.cE(k,1);
            d.g (       i.nV+1+i.pO+i.nE+sum(i.pE)+k-1+sum(i.pE(1:k-1))+j) =  z.cE(k,1);
      
          end

        end

        % Loop through inequality constraints
        for k = 1:i.nI

          % Loop through sample points for inequality constraint k
          for j = 1:1+i.pI(k)
  
            % Set constraint and linear objective term values for inequality constraint k for sample point j
            d.AE(1:i.nV,i.nV+1+i.pO+2*(i.nE+sum(i.pE))+k-1+sum(i.pI(1:k-1))+j) =  z.JI(k,:,j)';
            d.g (       i.nV+1+i.pO+2*(i.nE+sum(i.pE))+k-1+sum(i.pI(1:k-1))+j) = -z.cI(k,1);
      
          end
      
        end
        
        % Set subproblem penalty parameter
        d.bE(i.nV+1) = z.rho;

      else

        % Set linear objective terms for primal bounds
        d.g(1+i.pO+2*(i.nE+sum(i.pE))+i.nI+sum(i.pI)     +1:1+i.pO+2*(i.nE+sum(i.pE))+i.nI+sum(i.pI)+i.nV     ) = p.tr_radius*z.epsilon;
        d.g(1+i.pO+2*(i.nE+sum(i.pE))+i.nI+sum(i.pI)+i.nV+1:1+i.pO+2*(i.nE+sum(i.pE))+i.nI+sum(i.pI)+i.nV+i.nV) = p.tr_radius*z.epsilon;

        % Reset variable upper bounds
        d.u = inf*ones(1+i.pO+2*(i.nE+sum(i.pE))+i.nI+sum(i.pI)+2*i.nV,1);

        % Loop through sample points for objective
        for j = 1:1+i.pO

          % Set constraint and linear objective term values for objective for sample point j
          d.AE(1:i.nV,j) =  z.g(:,j);
          d.g (       j) = -z.f;
      
        end

        % Loop through equality constraints
        for k = 1:i.nE
  
          % Loop through sample points for equality constraint k
          for j = 1:1+i.pE(k)
  
            % Set constraint and linear objective term values for equality constraint k for sample point j
            d.AE(1:i.nV,1+i.pO               +k-1+sum(i.pE(1:k-1))+j) =  z.JE(k,:,j)';
            d.AE(1:i.nV,1+i.pO+i.nE+sum(i.pE)+k-1+sum(i.pE(1:k-1))+j) = -z.JE(k,:,j)';
            d.g (       1+i.pO               +k-1+sum(i.pE(1:k-1))+j) = -z.cE(k,1);
            d.g (       1+i.pO+i.nE+sum(i.pE)+k-1+sum(i.pE(1:k-1))+j) =  z.cE(k,1);
      
          end
      
        end

        % Loop through inequality constraints
        for k = 1:i.nI

          % Loop through sample points for inequality constraint k
          for j = 1:1+i.pI(k)
  
            % Set constraint and linear objective term values for inequality constraint k for sample point j
            d.AE(1:i.nV,1+i.pO+2*(i.nE+sum(i.pE))+k-1+sum(i.pI(1:k-1))+j) =  z.JI(k,:,j)';
            d.g (       1+i.pO+2*(i.nE+sum(i.pE))+k-1+sum(i.pI(1:k-1))+j) = -z.cI(k,1);
      
          end

        end
        
        % Set subproblem penalty parameter
        d.bE(i.nV+1) = z.rho;

      end
      
    end
    
    % Solve subproblem
    function solveSubproblem(d,i,c,z)
      
      % Check algorithm
      if i.algorithm == 0

        % Solve quadratic program
        if strcmpi(i.sp_solver, 'quadprog')
            [primals,~,flag,~,duals] = quadprog(d.H,d.g,d.AI,d.bI,d.AE,d.bE,d.l,d.u,[],i.sp_options);
		elseif strcmpi(i.sp_solver, 'gurobi')
            model.sense = char([ones(1, size(d.AI, 1))*'<', ones(1, size(d.AE, 1))*'=']);
            model.lb = full(d.l);
            model.ub =  full(d.u);
            model.Q = d.H;
            model.obj = full(d.g);
            model.A = [
                d.AI;
                d.AE
            ];
            model.rhs = [
                d.bI;
                d.bE
            ];
            result = gurobi(model, struct('OutputFlag', 0));
            if (strcmp(result.status, 'INFEASIBLE') || strcmp(result.status, 'INF_OR_UNBD'))
                [primals,~,flag,~,duals] = quadprog(d.H,d.g,d.AI,d.bI,d.AE,d.bE,d.l,d.u,[],i.sp_options);
            elseif ~strcmp(result.status, 'OPTIMAL')
                [primals,~,flag,~,duals] = quadprog(d.H,d.g,d.AI,d.bI,d.AE,d.bE,d.l,d.u,[],i.sp_options);
			else
                primals = result.x;
                flag = result.status;
                duals = struct(...
                    'ineqlin',	result.pi(1:size(d.AI, 1)),...
                    'eqlin',	result.pi(size(d.AI, 1) + (1:size(d.AE, 1))),...
                    'lower',	[],...
                    'upper',	[]...
                );
            end
		else
            if strcmpi(i.sp_solver, 'linprog')
                [primals,~,flag,~,duals] = linprog(d.H,d.g,d.AI,d.bI,d.AE,d.bE,d.l,d.u,[],i.sp_options);
			else
                [primals,~,flag,~,duals] = feval(i.sp_solver,d.H,d.g,d.AI,d.bI,d.AE,d.bE,d.l,d.u,[],i.sp_options);
            end
        end

      else

        % Solve linear program
        [primals,~,flag,~,duals] = feval(i.sp_solver,d.g,d.AI,d.bI,d.AE,d.bE,d.l,d.u,[],i.sp_options);
  
      end

      % Increment subproblem solver counter
      c.incrementSubproblemCount;

      % Check subproblem type
      if i.sp_problem == 0

        % Set primal step
        d.x = primals(1:i.nV);
        
        % Set dual vector
        vec = duals.ineqlin;
                
      elseif i.algorithm == 0

        % Set primal step
        d.x = primals(1:i.nV);

        % Set gradient multipliers
        vec = primals(i.nV+1:i.nV+1+i.pO+2*(i.nE+sum(i.pE))+i.nI+sum(i.pI));

      else

        % Set primal step
        d.x = -duals.eqlin(1:i.nV);

        % Set gradient multipliers
        vec = primals(1:1+i.pO+2*(i.nE+sum(i.pE))+i.nI+sum(i.pI));
  
      end
      
      % Set objective gradient multipliers
      d.lO = vec(1:1+i.pO);
        
      % Set row counter
      row = 2 + i.pO;

      % Loop through equality constraints
      for j = 1:i.nE

        % Set multipliers for gradients of equality constraint j
        d.lE(1:1+i.pE(j),j) = vec(row:row+i.pE(j)) - vec(row+i.nE+sum(i.pE):row+i.nE+sum(i.pE)+i.pE(j));
          
        % Increment row counter
        row = row + i.pE(j) + 1;

      end
  
      % Increment row counter
      row = row + i.nE + sum(i.pE);

      % Loop through inequality constraints
      for j = 1:i.nI

        % Set multipliers for gradients of inequality constraint j
        d.lI(1:1+i.pI(j),j) = vec(row:row+i.pI(j));

        % Increment row counter
        row = row + i.pI(j) + 1;

      end

      % Set flag
      d.flag = flag;
      
      % Compute step length
      d.x_norm = norm(d.x);
      
      % Compute inner product with Hessian
      if i.algorithm == 0, d.dHd = d.x'*z.H*d.x; else d.dHd = d.x'*d.x; end;
      
      % Compute KKT error approximation
      d.evalKKT(i,z);
      
    end
        
  end
  
end

% Output class
classdef Output < handle

  % Class properties (private access)
  properties (SetAccess = private, GetAccess = private)
  
    s       % Output stream
    l       % Line break string
    q       % Quantities string
    n       % Last iterate string
    
  end
  
  % Class methods
  methods

    % Constructor
    function o = Output(output_type)
      
      % Start clock
      tic;
      
      % Set output stream
      if isnumeric(output_type)
        % Print to console
        o.s = output_type; 
      else
        % To file
        o.s = fopen(output_type,'w');
        % Assert output stream has been opened
        assert(o.s~=-1,'SLQP-GS: Failed to open %s.',output_type);
      end
      
      % Store output strings
      o.l = '======+===================================================+===========================================+========================+===========';
      o.q = 'Iter. |  Objective     Infeas.  |  Pen. Par.     Merit    | Msg.   ||Step||    Mod. Red.       KKT    | Samp. Rad.   Inf. Tol. |  Stepsize';
      o.n =                                                             '----  ----------  -----------  ---------- | ----------  ---------- | ----------';

    end
    
    % Print acceptance information
    function printAcceptance(o,a)
      
      % Print steplengths
      fprintf(o.s,'%.4e\n',a.alpha);
      
    end
    
    % Print break in output
    function printBreak(o,c)
      
      % Print break every 20 iterations
      if mod(c.k,20) == 0, fprintf(o.s,'%s\n%s\n%s\n',o.l,o.q,o.l); end;
      
    end
    
    % Print direction information
    function printDirection(o,z,d)
      
      % Print direction information
      fprintf(o.s,'%4d  %.4e  %+.4e  %.4e | %.4e  %.4e | ',d.flag,norm(d.x),d.phi_red,z.kkt,z.epsilon,z.theta);
      
    end
    
    % Print output footer
    function printFooter(o,i,c,z,d)
      
      % Print final iterate information
      o.printIterate(c,z);
      
      % Print close of algorithm output
      fprintf(o.s,'%s\n%s\n\n',o.n,o.l);
      
      % Get solver result
      b = z.checkTermination(i,c,d);
      
      % Print solver result
      fprintf(o.s,'Final result\n');
      fprintf(o.s,'============\n');
      if b == 0, fprintf(o.s,'  EXIT: No termination message set\n'                 ); end;
      if b == 1, fprintf(o.s,'  EXIT: Solution appears stationary and feasible\n'   ); end;
      if b == 2, fprintf(o.s,'  EXIT: Solution appears stationary, but infeasible\n'); end;
      if b == 3, fprintf(o.s,'  EXIT: Iteration limit reached\n'                    ); end;
      fprintf(o.s,'\n');
      
      % Print iterate quantities
      fprintf(o.s,'Final values\n');
      fprintf(o.s,'============\n');
      fprintf(o.s,'  Objective function........................ : %+e\n',z.f);
      fprintf(o.s,'  Feasibility violation..................... : %+e\n',z.v);
      fprintf(o.s,'  Penalty parameter......................... : %+e\n',z.rho);
      fprintf(o.s,'  Sampling radius........................... : %+e\n',z.epsilon);
      fprintf(o.s,'  Primal-dual stationarity error estimate... : %+e\n',z.kkt);
      fprintf(o.s,'\n');
      
      % Print counters
      fprintf(o.s,'Final counters\n');
      fprintf(o.s,'==============\n');
      fprintf(o.s,'  Iterations................................ : %d\n',c.k);
      fprintf(o.s,'  Function evaluations...................... : %d\n',c.f);
      fprintf(o.s,'  Gradient evaluations...................... : %d\n',c.g);
      fprintf(o.s,'  Hessian evaluations....................... : %d\n',c.H);
      fprintf(o.s,'  Subproblems solved........................ : %d\n',c.s);
      fprintf(o.s,'  CPU seconds............................... : %d\n',ceil(toc));
      
    end
    
    % Print output header
    function printHeader(o,i)
      
      % Print SLQP-GS version
      fprintf(o.s,'+======================+\n');
      fprintf(o.s,'| SLQP-GS, version 1.3 |\n');
      fprintf(o.s,'+======================+\n');
      fprintf(o.s,'\n');
      
      % Print problem size
      fprintf(o.s,'Problem size\n');
      fprintf(o.s,'============\n');
      fprintf(o.s,'  # of variables.................... : %6d\n',i.nV);
      fprintf(o.s,'  # of equality constraints......... : %6d\n',i.nE);
      fprintf(o.s,'  # of inequality constraints....... : %6d\n',i.nI);
      fprintf(o.s,'\n');
      
      % Set output strings
      if i.algorithm  == 0, alg = 'SQP-GS'; else alg = 'SLP-GS'; end;
      if i.sp_problem == 0, sub = 'primal'; else sub = 'dual'  ; end;

      % Print subproblem size
      fprintf(o.s,'Subproblem size (%s,%s)\n',alg,sub);
      if i.algorithm == 0 & i.sp_problem == 0
        fprintf(o.s,'===============================\n');
        fprintf(o.s,'  # of variables.................... : %6d\n',i.nV+1+i.nE+i.nI);
        fprintf(o.s,'  # of linear equality constraints.. : %6d\n',0);
        fprintf(o.s,'  # of linear inequality constraints : %6d\n',1+i.pO+2*(i.nE+sum(i.pE))+i.nI+sum(i.pI));
        fprintf(o.s,'  # of bound constraints............ : %6d\n',i.nE+i.nI);
      elseif i.algorithm == 0
        fprintf(o.s,'=============================\n');
        fprintf(o.s,'  # of variables.................... : %6d\n',i.nV+1+i.pO+2*(i.nE+sum(i.pE))+i.nI+sum(i.pI));
        fprintf(o.s,'  # of linear equality constraints.. : %6d\n',i.nV+1);
        fprintf(o.s,'  # of linear inequality constraints : %6d\n',i.nE+i.nI);
        fprintf(o.s,'  # of bound constraints............ : %6d\n',1+i.pO+2*(i.nE+sum(i.pE))+i.nI+sum(i.pI));
      elseif i.sp_problem == 0
        fprintf(o.s,'===============================\n');
        fprintf(o.s,'  # of variables.................... : %6d\n',i.nV+1+i.nE+i.nI);
        fprintf(o.s,'  # of linear equality constraints.. : %6d\n',0);
        fprintf(o.s,'  # of linear inequality constraints : %6d\n',1+i.pO+2*(i.nE+sum(i.pE))+i.nI+sum(i.pI));
        fprintf(o.s,'  # of bound constraints............ : %6d\n',2*i.nV+i.nE+i.nI);
      else
        fprintf(o.s,'=============================\n');
        fprintf(o.s,'  # of variables.................... : %6d\n',1+i.pO+2*(i.nE+sum(i.pE))+i.nI+sum(i.pI)+2*i.nV);
        fprintf(o.s,'  # of linear equality constraints.. : %6d\n',i.nV+1);
        fprintf(o.s,'  # of linear inequality constraints : %6d\n',i.nE+i.nI);
        fprintf(o.s,'  # of bound constraints............ : %6d\n',1+i.pO+2*(i.nE+sum(i.pE))+i.nI+sum(i.pI)+2*i.nV);
      end
      fprintf(o.s,'\n');
      
      % If a file, close nonstandard output stream
      if ~ismember(o.s,[0 1 2]), fclose(o.s); end;
      
    end
    
    % Print iterate information
    function printIterate(o,c,z)
      
      % Print iterate information
      fprintf(o.s,'%5d | %+.4e  %.4e | %.4e  %+.4e | ',c.k,z.f,z.v,z.rho,z.phi);
      
    end
    
  end
  
end

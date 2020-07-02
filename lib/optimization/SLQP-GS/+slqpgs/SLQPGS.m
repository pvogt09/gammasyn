% SLQPGS class
classdef SLQPGS

  % Class properties (private access)
  properties (SetAccess = private, GetAccess = private)
    
    i           % Input object
    o           % Output object
    c           % Counter object
    p           % Parameter object
    z           % Iterate object
    l           % Log object
    d           % Direction object
    a           % Acceptance object
    output_on   % boolean
    logging_on  % boolean
    
  end
  
  % Class methods
  methods
  
    % Constructor
    function S = SLQPGS(prob)
      import slqpgs.*;
      % Construct classes
      S.i = Input(prob);
      S.output_on = ischar(S.i.output) || S.i.output;
      if S.output_on
        S.o = Output(S.i.output);
      end
      S.c = Counter;
      S.p = Parameter;
      S.z = Iterate(S.i,S.c,S.p);
      S.logging_on = ~isempty(S.i.log_fields);
      if S.logging_on
        S.l = Log(S.i,S.z);
      end
      S.d = Direction(S.i);
      S.a = Acceptance;

    end
    
    % Gets primal solution and the log 
    function [x,log] = getSolution(S)
	  import slqpgs.*;
	  x = S.z.getx;
      log = [];
      if S.logging_on 
        log = S.l.getLog();
      end
    end
	
	function [x, fval, exitflag, output, lambda, grad, hessian] = getResult(this)
		x = this.z.getx();
		if nargout >= 2
			fval = this.z.f;
			if nargout >= 3
				exitflag = this.z.checkTermination(this.i, this.c, this.d);
				if nargout >= 4
					output = struct(...
						'iterations',			this.c.k,...
						'funcCount',			this.c.f,...	
						'lssteplength',			NaN,...
						'constrviolation',		this.z.v,...
						'stepsize',				this.d.x_norm,...
						'algorithm',			'',...
						'cgiterations',			NaN,...
						'firstorderopt',		this.z.kkt,...
						'message',				''...
					);
					if nargout >= 5
						lambda = struct(...
							'lower',		[],...
							'upper',		[],...
							'ineqlin',		[],...
							'eqlin',		[],...
							'ineqnonlin',	[],...
							'eqnonlin',		[]...
						);
						if nargout >= 6
							grad = this.z.g(:, 1);
							if nargout >= 7
								hessian = this.z.H;
							end
						end
					end
				end
			end
		end
	end
    
    % Optimization algorithm
    function optimize(S)
	  import slqpgs.*;
      % Print header and line break
      if S.output_on 
        S.o.printHeader(S.i);
        S.o.printBreak(S.c);
      end
      
      % Iteration loop
      while ~S.z.checkTermination(S.i,S.c,S.d)    
        if S.output_on, S.o.printIterate(S.c,S.z); end
        S.d.evalStep(S.i,S.c,S.p,S.z);
        if S.output_on, S.o.printDirection(S.z,S.d); end
        S.a.lineSearch(S.i,S.c,S.p,S.z,S.d);
        if S.output_on, S.o.printAcceptance(S.a); end
        S.z.updateIterate(S.i,S.c,S.p,S.d,S.a,1);
        if S.logging_on, S.l.logData(); end
        S.c.incrementIterationCount;
        if S.output_on, S.o.printBreak(S.c); end
      end
      
      % Print footer and terminate
      if S.output_on
        S.o.printFooter(S.i,S.c,S.z,S.d);
      end
      
    end
        
  end
  
end

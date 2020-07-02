% Input class
classdef Input < handle
  
  % Class properties (private set access)
  properties (SetAccess = private)

    algorithm  % Algorithm option
    sp_problem % Subproblem option
    sp_solver  % Subproblem solver handle
    sp_options % Subproblem solver options
    stat_tol   % Stationarity tolerance
    eq_tol     % Feasibility violation tolerance of equality constraints
    ineq_tol   % Feasibility violation tolerance of inequality constraints
    iter_max   % Iteration limit
	funevals_max% Function Evaluation limit
    nV         % Number of variables
    nE         % Number of equality constraints
    nI         % Number of inequality constraints
    pO         % Number of sample points for objective
    pE         % Numbers of sample points for equality constraints
    pI         % Numbers of sample points for inequality constraints
    f          % Problem function handle
    d          % Data to pass to function/gradient evaluators
    x          % Initial primal point
    output     % Output type
    log_fields % Fields of Iterate to log at each accepted point
    
  end
  
  % Class methods
  methods
    
    % Constructor
    function i = Input(prob)
       
      dir_prob = false;
      if isstruct(prob)
        in = prob;
      else
        dir_prob = true;
        % Assert that problem function directory has been provided as a string
        assert(ischar(prob),'SLQP-GS: Problem prob must either be a struct of parameters or a string of the path.');
      
        % Assert that slqpgs_inputs.m exists in problem function directory
        assert(exist(sprintf('%s/slqpgs_inputs.m',prob),'file')~=0,sprintf('SLQP-GS: Problem input file, %s, does not exist.',sprintf('%s/slqpgs_inputs.m',prob)));
  
        % Add problem function directory to path
        addpath(prob);
      
        % Read problem data
        in = feval('slqpgs_inputs');
      end
      
      % Set output to file, console, or completely off
      if ~isfield(in,'output') % By default, print to console
        in.output = 1;
      end
      switch in.output
        case 0
          [i.output,out_str] = deal(0,[]);
        case 1 
          [i.output,out_str] = deal(1,'console');
        otherwise
          if ischar(in.output)
            [i.output,out_str] = deal(in.output);
          else
            error('SLQP-GS: Output target, i.output must be 0, 1, or a filename.');
          end
      end
          
      if ~isempty(out_str) && dir_prob
        % Print directory
        fprintf('SLQP-GS: Added to path the directory "%s"\n',prob);
      
        % Print inputs file
        fprintf('SLQP-GS: Loaded inputs from "%s"\n',which('slqpgs_inputs'));
      end
      
      % Set data for functions
      if isfield(in,'algorithm') && ~isempty(in.algorithm),  i.algorithm  = in.algorithm;  else i.algorithm  = 0;                         end;
      if isfield(in,'sp_problem') && ~isempty(in.sp_problem), i.sp_problem = in.sp_problem; else i.sp_problem = 0;                         end;
      if isfield(in,'sp_solver') && ~isempty(in.sp_solver),  i.sp_solver  = in.sp_solver;  else i.sp_solver  = 'quadprog';                end;
      if isfield(in,'sp_options') && ~isempty(in.sp_options), i.sp_options = in.sp_options; else i.sp_options = optimset('Display','off'); end;

      % Set data for functions
      if isfield(in,'stat_tol') && ~isempty(in.stat_tol), i.stat_tol = in.stat_tol; else i.stat_tol = 1e-06; end;
      if isfield(in,'eq_tol') && ~isempty(in.eq_tol),   i.eq_tol   = in.eq_tol;   else i.eq_tol   = 1e-04; end;
      if isfield(in,'ineq_tol') && ~isempty(in.ineq_tol), i.ineq_tol = in.ineq_tol; else i.ineq_tol = 1e-04; end;
      if isfield(in,'iter_max') && ~isempty(in.iter_max), i.iter_max = in.iter_max; else i.iter_max = 1e+03; end;
	  if isfield(in,'funevals_max') && ~isempty(in.funevals_max), i.funevals_max = in.funevals_max; else i.funevals_max = 1e+03; end;

      % Assert that numbers of variables and constraints has been specified
      assert(isfield(in,'nV') & length(in.nV) == 1 & i.isscalarinteger(in.nV),'SLQP-GS: Number of variables, i.nV, must be specified as a scalar integer.');
      assert(isfield(in,'nE') & length(in.nE) == 1 & i.isscalarinteger(in.nE),'SLQP-GS: Number of equality constraints, i.nE, must be specified as a scalar integer.');
      assert(isfield(in,'nI') & length(in.nI) == 1 & i.isscalarinteger(in.nI),'SLQP-GS: Number of inequality constraints, i.nI, must be specified as a scalar integer.');
      
      % Set number of variables
      i.nV = in.nV; i.nE = in.nE; i.nI = in.nI;

      % Assert that number of sample points for objective has been specified
                   assert(isfield(in,'pO') & length(in.pO) ==    1 & i.isscalarinteger(in.pO),'SLQP-GS: Number of sample points for objective, i.pO, must be specified as a scalar integer.');
      if i.nE > 0, assert(isfield(in,'pE') & length(in.pE) == i.nE & i.isvectorinteger(in.pE),'SLQP-GS: Numbers of sample points for equality constraints, i.pE, must be specified as a vector of integers of length i.nE.'); end;
      if i.nI > 0, assert(isfield(in,'pI') & length(in.pI) == i.nI & i.isvectorinteger(in.pI),'SLQP-GS: Numbers of sample points for inequality constraints, i.pI, must be specified as a vector of integers of length i.nI.'); end;

      % Set number of sample points for objective
                   i.pO = in.pO;
      if i.nE > 0, i.pE = in.pE; end;
      if i.nI > 0, i.pI = in.pI; end;
      
      % Assert that problem function has been specified
      assert(isfield(in,'f'),'SLQP-GS: Problem functions filename/handle, i.f, must be specified.');
      if ischar(in.f)
         in.f = str2func(in.f);
      else
         assert(isa(in.f,'function_handle'),'SLQP-GS: Problem functions filename/handle, i.f, must be specified as a string or a function handle.');
      end
      
      % Set problem function handle
      i.f = in.f;
      
      % Set data for functions
      if isfield(in,'d'), i.d = in.d; else i.d = []; end;
      
      % Assert that initial point has been specified
      assert(isfield(in,'x') & length(in.x) == i.nV,'SLQP-GS: Initial point, i.x, must be specified as a vector of length i.nV');
      
      % Set initial point
      i.x = in.x;
      
      if isfield(in,'log_fields')
        assert(iscell(in.log_fields) & min(size(in.log_fields)) == 1,'SLQP-GS: Logging, i.log_fields must be 1D cell array.');
        i.log_fields = in.log_fields;
      else
        i.log_fields = [];
      end
              
      if ~isempty(out_str) 
        % Print success message
        fprintf('SLQP-GS: Inputs appear loaded successfully\n');
        % Since i.f can now be an anonymous function, calling which is no
        % longer possible.  But since this info is basically the same as
        % which('slqpgs_inputs'), we don't really need to print this "again".
        % fprintf('SLQP-GS: Problem to be read from "%s"\n',which(i.f));
        fprintf('SLQP-GS: Use "S.optimize" to run optimizer\n');
        fprintf('SLQP-GS: Output to be printed to %s\n',out_str);
        fprintf('SLQP-GS: Use "S.getSolution" to return final iterate\n');
        fprintf('SLQP-GS: (After optimizing and getting iterate, advised to "clear S" before loading new problem)\n');
      end
      
    end
  
    % Function for asserting vector of integers
    function b = isvectorinteger(i,a)

      % Loop through elements
      for j = 1:length(a), b = i.isscalarinteger(a(j)); if b == 0, break; end; end;
      
    end
    
  end
  
  % Class methods (static)
  methods (Static)
    
    % Function for asserting scalar integers
    function b = isscalarinteger(a)

      % Check length
      if length(a) > 1, b = 0; return; end;

      % Evaluate boolean
      b = (isnumeric(a) & isscalar(a) & round(a) == a);
      
    end
    
  end
  
end

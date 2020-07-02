function i = slqpgs_inputs

% function i = slqpgs_inputs
%
% Author      : Frank E. Curtis
% Description : Inputs for sample problem
% Output      : i ~ inputs

% Set algorithm
i.algorithm = 0;

% Set subproblem options
i.sp_problem = 0;
i.sp_solver  = 'quadprog';
i.sp_options = optimset('Display','off');

% Set tolerances
i.stat_tol = 1e-06;
i.eq_tol   = 1e-04;
i.ineq_tol = 0;
i.iter_max = 1e+03;

% Set problem size
i.nV = 2;
i.nE = 0;
i.nI = 1;

% Set sample sizes
i.pO = 2*i.nV;
i.pE = 0;
i.pI = 2*i.nV;

% Set initial point
i.x = randn(i.nV,1);

% Set function handle
i.f = 'slqpgs_problem';
% Alternatively, this can be specified as a function handle
% i.f = @slqpgs_problem;

% Set function data
i.d.w = 8;
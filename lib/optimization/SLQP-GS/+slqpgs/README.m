% README
%
% Author       : Frank E. Curtis
% Contributor  : Tim Mitchell
% Description  : README file.

% Please cite:
%   F. E. Curtis and M. L. Overton, ``A Sequential Quadratic Programming
%   Algorithm for Nonconvex, Nonsmooth Constrained Optimization,''
%   in third round of review for SIAM Journal on Optimization.

% This code runs a sequential quadratic programming method with gradient
% sampling sampling (SQP-GS) or a sequential linear programming method with
% gradient sampling (SLP-GS) for nonconvex, nonsmooth constrained
% optimization.  The problem of interest is formulated as
%   minimize f(x) subject to cE(x) = 0, cI(x) <= 0,
% where f, cE, and cI are all continuously differentiable on open dense
% subsets of R^n.  Global convergence results hold with probability one if
% the problem functions are also locally Lipschitz on R^n; see the above
% citation.

% A sample problem can be run, if the Matlab Optimization Toolbox is
% installed, via the commands
%   >> S = SLQPGS('sample');
%   >> S.optimize;
%   >> S.getSolution;
% An output file, slqpgs.out, will be generated in the current directory.

% Please note that the algorithm is stochastic in nature, meaning that two
% runs for the same problem instance can yield different results!  The
% states of Matlab's built-in rand and randn functions should be set prior
% to a run to obtain reproducable results.

% To run the code, a user need only create either:
%   - an slqpgs_inputs.m file specifying the required algorithm parameters
%    (see below) and optimization problem definition, which is called via
%       >> S = SLQPGS('path_to_this_file') 
%   - or a struct params specifying the same field/value pairs, called via
%       >> S = SLQPGS(params).
%     Using the provided sample problem as an example:
%       >> params = slqpgs_inputs();
%       >> S = SLQPGS(params);
%     Note that in this latter case, the functions implementing the problem
%     must already be on the path, as opposed to the former approach where
%     SLQPGS automatically adds path_to_this_file to the MATLAB path.
% The data function evaluations implementing the desired optimization
% problem can either be provided as a:
%   - string of the name of the corresponding file which implement all the
%     required data function evaluations or 
%   - an anonymous function handle of the same.
% The function signature (input and output arguments) is the same for both
% types.  Please refer to the sample problem in ./sample to see how these
% functions/files should be formatted and see below for more details.
 
% If a QP or LP solver other than Matlab's built-in quadprog or linprog,
% respectively, is desired, then changes to the solveSubproblem method in
% the Direction class may be necessary.  We recommend the solver MOSEK
% (www.mosek.com).  For that solver one need only set the path to include
% the directory containing MOSEK's quadprog function; no changes to the
% solveSubproblem method are necessary as the formatting for MOSEK's
% quadprog is the same as that for Matlab's.

% The remainder of this file includes more detail about how to create a
% problem instance, how to read the output file, and how to change the
% input parameters.

% CREATING A PROBLEM INSTANCE

% Please see the sample problem in ./sample as a guide.  The required
% fields for the input structure i, returned by slqpgs_inputs.m or provided
% explicitly by the user, are:
%   i.nV ~ number of variables
%   i.nE ~ number of equality constraints
%   i.nI ~ number of inequality constraints
%   i.pO ~ number of sample points for the objective
%   i.pE ~ if i.nE > 0, vector of length i.nE indicating numbers of sample
%           points for each equality constraint
%   i.pI ~ if i.nI > 0, vector of length i.nI indicating numbers of sample
%           points for each inequality constraint
%   i.f  ~ handle for file evaluating the problem functions and gradients
%   i.x  ~ initial point, a column vector of length i.nV
% Optional fields for the input structure i are:
%   i.algorithm  ~ algorithm option, 0 = SQP-GS and 1 = SLP-GS; default is 0
%   i.sp_problem ~ subproblem option, 0 = primal and 1 = dual; default is 0
%   i.sp_solver  ~ subproblem solver handle; default is 'quadprog'
%   i.sp_options ~ subproblem solver options; default is optimset('Display','off')
%   i.stat_tol   ~ termination tolerance; default is 1e-06
%   i.eq_tol     ~ feasibility tolerance for equality constraints; default is 1e-04
%   i.ineq_tol   ~ feasibility tolerance for inequality constraints; default is 1e-04
%   i.iter_max   ~ maximum iterations; default is 1e+03
%   i.d          ~ data structure to be passed to problem function evaluator
%   i.output     ~ type of output, 0 = none, 1 = console, string = filename
%                  to write output to; default is 1
%   i.log_fields ~ cell array of string names corresponding to fields of
%                  Iterate that one wishes to log on every step.  This
%                  history of iterate information is obtained after
%                  optimization has terminated by calling 
%                       [x,log] = S.getSolution()
%                  where x is the computed solution and log is the log.
%
% The numbers of sample points --- i.pO, i.pE, and i.pI --- should be
% chosen based on the smoothness of each problem function.  If a given
% function is smooth, then the number of sample points for that function
% may be set to zero.  If a given function is nonsmooth, then the number of
% sample points should be positive.  The convergence theory in the paper
% cited above requires that at least i.nV+1 points be sampled for each
% nonsmooth function.  However, if a given function depends on only m < i.nV
% variables, then it is reasonable to set the number of sample points to m+1
% (if not more).  Generally speaking, more sample points will produce
% better search directions, but at a higher cost per iteration.

% The .m file or anonymous function handle for evaluating the problem
% function and gradient values should follow the format in the example;
% i.e., it should contain a switch (or if statements) on the input o and
% return the desired function or gradient value through the output v.  Note
% that the objective gradient is expected to be a column vector and any
% constraint gradient is expected to be a row vector.  Also note that
% constraint function and gradient values are to be returned individually
% according to the index j provided as an input.  This is in contrast to
% many optimization codes that return an entire Jacobian matrix at once.

% READING THE OUTPUT FILE AND THE RETURNED VALUES

% The problem size is printed before the output of the optimization
% routine.  It is advisable to verify that these sizes match the problem
% one is intending to solve.  The subproblem size is that for the QP or LP
% solve to compute the search direction during every iteration.  This size
% will change depending on if the primal or dual form of the subproblem is
% solved (see i.sp_problem in the list of inputs above).

% A short description of each item in the output file is as follows.
% Notation in parentheses refers to quantities as they are denoted in the
% paper cited above.
%   Iter.      ~ iteration number (k)
%   Objective  ~ objective value (f)
%   Infeas.    ~ infeasibility measure (v)
%   Pen. Par.  ~ penalty parameter value (rho)
%   Merit      ~ merit/penalty function value (phi)
%   Msg.       ~ subproblem solver message (see below)
%   ||Step||   ~ norm of computed search direction (||d||)
%   Mod. Red.  ~ reduction in model of penalty function for computed search
%                direction (Delta-q_rho)
%   Samp. Rad. ~ sampling radius (epsilon)
%   Inf. Tol.  ~ infeasibility tolerance (theta)
%   Stepsize   ~ steplength determined by line search (alpha)
% Note that the subproblem solver message should be observed to ensure that
% the subproblems are being solved sufficiently accurately to produce good
% steps for the nonlinear optimization.  For example, Matlab's quadprog
% returns a value of 1 for a successful solve.

% Finally, the summary results at the end of the output should be
% self-explanatory.  The only exception is the number of gradient
% evaluations reported.  The number of gradient evaluations is *not*
% necessarily the number of times the gradient of a given problem function
% is evaluated.  Rather, it is the number of times that the "gradient
% evaluator" is called, which may then internally evaluate a given
% function's gradient a number of times depending on the number of sample
% points specified for that function.  Overall, if one desires to know the
% number of times that, say, the objective's gradient is evaluated, then
% take the number of "Function Evaluations" and add to that the number of
% sample points for the objective times the number of "Iterations".

% CHANGING INPUT PARAMETERS

% There are a variety of parameters whose value may affect the performance
% of the code on a given problem instance.  Besides those listed above as
% inputs to the code (e.g., the initial point i.x), there are constants
% defined in the Parameters class that may be changed.  Expert users may
% change these values, though it is not recommended.

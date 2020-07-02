% Parameters class
classdef Parameter < handle
  
  % Class properties (constant)
  properties (Constant)
    
    rho_init       = 1e-01; % Penalty parameter initial value
    rho_factor     = 5e-01; % Penalty parameter reduction factor
    epsilon_init   = 1e-01; % Sampling radius initial value
    epsilon_factor = 5e-01; % Sampling radius reduction factor
    theta_init     = 1e-01; % Feasibility forcing tolerance initial value
    theta_factor   = 8e-01; % Feasibility forcing tolerance increase factor
    update_tol     = 1e+01; % Epsilon updating tolerance (nu1)
    ls_suff_dec    = 1e-08; % Line search sufficient decrease parameter
    ls_factor      = 5e-01; % Line search stepsize reduction factor
    tr_radius      = 1e+02; % Trust region radius constant (for SLP-GS)
    lbfgs_hist     = 10;    % L-BFGS history length
    lbfgs_tol      = 1e-06; % L-BFGS curvature tolerance
    
  end
  
end

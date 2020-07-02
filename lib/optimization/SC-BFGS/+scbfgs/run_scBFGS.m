function [x,F] = run_scBFGS

% function [x,F] = run_scBFGS
%
% Author      : Frank E. Curtis
% Description : Driver for self-correcting BFGS (scBFGS) algorithm
% Input       : [none]
% Output      : x ~ final parameter vector
%               F ~ vector of training errors obtained over the optimization process

% Set global variables
global X y m b

% Load training data
load(fullfile(mfilename('fullpath'), '..', 'rcv1'));

% Set number of data values
m = size(X,1);

% Set algorithm parameters
obj   = 'scbfgs.logloss'; % loss function
dmax  = m;         % accessed data point (ADP) limit
a0    = 1/16;      % stepsize constants
a1    = 0;         % ... for the formula alpha = 1/(a0 + k*a1)
b     = 64;        % batch size
eta   = 1/4;       % BFGS displacement vector constants
theta = 4;         % ... equation (9) in reference paper
C     = 0;         % max iterations in SC-BFGS-sub in reference paper
rho   = 1/8;       % input parameters
tau   = 8;         % ... for SC-BFGS-sub in reference paper
M     = 5;         % L-BFGS history length (0 ~ full BFGS)

% Reset random number generator
rng('default');

% Set initial point
x = randn(size(X,2),1);

% Run SG
[x,F] = scbfgs.scBFGS(obj,x,dmax,a0,a1,eta,theta,C,rho,tau,M);
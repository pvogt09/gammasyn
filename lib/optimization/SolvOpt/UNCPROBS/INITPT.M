
% initpt 
% ------ 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function [n,m,x]=initpt(nprob,factor) 
% This function generates the starting points for each problem    
% by calling the function initf, which sets m, n and the starting 
% point xo for the number of problem given to it. If xo is the   
% standard starting point, then x will contain factor*xo, except  
% if xo is the zero vector and factor is not unity, then all the
% components of x will be set to factor.                      
%
% Created by Madhu Lamba on 10/30/94.                       
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [n,m,x] = initpt(nprob,factor)

	[n,m,x] = initf(nprob)
        x=factor*x
%

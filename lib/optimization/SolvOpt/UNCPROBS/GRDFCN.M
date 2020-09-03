% grdfcn function 
% ---------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function g=grdfcn(n,m,x,nprob)
% It is an interface function which calls the function func(which selects  
% appropriate test fuction based on nprob) to return fvec & J then it    
% calculates the gradient function's value.                               
%                                                                        
% Created on 10/22/94 by Madhu Lamba                                    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                                                             
function g = grdfcn(n,m,x,nprob)

  [fvec,J] = func(n,m,x,nprob,3);
  
  g = 2 * fvec' * J ;
  g=g'; 

%

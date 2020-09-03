% objfcn function 
% --------------- 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function f=objfcn(n,m,x,nprob)
% It is an interface function which calls the function func(which selects  
% appropriate test fuction based on nprob) to return fvec, then it 
% calculates the objective function by fvec'*fvec.      
%                                                                        
% Created on 10/22/94 by Madhu Lamba                                    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function f = objfcn(n,m,x,nprob)

  fvec = func(n,m,x,nprob,1);
  f=fvec' * fvec;
%

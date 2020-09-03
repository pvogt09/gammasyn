% vecjac function 
% ---------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function J=vecjac(n,m,x,nprob)
% It is an interface function which calls the function func (which selects  
% appropriate test function based on nprob) with option=2 inorder to return
% J, the vector.                                                      
%                                                                        
% Created on 10/30/94 by Madhu Lamba                                   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                                                                            


function J = vecjac(n,m,x,nprob)
  
  
   [fvec,J] = func(n,m,x,nprob,2) ;
%

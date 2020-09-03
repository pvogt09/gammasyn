% vecfcn function 
% ---------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function fvec=vecfcn(n,m,x,nprob)
% It is an interface function which calls the function func (which selects  
% appropriate test function based on nprob) with option=1 inorder to return
% fvec , the vector.                                                      
%                                                                        
% Revised on 10/22/94 by Madhu Lamba                                   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                                                                            


function fvec = vecfcn(n,m,x,nprob)
  
  
    fvec = func(n,m,x,nprob,1);
%

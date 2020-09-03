% Beale function 
% -------------- 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function [fvec,J]=beale(n,m,x,option)
% Problem no. 5
% Dimensions -> n=2, m=3              
% Standard starting point -> x=(1,1) 
% Minima -> f=0 at (3,0.5)            
%                                     
% Revised on 10/22/94 by Madhu Lamba  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [fvec,J] = beale(n,m,x,option)

if (option==1 | option==3)
        fvec = [  1.5-x(1)*(1-x(2))
                  2.25-x(1)*(1-x(2)^2)
                  2.625-x(1)*(1-x(2)^3)  ]; 
end;        
if (option==2 | option==3)
        J    = [  -(1-x(2))      x(1)
                  -(1-x(2)^2)    x(1)*2*x(2)
                  -(1-x(2)^3)    x(1)*3*x(2)^2  ]; 
        
end;
%

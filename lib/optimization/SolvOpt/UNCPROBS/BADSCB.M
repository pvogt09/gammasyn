% Brown badly scaled function 
% --------------------------- 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function [fvec,J]=badscb(n,m,x,option)
% Problem no. 4
% Dimensions -> n=2, m=3              
% Standard starting point -> x=(1,1)  
% Minima -> f=0 at (1e+6,2e-6)        
%                                     
% Revised on 10/22/94 by Madhu Lamba  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [fvec,J] = badscb(n,m,x,option)  
if (option==1 | option==3)
        fvec = [  x(1)-10^6
                  x(2)-(2e-6)
                  x(1)*x(2)-2  ]  ;
end;        
if (option==2 | option==3)
        J    = [  1      0
                  0      1
                  x(2)   x(1)  ] ;
end;

%


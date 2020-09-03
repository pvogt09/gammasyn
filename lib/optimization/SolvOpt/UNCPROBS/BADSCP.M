% Powell badley scaled function 
% ----------------------------- 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function [fvec,J]=badscp(n,m,x,option)   
% Problem no. 3
% Dimensions -> n=2, m=2                      
% Standard starting point -> x=(0,1)          
% Minima -> f=0 at (1.098...10-E5,9.106...)   
%                                             
% Revised on 10/22/94 by Madhu Lamba          
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [fvec,J] = badscp(n,m,x,option)

if (option==1 | option==3)
        fvec = [  10^4*x(1)*x(2)-1
                  exp(-x(1))+exp(-x(2))-1.0001  ] ;
end;
if (option==2 | option==3)
        J    = [  10^4*x(2)        10^4*x(1)
                  -exp(-x(1))      -exp(-x(2))  ] ;
end;

%

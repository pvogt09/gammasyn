% Freudenstein and Roth function 
% ------------------------------ 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
% function [fvec,J]=froth(n,m,x,option)     
% Problem no. 2
% Dimensions -> n=2, m=2                           
% Standard starting point -> x=(0.5,-2)            
% Minima -> f=0 at (5,4)                           
%           f=48.9842... at (11.41...,-0.8968...)  
%                                                  
% Revised on 10/22/94 by Madhu Lamba               
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [fvec,J] = froth(n,m,x,option)

if (option==1 | option==3)
        fvec = [ -13+x(1)+((5-x(2))*x(2)-2)*x(2)
                 -29+x(1)+((x(2)+1)*x(2)-14)*x(2) ]; 
end;        
if (option==2 | option==3)
        J    = [ 1       10*x(2)-3*x(2)^2-2
                 1       3*x(2)^2+2*x(2)-14  ] ;
        
end;
%

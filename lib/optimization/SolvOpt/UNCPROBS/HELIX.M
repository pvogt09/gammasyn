function [fvec,J] = helix(n,m,x,opt)

% *******************************************
% *******************************************
% function [ fvec, J]= helix(n,m,x,opt)
%
% Helical valley function  [7]
% Dimensions    n=3,   m=3
% Function Definition:
%       f1(x) = 10[x3 - 10*(x1,x2)]
%       f2(x) = 10[((x1)^2 + (x2)^2)^.5 -1]
%       f3(x) = x3
% Standard starting point  x= (-1,0,0)
% Minima of f=0 at (1,0,0)
%
% Revised 10/23/94   PLK
% *********************************************
  
if ((opt==1) | (opt ==3))
    if x(1) > 0
	   fvec(1)  =  10*(x(3)-10*((1/(2*pi))*atan(x(2)/x(1))));                                        
    elseif x(1) < 0
	   fvec(1)  =  10*(x(3)-10*((1/(2*pi))*atan(x(2)/x(1))+.5));                       
    end
    fvec(2)  = 10*((x(1)^2+x(2)^2)^.5-1);
    fvec(3)  = x(3);
    fvec=fvec';                   
end;

if ((opt ==2) | (opt == 3))
        J(1,1)   =    (50/pi)*(x(2)/x(1)^2)*(1/(1+(x(2)/x(1))^2));
        J(1,2)   =    (-50/pi)*(1/x(1))*(1/(1+(x(2)/x(1))^2));
        J(1,3)   =    10;

        J(2,1)   =    (10*x(1))/sqrt(x(1)^2+x(2)^2);
        J(2,2)   =    (10*x(2))/sqrt(x(1)^2+x(2)^2);
        J(2,3)   =    0;

        J(3,1)   =    0;
        J(3,2)   =    0;
        J(3,3)   =    1;
end;
 
if ((opt <1) | (opt >3))
        disp('Error: Option value sent to HELIX.M is either <1 or >3');
end;


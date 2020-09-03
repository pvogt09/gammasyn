function [fvec,J] = kowosb(n,m,x,opt)

% ***********************************************************
% Function [fvec, J]=kowosb(n,m,x,opt)
% Kowalik and Osborne function    [15]
% Dimensions:     n=4   m=11
% Function Definition:
%       f(x)= y(i) - [x1(u^2 +u*x2) / (u^2 + u*x3 + x4)]
% Standard starting point: (0.25,0.39,0.415,0.39)
% Minima of f= 3.07505...10^-4 and
%           f= 1.02734...10^-3  at (inf,-14.07...,-inf,-inf)
%
% Coded in Matlab   October 1994        PLK
% **********************************************************

y     = [.1957  .1947  .1735  .1600  .0844  .0627  
         .0456  .0342  .0323  .0235  .0246      0]' ;

u     = [4.0000  2.0000  1.0000  0.5000  0.2500  0.1670
         0.1250  0.1000  0.0833  0.0714  0.0625       0]' ;
         


for i = 1:m
   c1 = u(i)^2 + u(i)*x(2);
   c2 = (u(i)^2 + u(i)*x(3) +x(4));
  
   if((opt==1)|(opt==3))
    fvec(i) =  y(i)-(x(1)*c1)/c2;
   end;

   if((opt==2)|(opt==3))
    J(i,1) =  -c1/c2;
    J(i,2) = (-x(1)*u(i) ) / c2;
    J(i,3) = x(1)*c1*(c2^(-2))*u(i); 
    J(i,4) = x(1)*c1*(c2^(-2));
   end;
end;

fvec=fvec';

if((opt<1)|(opt>3))
        disp('Error: Option value for KOWOSB.M is either <1 or >3');
end;
    

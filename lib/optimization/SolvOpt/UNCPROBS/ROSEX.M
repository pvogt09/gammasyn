% Extended Rosenbrock function 
% ---------------------------- 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function [fvec,J]=rosex(n,m,x,option)
% Dimensions -> n=variable but even, m=n 
% Problem no. 21            
% Standard starting point -> x=(s(j)) where 
%                            s(2*j-1)=-1.2, 
%                            s(2*j)=1 
% Minima -> f=0 at (1,.....,1)              
%                                     
% 11/21/94 by Madhu Lamba  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [fvec,J] = rosex(n,m,x,option)  

J=zeros(m,n);

for i=1:m/2

   if (option==1 | option==3)
        fvec(2*i-1)=10*(x(2*i)-x(2*i-1)^2);
        fvec(2*i)=1-x(2*i-1);
   end;        

   if (option==2 | option==3)
        J(2*i-1,2*i-1) = -20*x(2*i-1);
        J(2*i-1,2*i)   = 10; 
        J(2*i,2*i-1)   = -1;
   end;

end;

if (option==1 | option==3)
	fvec=fvec';
end;

 
%

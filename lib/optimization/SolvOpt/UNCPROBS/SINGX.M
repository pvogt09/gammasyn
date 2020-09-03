% Extended Powell Singular function
% --------------------------------- 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function [fvec,J]=singx(n,m,x,option)
% Dimensions -> n=variable but a multiple of 4, m=n             
% Problem no. 22
% Standard starting point -> x=(s(j)) where 
%                            s(4*j-3)=3, 
%                            s(4*j-2)=-1,
%                            s(4*j-1)=0,
%                            s(4*j)=1 
% Minima -> f=0 at the origin.            
%                                     
% 11/21/94 by Madhu Lamba  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [fvec,J] = singx(n,m,x,option)  

J=zeros(m,n);
for i=1:m/4

   if (option==1 | option==3)
        fvec(4*i-3)=x(4*i-3)+10*(x(4*i-2));
        fvec(4*i-2)=sqrt(5)*(x(4*i-1)-x(4*i));
        fvec(4*i-1)=(x(4*i-2)-2*(x(4*i-1)))^2;
        fvec(4*i)  =sqrt(10)*(x(4*i-3)-x(4*i))^2;
   end;        

   if (option==2 | option==3)
	J(4*i-3,4*i-3) = 1;
        J(4*i-3,4*i-2) = 10;
        J(4*i-2,4*i-1) = sqrt(5);
        J(4*i-2,4*i)   = -sqrt(5);
        J(4*i-1,4*i-2) = 2*x(4*i-2)-4*x(4*i-1);
        J(4*i-1,4*i-1) = 8*x(4*i-1)-4*x(4*i-2);
        J(4*i,4*i-3)   = 2*sqrt(10)*(x(4*i-3)-x(4*i));
        J(4*i,4*i)     = 2*sqrt(10)*(x(4*i)-x(4*i-3));
   end;
end;

if (option==1 | option==3)
   fvec=fvec';
end;

%

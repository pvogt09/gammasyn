% Discrete boundary value function
% -------------------------------- 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function [fvec,J]=bv(n,m,x,option)
% Dimensions -> n=variable, m=n
% Standard starting point -> x=(s(j)) where
%                            s(j)=t(j)*(t(j)-1) where
%                            t(j)=j*h & h=1/(n+1)
% Minima -> f=0 
%                                     
% 12/4/94 by Madhu Lamba  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [fvec,J] = bv(n,m,x,option)  

J=zeros(m,n);
h=1/(n+1);
for i=1:n
   t(i)=i*h;

   if (option==1 | option==3)
     x(n+1)=0;
     if (i==1)
        fvec(i)=2*x(i)-x(i+1)+(h^2*(x(i)+t(i)+1)^3)/2;    
     elseif (i==n)
        fvec(i)=2*x(i)-x(i-1)+(h^2*(x(i)+t(i)+1)^3)/2;
     else
        fvec(i)=2*x(i)-x(i-1)-x(i+1)+(h^2*(x(i)+t(i)+1)^3)/2;
     end;
   end;

   if (option==2 | option==3)
        J(i,i)=2+h^2/2*3*(x(i)+t(i)+1)^2;
        if (i<n)
           J(i,i+1)=-1;
           J(i+1,i)=-1;
        end;
   end;
end;
 
if (option==1 | option==3)
   fvec=fvec';
end;

%


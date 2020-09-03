% Broyden tridiagonal function
% ---------------------------- 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function [fvec,J]=trid(n,m,x,option)
% Dimensions -> n=variable, m=n
% Problem no. 30
% Standard starting point -> x=(-1,..,-1)
% Minima -> f=0 
%                                     
% 11/21/94 by Madhu Lamba  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [fvec,J] = trid(n,m,x,option)  

J=zeros(m,n);
for i=1:n

   if (option==1 | option==3)
      x(n+1)=0;
      if (i==1)
         fvec(i)=(3-2*x(i))*x(i)-2*x(i+1)+1;
      elseif (i==n)
         fvec(i)=(3-2*x(i))*x(i)-x(i-1)+1;
      else
         fvec(i)=(3-2*x(i))*x(i)-x(i-1)-2*x(i+1)+1;
      end; 
   end;       

   if (option==2 | option==3)
      J(i,i)=3-4*x(i);
      if (i<n)
         J(i,i+1)=-2;
         J(i+1,i)=-1;
      end;
   end;
end; 

if (option==1 | option==3)
   fvec=fvec';
end;

%


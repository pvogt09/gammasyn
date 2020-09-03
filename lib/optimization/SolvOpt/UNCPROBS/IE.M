% Discrete integral equation function
% ----------------------------------- 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function [fvec,J]=ie(n,m,x,option)
% Dimensions -> n=variable, m=n
% Standard starting point -> x=(s(j)) where
%                            s(j)=t(j)*(t(j)-1) where
%                            t(j)=j*h & h=1/(n+1)
% Minima -> f=0 
%                                     
% 12/4/94 by Madhu Lamba  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [fvec,J] = ie(n,m,x,option)  

J=zeros(m,n);
h=1/(n+1);
for i=1:n
   t(i)=i*h;
end;

for i=1:n

   if (option==1 | option==3)
        x(n+1)=0;
        sum1=0;
        for j=1:i
            sum1=sum1+t(j)*(x(j)+t(j)+1)^3;
        end;
        sum2=0;
        if (n>i) 
          for j=i+1:n
              sum2=sum2+(1-t(j))*(x(j)+t(j)+1)^3;
          end;
        end;
        fvec(i)=x(i)+h*((1-t(i))*sum1+t(i)*sum2)/2;
   end;
 
   if (option==2 | option==3)
      for j=1:n
         if (j<i)
            J(i,j)=3*h/2*(1-t(i))*t(j)*(x(j)+t(j)+1)^2;
         elseif (j>i)
            J(i,j)=3*h/2*t(i)*(1-t(j))*(x(j)+t(j)+1)^2; 
         elseif (j==i)
            J(i,i)=1+3*h/2*(1-t(i))*t(i)*(x(i)+t(i)+1)^2;
         end;
      end;
   end;
end;   
if (option==1 | option==3)
   fvec=fvec';
end;

%


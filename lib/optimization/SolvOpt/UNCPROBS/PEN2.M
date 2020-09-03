% Penalty II  function
% ------------------- 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function [fvec,J]=pen2(n,m,x,option)
% Dimensions -> n=variable, m=2*n
% Problem no. 24             
% Standard starting point -> x=(1/2,......,1/2)
% Minima -> f=9.37629...10^(-6)  if n=4
%           f=2.93660...10^(-4)  if n=10            
%                                     
% 11/21/94 by Madhu Lamba  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [fvec,J] = pen2(n,m,x,option)  


J=zeros(m,n);
if (option==1 | option==3)
   fvec(1)=x(1)-0.2;
end;

if (option==2 | option==3)
   J(1,1)=1;
end;

if(n>=2)

   for i=2:n
     y(i)=exp(i/10)+exp((i-1)/10);

     if (option==1 | option==3)
        fvec(i)=sqrt(1.e-5)*(exp(x(i)/10)+exp(x(i-1)/10)-y(i));
     end;        

     if (option==2 | option==3)
	J(i,i)   = sqrt(1.e-5)*exp(x(i)/10)*(1/10);
        J(i,i-1) = sqrt(1.e-5)*exp(x(i-1)/10)*(1/10);
     end;
   end;
  
   for i=n+1:(2*n-1)
       
     if (option==1 | option==3)
        fvec(i)=sqrt(1.e-5)*(exp(x(i-n+1)/10)-exp(-1/10));
     end;
 
     if (option==2 | option==3)
	J(i,i-n+1) = sqrt(1.e-5)*exp(x(i-n+1)/10)*(1/10);
     end; 
   end;
end;

if (option==1 | option==3) 

    sum=0;
    for j=1:n
        sum=sum+(n-j+1)*x(j)^2;
    end;
    fvec(2*n)=sum-1; 
end;
if (option==2 | option==3)
    for j=1:n
        J(m,j) = (n-j+1)*2*x(j);
    end;
end;
if (option==1 | option==3)
   fvec=fvec';
end;

%


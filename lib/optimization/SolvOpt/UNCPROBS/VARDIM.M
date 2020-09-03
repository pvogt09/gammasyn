% Variably Dimensioned function
% ----------------------------- 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function [fvec,J]=vardim(n,m,x,option)
% Dimensions -> n=variable, m=n+2
% Problem no. 25
% Standard starting point -> x=(s(j)) where 
%                            s(j)=1-(j/n) 
% Minima -> f=0 at (1,.....1)
%                                     
% 11/21/94 by Madhu Lamba  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [fvec,J] = vardim(n,m,x,option)  

J=zeros(m,n);
for i=1:n

   if (option==1 | option==3)
        fvec(i)=x(i)-1;
   end;        

   if (option==2 | option==3)
	J(i,i)=1;
   end;
end 

var_1=0;
for j=1:n
    var_1=var_1+j*(x(j)-1);
end;
if (option==1 | option==3)
        fvec(n+1)=var_1;
        fvec(n+2)=(var_1)^2;
end;

if (option==2 | option==3)
        for j=1:n
 	    J(n+1,j) = j;
            J(n+2,j) = (2*var_1*j);
        end;
end;

if (option==1 | option==3)
   fvec=fvec';
end;

%


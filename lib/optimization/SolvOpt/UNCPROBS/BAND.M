function [fvec,J] = band(n,m,x,opt)

%******************************************
% Function [Fvec, J]= band (n,m,x,opt)
% Broyden banded function   [31]
% Dimensions: n variable,   m=n
% Standard starting point: (-1,...,-1)
% minima of f=0
%
% coded in MATLAB  11/94        plk
% *****************************************   

ml=5;
mu=1;
 
J=zeros(m,n);
for i=1:m
        sum=0;
        lb=max(1,i-ml);
        lu=min(n,i+mu);
         
        for j=1:n
           if (j ~= i)
              if((j>=lb)&(j<=lu))
                 sum=sum + x(j)*(1+x(j));
              end;
           end;
        end;


        if((opt==1)|(opt==3))
          fvec(i)=x(i)*(2+5*(x(i)^2))+1-sum;
        end;

        if((opt==2)|(opt==3))
          for j=1:n
            if i==j
               J(i,j)=2+15*(x(i)^2);          
            elseif((j>=lb)&(j<=lu)) 
               J(i,j)=-1-2*x(j);
            end;
          end;
        end;
end;                 
fvec=fvec';

if((opt<1)|(opt>3))
   disp('Error: Option value sent to BAND.M is either <1 or >3 ');
end;

function [fvec,J]= lin(n,m,x,opt)
%Function [fvec,J]= lin(n,m,x,opt)
%Linear function - full rank [32]
%Dimensions: n variable,      m>=n
%Standard starting point: (1,...,1)
%Minima of f=m-n at (-1,...,-1)
%
%Coded in MATLAB   11/94        plk

J=zeros(m,n);
for i=1:n

    sum1=sum(x);
        
    if((opt==1)|(opt==3))
        fvec(i)= x(i)-(2/m)*sum1-1;
    end;

    if((opt==2)|(opt==3))
        for j=1:n
           if i==j
                J(i,j)=1-(2/m);
           else J(i,j)=-(2/m);
           end;
        end;
    end;
end;

for i=n+1:m
        
    if((opt==1)|(opt==3))
        fvec(i)= -(2/m)*sum1-1;
    end;

    if((opt==2)|(opt==3))
        for j=1:n
                J(i,j)=-(2/m);
        end;
    end;
end;
       
fvec=fvec';

if((opt<1)|(opt>3))
        disp('Error: Option value sent to LIN.M is either <1 or >3');
end;


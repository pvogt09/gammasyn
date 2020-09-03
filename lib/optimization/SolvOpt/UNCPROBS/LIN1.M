function [fvec,J]= lin1(n,m,x,opt)

% **********************************
% Function [fvec,J] = lin1(n,m,x,opt)
% Linear function - rank 1   [33]
% Dimensions:  n variable,    m>=n
% Standard starting point: (1,....,1)
% Minima of f=[(m(m-1))/(2(2m+1))]
%
% Coded in MATLAB    11/94      PLK
% **********************************
J=zeros(m,n);
for i = 1:m
   
    sum1=0;
    for j = 1:n
       sum1=sum1 + j*x(j);
    end;

    if((opt==1)|(opt==3))
        fvec(i)= i*sum1 - 1;
    end;

    if((opt==2)|(opt==3))
       
        for j= 1:n
           J(i,j)=i*j;
        end;

    end;
end;
fvec=fvec';

if((opt<1)|(opt>3))
        disp('Error: Option value sent to LIN1.M is either <1 or >3');
end;


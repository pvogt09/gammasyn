function[fvec,J]=lin0(n,m,x,opt)
% ****************************************************
% Function[fvec,J] = lin0(n,m,x,opt)
% Linear function - rank 1 with zero columns and rows  [34] 
% Dimensions: n variable,     m>=n
% Standard starting point: (1,...1)
% Minima of f=(m^2 + 3m - 6)/2(2m - 3)
%
% Coded in MATLAB    11/94      PLK
% *****************************************************

J=zeros(m,n);
for i=2:m-1
        sum=0;
        for j=2:n-1
                sum=sum + j*x(j);
        end;
        
        if((opt==1)|(opt==3))
           fvec(i)=(i-1)*sum -1;
        end;

        if((opt==2)|(opt==3))
          for j=2:n-1
             J(1,j)=0;
             J(i,j)=j*(i-1);
             J(m,j)=0;
          end;
        end;
end;
 
if((opt==1)|(opt==3))
    fvec(1)=-1;
    fvec(m)=-1;
end;
if((opt==2)|(opt==3))
    J(1,1)=0;
    J(m,n)=0;
end;
fvec=fvec';

if((opt<1)|(opt>3))
        disp('Error: Option value sent to LIN0.M is either <1 or <3');
end;

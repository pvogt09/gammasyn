function [fvec,J] = bard(n,m,x,opt)
% **************************************************************
% **************************************************************
%  function [fvec,J]= bard(n,m,x,opt)
%  Bard function       [8] 
%  Dimensions  n=3,    m=15
%  Function definition:
%       f(x) = y(i) - [x1 + (u(i) / v(i)x2 + w(i)x3)]
%       where u(i) = i, v(i) = 16-i, w(i) = min(u(i),v(i))
%  Standard starting point at x= (1,1,1)
%  Minima f=8.21487...10^(-3)   and f=17.4286 at (0.8406...,-inf,-inf)
%
%  Revised 10/23/94   PLK
% **************************************************************


y    = [.14  .18  .22  .25  .29  .32  .35  .39  .37  .58
        .73  .96  1.34 2.10 4.39   0    0    0    0    0 ]' ;

J=zeros(m,n);

for i = 1:m
     
    u(i) = i;
    v(i) = 16 - i;
    w(i) = min(u(i),v(i));

    if ( (opt ==1) | (opt == 3))
        fvec(i) = y(i)-(x(1)+(u(i)/(v(i)*x(2)+w(i)*x(3))));
    end;

    if ((opt ==2) | (opt ==3))    
        J(i,1) =  -1;
        J(i,2) =  (u(i)*v(i))/((v(i)*x(2)+w(i)*x(3))^2);
        J(i,3) =  (u(i)*w(i))/((v(i)*x(2)+w(i)*x(3))^2);
    end;
    
    
end;
fvec=fvec';
    if ((opt<1)|(opt>3))
        disp('Error: Option value sent to BARD.M is either <1 or >3');
    end;



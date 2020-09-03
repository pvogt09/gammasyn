function [fvec,J] = biggs(n,m,x,opt)

% ******************************************
%  function [fvec,J] = biggs(n,m,x,opt)
%  Biggs EXP6 function   [18]
%  Dimensions :  n=6,  m=13
%  Standard starting point (1,2,1,1,1,1)
%  Minima of f=5.65565...10^(-3)   if m=13
%            f=0 at (1,10,1,5,4,3)
%
%  Revised  11/94               PLK
% ******************************************
J=zeros(m,n);

for i = 1:m
  t(i) = .1*i;
  y(i) = exp(-t(i))-5*exp(-10*t(i))+3*exp(-4*t(i));

 if((opt==1) | ( opt==3))
  fvec(i) = x(3)*exp(-t(i)*x(1))-x(4)*exp(-t(i)*x(2))+x(6)*exp(-t(i)*x(5))-y(i);
 end;

 if((opt==2) | (opt==3))
  J(i,1)  = -t(i)*x(3)*exp(-t(i)*x(1));   
  J(i,2)  =  t(i)*(x(4))*exp(-t(i)*x(2));
  J(i,3)  = exp(-t(i)*x(1));
  J(i,4)  = -exp(-t(i)*x(2));
  J(i,5)  = x(6)*(-t(i))*exp(-t(i)*x(5));
  J(i,6)  = exp(-t(i)*x(5));
 end;

end; 
fvec=fvec';

if((opt<1) | (opt>3))
        disp('Error: Option value sent to BIGGS.M is either <1 or >3');
end;


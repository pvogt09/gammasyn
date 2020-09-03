function [fvec,J] = watson(n,m,x,opt)

% **************************************************
%  Function [fvec,J]= watson(n,m,x,opt)
%  Watson function    [20]
%  Dimensions : n=20,  m=31
%  Standard starting point : (0,....,0)
%  Minima of f=2.28767...10^(-3)    if n=6
%            f=1.39976...10^(-6)    if n=9
%            f=4.72238...10^(-10)   if n=12
%	     f=2.48631...10^(-20)   if n=20
%
% Revised  11/94        PLK
% **************************************************


for i = 1:29
    
    t(i) = i / 29;
    sum1 = 0;
    for j = 2:n
        sum1 = sum1 + (j-1) *(x(j) * (t(i)^(j-2)));
    end;

    sum2 = 0;
    for j = 1:n
        sum2 = sum2 + x(j) * (t(i)^(j-1));
    end;

  if((opt==1) | ( opt==3))
    fvec(i) =  sum1-(sum2^2)-1;
  end;

  if ((opt==2)|(opt==3))
    J(i,1)  =  -(2*sum2);
   
    for j = 2:n
        J(i,j) =  (j-1)*((t(i))^(j-2))-2*sum2*(t(i))^(j-1);
    end;
  end;

end;

if((opt==1)|(opt==3)) 
  fvec(30) =  x(1);
  fvec(31) =  x(2)-((x(1))^2)-1;
end;

if((opt==2)|(opt==3))
J(30,1)  =  1;

for r = 2:n
    J(30,r) = 0;
end;



J(31,1)  =  -2*x(1);
J(31,2)  =  1;

for r = 3:n
    J(31,r) = 0;
end;
end;%if statement
 
fvec=fvec';

if((opt<1)|(opt>3))
        disp('Error: Option value sent to WATSON.M is either <1 or >3');
end;




%

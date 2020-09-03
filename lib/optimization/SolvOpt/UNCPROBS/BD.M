function [fvec,J] = bd(n,m,x,opt)

%  function [fvec,J] = bd(n,m,x,opt)
%  Brown and Dennis function  [16]
%  Dimensions:  n=4,  m=20
%  Function Definition:
%       f(x)=(x1 + t(i)x2- exp[t(i)])^2 +(x3 + x4sin(t(i))- cos(t(i)))^2
%       where t(i)=i/5
%  Standard starting point (25,5,-5,-1)
%  Minima of f=85822.2... if m=20
%
%  Revised  11/94               PLK
%
      two = 2.d0;
      point2 = .2d0;
      x1 = x(1);
      x2 = x(2);
      x3 = x(3);
      x4 = x(4);

     if opt==1
	for i = 1: m
        	ti   = (i)*point2;
        	ei   = exp(ti);
        	si   = sin(ti);
       		ci   = cos(ti);
        	fvec(i) = (x1 + ti*x2 - ei)^2 + (x3 + x4*si - ci)^2;
	end
	fvec=fvec';

     elseif opt==2
	for i=1:m
        	ti = (i)*point2;
        	ei = exp(ti);
        	si = sin(ti);
        	ci = cos(ti);
        	f1 = two*(x1 + ti*x2 - ei);
        	f3 = two*(x3 + x4*si - ci);
        	J( i, 1) = f1;
        	J( i, 2) = f1 * ti;
        	J( i, 3) = f3;
        	J( i, 4) = f3 * si;
	end

     elseif opt==3
	for i=1:m
        	ti = (i)*point2;
        	ei = exp(ti);
        	si = sin(ti);
        	ci = cos(ti);
        	f1 = two*(x1 + ti*x2 - ei);
        	f3 = two*(x3 + x4*si - ci);
        	fvec(i) = (x1 + ti*x2 - ei)^2 + (x3 + x4*si - ci)^2;
        	J( i, 1) = f1;
        	J( i, 2) = f1 * ti;
        	J( i, 3) = f3;
        	J( i, 4) = f3 * si;
	end
	fvec=fvec';

     else 
	error('Error: bd.m - Invalid option')
     end;

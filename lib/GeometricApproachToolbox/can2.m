function [a2,b2,c2,d2] = can2(a,b,c,d)
%CAN2     Controller canonical realization.
%       [a2,b2,c2,d2] = CAN2(a,b,c,d) calculates the controller
%       canonical realization for the SISO quadruple (a,b,c,d) .

%       G.Marro 11-18-91

narginchk(4,4);
error(abcdchk(a,b,c,d));
if length(a)==0
	a2=a; b2=b; c2=c; d2=d;
	return
end
[t,mb] = size(b);
[nc,t] = size(c);
if (mb~=1)||(nc~=1)
	error('GEOAPPR:input', 'not a SISO system in can2')
end
T2=ctrb(a,b)*maqu(a);
a2=T2\a*T2;
b2=T2\b;
c2=c*T2;
d2=d;
% --- last line of can2 ---

function q = maqu(a)
%MAQU     Axis transformation for canonical realizations.
%        Q = MAQU(A) gives the tranformation matrix Q to transform the
%        controllability to the controller canonical form as a function of
%        system matrix A.

%        G. Marro 3-22-91 rev. 11-11-91

n = length(a);
p = poly(a);
q = eye(n);
for k=1:n-1
	q(k,k+1:n)=p(2:n+1-k);
end
% --- last line of maqu ---
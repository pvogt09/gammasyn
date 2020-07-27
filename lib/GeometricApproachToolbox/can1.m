function [a1,b1,c1,d1] = can1(a,b,c,d)
%CAN1     Controllability canonical realization.
%       [a1,b1,c1,d1] = CAN1(a,b,c,d) calculates the controllability
%       canonical realization for the SISO quadruple (a,b,c,d) .

%       G.Marro 11-18-91

narginchk(4,4);
error(abcdchk(a,b,c,d));
if length(a)==0
	a1=a; b1=b; c1=c; d1=d;
	return
end
[t,mb] = size(b);
[nc,t] = size(c);
if (mb~=1)||(nc~=1)
	error('GEOAPPR:input', 'not a SISO system in can1')
end
T1=ctrb(a,b);
a1=T1\a*T1;
b1=T1\b;
c1=c*T1;
d1=d;
% --- last line of can1 ---
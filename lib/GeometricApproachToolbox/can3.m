function [a3,b3,c3,d3] = can3(a,b,c,d)
%CAN3     Observability canonical realization.
%       [a3,b3,c3,d3] = CAN3(a,b,c,d) calculates the observability
%       canonical realization for the SISO quadruple (a,b,c,d) .

%       G.Marro 11-18-91

narginchk(4,4);
error(abcdchk(a,b,c,d));
if length(a)==0
	a3=a; b3=b; c3=c; d3=d;
	return
end
[t,mb] = size(b);
[nc,t] = size(c);
if (mb~=1)||(nc~=1)
	error('GEOAPPR:input', 'not a SISO system in can3')
end
[aa,bb,cc,dd]=can1(a',c',b',d);
a3=aa';
b3=cc';
c3=bb';
d3=d;
% --- last line of can3 ---
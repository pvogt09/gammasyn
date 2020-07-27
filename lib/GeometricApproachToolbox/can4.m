function [a4,b4,c4,d4] = can4(a,b,c,d)
%CAN4     Observer canonical realization.
%       [a4,b4,c4,d4] = CAN4(a,b,c,d) calculates the observer
%       canonical realization for the SISO quadruple (a,b,c,d) .

%       G.Marro 11-18-91

narginchk(4,4);
error(abcdchk(a,b,c,d));
if length(a)==0
	a4=a; b4=b; c4=c; d4=d;
	return
end
[t,mb] = size(b);
[nc,t] = size(c);
if (mb~=1)||(nc~=1)
	error('GEOAPPR:input', 'not a SISO system in can3')
end
[aa,bb,cc,dd]=can2(a',c',b',d);
a4=aa';
b4=cc';
c4=bb';
d4=d;
% --- last line of can4 ---
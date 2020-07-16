function [Ad,Bd,Cd,Dd] = dualsys(A,B,C,D)
%DUALSYS  Dual system.
%  [Ad,Bd,Cd,Dd] = DUALSYS(A,B,C,D) or [Ad,Bd,Cd,Dd] = DUALSYS(sys)
%  or sysd = DUALSYS(sys) provides the dual of (A,B,C,D) or of sys.

%  G.Marro 8-20-01

if nargin==1
	[A,B,C,D,Tc]=ssdata(ss(A));
	A1=A'; B1=C'; C1=B'; D1=D';
	Ad=ss(A1,B1,C1,D1,Tc); Bd=[]; Cd=[]; Dd=[];
elseif nargin==4
	Ad=A'; Bd=C'; Cd=B'; Dd=D';
else
	disp(' *** illegal number of arguments in dualsys'), Ad=[]; Bd=[]; Cd=[]; Dd=[];
end
% --- last line of dualsys ---
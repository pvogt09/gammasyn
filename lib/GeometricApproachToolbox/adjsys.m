function [Ad,Bd,Cd,Dd] =adjsys(A,B,C,D,E)
%ADJSYS   Adjoint system.
%  [Ad,Bd,Cd,Dd] = ADJSYS(A,B,C,D[,1]) or sys1 = ADJSYS(sys) provides
%  the adjoint of (A,B,C,D).
%  Five-arguments call refer to discrete-time systems.
%  In the discrete-time case matrix`A must be invertible.

%  G.Marro 3-19-06

if nargin==4
	sys0=ss(A,B,C,D);
elseif nargin==5
	sys0=ss(A,B,C,D,-1);
elseif nargin==1
	sys0=A;
else
	disp(' *** illegal number of arguments in adjsys'), Ad=[]; Bd=[]; Cd=[]; Dd=[];
end
sys0a=dualsys(revsys(sys0));
if nargin==1
	Ad=sys0a; Bd=[]; Cd=[]; Dd=[];
else
	[Ad,Bd,Cd,Dd]=ssdata(sys0a);
end
% --- last line of adjsys ---
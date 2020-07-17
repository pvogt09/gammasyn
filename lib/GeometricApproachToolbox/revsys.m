function [Ar,Br,Cr,Dr] = revsys(A,B,C,D,E)
%REVSYS   Reverse-time system.
%  [Ar,Br,Cr,Dr] = REVSYS(A,B,C,D,[1]) or sys1 = REVSYS(sys)
%  or sysr = REVSYS(sys) provides the reverse-time of (A,B,C,D) or of sys.
%  Five-arguments call refer to discrete-time systems.
%  In the discrete-time case matrix A must be invertible.

%  G.Marro 8-20-01

if nargin==4
	Ts=0;
elseif nargin==5
	Ts=-1;
elseif nargin==1
	[A,B,C,D,Ts]=ssdata(A);
else
	disp(' *** illegal number of arguments in revsys'), Ar=[]; Br=[]; Cr=[]; Dr=[]; return
end
if Ts==0
	A1=-A; B1=-B; C1=C; D1=D;
else
	A1=inv(A); B1=-inv(A)*B; C1=C*inv(A); D1=D-C*inv(A)*B;
end
if nargin==1
	Ar=ss(A1,B1,C1,D1,Ts); Br=[]; Cr=[]; Dr=[];
else
	Ar=A1; Br=B1; Cr=C1; Dr=D1;
end
% --- last line of revsys ---
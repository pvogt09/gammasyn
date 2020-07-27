function [z,W] = gazero(A,B,C,D)
%GAZERO   Invariant zeros and invariant zero structure.
%  z=GAZERO(A,B,C[,D]) or z=GAZERO(sys) returns in a column vector the invariant zeros
%  of the LTI system sys=(A,B,C,D[,-1]]). In the two-output call [z,X]=GAZERO(A,B,C[,D])
%  X is a matrix representing the invariant zero stucture of the system.
%  The algorithm is based on the standard geometric approach definition
%  of invariant zeros. A different algorithm is used in the program TZERO
%  or ZERO of the Matlab Control System Toolbox.

%  G.Marro 8-20-01 (rev 2007)

if nargin==1, [A,B,C,D]=ssdata(A); end
tol=norm(A,'fro')*eps*10^6;
[ma,na] = size(A);
[mb,nb] = size(B);
[mc,nc] = size(C);
flg=1;
if (nargin==3)||(norm(D,'fro')<tol)
	flg=0;
	D=zeros(mc,nb);
end
error(abcdchk(A,B,C,D));
if flg==1
	A=[A zeros(na,mc); C zeros(mc)];
	B=[B; D];
	C=[zeros(mc,na) eye(mc)];
end
kC=ker(C);
Vstar=mainco(A,B,kC); nV=size(Vstar,2);
if ~any(any(Vstar)), z=[]; W=[]; return, end
Sstar=miinco(A,kC,B);
Vr=ints(Vstar,Sstar);
if ~any(any(Vr)), Vr=[]; end
nVr=size(Vr,2);
if nVr>0
	VV=ima([Vr,Vstar],0); V1=VV;
else
	V1=Vstar;
end
M=pinv([V1 -B])*A*V1;
W=M(nVr+1:nV,nVr+1:nV);
z=eig(W);
if isempty(z), z=[]; end

% --- last line of gazero ---
function r = rhomin(A,B,C,D)
%RHOMIN   Minimum response delay of a space-state system.
%  r = RHOMIN(A,B,C) or r = RHOMIN(A,B,C,D) or r = RHOMIN(sys)
%  computes the minimum delay of a state space system.

%  G. Marro 12-22-00

if nargin==1, [A,B,C,D]=ssdata(ss(A,'minimal')); end
[q,n]=size(C);
flg=1;
if (nargin==3)||(norm(D,'fro')<eps)
	flg=0;
	p=size(B,2); D=zeros(q,p);
end
if flg
	A=[A zeros(n,q); C zeros(q,q)];
	B=[B;D];
	C=[zeros(q,n) eye(q)];
	[q,n]=size(C);
end
kerC=ker(C);
% Computation of the minimum response delay
r=0;
Q=zeros(n,1);
stopcond=0;
while stopcond==0
	r=r+1;
	Q = ima([B A*Q],0);
	CS=C*Q; if norm(CS,'fro')~=0; stopcond=1; end
	if r==n, stopcond=1; end
end
% Adjustment for quadruples
if flg, r=r-1; end
% --- Last line of rhomin ---
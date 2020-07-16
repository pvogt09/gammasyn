function r = rhominv(A,B,C,D)
%RHOMINV  Vector minimum response delay of a space-state system.
%  r = RHOMINV(A,B,C) or r = RHOMINV(A,B,C,D) or r = RHOMINV(sys)
%  computes the vector minimum delay of state space system.

%  G. Marro 12-22-00

if nargin==1, [A,B,C,D]=ssdata(A); end
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
% Computation of the minimum delay
r=zeros(q,1);
for i=1:q
Q=zeros(n,1);
stopcond=0;
	while stopcond==0
		r(i)=r(i)+1;
		Q = ima([B A*Q],0);
		CS=C(i,:)*Q; if norm(CS,'fro')~=0; stopcond=1; end
		if r(i)==n, stopcond=1; end
	end
end
% Adjustment for quadruples
if flg, r=r-1; end
% --- Last line of rhominv ---
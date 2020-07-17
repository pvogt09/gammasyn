function r = reldeg(A,B,C,D,E)
%RELDEG   Relative degree.
%  r = RELDEG(A,B,C[,D]) or r = RELDEG(sys) computes the relative degree
%  of a right or left-invertible LTI system.

%  G. Marro 12-22-00


mess=' *** illegal number of arguments in reldeg';
if nargin==1
	[A,B,C,D]=ssdata(A);
elseif nargin==2
	disp(mess), r=[]; return
elseif nargin==3
	[q,n]=size(C); p=size(B,2); D=zeros(q,p);
elseif nargin==4
else
	disp(mess), r=[]; return
end

[q,n]=size(C); p=size(B,2);
if size(ima([B;D]',0),2)~=p
	fprintf(' *** warning: matrix [B;D] is not maximum rank in reldeg\n')
end
if size(ima([C,D],0),2)~=q
	fprintf(' *** warning: matrix [C,D] is not maximum rank in reldeg\n')
end

flg=1;
if (norm(D,'fro')<eps)
	flg=0;
	D=zeros(q,p);
end
if flg
	A=[A zeros(n,q); C zeros(q,q)];
	B=[B;D];
	C=[zeros(q,n) eye(q)];
	[q,n]=size(C);
end
%A1=A; B1=B; C1=C; D1=D;
kerC=ker(C);
%
% Computation of the relative degree
%
r=0;
Q=zeros(n,1);
VV=vstar(A,B,C); SS=sstar(A,B,C); W=sums(VV,SS); nW=size(W,2);
stopcond=0;
while stopcond==0
	r=r+1;
	Q = ima([B A*(ints(Q,kerC))],0);
	CS=sums(Q,VV); if (size(CS,2)==1)&&(~any(CS)), CS=[]; end
	if size(CS,2)==nW, stopcond=1; end
	if r>n, fprintf('   *** lack of convergence in reldeg\n'), stopcond=1; r=[]; end
end
if isempty(r), return, end
% Adjustment for quadruples
if flg, r=r-1; end

% --- Last line of reldeg ---
function r = reldegv(A,B,C,D)
%RELDEGV  Vector relative degree.
%  r = RELDEGV(A,B,C) or r = RELDEGV(A,B,C,D) or r = RELDEGV(sys)
%  computes the vector relative degree of a LTI right or left-invertible system.

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
imB=ima(B);
kerC=ker(C);
% Controls if the system is right invertible
Vstar=mainco(A,B,kerC);
Sstar=miinco(A,kerC,B);
W=sums(Vstar,Sstar);
if size(W,2)~=n
	fprintf(' **** the system isn''t right-invertible in reldegv:')
	fprintf(' the dual system will be used\n')
	[A,B,C,D]=dualsys(A,B,C,D); [q,n]=size(C);
end
kerC=ker(C);
Vstar=mainco(A,B,kerC);
Sstar=miinco(A,kerC,B);
W=sums(Vstar,Sstar);
if size(W,2)~=n
	fprintf(' **** the dual system isn''t right-invertible in reldegv:')
	fprintf(' computation is impossible\n'), r=[]; return
end
% Computation of the relative degree
tol=10^(-8);
r=zeros(q,1);
for i = 1:q
	E=[C(1:i-1,1:n);C(i+1:q,1:n)];
	E1=C(i,1:n);
	Vstar=mainco(A,B,ker(E));
	%Sstar=miinco(A,ker(E),B);
	degree=1;
	z=imB;
	stopcond=0; kk=0;
	while (stopcond==0)&&(kk<n)
		kk=kk+1;
		if any(abs(E1*ints(z,Vstar))>tol), r(i)=degree; stopcond=1; end
		degree=degree+1;
		z=sums(A*ints(z,Vstar),imB);
		if kk==n, fprintf('   *** lack of convergence in reldeg\n'), r=[]; break, end
	end
	if isempty(r), break, end
end
% Adjustment for quadruples
if flg, r=r-1; end

% --- Last line of reldegv ---
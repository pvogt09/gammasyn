function [ak,bk,ck,dk,n1,n2,n3,n4]=kalmcd(a,b,c,d,tol)
%KALMCD   Kalman canonical decomposition.
%  [ak,bk,ck,dk,n1,n2,n3,n4]=kalmcd(a,b,c,d) provides as (ak,bk,ck,dk) the
%  Kalman equivalent realization of (a,b,c,d); the other outputs are
%  n1 - state dimension of the controllable and unobservable subsystem;
%  n2 - state dimension of the controllable and observable subsystem;
%  n3 - state dimension of the uncontrollable and unobservable subsystem;
%  n4 - state dimension of the uncontrollable and observable subsystem;

if nargin==4, tol=norm(a,'fro')*eps*10^6; end
n=length(a);
R=mininv(a,b);
[mx,nr]=size(R);
if (nr==1)&&(norm(R,'fro')<tol), nr=0; end
Q=maxinv(a,ker(c));
[mx,nq]=size(Q);
if (nq==1)&&(norm(Q,'fro')<tol), nq=0; end
RQ=ints(R,Q);
[mx,nrq]=size(RQ);
if (nrq==1)&&(norm(RQ,'fro')<tol), nrq=0; end
n1=nrq;
n2=nr-nrq;
n3=nq-nrq;
n4=n-n1-n2-n3;
T1=ima([RQ,R],0);
T2=ima([RQ,Q],0);
if n1==0
	T=[T1(:,1:nr),T2(:,1:nq)];
else
	T=[RQ,T1(:,nrq+1:nr),T2(:,nrq+1:nq)];
end
if (length(T)==0)||(norm(T,'fro')<tol)
	T=eye(n);
else
	Tc=ortco(T);
	if ~(norm(Tc,'fro')<tol), T=[T,Tc]; end
	[mt,nt]=size(T);
	mess=' **** ill-conditioned transform to the kalman form';
	if mt~=nt, disp(mess), return, end
	if abs(det(T))<eps, disp(mess), return, end
end
ak=T\a*T;
bk=T\b;
ck=c*T;
dk=d;
%
% puts true zeros in the computed matrices
%
for k=1:n
	ii=find(abs(ak(k,:))<tol); ll=length(ii);
	if ll~=0, ak(k,ii)=zeros(1,ll); end
end
for k=1:n
	ii=find(abs(bk(k,:))<tol); ll=length(ii);
	if ll~=0, bk(k,ii)=zeros(1,ll); end
end
for k=1:n
	ii=find(abs(ck(:,k))<tol); ll=length(ii);
	if ll~=0, ck(ii,k)=zeros(ll,1); end
end
% ----- last line of kalmcd -----
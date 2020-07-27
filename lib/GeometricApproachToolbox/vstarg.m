function [V,F]=vstarg(A,B,C,D,disc)
%VSTARG   Maximum internally stabilizable output nulling controlled invariant.
%    [V,F]=vstarg(A,B,C,D[,1]) where the five arguments call refers to discrete time.
%    [V,F]=vstarg(sys) where sys is an LTI system.

if nargin==1, [A,B,C,D,disc]=ssdata(A);
elseif nargin==3, nb=size(B,2); mc=size(C,1); D=zeros(mc,nb); disc=0;
elseif nargin==4, disc=0;
elseif nargin==5, disc=1;
end
if disc~=0, disc=1; end
tol=norm(A,'fro')*eps*10^6;
[ma,na] = size(A);
[mb,nb] = size(B);
[mc,nc] = size(C);
flg=1;
if (norm(D,'fro')<tol)
	flg=0;
	D=zeros(mc,nb);
end
error(abcdchk(A,B,C,D));
if flg==1
	A=[A zeros(na,mc); C zeros(mc)];
	B=[B; D];
	C=[zeros(mc,na) eye(mc)];
end
D=zeros(mc,nb);
Vstar=vstar(A,B,C,D);
if ~any(any(Vstar)), V=zeros(na,1); F=zeros(1,na); return, end
Sstar=sstar(A,B,C,D);
Vr=ints(Vstar,Sstar);
if ~any(any(Vr)), Vr=[]; end
nVr=size(Vr,2);
if nVr>0
	V1=ima([Vr,Vstar],0);
else
	V1=Vstar;
end
nV=size(V1,2);
M=pinv([V1 B])*A*V1;
U=M(nV+1:nV+nb,:);
W=M(nVr+1:nV,nVr+1:nV);
if disc==0, Ws=subsplit(W); else, Ws=subsplit(W,1); end
nws=size(Ws,2); if (isempty(Ws))||(~any(Ws)), nws=0; end
T=ima([Ws,eye(size(Ws,1))],0);
%Wst=inv(T)*Ws*T;
T1=eye(nV);
T1(nVr+1:nV,nVr+1:nV)=T;
V1=V1*T1;
V=V1(:,1:nVr+nws);
if isempty(V), V=zeros(size(Vstar,1),1); end
if nargout==2, F=effesta(A,B,V); end
if flg
	V=V(1:na,:);
	if nargout==2, F=F(:,1:na); end
end

% --- last line of vstarg ---
function [F,T,nnt,U,nnu,Ap]=effest(A,B,V,Pv,Pe)
%EFFEST   Stabilizing state feedback matrix for a controlled invariant.
%  F=effest(A,B,V,Pv,Pe))
%  Given V, (A,imB)-controlled invariant, an F is derived such that:
%  -> V is (A+B*F)-invariant
%  -> (A+B*F) on the reachable subspace of V has the eigenvalues of Pv
%     where Pv is a given column vector with at least dim(Rv) components
%  -> (A+B*F) external to V has the eigenvalues of Pe
%     where Pe is a given column vector with at least n-dim(Rv) components
%
%  NOTE: If Pv=[] or Pe=[] the eigenvalues internal and/or external
%	 with respect to V are not changed
% The call [F,T,nnt,U,nnu]=effest(A,B,V,Pv,Pe)) is also possible, where the
% optional outputs T,nnt,U,nnu provide information on the changes of bases.

if nargin~=5, error('GEOAPPR:input', ' **** call error in effest'), end
tol=norm(A,'fro')*eps*10^4;
n=length(A);					    % n=dim(A)
nb=size(B,2);					    % nb=dim(B)
nv=size(V,2);
if ~any(V), nv=0; end
VV=mainco(A,B,V);
ny=size(VV,2);
if ~any(VV), ny=0; end
if ny~=nv, error('GEOAPPR:input', 'V is not a controlled invariant in EFFEST'), end
if norm(V,'fro')<tol, nv=0; end                     % nv=dim(V)
ny=size(ima(B,0),2);
if ny~=nb, error('GEOAPPR:input', 'B is not full rank in EFFEST'), end
S=miinco(A,V,B);
%
% Construction of matrix T (change of basis in the state space)
%
% 1 - Rv
%
T1=ints(V,S); if (size(T1,2)==1)&&(~any(T1)), T1=[]; end
n1=size(T1,2);					    % n1=dim(T1)
%
% 2 - V
%
T1T2=ima([T1,V],0);  if (size(T1T2,2)==1)&&(~any(T1T2)), T1T2=[]; end
n2=size(T1T2,2);				    % n2=dim(T1T2)
%
% 3 - S
%
Sb=ima([T1,S],0); if (size(Sb,2)==1)&&(~any(Sb)), Sb=[]; end
nsb=size(Sb,2);
T3=Sb(:,n1+1:nsb);
T1T2T3=[T1T2,T3]; if (size(T1T2T3,2)==1)&&(~any(T1T2T3)), T1T2T3=[]; end
n3=size(T1T2T3,2);				    % n3=dim(T1T2T3)
%
% 4 - compl
%
if n3<n
	T=[T1T2T3,ortco(T1T2T3)];
else
	T=T1T2T3;
end
%
% Information on partitions of T in the output vector nnt
%
nnt=zeros(1,4); n4=n;
for kk=1:4
	eval(['nnt(kk)=n',int2str(kk),';'])
end
%
% Construction of matrix U (change of basis in the input space)
%
if isempty(T1), T1=zeros(n,1); end
U1=invt(B,T1);
U2=ortco(U1);
if (size(U1,2)==1)&&(~any(U1)), U1=[]; end
nu1=size(U1,2);
if (size(U2,2)==1)&&(~any(U2)), U2=[]; end
U=[U1 U2];
nnu=[nu1 nb];
%
Ap=T\A*T;
Bp=T\B*U;
Fp=zeros(nb,n);
%return
%
%  invariance of A+BF
%
Fp(nu1+1:nb,1:n2)=-pinv(Bp(n2+1:n3,nu1+1:nb))*Ap(n2+1:n3,1:n2);
%
%  internal pole placement
%
if ~isempty(Pv)
	flg=0;
	if length(Pv)<n1
		disp(' **** lack of elements in Pv'), flg=1;
	end
	if (n1>0)&&(flg==0)
		Pv1=Pv(1:n1);
		if ~isempty(Fp(nu1+1:nb,1:n1))
			Fp(1:nu1,1:n1)=-place(Ap(1:n1,1:n1)+...
			Bp(1:n1,nu1+1:nb)*Fp(nu1+1:nb,1:n1),Bp(1:n1,1:nu1),Pv1);
		else
			Fp(1:nu1,1:n1)=-place(Ap(1:n1,1:n1),Bp(1:n1,1:nu1),Pv1);
		end
	end
end
%
%return
%Pe=[]
%
%  external pole placement
%
if ~isempty(Pe)
	flg=0;
	Rext=mininv(Ap(n2+1:n4,n2+1:n4),Bp(n2+1:n4,nu1+1:nb));
	if size(Rext,2)~=length(Ap(n2+1:n4,n2+1:n4))
		%  disp(' **** V is not externally stabilizable'), flg=1;
			disp(' **** Not all the external eigenvalues of V are assignable'), flg=1;
	end
	if length(Pe)<n4-n2
		disp(' **** lack of elements in Pe'), flg=1;
	end
	if (n4-n2>0)&&(flg==0)
		Pe1=Pe(1:n4-n2);
		Fp(nu1+1:nb,n2+1:n4)=-place(Ap(n2+1:n4,n2+1:n4),Bp(n2+1:n4,nu1+1:nb),Pe1);
	end
end
Ap3=Ap+Bp*Fp; Ap=Ap3;
F=U*Fp/T;
%F=cleanma(F);
% --- last line of effest ---
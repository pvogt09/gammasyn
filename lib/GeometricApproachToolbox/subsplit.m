function [Xs,Xu,X0]=subsplit(A,opt)
%SUBSPLIT Invariant subspaces of stable/unstable/boundary modes of a matrix.
%  [Xs,Xu,X0]=subsplit(A) gives, for a continuous-time system:
%     Xs: a basis matrix for the subspace of strictly stable modes of A
%     Xu: a basis matrix for the subspace of strictly unstable modes of A
%     X0: a basis matrix for the subspace of modes with zero real part.
%  [Xs,Xu,X0]=subsplit(A,1) gives the same in the discrete-time case.
%  [Xs,Xu,X0]=subsplit(A,2+nz) in the discrete-time case moves some zero modes to X0.

%  G. Marro 3-20-99

if (nargin<1)||(nargin>3)
	disp('   illegal number of arguments in subsplit')
end
Xs=[]; Xu=[]; X0=[];
if isempty(A), return, end
if nargin==1, flg=0; else, flg=1; end
nr=length(A);
% set tolerance for splitting subspaces
tol=norm(A,'fro')*eps*10^12;
%  computing the schur form
[T1,A1]=schur(A);
[T1,A1]=rsf2csf(T1,A1);
d1=diag(A1);
if flg==0
	rd1=real(d1);
else
	if opt>=2
		nzt=opt-2;
		[xxx,ij]=sort(abs(d1)); ij=ij(1:nr-nzt); d1(ij)=1;
	end
	rd1=abs(d1)-1;
end
%  reordering the schur form for stable modes
ii=sign(rd1+tol*ones(nr,1)); in=length(find(ii<0));
[T,Temp]=schord(T1,A1,ii);
if in==0
	Xs=zeros(nr,1);
else
	Ts=T(:,1:in); Xs=-ima([real(Ts)]);
end
%  reordering the schur form for unstable modes
ii=sign(-rd1+tol*ones(nr,1)); in=length(find(ii<0));
[T,Temp]=schord(T1,A1,ii);
if in==0
	Xu=zeros(nr,1);
else
	Tu=T(:,1:in); Xu=-ima([real(Tu)]);
end
%  reordering the schur form for zero variety modes
ii=sign(abs(rd1)-tol*ones(nr,1)); in=length(find(ii<0));
[T,Temp]=schord(T1,A1,ii);
if in==0
	X0=zeros(nr,1);
else
	T0=T(:,1:in); X0=-ima([real(T0)]);
end
% --- last line of subsplit
function [out1,out2,VV1,z]=stoor(A,B,C,D,disc)
%STOOR    Provides output matrices [C1,D1] for the Stoorvogel equivalence.
%   [C1,D1]=stoor(A,B,C,D) or sys1=stoor(sys) (with sys1=ss(A,B,C1,D1))
%   computes C1 and D1 such that the exact disturbance decoupling problem
%   for (A,B,C1,D1) is equivalent to the minimum H2 norm decoupling
%   for the original system (A,B,C,D). Left invertibility is requested.
%   It also works in the discrete-time case with the calls [C1,D1]=stoor(A,B,C,D,1)
%   or sys1=stoor(sys) with sys being defined as a discrete-time LTI system.
%   Left invertibility is not required.

%   G. Marro, 12-22-04

out1=[]; out2=[]; out3=[]; out4=[]; flg=0; z=[];

mm='The number of input arguments is not acceptable in stoor';
if nargin==4, disc=0; elseif nargin==5, disc=-1; end
if (nargin==2)||(nargin==3), disp(mm), return, end
if nargin==1
	sysv=A;
	[A,B,C,D,disc]=ssdata(sysv);
end

na=size(A,1); nb=size(B,2); mc=size(C,1); V=vstar(A,B,C,D); S=sstar(A,B,C,D);

rvs=ints(V,S);

if any(rvs)==1
	disp(' '), disp('   Non-left invertible system in stoor')
end

xvs=sums(V,S);

if size(xvs,2)==na % for right-invertible systems

	z=gazero(A,B,C,D); if isempty(z), flg=1; end

	if disc==0
		if all(real(z)<0), flg=1; end
	else
		if all(abs(z)<1), flg=1; end
	end

	if flg
		disp(' '), disp('   The system is already minimum-phase in stoor')
		if nargin==1
			out1=sysv; return
		else
			out1=C; out2=D; return
		end
	end

else
		disp(' '), disp('   Non-right invertible system in stoor')
end

flgd=1;
if norm(D,'fro') < 100*eps
	flgd=0;
	D=zeros(mc,nb);
end

if disc==0

	p=eig(A); flll=0;
	if any(abs(p)<10^(-8)*ones(na,1)), flll=1; end
	if flll, disp(' '), disp('   Stabilizing matrix A in stoor'), end
	if flll, Aold1=A; A=A-B*randn(nb,na); end

	if flgd
		Aold=A; Bold=B; Cold=C; Dold=D;
		MM=-eye(nb);
		A=[A B; zeros(nb,na),MM];
		B=[zeros(na,nb);eye(nb)];
		C=[C D]; D=zeros(mc,nb);
	end

	SS=sstar(A,B,C,D);
	%[T1,T2,T3,VV1,z]=singcare(A,B,C,D);
	VV1=vstargh2(A,B,C,D);
	VVr=sums(VV1,ints(SS,ker(C)));
	C1=ortco(VVr)';
	G=-C/(A)*B; G1=-C1/(A)*B;
	M1=G*pinv(G1)*C1;
	if mc>nb, M1=fattmat(M1'*M1); end % for a square spectral factor
	%M1=(C*SS)*pinv(C1*SS)*C1;

else

	p=eig(A); flll=0;
	p=eig(eye(na)-A); flll=0;
	if any(abs(p)<10^(-8)*ones(na,1)), flll=1; end
	if flll, disp(' '), disp('   Eliminating a unit eigenvalue in stoor'), end
	if flll, Aold1=A; A=A-B*randn(nb,na); end

	if flgd
	Aold=A; Bold=B; Cold=C; Dold=D;
	MM=0.5*eye(nb);
	A=[A B; zeros(nb,na),MM];
	B=[zeros(na,nb);eye(nb)];
	C=[C D]; D=zeros(mc,nb);
	end
	SS=sstar(A,B,C,D);
	VV1=vstargh2(A,B,C,D,1);
	VVr=sums(VV1,ints(SS,ker(C)));
	C1=ortco(VVr)';
	G=C/(eye(na)-A)*B; G1=C1/(eye(na)-A)*B;
	M1=G*pinv(G1)*C1;
	if mc>nb, M1=fattmat(M1'*M1); end % for a square spectral factor
	%M1=(C*SS)*pinv(C1*SS)*C1;
end

if flgd
	A=Aold; B=Bold; C=Cold; D=Dold;
	na=size(A,1); nb=size(B,2); mc=size(C,1);
	C1=M1(:,1:na); D1=M1(:,na+1:na+nb);
else
	C1=M1; D1=zeros(size(C1,1),nb);
end
if flll, A=Aold1; end

if nargin==1
	out1=ss(A,B,C1,D1,disc); out2=z;
else
	out1=C1; out2=D1;
end

% --- end of stoor ---
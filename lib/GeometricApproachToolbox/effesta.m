function F=effesta(A,B,V,ff)
%EFFESTA  Assigns all the assignable eigenvalues of a controlled invariant.
%  F=effesta(A,B,V), with im(V) being an (A,B)-controlled invariant, computes
%  a state feedback matrix F that makes im(V) to be an (A+BF)-invariant and
%  assigns the internal assignable eigenvalues of (A+BF), i.e. eig[(A+BF)/Rv],
%  that are requested ininteractive mode.
%  F=effesta(A,B,V,1) also assigns the external eigenvalues of im(V), i.e.
%  eig[(A+BF)|V+R], where R denotes the reachable subspace of (A,B).

% Checks and messages
if nargin==3, ff=0; else, ff=1; end
na=length(A); nb=size(B,2);
[my,ny]=size(ima(B,1));
if ny~=nb
	warning('GEOAPPR:input', 'B is not full rank')
end
nv=size(V,2);
if ~any(V), nv=0; end
if (~any(V)||isempty(V)), F=zeros(nb,na); return, end
VV=mainco(A,B,V);
ny=size(VV,2);
if ~any(VV), ny=0; end
if ny~=nv, warning('GEOAPPR:input', 'V is not a controlled invariant in EFFESTA'), end
%
XU=pinv([V B])*A*V;
KK=ker([V B]);
if ~any(KK), KK=[]; end
W=XU(1:nv,:);
if isempty(KK)
	L=XU(nv+1:nv+nb,:);
	F1=-L*pinv(V);
	Rv=[];
else
	Rv=ints(V,miinco(A,V,B));
	ncol=size(Rv,2);
	V1=ima([Rv V],0);
	XU=pinv([V1 B])*A*V1;
	KK=ker([V1 B]);
	AA=XU(1:ncol,1:ncol);
	BB=KK(1:ncol,:);
	disp(' ')
	disp(' **** the model is not left-invertible:')
	disp(' ')
	fprintf(' you have to define %2i',ncol)
	fprintf(' internal eigenvalue(s)\n')
	kk=0; pa=[];
	while kk<ncol
		pp=input(' enter a value or a vector : ');
		if ~isempty(pp)
			[mp,np]=size(pp);
			if np>mp, pp=pp'; end
			pa=[pa;pp]; kk=length(pa);
			if kk>ncol, pa=pa(1:ncol); end
		end
	end
	FF=-place(AA,BB,pa);
	XU(:,1:ncol)=XU(:,1:ncol)+KK*FF;
	W=XU(1:nv,:);
	L=XU(nv+1:nv+nb,:);
	F1=-L*pinv(V1);
end
F=F1;
%
if ff
	R=mininv(A,B);
	T1=V;
	c1=size(T1,2);
	c=size(ima([T1,R],0),2);
	ccc=ima([T1,R],0);
	if c>=c1+1
		T2=ccc(:,c1+1:c); c2=size(T2,2);
	else
		T2=[]; c2=0;
	end
	if c<na
		T3=ortco([T1 T2]); if ~any(T3), T3=[]; end
		c3=size(T3,2);
		T=[T1 T2 T3];
		disp(' **** The model is not externally stabilizable')
	else
		c3=0;
		T=[T1 T2];
	end
	if c2==0
		disp(' no external eigenvalue can be assigned'), disp(' ')
		F=F1;
	else
		disp(' ')
		fprintf(' you can define %2i',c2)
		fprintf(' external eigenvalue(s)\n')
		%pro=input(' do you want to proceed ? (1) : ');
		%if isempty(pro), pro=0; end
		%if pro~=1, F=F1; return, end
		Ap=T\(A+B*F1)*T;
		Bp=T\B;
		Fp=F1*T;
		pb=[];
		k=0;
		while k<c2
			ppp=input(' enter a value or a vector : ');
			[mp,np]=size(ppp);
			if np>mp, ppp=ppp'; end
			pb=[pb;ppp];
			k=k+length(ppp);
			k=length(pb);
			if k>c2, pb=pb(1:c2); end
		end
		As=Ap(c1+1:c1+c2,c1+1:c1+c2);
		Bs=Bp(c1+1:c1+c2,:);
		F2=-place(As,Bs,pb);
		Fp(:,c1+1:c1+c2)=F2;
		F=Fp/T;
	end
end
% --- last line of effesta ----
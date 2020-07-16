function [V,F,X]=vstargh2(A,B,C,D,fl)
%VSTARGH2 Maximum internally stabilizable H2-minimizing controlled invariant.
%    [V,F]=vstargh2(A,B,C,D), referring to a continuous-time quadruple.
%    [V,F]=vstargh2(A,B,C,D,1), referring to a discrete-time quadruple.
%    [V,F]=vstargh2(sys) referring to an LTI system (continuous or discrete).
%    The quadruple (A,B,C,D) can be non-left-invertible.

%   G. Marro, 11-24-08

if nargin==1, [A,B,C,D,disc]=ssdata(A); end
if nargin==5, disc=1; elseif nargin==4, disc=0; end

na=length(A);

Q=C.'*C; S=C.'*D; R=D.'*D;

if ~disc

	AA=[A,zeros(na,na);-Q,-A'];
	BB=[B;-S];
	CC=[S',B'];
	DD=R;

	[VV,FF]=vstarg(AA,BB,CC,DD); nas=size(VV,2);

	V1=VV(1:na,:); X1=VV(na+1:2*na,:);

	nV=nas; if nV==na, MM=inv(V1(1:na,:)); else, MM=eye(nV); end
	V=V1*MM; X1=X1*MM; X=V'*X1; X=(X'+X)/2;
	F1=FF(:,1:na); F2=FF(:,na+1:2*na); F=(F1+F2*X1*pinv(V));

else

	RV=rvstar(A,B,C,D);
	if any(RV)
		[Am,Bm,Cm,Dm,Fs,Us]=extendf(A,B,C,D);
		Q=Cm.'*Cm; S=Cm.'*Dm; R=Dm.'*Dm;
		[X1,L,Km]=dare(Am,Bm,Q,R,S); Fm=-Km;
		AFm=Am-Bm*Km;
		z=gazero(Am,Bm,Cm,Dm); nz=length(z);
		V=subsplit(AFm,2+nz);
		F=Us*Fm+Fs;
		X=V'*X1*V;
	else
		[X1,L,K]=dare(A,B,Q,R,S); F=-K;
		AF=A-B*K;
		z=gazero(A,B,C,D); nz=length(z);
		V=subsplit(AF,2+nz);
		% X1=dlyap(AF',(C+D*F)'*(C+D*F));
		X=V'*X1*V;
	end

end

% --- end of vstargh2 ---
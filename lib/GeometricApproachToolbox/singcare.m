function [X,L,K,V,z] = singcare(A,B,C,D)
%SINGCARE Regular or singular or cheap continuous-time LQR problem.
%         [X,L,K,V] = singcare(A,B,C,D) or [X,L,K,V] = singcare(sys) gives:
%         V a basis matrix for the subspace of the admissible initial states x(0) = V * alpha_0
%         X the matrix of cost : c_[0,inf] = alpha_0'*X*alpha_0
%         L the eigenvalues corresponding to the optimal modes
%         K the state feedback matrix : dot alpha(t) = pinv(V)*(A-BK)*V*alpha(t)
%         The pair A,B is assumed to be stabilizable.

%        G.Marro 25-10-05

if nargin==1, [A,B,C,D,T]=ssdata(A);
if T~=0, error('GEOAPPR:input', 'the system in singcare must be continuous-time'), end, end
error(abcdchk(A,B,C,D));
tol=norm(A,'fro')*eps*10^6;
na=length(A); nb=size(B,2); mc=size(C,1);

Q=C.'*C; S=C.'*D; R=D.'*D;

AA=[A,zeros(na,na);-Q,-A'];
BB=[B;-S];
CC=[S',B'];
DD=R;

[VV1,FF]=vstar(AA,BB,CC,DD); AAF=AA+BB*FF;
AAF=AA+BB*FF;
T=ima([VV1,eye(size(VV1,1))],0);
AAF_T=T\AAF*T;
cVV1=size(VV1,2); if ~any(VV1), cVV1=0; end
AAA=AAF_T(1:cVV1,1:cVV1); z=eig(AAA);

As=subsplit(AAA); nas=size(As,2); if ~any(As), nas=0; end
if nas~=0
	T1=ima([As,eye(size(As,1))],0);
	Tas=T1\AAA*T1;
	MM1=Tas(1:nas,1:nas);
	VV1d=T1*[MM1;zeros(size(AAA,1)-nas,nas)];
	VV1d=T*[VV1d;zeros(size(AA,1)-size(VV1d,1),size(VV1d,2))];
	V1=VV1d(1:na,:); X1=VV1d(na+1:2*na,:);
else
	V=zeros(na,1); X=0; L=[]; K=zeros(1,na); return
end

nV=nas; if nV==na, MM=inv(V1(1:na,:)); else, MM=eye(nV); end
V=V1*MM; X1=X1*MM; L=eig(MM1); X=V'*X1; X=(X'+X)/2;
F1=FF(:,1:na); F2=FF(:,na+1:2*na); K=-(F1+F2*X1*pinv(V));

% --- end of singcare ---
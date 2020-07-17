function [A1,B11,B12,C1,D11,D12] = redobs(A,B,C,D)
%REDOBS   Reduced-order observer.
%
%                                +---Do1--+
%            +------------------>|Bo1     |
%            |    +--D--+        | (Ao) Co|-----> xo
%            |    | (A) |   +--->|Bo2     |
%       u ------> |B   C|---+    +---Do2--+
%                 |     |
%                 +-----+           SYSo
%                   SYS
%
%   SYSo = REDOBS(SYS) provides a reduced-order observer of SYS.
%   In SYSo the input matrices are together as [Bo1 Bo2] and [Do1 Do2]
%   [Ao,Bo1,Bo2,Co,Do1,Do2]=REDOBS(A,B,C,D) is another possible call.

%  G.Marro 2-15-07

if (nargin==1)
	[A,B,C,D,Tc]=ssdata(A);
end
%
na=length(A); nb=size(B,2); mc=size(C,1);
%
% similarity transformation and matrix partitioning
%
Cp=pinv(C);
T=ima([Cp,eye(na)],0); T(:,1:mc)=Cp;
A1=T\A*T; B1=T\B; C1=C*T;
%
A1_11=A1(1:mc,1:mc); A1_12=A1(1:mc,mc+1:na);
A1_21=A1(mc+1:na,1:mc); A1_22=A1(mc+1:na,mc+1:na);
B1_1=B1(1:mc,:); B1_2=B1(mc+1:na,:);
T1=T(:,1:mc); T2=T(:,mc+1:na);
%
% pole assignment
%
fprintf(' you have to define %2i',na-mc)
fprintf(' stable eigenvalue(s)\n')
kk=0;
while kk<na-mc
	pp=input(' enter a column vector : ');
	if ~isempty(pp)
		kk=length(pp);
	end
end
pp=pp(1:na-mc);
K2=(place(A1_22',A1_12',pp))';
%
% computing the observer matrices
%
L=B1_2-K2*B1_1;
M=(A1_22-K2*A1_12)*K2+A1_21-K2*A1_11;
N=A1_22-K2*A1_12;
%
A1=N;
B11=L-M*D;
B12=M;
C1=T2;
D11=-(T1+T2*K2)*D;
D12=T1+T2*K2;
%
if (nargout==1)
	A1=ss(A1,[B11,B12],C1,[D11,D12],Tc); B11=[]; B12=[]; C1=[]; D11=[]; D12=[];
	return
end
%
% --- last line of redobs ---
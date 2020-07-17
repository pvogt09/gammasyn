function [A1,B11,B12,C1,D11,D12] = fullobs(A,B,C,D)
%FULLOBS  Full-order observer.
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
%   SYSo = FULLOBS(SYS) provides a full-order observer of SYS.
%   In SYSo the input matrices are together as [Bo1 Bo2] and [Do1 Do2]
%   [Ao,Bo1,Bo2,Co,Do1,Do2]=FULLOBS(A,B,C,D) is another possible call.

%  G.Marro 2-15-07

if (nargin==1)
	[A,B,C,D,Tc]=ssdata(A);
end
%
na=length(A); nb=size(B,2); mc=size(C,1);
%
% pole assignment
%
fprintf(' you have to define %2i',na)
fprintf(' stable eigenvalue(s)\n')
kk=0;
while kk<na
	pp=input(' enter a column vector : ');
	if ~isempty(pp)
		kk=length(pp);
	end
end
pp=pp(1:na);
KK=(place(A',C',pp))';
%
A1=A-KK*C;
B11=B-KK*D;
B12=KK;
C1=eye(na);
D11=zeros(na,nb);
D12=zeros(na,mc);
%
if (nargout==1)
	A1=ss(A1,[B11,B12],C1,[D11,D12],Tc); B11=[]; B12=[]; C1=[]; D11=[]; D12=[];
	return
end
%
% --- last line of fullobs ---
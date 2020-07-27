function [V,F,ni] = vstar(A,B,C,D)
%VSTAR    Maximum output-nulling controlled invariant.
%  [V,F] = VSTAR(A,B,C) or [V,F] = VSTAR(A,B,C,D) or [V,F] = VSTAR(sys)
%  provides the maximum output-nulling (A,imB)-controlled invariant,
%  i.e. the maximum subspace imV such that (A+BF)imV is contained in imV
%  and imV is contained in ker(C+DF), and a friend F, namely a matrix F
%  such that (A+BF)imV is contained in imV.

%  G.Marro 8-20-01

if nargin==1, [A,B,C,D]=ssdata(A); end
if (isempty(A))|(isempty(B))|(isempty(C)), V=[]; F=[]; ni=0; return, end
tol=norm(A,'fro')*10^(-8); F=[];
na=length(A); [mb,nb] = size(B); [mc,nc] = size(C);
if (nargin==3), D=zeros(mc,nb); end
flgd=1; if (norm(D,'fro')<tol), flgd=0; end

if flgd
	na1=length(A);
	A=[A zeros(na,mc); C zeros(mc,mc)];
	B=[B;D];
	C=[zeros(mc,na) eye(mc)];
end

[V,ni]=mainco(A,B,ortco(C'));
if nargout>=2, F=effe(A,B,V); end

if flgd
	V=V(1:na1,:); if length(ni)>1, ni=ni(2:length(ni)); end
	if nargout>=2, F=F(:,1:na1); end
end

% --- last line of vstar ---
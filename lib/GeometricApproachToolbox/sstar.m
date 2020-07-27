function [Q,G,ni] = sstar(A,B,C,D)
%SSTAR    Minimum input-containing conditioned invariant.
%  [S,G] = SSTAR(A,B,C) or [S,G] = SSTAR(A,B,C,D) or [S,G] = SSTAR(sys)
%  provides the maximum input containing (A,kerC)-conditioned invariant,
%  i.e. the minimum subspace imS such that imS is contained in (A+GC)imS
%  and im(B+GD)is  contained in imS, and a friend G, namely a matrix G
%  such that (A+GC)imS is contained in imS.


%  G.Marro 8-20-01

if nargin==1, [A,B,C,D]=ssdata(A); end
tol=norm(A,'fro')*eps*10^6;
[mb,nb] = size(B); [mc,nc] = size(C);
if (nargin==3), D=zeros(mc,nb); end

G=[];
if nargout>=2
	[Q,G,nv]=vstar(A',C',B',D'); G=G'; ni=mb-nv;
else
	Q=vstar(A',C',B',D');
end
Q=ortco(Q);

% --- last line of sstar ---
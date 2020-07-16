function R = rvstar(A,B,C,D)
%RVSTAR   Maximum output-nulling reachable subspace.
%  R= RVSTAR(A,B,C) or R = RVSTAR(A,B,C,D) or R = RVSTAR(sys)
%  provides the max output nulling reachable subspace of (A,B,C,D) or of sys.

%  G.Marro 8-20-01

if nargin==1
	R=ints(vstar(A),sstar(A));
	return
end
[mb,nb] = size(B); [mc,nc] = size(C);
tol=norm(A,'fro')*eps*10^6;
if (nargin==3)||(norm(D,'fro')<tol)
	D=zeros(mc,nb);
end
R = ints(vstar(A,B,C,D),sstar(A,B,C,D));
% --- last line of rvstar ---
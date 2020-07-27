function [P,Q] = stabi(A,X)
%STABI    Internal and external stability of an invariant.
%  [P,Q] = stabi(A,X) provides as P and Q the critical matrices for the
%  internal and external stability of the A-invariant imX.

%  Basile and Marro 4-20-90

% Checks and messages
tol=norm(A,'fro')*eps*10^6;
[mx,nx] = size(X);
no = norm(X,'fro');
if (nx == 1) && (no < tol)
	nx = 0;
end
T=maxinv(A,X);
[my,ny] = size(T);
no = norm(T,'fro');
if (ny == 1)&&(no < tol)
	ny = 0;
end
if ny ~= nx
	warning('GEOAPPR:notinvariant', 'X is not an A-invariant in STABI')
end
%
X=T;
[ma,na] = size(A);
T = ima(X,1);
[m,n] = size(T);
no = norm(T,'fro');
if (n == 1)&&(no < tol)
	n = 0;
	T = eye(m);
else
	T = ima([T ortco(T)],0);
end
A1 = T\A*T;
P = A1(1:n,1:n); % P = cleanma(P);
n1 = n+1;
Q = A1(n1:na,n1:na); % Q = cleanma(Q);
% --- last line of stabi ---
function [P,Q] = stabv(A,B,X,E)
%STABV    Internal and external stabilizability of a controlled invariant.
%  [P,Q] = stabv(A,B,X) provides as P and Q the critical matrices for the
%  internal and external stabilizability of (A,B)-controlled invariant imX.

%        Basile and Marro 4-20-90

tol = norm(A,'fro')*eps*10^6;
[ma,na] = size(A);
if nargin==3
	% Check and message
	[mx,nx] = size(X);
	T=mainco(A,B,X);
	[my,ny] = size(T);
	if (ny ~= nx)
		error('GEOAPPR:input', 'X is not a controlled invariant in STABV')
	end
	P = miinco(A,X,B); X = T;
else
	P = miinco(A,E,B);
end
R = ints(X,P);
[m1,n1] = size(R);
no = norm(R,'fro');
if (n1 == 1)&&(no < tol)
	n1 = 0;
end
if n1 == 0
	T = ima(X,0);
	Q = P;
else
	T = ima([R X],0);
	Q = ima([R P],0);
end
[m2,n2] = size(T);
[mx,nx] = size(Q);
Q = Q(:,(n1+1):nx);
no = norm(T,'fro');
if (n2 == 1)&&(no < tol)
	n2 = 0;
end
if n2 == 0
	T = ima([P ortco(P)],0);
else
	T = ima([T ortco(T)],0);
	T(:,(n2+1):(n2+nx-n1)) = Q;
	if (n2+nx-n1) < na
		T=[T(:,1:n2+nx-n1),ortco(T(:,1:n2+nx-n1))];
	end
end
[m3,n3] = size(T);
if (n3 == 1)&&(no < tol)
	n3 = 0;
	T = eye(m3);
end
A1 = T\A*T;
ni = n1+1;
P = A1(ni:n2,ni:n2); % P = cleanma(P);
[T Q] = stabi(A,sums(X,mininv(A,B))); % Q = cleanma(Q);
% --- last line of stabv ---
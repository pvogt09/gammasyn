function [s,b] = kalmantest(A,B,C)
% Test the controllability and observability of a system.
%
% OUTPUT:
% s - controllability index (0=not controllable,1=controllable)
% b - observability index  (0=not observable,1=observable)
%
% INPUT:
% A 		- system matrix
% B 		- input matrix
% C 		- output matrix

n = size(A,1);
p = size(B,2);
q = size(C,1);
Qs = zeros(n,n*p);
Qb = zeros(n*q,n);

bs = B;
cs = C;
for i=1:n
	Qs(:,(i-1)*p+1:i*p) = bs;
	Qb((i-1)*q+1:i*q,:) = cs;
	if i<n
		bs = A*bs;
		cs = cs*A;
	end
end

if rank(Qs) == n
	s = 1;	% System ist steuerbar
else
	s = 0;	% System ist nicht steuerbar
end
if rank(Qb) == n
	b = 1;	% System ist beobachtbar
else
	b = 0;	% System ist nicht beobachtbar
end
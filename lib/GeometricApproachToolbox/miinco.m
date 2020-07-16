function [Q,ni] = miinco(A,C,X)
%MIINCO   Minimum (A,imC)-conditioned invariant containing imX.
%  Q = miinco(A,C,X) is an orthonormal basis for the minimum
%  (A,imC)-conditioned invariant containing imX.
%  [Q,ni] = miinco(A,C,X) gives in the row vector ni the dimensions of
%  the subsequent subspaces computed by the recursive algorithm.
%  The routine implements Algorithm 4.1-1 of "Controlled and Conditioned
%  Invariants in Linear System Theory"

%  Basile and Marro 4-20-90

nv = 0; ni = [];
X1 = ima(X);
Q = X1;
[mq,nq] = size(Q);
while (nq-nv) > 0
	ni = [ni,nq];
	nv = nq;
	Q = ima([X1 A*(ints(Q,C))],0);
	[mq,nq] = size(Q);
end
% --- last line of miinco ---
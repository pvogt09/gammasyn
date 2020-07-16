function [Q,ni] = mininv(A,B)
%MININV   Minimum A-invariant containig imB.
%  Q = mininv(A,B) is an orthonormal basis for the minimum A-invariant
%  containing imB.
%  [Q,ni] = mininv(A,B) gives in the row vector ni the dimensions of the
%  subsequent subspaces computed by the recursive algorithm.
%  The routine implements Algorithm 3.2-1 of "Controlled and Conditioned
%  Invariants in Linear System Theory"

%  Basile and Marro 4-20-90

nv = 0; ni = [];
B1 = ima(B,0);
Q = B1;
[mq,nq] = size(Q);
while (nq-nv) > 0
	ni = [ni,nq];
	nv = nq;
	Q = ima([B1 A*Q],0);
	[mq,nq] = size(Q);
end
% --- last line of mininv
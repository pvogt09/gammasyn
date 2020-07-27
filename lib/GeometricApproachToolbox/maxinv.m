function [Q,ni] = maxinv(A,X)
%MAXINV   Maximum A-invariant contained in imX.
%  Q = maxinv(A,X) is an orthonormal basis for the maximal A-invariant
%  contained in imX.
%  [Q,ni] = maxinv(A,B) gives in the row vector ni the dimensions of
%  the subsequent subspaces computed by the recursive algorithm.
%  The routine implements Relation 3.2.7 of "Controlled and Conditioned
%  Invariants in Linear System Theory"

%  Basile and Marro 4-20-90

[Q,ni] = mininv(A',ortco(X));
Q = ortco(Q);
ni = length(A) - ni;
% --- last line of maxinv ---
function [Q,ni] = mainco(A,B,X)
%MAINCO   Maximum (A,imB)-controlled invariant contained in imX.
%  Q = mainco(A,B,X) is an orthonormal basis for the maximum
%  (A,imB)-controlled invariant contained in imX.
%  [Q,ni] = mainco(A,B,X) gives in the row vector ni the dimensions of
%  the subsequent subspaces computed by the recursive algorithm.
%  The routine implements Relation 4.1.8 of "Controlled and Conditioned
%  Invariants in Linear System Theory"

%  Basile and Marro 4-20-90

[Q,ni] = miinco(A',ortco(B),ortco(X));
Q = ortco(Q);
ni = length(A) - ni;
% --- last line of mainco ---
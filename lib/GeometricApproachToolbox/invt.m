function Q = invt(A,X)
%INVT     Inverse transform of a subspace.
%  Q=invt(A,X) is an orthonormal basis for the inverse map of imX in A.

%  Basile and Marro 4-20-90

Q=ortco(A'*ortco(X));
% --- last line of invt ---
function Q = robcoin(A,B,E)
%ROBCOIN  Maximal robust controlled invariant.
%         Q = robcoin(A,B,E) is an orthonormal basis for the maximal robust
%         (A(k),imB(k))-controlled invariant contained in ints(kerE(k)).
%         The input matrices are A := [A(1) A(2) ...], B := [B(1) B(2) ... ],
%         E := [E(1) E(2) ... ]. The program runs with the geometric approach
%         m-files of "Controlled and Conditioned Invariants in Linear System
%         Theory" (G. Basile and G. Marro - Prentice Hall 1992).

%         G. Marro and A. Piazzi 4-20-92

[ma,na] = size(A);
[t,nb] = size(B);
if na > ma
	h = na/ma;
	nb = nb/h;
else
	Q = mainco(A,B,ker(E));
	return
end
V1 = eye(ma);
for k = 0:(h-1)
	Ek = E(:,(ma*k+1):(ma*k+ma));
	V1 = ints(ker(Ek),V1);
end
in = 1;
V = V1;
while in == 1
	for k = 0:(h-1)
		Ak = A(:,(ma*k+1):(ma*k+ma));
		Bk = B(:,(nb*k+1):(nb*k+nb));
		V = mainco(Ak,Bk,V);
	end
	[t,nv1] = size(V1);
	[t,nv] = size(V);
	if nv < nv1
		V1 = V;
	else
		Q = V;
		in = 0;
	end
end
% --- last line of robcoin ---
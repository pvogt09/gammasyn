function W = maxpcs(A,B,C,X,ii)
%MAXPCS   Maximal perfect controllability subspace.
%        W = maxpcs(A,B,C,X,i) is an orthonormal basis for the maximal
%        perfect controllability subspace with respect to the i-th
%        derivative contained in subspace X.

%        G.Marro 4-27-95

nv=length(A);
W=X;
[mw,nw]=size(W);
B1=ima(B,0);
C1=ker(C);
h=0;
while (nv-nw)>0 || h==0
	nv=nw;
	Z=B1;
	for jj=1:(ii-1)
		Z=ima([B1,A*ints(ints(Z,W),C1)],0);
	end
	W=mainco(A,B,sums(ints(W,Z),C1));
	[mw,nw]=size(W);
	h=h+1;
end
% --- last line of maxpcs ---
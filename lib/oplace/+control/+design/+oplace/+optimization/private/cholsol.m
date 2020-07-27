function X=cholsol(N,A,B,X)
%     UP FUER CHOLESKY-FAKTORISIERTE MATRIZEN, DOPPELT LANG
%     BERECHNET X AUS L*D*LT*X = B
%     N      - DIMENSION DES VEKTORRAUMS
%     A      - EINDIM. BEREICH DER MATRIX A BZW. LD
%     B      - VEKTOR DER RECHTEN SEITE B
%     X      - LOESUNGSVEKTOR X
%     REAL*8 A(1),B(1),X(1),Z

N1=N-1;
NP=N+1;
NN=N*NP/2;
X(1)=B(1);
for I=2:N,
	IJ=I;
	I1=I-1;
	Z=B(I);
	for J=1:I1,
		Z=Z-A(IJ)*X(J);
		IJ=IJ+N-J;
	end
	X(I)=Z;
end
X(N)=X(N)/A(NN);
IJ=NN;
for I=1:N1,
	IJ=IJ-1;
	Z=0.0;
	for J=1:I,
		Z=Z+A(IJ)*X(NP-J);
		IJ=IJ-1;
	end
	X(N-I)=X(N-I)/A(IJ)-Z;
end
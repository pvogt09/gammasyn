function A = cholmod(N,A,SIGMA,Z,H,IPOSD)
%     UP FUER CHOLESKY-FAKTORISIERTE MATRIZEN, DOPPELT LANG
%     BERECHNET LS*DS*LST = L*D*LT + SIGMA*Z*ZT
%     N      - DIMENSION DES VEKTORRAUMS
%     A      - EINDIM. BEREICH DER MATRIX A BZW. LD
%     SIGMA  - SKALARER FAKTOR DER RANG1-MATRIX
%     Z      - EIGENVEKTOR DER RANG1-MATRIX
%     H      - HILFSVEKTOR DER LAENGE N
%     IPOSD  - STEUERGROESSE ZUR KONTROLLE DER POSITIV-DEFINITHEIT
%              = 0   KEINE KONTROLLE
%              = 1   ERGEBNISMATRIX WIRD POSITIV DEFINIT
%      REAL*8 A(1),Z(1),H(1),SIGMA,DELTA,D,DS,DEL,SIG,BETA,V

DELTA=1.0D-17;
SIG=SIGMA;
for I=1:N,
   H(I)=Z(I);
end 
K=1;
for J=1:N,
   V=H(J);
   D=A(K);
   DS=D+SIG*V*V;
   if (abs(DS)< DELTA)
      DS=sign_fortran(DELTA,DS);
   end 
   if (IPOSD ~= 0)
      DS=abs(DS);
   end 
   A(K)=DS;
   DEL =SIG/DS;
   BETA=DEL*V;
   SIG =DEL*D;
   if (J == N)
      return;
   end 
   M=J+1;
   for I=M:N,
      K=K+1;
      H(I)=H(I)-V*A(K);
      A(K)=A(K)+BETA*H(I);
   end 
   K=K+1;
end 

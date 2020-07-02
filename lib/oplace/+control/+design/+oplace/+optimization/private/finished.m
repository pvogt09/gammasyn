function [IV,DV]=finished(N,X,IU,IV,DV,fun)

% ENDE DER ITERATIONSSCHLEIFE
% AUSDRUCKEN UND RETTEN DER AKTUELLEN WERTE

ISTART=1;
IPRINT=3;
IEXIT=11;
IOLDN=12;
IOLDX=13;

switch fun
   case 1
      IV(IEXIT)=6;
      if (IV(IPRINT)~=0)
         J=IV(IPRINT);
         IV(IPRINT)=sign_fortran(1,J);
         monit(N,0,X,IV,DV,3);
         IV(IPRINT)=J;
      end
   case 2
      if (IV(IPRINT)~=0)
         J=IV(IPRINT);
         IV(IPRINT)=sign_fortran(1,J);
         monit(N,0,X,IV,DV,3);
         IV(IPRINT)=J;
      end
end
IV(ISTART)=IV(IEXIT);
IV(IOLDN) =N;
IV(IOLDX) =IU;
%CALL VCOPY(N,DV(IU),X)
DV(IU:IU+N-1)=X(1:N);

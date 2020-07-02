function monit(N,M,X,IV,DV,K)
%*******************************************************************************
%
%   AUSDRUCKEN VON ZWISCHENERGEBNISSEN BEI OPTIMIERUNGSALGORITHMEN
%
%   EINGABEPARAMETER:
%   ****************
%   N       = ANZAHL DER VARIABLEN X(J)
%   M       = ANZAHL DER TEILFUNKTIONEN TF(I)
%   X       = FELD DER VARIABLEN X(J)
%   IV      = DATENFELD FUER INTEGER-VARIABLE
%   DV      = DATENFELD FUER DOUBLE-PRECISION-VARIABLE
%   K       = STEUERUNG DES DRUCKBILDES. ES WERDEN GEDRUCKT :
%               FUER K = 1 : IT,IF,F,X
%               FUER K = 2 : IT,IF,F,X,TF
%               FUER K = 3 : IT,IF,IG,F,X,G
%               FUER K = 4 : IT,IF,IG,F,X,G,TF
%               FUER K = 5 : ABBRUCHKRITERIUM
%
%   IM DATENFELD IV MUESSEN UEBERGEBEN WERDEN :
%   IV(2)   = IV(IOUTNR)  : NUMMER DES AUSGABEMEDIUMS
%   IV(3)   = IV(IPRINT)  : STEUERUNG DER DRUCKAUSGABE
%                = 0    : KEINE DRUCKAUSGABE
%                =+1    : JEDE I-TE ITERATION WIRD GEDRUCKT
%                =-1    : JEDE I-TE ITERATION WIRD GEDRUCKT,
%                         OHNE GRADIENTENVEKTOR UND OHNE
%                         TEILFUNKTIONSWERTE
%   IV(4)   = IV(MAXITN)  : MAXIMALE ANZAHL DER ITERATIONEN
%   IV(5)   = IV(MAXIFN)  : MAXIMALE ANZAHL DER FUNKTIONSWERT-
%                           BERECHNUNGEN
%   IV(6)   = IV(ITN)     : ANZAHL DER DURCHGEFUEHRTEN ITERATIONEN
%   IV(7)   = IV(IFN)     : ANZAHL DER FUNKTIONSWERTBERECHNUNGEN
%   IV(8)   = IV(IGN)     : ANZAHL DER GRADIENTENBERECHNUNGEN
%   IV(11)  = IV(IEXIT)   : NUMMER DES ABBRUCHKRITERIUMS
%   IV(15)  = IV(IDVTF)   : FELDANFANG FUER DEN VEKTOR TF(I)
%
%   IM DATENFELD DV MUESSEN UEBERGEBEN WERDEN :
%   DV(1)   = DV(IF)      : FUNKTIONSWERT
%   DV(2)   = DV(IEPS)    : KLEINSTER WERT FUER DIE SCHRITTWEITE IN
%                           EINEM ITERATIONSSCHRITT
%   DV(3)   = DV(IACC)    : KLEINSTER WERT FUER DIE NORM DES GRADIENTEN
%   DV(10)  = DV(IG)      : GRADIENTENVEKTOR
%    :
%   DV(10+N-1)
%   DV(IDVTF)             : VEKTOR DER TEILFUNKTIONEN
%    :
%   DV(IDVTF+N-1)
%
%*******************************************************************************
%     IMPLICIT DOUBLE PRECISION (A-H,O-Z)
%     DIMENSION X(1),IV(1),DV(1)

IBL=' ';
IOUTNR=2;
IPRINT=3;
MAXITN=4;
MAXIFN=5;
ITN=6;
IFN=7;
IGN=8;
IEXIT=11;
IDVA=14;
IDVTF=15;
IF=1;
IEPSG=2;
IEPSF=3;
IEPSX=4;
IDELX=5;

IP=IV(IPRINT);
if (IP ~= 0)
   if (K ~= 5)
      if (mod(IV(ITN),abs(IP))== 0)
         IOUT=IV(IOUTNR);
         IG=IV(IDVA);
         IGN1=IG+N-1;
         ITF1=IV(IDVTF)-1;
         G2N=v2norm(N,DV(IG:IGN1));
         if (IV(ITN)==0)
            disp(sprintf('\n'));
         end
         if (IP<0)
            disp(sprintf('IT=%d\t\t\tIF=%d\t\t\tF=%0.5g\t\t\tG=%0.5g',IV(ITN),IV(IFN),DV(IF),G2N));
            % WRITE (IOUT,1000) IV(ITN),IV(IFN),DV(IF)
            % 1000 FORMAT(1H ,'IT=',I5,10X,'IF=',I5,10X,'F=',1PD20.13)
         else
            disp(sprintf('IT=%d\t\t\tIF=%d\t\t\tF=%0.5g\t\t\tG=%0.5g',IV(ITN),IV(IFN),DV(IF),G2N));
            % WRITE (IOUT,1001) IV(ITN),IV(IFN),IV(IGN),DV(IF)
            % 1001 FORMAT(1H0,'IT=',I5,10X,'IF=',I5,10X,'IG=',I5,10X,'F=',1PD20.13)
            disp(sprintf('Variables:\n'));
            disp(X);
            % WRITE(IOUT,1002) (IBL,I,X(I),I=1,N)
            % 1002 FORMAT(5(A1,' X(',I2,')=',1PD13.6,2X))
         end 
         if (IP>=0 & K~=1)
            if (K~=2)
               disp(sprintf('Gradient:\n'));
               X(:)=DV(IG:IGN1);
               disp(X);
               % WRITE(IOUT,1003) (IBL,I,DV(IG1+I),I=1,N)
               % 1003 FORMAT(5(A1,' G(',I2,')=',1PD13.6,2X))
            end 
         end 
      end 
   end 
end

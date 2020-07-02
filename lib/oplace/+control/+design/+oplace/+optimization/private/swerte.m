function [IV,DV]=swerte(IV,DV)
%********************************************************************
% DIESES UNTERPROGRAMM LIEFERT STANDARDWERTE FUER DIE IN DEN
% DATENFELDERN IV UND DV ENTHALTENEN VARIABLEN.
%********************************************************************
%     IMPLICIT DOUBLE PRECISION (A-H,O-Z)
%     DIMENSION IV(1),DV(1)
%     EXTERNAL DPMDC
ISTART=1;
IOUTNR=2;
IPRINT=3;
MAXITN=4;
MAXIFN=5;
IDIFF=9;
IEXIT=11;
IDVA=14;
IEPSG=2;
IEPSF=3;
IEPSX=4;
IDELX=5;

% WERTE FUER DATENFELD IV

IV(ISTART)=10;
IV(IOUTNR)=6;
IV(IPRINT)=-2;
IV(MAXITN)=100;
IV(MAXIFN)=200;
IV(IDIFF) =1;
IV(IEXIT) =10;
IV(IDVA)  =10;

% WERTE FUER DATENFELD DV

EPS=control.design.oplace.machconst(3);
DV(IEPSG)=max(1.0D-8,1.0D3*EPS);
DV(IEPSF)=max(1.0D-8,1.0D3*EPS);
DV(IEPSX)=max(1.0D-9,1.0D3*EPS);
SQTEPS   =control.design.oplace.machconst(4);
DV(IDELX)=max(1.0D-9,SQTEPS);
%IA=IV(IDVA)-1;
%for I=1:N,
%   DV(IA+I)=0.0;
%end

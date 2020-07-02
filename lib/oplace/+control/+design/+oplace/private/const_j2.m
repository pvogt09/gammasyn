function [G_xi,p_xi] = const_j2(st_opt,ew,ews,A,B,C,npq)
% [G_xi,p_xi] = CONST_J2(st_opt,ew,A,B,C,npq) berechnet 
% die konstanten Parameter, die für die Bestimmung der Güte-
% funktion J2 erforderlich sind.
%
% OUTPUT:
% G_xi   - 3-dim. Übertragungsmatrix der Regelstrecke 
% p_xi   - konst. Wert, erforderlich, weil Stützstelle kein EW
% 
% INPUT:
% st_opt - für die Optimierung zu nutzende Stützstellen
% ew     - Vektor mit den vorgegebenen Eigenwerten
% A      - Systemmatrix
% B      - Eingangsmatrix
% C      - Ausgangsmatrix
% npq    - Dimensionen der vorliegenden Matrizen bzw. Vektoren

In = eye(npq(1));
G_xi = zeros(npq(3),npq(2),npq(5));
for i=1:npq(5)
   G_xi(:,:,i) = C*inv(st_opt(i)*In-A)*B;
end; 
p_xi = st_const(st_opt,ew,ews,npq(1),npq(5));

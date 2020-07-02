function j2 = criterion_2(K,w2,G_xi,p_xi,npq)
% j2 = GUETEFUNKTION_2(K,w2,G_xi,p_xi,npq) berechnet das Gütekriterium 
% J2, welches die gewählten Stützstellen berücksichtigt.
%
% OUTPUT:
% j2     - Gütefunktionswert für K
%
% INPUT:
% K      - die zu optimierende Matrix
% w2     - Gewichtungsfaktoren für J2
% G_xi   - 3-dim. Übertragungsmatrix der Regelstrecke 
% p_xi   - konst. Wert, weil Stützstelle kein EW
% npq    - Dimensionen der vorliegenden Matrizen bzw. Vektoren  

Ip = eye(npq(2)); 
j2 = 0;
for i=1:npq(5)
   Z = Ip+K*G_xi(:,:,i);
   j2 = j2 + .5*w2(i)*(det(Z)-p_xi(i))^2;
end

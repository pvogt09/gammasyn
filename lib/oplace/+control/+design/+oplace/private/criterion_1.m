function j1 = criterion_1(K,w1,G_lam,npq,ew)
% j1 = GUETEFUNKTION_1(K,w1,G_lam,npq) berechnet das G�tekriterium 
% J1, welches die vorgegebenen EW ber�cksichtigt.
%
% OUTPUT:
% j1     - G�tefunktionswert f�r K
%
% INPUT:
% K      - die zu optimierende Matrix
% w1     - Gewichtungsfaktoren f�r J1
% G_lam  - 3-dim. �bertragungsmatrix der Regelstrecke 
% npq    - Dimensionen der vorliegenden Matrizen bzw. Vektoren  

Ip = eye(npq(2)); 
j1 = 0;
i = 1;
while i<=npq(4)
   if isreal(ew(i))
      Z = Ip+K*G_lam(:,:,i);
      j1 = j1 + .5*w1(i)*abs(det(Z))^2;
      i = i + 1;
   else
      Z = Ip+K*G_lam(:,:,i);
      j1 = j1 + w1(i)*abs(det(Z))^2;
      i = i + 2;
   end
end

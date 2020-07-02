function j3 = criterion_3(K,w3,P,H,npq)
% j3 = GUETEFUNKTION_3(K,w3,P,H,npq) berechnet das G�tekriterium 
% J3, welches die vorgegebenen Parametervektoren ber�cksichtigt.
%
% OUTPUT:
% j4     - G�tefunktionswert f�r K
%
% INPUT:
% K      - die zu optimierende Matrix
% w3     - Gewichtungsfaktoren f�r die einzelnen Parametervektoren
% P      - Matrix mit den Parametervektoren 
% H      - aus den Parametervektoren berechnete Vektoren
% npq    - Dimensionen der vorliegenden Matrizen bzw. Vektoren  

j3 = 0;
p=npq(2);
for i=1:npq(6)
   Z = P(1:p,i)+K*H(:,i);
   j3 = j3 + w3(i)*(Z'*Z);   
end
j3 = real(0.5*j3);


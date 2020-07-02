function dK = gradient_3(K,S,npq,P,w3,H)
% dK = GRADIENT_3(K,S,npq,P,w3,H) berechnet den Gradienten
% für das Gütekriterium J3
%
% OUTPUT:
% dK     - Gradient der zu optimierenden Matrix für J3
%
% INPUT:
% K      - die zu optimierende Matrix
% S      - Matrix für eine mögliche Strukturbeschränkung
% npq    - Dimensionen der vorliegenden Matrizen bzw. Vektoren  
% P      - Matrix mit den Parametervektoren 
% w3     - Gewichtungsfaktoren für die einzelnen Parametervektoren
% H      - aus den Parametervektoren berechnete Vektoren

p = npq(2);
dK = zeros(npq(2),npq(3));
%--- Berechnung der Gradientenmatrix

for i=1:npq(6)
   dK = dK + w3(i) * (P(1:p,i)+K*H(:,i)) * H(:,i)';
end;
%--- Strukturbeschränkung an der Stelle (i,j) -> dK(i,j) = 0
dK=real(dK);
if ~isempty(S)
   dK = dK.*(~S);   
end

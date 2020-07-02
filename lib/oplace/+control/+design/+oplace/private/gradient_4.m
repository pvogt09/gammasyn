function dK = gradient_4(K,W4,T1)
% dK = GRADIENT_4(K,W4) berechnet den Gradienten
% f�r das G�tekriterium J4
%
% OUTPUT:
% dK     - Gradient der zu optimierenden Matrix f�r J4
%
% INPUT:
% K      - die zu optimierende Matrix
% W4     - Matrix mit den Gewichtungsfaktoren f�r das G�tekriterium 
%          der minimalen Reglernorm

%--- Berechnung der Gradientenmatrix (Strukturbeschr�nkung in W4 enthalten)
if isempty(T1)
   dK = K.*W4;
else
   Kh = K*T1';
   dK = (Kh.*W4)*T1;
end



      
      
      
      

function dK = gradient_4(K,W4,T1)
% dK = GRADIENT_4(K,W4) berechnet den Gradienten
% für das Gütekriterium J4
%
% OUTPUT:
% dK     - Gradient der zu optimierenden Matrix für J4
%
% INPUT:
% K      - die zu optimierende Matrix
% W4     - Matrix mit den Gewichtungsfaktoren für das Gütekriterium
%          der minimalen Reglernorm

%--- Berechnung der Gradientenmatrix (Strukturbeschränkung in W4 enthalten)
if isempty(T1)
	dK = K.*W4;
else
	Kh = K*T1';
	dK = (Kh.*W4)*T1;
end
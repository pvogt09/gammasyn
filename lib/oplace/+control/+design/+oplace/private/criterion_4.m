function j4 = criterion_4(K,W4,T1)
% j4 = GUETEFUNKTION_4(K,W4) berechnet das Gütekriterium
% J4, welches die minimale Reglernorm berücksichtigt.
%
% OUTPUT:
% j4     - Gütefunktionswert für K
%
% INPUT:
% K      - die zu optimierende Matrix
% W4     - Matrix mit den Gewichtungsfaktoren für das Gütekriterium
%          der minimalen Reglernorm

if isempty(T1)
	j4 = 0.5*sum(sum(W4.*(K.^2)));
else
	Kh = K*T1';
	j4 = 0.5*sum(sum(W4.*(Kh.^2)));
end
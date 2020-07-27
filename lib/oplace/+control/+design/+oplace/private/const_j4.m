function W4 = const_j3(W4,S,npq)
% W4 = CONST_J4(W4,S) kontrolliert die Gewichtungsmatrix W4,
% und setzt bei einem konstanten Element die Gewichtung auf
% Null.
%
% OUTPUT:
% W4     - ggf. veränderte Matrix mit den Gewichtungsfaktoren
%          für das Gütekriterium der minimalen Reglernorm
%
% INPUT:
% W4     - Matrix mit den Gewichtungsfaktoren für das Gütekriterium
%          der minimalen Reglernorm
% S      - Matrix für eine mögliche Strukturbeschränkung
% npq    - Dimensionen der vorliegenden Matrizen bzw. Vektoren

for i=1:npq(2)
		for j=1:npq(3)
			if S(i,j)
				W4(i,j) = 0;
		end;
	end;
end;
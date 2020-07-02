% A 		- Systemmatrix
% B 		- Eingangsmatrix 
% C 		- Ausgangsmatrix
% ew		- Vektor mit den vorgegebenen Eigenwerten
% K0		- Startrückführungsmatrix für die Optimierung 
% S		- Matrix für eine mögliche Strukturbeschränkung
% P		- Matrix mit den Parametervektoren 
% w3		- Gewichtungsfaktoren für die einzelnen Parametervektoren
% W4		- Matrix mit den Gewichtungsfaktoren für das Gütekriterium 
%          der minimalen Reglernorm
% ew_opt	- für die Optimierung zu nutzende EW (nicht mit 0 gewichtet)
% st_opt	- für die Optimierung zu nutzende Stützstellen
% npq    - Dimensionen der vorliegenden Matrizen bzw. Vektoren  
% flag   - Vektor, der anzeigt, welche EW zu ändern sind
% H    	- aus den Parametervektoren berechnete Vektoren
% G_lam 	- 3-dim. Übertragungsmatrix der Regelstrecke 
% w1	  	- Gewichtungsfaktoren für J1
% G_xi   - 3-dim. Übertragungsmatrix der Regelstrecke 
% w2	   - Gewichtungsfaktoren für J2
% p_xi   - konst. Wert, weil Stützstelle kein EW
% merker	- Vektor, der aufschluß gibt, welcher EW mit 0 gewichtet ist 
% dK 		- Gradient der zu optimierenden Matrix
% K      - die zu optimierende Matrix
% c		- Vorgabevektor für die einzelnen Gütekriterien
% zeta 	- Optimierungskonstante: zeta = max[Ji(K0)/c(i)]
% Ko   	- Matrix, durch die sämtliche Parametervektor und die dazugehörigen 
%          EW festlegt werden
% T1   	- Matrix für die Berechnung des neuen Systems
 
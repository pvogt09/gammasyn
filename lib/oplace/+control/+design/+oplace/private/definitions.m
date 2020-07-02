% A 		- Systemmatrix
% B 		- Eingangsmatrix 
% C 		- Ausgangsmatrix
% ew		- Vektor mit den vorgegebenen Eigenwerten
% K0		- Startr�ckf�hrungsmatrix f�r die Optimierung 
% S		- Matrix f�r eine m�gliche Strukturbeschr�nkung
% P		- Matrix mit den Parametervektoren 
% w3		- Gewichtungsfaktoren f�r die einzelnen Parametervektoren
% W4		- Matrix mit den Gewichtungsfaktoren f�r das G�tekriterium 
%          der minimalen Reglernorm
% ew_opt	- f�r die Optimierung zu nutzende EW (nicht mit 0 gewichtet)
% st_opt	- f�r die Optimierung zu nutzende St�tzstellen
% npq    - Dimensionen der vorliegenden Matrizen bzw. Vektoren  
% flag   - Vektor, der anzeigt, welche EW zu �ndern sind
% H    	- aus den Parametervektoren berechnete Vektoren
% G_lam 	- 3-dim. �bertragungsmatrix der Regelstrecke 
% w1	  	- Gewichtungsfaktoren f�r J1
% G_xi   - 3-dim. �bertragungsmatrix der Regelstrecke 
% w2	   - Gewichtungsfaktoren f�r J2
% p_xi   - konst. Wert, weil St�tzstelle kein EW
% merker	- Vektor, der aufschlu� gibt, welcher EW mit 0 gewichtet ist 
% dK 		- Gradient der zu optimierenden Matrix
% K      - die zu optimierende Matrix
% c		- Vorgabevektor f�r die einzelnen G�tekriterien
% zeta 	- Optimierungskonstante: zeta = max[Ji(K0)/c(i)]
% Ko   	- Matrix, durch die s�mtliche Parametervektor und die dazugeh�rigen 
%          EW festlegt werden
% T1   	- Matrix f�r die Berechnung des neuen Systems
 
function p_xi = st_const(st_opt,ew,ews,n,anz)
% p_xi = ST_CONST(st_opt,ew,A,anz) berechnet den konstanten Anteil 
% für das Gütekriterium J2.
%
% OUTPUT:
% p_xi   - konst. Wert,erforderlich, weil Stützstelle kein EW
%
% INPUT:
% st_opt - für die Optimierung zu nutzende Stützstellen
% ew     - Vektor mit den vorgegebenen Eigenwerten
% ews    - Streckeneigenwerte
% npq    - Dimensionen

p_xi = zeros(anz,1);

for i=1:anz
   nom = 1;
   stst = st_opt(i);
   for j=1:n
      nom = nom*(stst-ew(j))/(stst-ews(j));
   end
   p_xi(i) = real(nom);
end

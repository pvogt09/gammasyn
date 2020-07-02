function w2 = weight_2(st_opt,ew,ews,anz)
% w2 = GEWICHTUNG_2(st_opt,ew,A,anz) berechnet die Gewichtungs-
% faktoren für die einzelnen Stützstellen, die für das Güte-
% kriterium J2 benötigt werden.
%
% OUTPUT:
% w2     - Gewichtungsfaktoren für J2
%
% INPUT:
% st_opt - für die Optimierung zu nutzende Stützstellen
% ew     - Vektor mit den vorgegebenen Eigenwerten
% ews    - Streckeneigenwerte
% anz    - Anzahl der Stützstellen  

w2 = zeros(anz,1);
n = length(ews);

for i=1:anz
   nom = 1;
   nenn1 = 0;
   nenn2 = 0;
   stst = st_opt(i);
   for j=1:n
      nenn1 = nenn1 + 1/(stst - ew(j));
      nenn2 = nenn2 + 1/(stst - ews(j));
      nom = nom*(stst-ews(j))/(stst-ew(j));
   end
   w2(i) = abs(nom/(nenn1 - nenn2))^2;
end

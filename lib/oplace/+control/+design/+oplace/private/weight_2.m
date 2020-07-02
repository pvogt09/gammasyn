function w2 = weight_2(st_opt,ew,ews,anz)
% w2 = GEWICHTUNG_2(st_opt,ew,A,anz) berechnet die Gewichtungs-
% faktoren f�r die einzelnen St�tzstellen, die f�r das G�te-
% kriterium J2 ben�tigt werden.
%
% OUTPUT:
% w2     - Gewichtungsfaktoren f�r J2
%
% INPUT:
% st_opt - f�r die Optimierung zu nutzende St�tzstellen
% ew     - Vektor mit den vorgegebenen Eigenwerten
% ews    - Streckeneigenwerte
% anz    - Anzahl der St�tzstellen  

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

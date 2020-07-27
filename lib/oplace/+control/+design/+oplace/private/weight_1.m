function w1 = weight_1(ew_opt,ew,ews,npq)
% w1 = GEWICHTUNG_1(ew_opt,ew,A,npq) berechnet die Gewichtungs-
% faktoren für die einzelnen Eigenwerte, die für das Gütekriterium
% J1 benötigt werden.
%
% OUTPUT:
% w1     - Gewichtungsfaktoren für J1
%
% INPUT:
% ew_opt - für die Optimierung zu nutzende EW (nicht mit 0 gewichtet)
% ew     - Vektor mit den vorgegebenen Eigenwerten
% A      - Streckeneigenwerte
% npq    - Dimensionen der vorliegenden Matrizen bzw. Vektoren

nopt=npq(4);
n=npq(1);
w1 = zeros(nopt,1);

for i=1:nopt
	for j=1:n
		fak=1;
		for k=1:n
			if k~=j
				fak = fak*(ew_opt(i)-ew(k))/(ew_opt(i)-ews(k));
			end
		end
		w1(i) = w1(i) + fak/(ew_opt(i)-ews(j));
	end
end

w1 = abs(w1).^2;
fak=median(w1);
if fak < 1e-16		% vorgegebener Wert
	fak = 1;
end;

for i=1:nopt
	if w1(i) < 1e-16	% vorgegebener Wert
		w1(i) = fak;
	end
	w1(i)=1/w1(i);
end
function flag = ev_test(ew,ews,npq)
% flag = KONTROLLE(ew,A,B,C,npq) überprüft die
% vorgegeben Eigenwerte und setzt im Falle einer erforderlichen
% Änderung ein Flag.
%
% OUTPUT:
% flag   - Vektor, der anzeigt, welche Eigenwerte zu ändern sind
%
% INPUT:
% ew     - Vektor mit den vorgegebenen Eigenwerten
% ews    - Vektor mit den Streckeneigenwerten
% npq    - Dimensionen der vorliegenden Matrizen bzw. Vektoren

n=npq(1);
flag = zeros(n,1);
fak = flag;
%--- Kontrolle mit den Streckeneigenwerte
for i=1:n
	for j=1:n
		fak(j) = abs(ew(i)-ews(j));
	end;
	minfak = min(fak);
	if minfak < 1e-3   % vorgegebener Wert
		flag(i) = 1;
	end;
end;
%--- �berpr�fung der einzelnen Eigenwerte untereinander
for i=1:n-1
	for j=i+1:n
		diff = abs(ew(i)-ew(j));
		if diff < 1e-3   % vorgegebener Wert
			flag(j) = 2;
		end;
	end;
end;
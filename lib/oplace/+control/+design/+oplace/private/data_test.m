function okay = data_test(wert,ew_opt,st_opt,ews,n)
% okay = KONTROLLE_WERT(wert,ew_opt,st_opt,A,B,C,npq) überprüft,
% ob der eingegebene Wert als Stützstelle akzeptabel ist.
% 
% OUTPUT:
% okay   - wenn okay == 1: Wert i.O.
%          wenn okay == 0: Wert nicht akzeptabel
%
% INPUT:
% wert   - der neu eingegebene Wert
% ew_opt - für die Optimierung zu nutzende EW (nicht mit 0 gewichtet)
% st_opt - für die Optimierung zu nutzende Stützstellen
% ews    - Streckeneigenwerte
% n      - Systemordnung

okay = 1;
stelle = [ew_opt;st_opt];
%--- Kontrolle mit den Streckeneigenwerte
fak = zeros(n,1);
for i=1:n
	fak(i) = abs(wert-ews(i));
end;
minfak = min(fak);
if minfak < 1e-3   % vorgegebener Wert
	okay = 0;
	return;
end
%--- Überprüfung mit den bereits vorgegebenen EW bzw.Stützstellen
d = size(stelle,1);
if d <=0
	return;
end

for i=1:d
	diff = abs(wert-stelle(i));
	if diff < 1e-3   % vorgegebener Wert
		okay = 0;
	end
end
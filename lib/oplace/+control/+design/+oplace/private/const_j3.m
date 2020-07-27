function H = const_j3(ew,A,B,C,P,npq)
% H = CONST_J3(ew,A,B,C,P,npq) berechnet die konstanten
% Parameter, die für die Bestimmung der Gütefunktion J3
% erforderlich sind.
%
% OUTPUT:
% H      - aus den Parametervektoren berechnete Vektoren
%
% INPUT:
% ew     - Vektor mit den vorgegebenen Eigenwerten
% A      - Systemmatrix
% B      - Eingangsmatrix
% C      - Ausgangsmatrix
% P      - Matrix mit den Parametervektoren
% npq    - Dimensionen der vorliegenden Matrizen bzw. Vektoren

%H = zeros(npq(3),npq(6));
H = [];
In = eye(npq(1));
p = npq(2);
p1 = p+1;
for i=1:npq(6)
	for j=1:npq(1)
		if j<=npq(4) & P(p1,j)==i
			H(:,i) = C*inv(ew(i)*In-A)*B*P(1:p,i);
		end
	end
end
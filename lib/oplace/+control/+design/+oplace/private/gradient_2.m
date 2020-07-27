function dK = gradient_2(K,S,npq,w2,G_xi,p_xi)
% dK = GRADIENT_2(K,S,npq,w2,G_xi,p_xi) berechnet den Gradienten
% für das Gütekriterium J2
%
% OUTPUT:
% dK     - Gradient der zu optimierenden Matrix für J2
%
% INPUT:
% K      - die zu optimierende Matrix
% S      - Matrix für eine mögliche Strukturbeschränkung
% npq    - Dimensionen der vorliegenden Matrizen bzw. Vektoren
% w2     - Gewichtungsfaktoren für J2
% G_xi   - 3-dim. Übertragungsmatrix der Regelstrecke
% p_xi   - konst. Wert, weil Stützstelle kein EW

dK = zeros(npq(2),npq(3));
Ip = eye(npq(2));
if isempty(S)
	S = dK;
end

f2 = zeros(npq(5),1);
for i=1:npq(5)
	H(:,:,i) = Ip+K*G_xi(:,:,i);
	f2(i) = det(H(:,:,i))-p_xi(i);
end

for l=1:npq(2)
	for m=1:npq(3)
		if ~S(l,m)
			for i=1:npq(5)
				H1 = H(:,:,i);
				H1(l,:) = [G_xi(m,:,i)];
				dK(l,m) = dK(l,m) + w2(i)*det(H1)*f2(i);
			end
		end
	end
end
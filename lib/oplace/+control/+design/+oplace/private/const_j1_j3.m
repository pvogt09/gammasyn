function [G_lam,H,ew,P,npq] = const_j1_j3(ew,A,B,C,P,npq)
% [G_lam,H,npq] = CONST_J1_J3(ew,A,B,C,P,npq) berechnet die
% konstanten Parameter, die für die Bestimmung der Güte-
% funktion J1 erforderlich sind.
%
% OUTPUT:
% G_lam  - 3-dim. Übertragungsmatrix der Regelstrecke
%
% INPUT:
% ew_opt - für die Optimierung zu nutzende EW (nicht mit 0 gewichtet)
% A      - Systemmatrix
% B      - Eingangsmatrix
% C      - Ausgangsmatrix
% npq    - Dimensionen der vorliegenden Matrizen bzw. Vektoren

H = []; H1 = []; G_lam = []; G_lam1 = [];
In = eye(npq(1));
p = npq(2);
p1 = p+1;
n = 1;
i = 1;
jj = 1;
ew1 = []; ew2 = []; P1 = []; P2 = []; vh = [];
while i<=npq(6)
	for j=1:npq(4)
		if P(p1,j)==i
			G_lam(:,:,jj) = C*inv(ew(j)*In-A)*B;
			H(:,jj) = G_lam(:,:,jj)*P(1:p,j);
			ew1(jj,1) = ew(j);
			vh(jj) = j;
			P1(:,jj) = P(:,j);
			jj = jj+1;
		end
	end
i = i+1;
end

h = length(vh);
jj = 1;
for j=1:npq(4)
	if h
		chosen = 0;
		for i=1:h
			if vh(i)==j
				chosen = 1;
			end
		end
		if ~chosen
			G_lam1(:,:,jj) = C*inv(ew(j)*In-A)*B;
			H1(:,jj) = G_lam1(:,:,jj)*P(1:p,j);
			ew2(jj,1) = ew(j);
			P2(:,jj) = P(:,j);
			jj = jj+1;
		end
	else
		G_lam1(:,:,jj) = C*inv(ew(j)*In-A)*B;
		H1(:,jj) = G_lam1(:,:,jj)*P(1:p,j);
		ew2(jj,1) = ew(j);
		P2(:,jj) = P(:,j);
		jj = jj+1;
	end
end

warning on;
if npq(6)>0
	disp (' ');
	if ~h
		warning(sprintf('\nNone of the predefined %d parametervectors can be used!',npq(6)));
	elseif h~=npq(6)
		warning(sprintf('\nOnly %d of the predefined %d parametervectors can be used!',h,npq(6)));
	end
end

npq(6) = h;
if isempty(G_lam)
	G_lam = G_lam1;
elseif isempty(G_lam1)
	;
else
	G_lam = cat(3,G_lam,G_lam1);
end
H = [H,H1];
if npq(4) < npq(1)
	P = [P1,P2,P(:,npq(4)+1:npq(1))];
	ew = [ew1;ew2;ew(npq(4)+1:npq(1))];
else
	P = [P1,P2];
	ew = [ew1;ew2];
end
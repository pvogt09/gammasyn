function [Ks,npq_s,w1_s,ew_s,G_lam_s,G_xi_s,rho,w2_s] = transsys(A,B,C,K0,Ko,T1,npq,ew)
% [Ks,npq_s,w1_s,G_lam_s,G_xi_s] = SCHLANGE(A,B,C,K0,Ko,N,T1,npq,w1,ew_n)
% berechnet die veränderten Größen, die für die Optimierung
% erforderlich sind. Dieses ist notwendig, da bereits die EW, die mit
% einem Parametervektor versehen worden sind, durch Ko festliegen.
% Die anschließende Optimierung nähert die restlichen Vorgaben an.
%
% OUTPUT:  
% Ks      - neue Startmatrix für die Optimierung
% npq_s   - überarbeitetes npq für das neue System
% w1_s    - Gewichtungsfaktoren der restlichen EW
% G_lam_s - 3-dim. Übertragungsmatrix der Regelstrecke des neuen Systems
% G_xi_s  - 3-dim. Übertragungsmatrix der Regelstrecke des neuen Systems
%
% INPUT:
% A      - Systemmatrix
% B      - Eingangsmatrix
% C      - Ausgangsmatrix
% K0     - Startrückführungsmatrix für die Optimierung
% Ko     - Matrix, durch die sämtliche Parametervektoren und die dazugehörigen
%          EW festlegt werden
% N      - Matrix N = T1*T1'
% T1     - Matrix für die Berechnung des neuen Systems
% npq    - Dimensionen der vorliegenden Matrizen bzw. Vektoren
% w1     - Gewichtungsfaktoren für J1
% ew_n   - die zu nutzende EW (nicht mit 0 gewichtet)
% st_n   - die zu nutzende Stützstellen

As = A + B * Ko * C;
Cs = T1'* C;
r = size(Cs,1);
npq_s = [npq(1);npq(2);r;npq(4)-npq(6);npq(5);0];
In = eye(npq(1));
ews = eig(As);
ew_s = []; ew1 = [];
j = 1;
n = 1;
bad = ev_test(ew,ews,npq);
for i=1:npq(1)
	if i>npq(6) & ~bad(i)
		ew_s(j,1) = ew(i);
		j = j+1;
	else
		ew1(n,1) = ew(i);
		n = n+1;
	end
end
npq_s(4) = length(ew_s);
npq_s(5) = length(ew1)-npq(6);
ew_s = [ew_s;ew1];
d = npq_s(5);
st_s = [];
fak = sqrt(npq(1));
for i=1:d
	okay = 0;
	while ~okay
		sth = rand*fak;
		okay = data_test(sth,ew_s,st_s,ews,npq(1));
	end
	st_s(i,1) = sth;
end
G_lam_s = [];
w1_s = [];
if npq_s(4)>0
	G_lam_s = zeros(npq_s(3),npq_s(2),npq_s(4));
	for i=1:npq_s(4)
		G_lam_s(:,:,i) = Cs*inv(ew_s(i)*In-As)*B;
	end
	w1_s = weight_1(ew_s,ew_s,ews,npq_s);
end
G_xi_s = [];
rho = [];
w2_s = [];
if d>0
	rho = real(st_const(st_s,ew_s,ews,npq_s(1),d));
	w2_s = weight_2(st_s,ew_s,ews,d);
	G_xi_s = zeros(npq_s(3),npq_s(2),d);
	for i=1:d
		G_xi_s(:,:,i) = Cs*inv(st_s(i)*In-As)*B;
	end
end
% Berechnung der Startmatrix f�r das neue System
Ks = (K0+Ko)*T1;
function K = analytic(A,B,C,ew,P,H,G_lam,G_xi,p_xi,w1,w2,anaok,npq)
% K = ANALYTISCH(A,B,C,ew,ew_n,st_n,P,npq) bestimmt bei Erfüllung der
% nachfolgenden Bedingungen die gesuchte Ausgangsrückführungsmatrix K
% analytisch.
%
% Bed.:
% 1. r <= (p*q-n) / (p-1)  mit  r - Anz. der Parametervektoren [npq(6)],
%                               p - Anz. der Eingangsgrößen [npq(2)],
%                               q - Anz. der Ausgangsgrößen [npq(3)],
%                               n - Systemordnung [npq(1)].
% 2. r == q-1
%
% OUTPUT:
% K      - gesuchte Rückführungsmatrix
%
% INPUT:
% A      - Systemmatrix
% B      - Eingangsmatrix
% C      - Ausgangsmatrix
% ew     - vorgegebene EW
% P      - Matrix mit den Parametervektoren
%          (P(:,1) wird ew_n(1) zugeordnet,usw.)
% H      - aus den Parametervektoren berechnete Vektoren
% npq    - Dimensionen des Systems (siehe Bed., npq(4) - Anz der zu
%          nutzenden EW und npq(5) - Anz. der Stützstellen)

if anaok==2
	K = - P(1:npq(2),:) * inv(H);
	return;
end

if anaok==1
	In = eye(npq(1));
	Iq = eye(npq(3));
	Hi = pinv(H(:,1:npq(6)));
	Ko = real(P(1:npq(2),1:npq(6)) * Hi);
	N = real(Iq - H(:,1:npq(6)) * Hi);
	[U,S,V] = svd(N);
	T1(:,1)=V(:,1);
	T1 = T1';
	As = A + B * Ko * C;
	Cs = T1 * C;
	ews = eig(As);
	d = npq(1) - npq(6);
	sth = [];
	fak = 40;
	for i=1:d
		okay = 0;
		while ~okay
			st = (rand-.5)*fak;
			okay = data_test(st,[],sth,ews,npq(1));
		end
		sth(i,1) = st;
	end
	rho = real(st_const(sth,ew,ews,npq(1),d));

	for i=1:d
			Gamma(i,:) = Cs * inv(sth(i)*In-As) * B;
	end

	Ks = pinv(Gamma) * (rho - ones(d,1));
	K = -Ko + Ks * T1;
	return;
end

if anaok==3
	In = eye(npq(1));
	Iq = eye(npq(3));
	Hi = pinv(H(:,1:npq(4)));
	Ko = P(1:npq(2),1:npq(4)) * Hi;
	N = Iq - H * Hi;
	[U,S,V] = svd(N);
	T1(:,1)=V(:,1);
	T1 = T1';
	As = A + B * Ko * C;
	Cs = T1 * C;
	ews = eig(As);
	d = npq(1) - npq(4);
	sth = [];
	fak = sqrt(npq(1));
	for i=1:d
		okay = 0;
		while ~okay
			st = rand*fak;
			okay = data_test(st,[],sth,ews,npq(1));
		end
		sth(i) = st;
	end
	rho = st_const(sth,ew,ews,npq(1),d);

	for i=1:d
			Gamma(i,:) = Cs * inv(sth(i)*In-As) * B;
	end

	Ks = pinv(Gamma) * (rho - ones(d,1));
	K = -Ko + Ks * T1;
	return;
end

if anaok == 4
	Gamma = [];
	for i=1:npq(4)
		fak = sqrt(w1(i));
		Gamma(:,i) = G_lam(:,:,i)*fak;
		rho(1,i) = -fak;
	end
	for i=1:npq(5)
		fak = sqrt(w2(i));
		Gamma(:,i+npq(4)) = G_xi(:,:,i)*fak;
		rho(1,i+npq(4)) = (p_xi(i)-1)*fak;
	end
K = rho*pinv(Gamma);
end
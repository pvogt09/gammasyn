function [f, G, Rcode] = critgrad(K, S, npq, ew, w1, G_lam, w2, G_xi, p_xi, w3, P, H, W4, T1, c, optflag)
	% f = KTITERIUM(K,K0,S,npq,w1,G_lam,w2,G_xi,p_xi,w3,P,H,W4,c,zeta)
	% berechnet einen zusammengefaßten Funktionswert aus den einzelnen
	% Gütefunktionen.
	%
	% OUTPUT:
	% f      - zusammengefaßter Funktionswert
	%
	% INPUT:
	% K      - state or output feedback matrix to be calculated
	% S      - Matrix für eine mögliche Strukturbeschränkung
	% npq    - Dimensionen der vorliegenden Matrizen bzw. Vektoren
	% w1     - Gewichtungsfaktoren für J1
	% G_lam  - 3-dim. Übertragungsmatrix der Regelstrecke
	% w2     - Gewichtungsfaktoren für J2
	% G_xi   - 3-dim. Übertragungsmatrix der Regelstrecke
	% p_xi   - konst. Wert, weil Stützstelle kein EW
	% w3     - Gewichtungsfaktoren für die einzelnen Parametervektoren
	% P      - Matrix mit den Parametervektoren
	% H      - aus den Parametervektoren berechnete Vektoren
	% W4     - Matrix mit den Gewichtungsfaktoren für das Gütekriterium
	%          der minimalen Reglernorm
	% T1		- transformation matrix
	% c      - Vorgabevektor für die einzelnen Gütekriterien
	Rcode = 1;  %Kein Fehler
	y = zeros(npq(2), npq(3));
	J = zeros(4, 1);
	x = J;
	ih = J;
	GH = zeros(npq(2), npq(3), 4);

	if optflag
		G = y;
		if npq(4)
			f = criterion_1(K, w1, G_lam, npq, ew);
			G = gradient_1(K, S, npq, w1, G_lam, ew);
		elseif npq(5)
			f = criterion_2(K, w2, G_xi, p_xi, npq);
			G = gradient_2(K, S, npq, w2, G_xi, p_xi);
		end
		return;
	end

	if npq(4)
		J(1) = criterion_1(K, w1, G_lam, npq, ew)/c(1);
		GH(:, :, 1) = gradient_1(K, S, npq, w1, G_lam, ew);
		ih(1) = 1;
	end

	if npq(5)
		J(2) = criterion_2(K, w2, G_xi, p_xi, npq)/c(2);
		GH(:, :, 2) = gradient_2(K, S, npq, w2, G_xi, p_xi);
		ih(2) = 1;
	end

	if npq(6)
		J(3) = criterion_3(K, w3, P, H, npq)/c(3);
		GH(:, :, 3) = gradient_3(K, S, npq, P, w3, H);
		ih(3) = 1;
	end

	if ~isempty(W4)
		J(4) = criterion_4(K, W4, T1)/c(4);
		GH(:, :, 4) = gradient_4(K, W4, T1);
		ih(4) = 1;
	end

	eta = control.design.oplace.machconst(1);
	zeta = max(J) + eta;
	for ii = 1:4
		if ih(ii)
			x(ii) = exp(20*(J(ii) - zeta));
		end
	end

	z = sum(x);
	if (z <= eta)
		z = eta;
		Rcode = 0;
	end

	f = zeta + 0.05*log(z);
	for ii=1:4
		if ih(ii)
			y = y + (x(ii)/c(ii))*GH(:, :, ii);
		end
		G = y/z;
	end
end
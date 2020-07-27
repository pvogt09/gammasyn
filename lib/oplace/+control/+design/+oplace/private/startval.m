function [c, optflag] = startval(K, npq, ew, w1, G_lam, w2, G_xi, p_xi, w3, P, H, W4, T1, interactive)
	% [c,zeta] = VORGABE_C(K,npq,w1,G_lam,w2,G_xi,p_xi,w3,P,H,W4)
	% berechnet vor der eigentlichen Optimierung die Startgütefunktionen.
	% Außerdem kann der Anwender die Werte modifizieren, um so gezielt
	% die Gütekriterien untereinander zu gewichten.
	%
	% OUTPUT:
	% c      - Vorgabevektor für die einzelnen Gütekriterien
	% zeta   - Optimierungskonstante: zeta = max[Ji(K0)/c(i)]
	%
	% INPUT:
	% K      - die zu optimierende Matrix (Anfangswert)
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
	if nargin <= 13
		interactive = true;
	end
	if ~islogical(interactive)
		error('oplace:arguments', 'Interactive indicator must be of type ''logical''.');
	end

	optflag = 1;
	c = zeros(4, 1);
	eta = control.design.oplace.machconst(1);

	if npq(4)
		c(1) = criterion_1(K, w1, G_lam, npq, ew) + eta;
	end

	if npq(5)
		c(2) = criterion_2(K, w2, G_xi, p_xi, npq) + eta;
		if npq(4)
			optflag = 0;
		end
	end

	if npq(6)
		optflag = 0;
		c(3) = criterion_3(K,w3,P,H,npq) + eta;
	end

	if ~isempty(W4)
		optflag = 0;
		c(4) = criterion_4(K, W4, T1) + eta;
	end

	if optflag
		c = [];
		return;
	end

	h = c;
	c = h ./ 0.99;

	if interactive
		disp(' ');
		disp('For each criterion J(i) you can define an upper bound c(i) (c(i) > J(i))');
		disp('to individually influence the following multiobjective minimization.');

		while 1
			fprintf('\nStarting values and upper bounds of the individual criterions:\n');
			for ii = 1:4
				if c(ii)
					s = num2str(h(ii));
					if (length(s)+7)>=16
						fprintf('J(%d) : %s\t- upper bound c(%d) : %s\t(J/c = %s)\n', ii, s, ii, num2str(c(ii)), num2str(h(ii)/c(ii)));
					else
						fprintf('J(%d) : %s\t\t- upper bound c(%d) : %s\t(J/c = %s)\n', ii, s, ii, num2str(c(ii)), num2str(h(ii)/c(ii)));
					end
				else
					fprintf('J(%d) : %s\t\t- upper bound c(%d) : %s\n', ii, num2str(c(ii)), ii, num2str(c(ii)));
				end
			end
			inp = inp_from_keyb('\nWhich value do you want to change (end=0 or RETURN)? ', 2);
			if isempty(inp)
				choice = 0;
			else
				choice = str2double(inp);
			end
			if choice == 0
				break;
			end;
			if ~isempty(choice) && choice > 0 && choice < 5
				value = inp_from_keyb(sprintf('Input new upper bound c(%d) : ', choice));
				if value > h(choice)
					c(choice) = value;
				end
			end
		end
	end
end
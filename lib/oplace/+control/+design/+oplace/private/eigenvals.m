function [P_opt, ew_opt, st_opt, ews, w1, w2, npq, index] = eigenvals(ew, A, P, npq, interactive)
	% [ew_opt,st_opt,ew,w1,w2,npq] = EIGENVALUES(ew,A,B,C,npq) dient 
	% zur Kommunikation mit dem Anwender: hier werden sämtliche
	% Abfragen bezüglich der Eigenwerte, der Gewichtungsfaktoren w1,
	% der Stützstellen sowie der Gewichtungsfaktoren w2 durchgeführt.
	%
	% OUTPUT:
	% ew_opt - für die Optimierung zu nutzende EW (nicht mit 0 gewichtet)
	% st_opt - für die Optimierung zu nutzende Stützstellen
	% ew     - Vektor mit den vorgegebenen Eigenwerten
	% ews    - Vektor mit den Streckeneigenwerten
	% w1     - Gewichtungsfaktoren für J1
	% w2     - Gewichtungsfaktoren für J2
	% npq    - Dimensionen der vorliegenden Matrizen bzw. Vektoren  
	%
	% INPUT:
	% ew     - Vektor mit den vorgegebenen Eigenwerten
	% A      - Systemmatrix
	% B      - Eingangsmatrix 
	% C      - Ausgangsmatrix
	% npq    - Dimensionen der vorliegenden Matrizen bzw. Vektoren
	if nargin <= 4
		interactive = true;
	end
	if ~islogical(interactive)
		error('oplace:arguments', 'Interactive indicator must be of type ''logical''.');
	end
	% Teil I: EW-Kontrolle und ggf. EW-Änderung
	ende = 0;
	ews = eig(A);
	ews = sort(ews);
	flag = ev_test(ew, ews, npq);
	if interactive
		disp(' ');
		disp('It is checked wether there are multiple predefined closed-loop eigenvalues and/or');
		disp('wether some of them are too close to eigenvalues of the uncontrolled system. In');
		disp('these cases you can change the corresponding closed-loop eigenvalues or define');
		disp('almost arbitrary real points instead (they must only be distinct from each other');
		disp('and the open-loop eigenvalues). Thus, together with the different closed-loop');
		disp('eigenvalues they uniquely determine the desired closed-loop characteristic polynomial.');
		while ende ~= 1
			evalue_output(ew, ews, flag, npq);
			inp = inp_from_keyb('Which eigenvalue shall be changed? (end=0 or RETURN) ', 2);
			if isempty(inp)
				wahl = 0;
			else
				wahl = str2double(inp);
			end
			if wahl > 0 && wahl <= npq(1)
				inp = inp_from_keyb('Input new eigenvalue: ', 2);
				wert = str2double(inp);
				if ~isempty(wert)
					ew(wahl) = wert;
					[ew, P] = sortewpv(ew, P, npq);
					flag = ev_test(ew, ews, npq);	
				end
			elseif wahl == 0
				ende = 1;
			end
		end
	end
	% Teil II: die nicht mit 0 gewichteten EW gewichten
	npq(4) = sum(~flag); 
	[ew_opt, P_opt, index] = weight_ev(ew, ~flag, P, npq);
	w1 = weight_1(ew_opt, ew, ews, npq);	% Vorschlagswerte
	flag = 1;
	if interactive
		while ende ~= 2
			output_w(npq, ew_opt, w1, flag);
			inp = inp_from_keyb('Which weighting factor shall be changed? (end=0 or RETURN) ', 2);
			if isempty(inp)
				wahl = 0;
			else
				wahl = str2double(inp);
			end
			if wahl <= npq(4) && wahl > 0
				inp = inp_from_keyb(sprintf('Input w1(%d): ', wahl), 2);
				wert = str2double(inp);
				if ~isempty(wert) && wert >= 0
					w1(wahl) = wert;         
				end
			elseif wahl == 0
				ende = 2;
			end
		end
	end
	% falls ein EW mit 0 gewichtet ist, EW aus ew_opt eliminieren
	check = sum(sign(w1));
	if check && check < npq(4)
		h_ew1 = zeros(check, 1);	% Hilfsvariable für die gewichteten EW
		h_ew2 = zeros(npq(4) - check, 1);	% Hilfsvariable für die gewichteten EW
		h_w1 = zeros(check, 1);	% Hilfsvariable für die Gewichtungsfaktoren 
		h_P1 = zeros(npq(2) + 1, check);	% Hilfsvariable für die Parametervektoren
		h_P2 = zeros(npq(2) + 1, npq(4) - check);	% Hilfsvariable für die Parametervektoren
		idx_ew1 = zeros(check, 1);
		idx_ew2 = zeros(npq(4) - check, 1);
		jj = 1;
		nn = 1;
		for ii = 1:npq(4)
			if w1(ii) > 0
				h_ew1(jj) = ew_opt(ii); 
				idx_ew1(jj) = ii;
				h_w1(jj) = w1(ii);
				h_P1(:, jj) = P_opt(:, ii);
				jj = jj + 1;
			else
				h_ew2(nn) = ew_opt(ii); 
				idx_ew2(nn) = ii;
				h_P2(:, nn) = P_opt(:, ii);
				nn = nn+1;
			end
		end
		if npq(4) >= npq(1)
			ew_opt = [h_ew1; h_ew2];
			P_opt = [h_P1, h_P2];
			w1 = h_w1;
			index = [
				idx_ew1;
				idx_ew2
			];
		else
			ew_opt = [h_ew1; h_ew2; ew_opt(npq(4) + 1:npq(1))];
			P_opt = [h_P1, h_P2, P_opt(:, npq(4) + 1:npq(1))];
			w1 = h_w1;
			index = [
				idx_ew1;
				idx_ew2;
				index(npq(4) + 1:npq(1))
			];
		end
	end
	npq(4) = check;
	npq(5) = npq(1) - npq(4);

	% Teil III: Stützstellen abfragen
	if npq(5)
		st_opt = [];
		d = npq(5);
		fak = 40;
		for ii = 1:d
			okay = 0;
			while ~okay
				st = (rand - .5)*fak;
				okay = data_test(st, ew_opt, st_opt, ews, npq(1));
			end
			st_opt(ii,1) = st;
		end
		fprintf('\n\nThere are %d unweighted eigenvalues. Therefore, %d additional real data points\n', d, d);
		fprintf('have been defined to take these eigenvalues into account for the solution.\n');

		% Teil IV: Ausgabe der Stützstellen plus Vorschlag für Gewichtungsfaktor
		w2 = weight_2(st_opt, ew, ews, d);
		flag = 2;
		if interactive
			while ende ~= 3
				output_w(npq, st_opt, w2, flag);
				inp = inp_from_keyb('Which weighting factor shall be changed? (end=0 or RETURN) ', 2);
				if isempty(inp)
					wahl = 0;
				else
					wahl = str2double(inp);
				end
				if wahl <= d && wahl > 0
					inp = inp_from_keyb(sprintf('Input w2(%d): ', wahl), 2);
					wert = str2double(inp);
					if ~isempty(wert) && wert >= 0
						w2(wahl) = wert;
					end
				elseif wahl == 0
					ende = 3;
				end
			end
		end
	else
		st_opt = [];
		w2 = [];
	end

	% falls eine Stützstelle mit 0 gewichtet ist, Stützstelle aus st_opt eliminieren
	check = sum(sign(w2));
	if check && check < npq(5)
		st_1 = zeros(check, 1);	% Hilfsvariable für die gewichteten Stützstellen
		h_w2 = zeros(check, 1);	% Hilfsvariable für die Gewichtungsfaktoren
		jj = 1;
		for ii = 1:npq(5)
			if w2(ii) > 0       
				st_1(jj) = st_opt(ii); 
				h_w2(jj) = w2(ii);
				jj = jj + 1;
			end
		end
		npq(5) = check;
		st_opt = st_1;
		w2 = h_w2;
	elseif ~check
		npq(5) = 0;
		st_opt = [];
		w2 = [];
	end
	if (npq(4) + npq(5)) == 0
		error('oplace:arguments:constraints', 'No equations to solve. (w1 and w2 are both zero) !!!');
	end
end